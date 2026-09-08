#!/usr/bin/env python3
"""Optional pinned pedsim_ros adapter. No goals or internal forces reach planner."""
import math
import threading
from collections import OrderedDict
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from onboard_detector.msg import DynamicObstacles
from pedsim_msgs.msg import AgentStates


def fresh(stamp, now, age=.5):
    delta=(now-stamp).to_sec()
    return stamp.to_sec()>0 and 0<=delta<=age


def pedestrian_values(msg):
    people=[a for a in msg.agent_states if a.type != 2]
    if len(people)>1: raise ValueError('single pedestrian required')
    result=[]
    for a in people:
        if a.header.frame_id!=msg.header.frame_id or abs((a.header.stamp-msg.header.stamp).to_sec())>.05:
            raise ValueError('inconsistent agent header')
        p=a.pose.position;v=a.twist.linear
        if not all(math.isfinite(x) for x in [p.x,p.y,p.z,v.x,v.y,v.z]): raise ValueError('nonfinite agent')
        result.append((p,v))
    return result


class Bridge:
    def __init__(self):
        self.lock=threading.Lock();self.odoms=OrderedDict();self.inputs=OrderedDict();self.agents=OrderedDict()
        self.started=False;self.failed=False;self.last_now=None;self.last_pair=None
        self.auto_start=rospy.get_param('~auto_start',False)
        self.goal_time=None;self.guard_state='';self.release_attempted=False;self.workflow_state=''
        self.status_pub=rospy.Publisher('~status',String,queue_size=1,latch=True)
        self.goal_sub=rospy.Subscriber('/move_base_simple/goal',PoseStamped,self.on_goal,queue_size=1)
        self.guard_sub=rospy.Subscriber('/pedsim_simulator/guard_status',String,self.on_guard,queue_size=1)
        self.tf=tf2_ros.TransformBroadcaster()
        self.pub=rospy.Publisher('/onboard_detector/dynamic_obstacles_info',DynamicObstacles,queue_size=1)
        self.odom_sub=rospy.Subscriber('/sim_odom',Odometry,self.on_odom,queue_size=20)
        self.input_sub=rospy.Subscriber('/pedsim_simulator/robot_input',Odometry,self.on_input,queue_size=20)
        self.agent_sub=rospy.Subscriber('/pedsim_simulator/simulated_agents',AgentStates,self.on_agents,queue_size=20)
        self.timer=rospy.Timer(rospy.Duration(.1),self.watchdog)

    def on_goal(self,msg):
        with self.lock:
            if self.goal_time is None and not self.started:
                self.goal_time=rospy.Time.now()

    def on_guard(self,msg):
        with self.lock:
            self.guard_state=msg.data.split()[0].replace('state=','',1) if msg.data.split() else ''
            if self.guard_state.startswith('LATCHED_'):self.failed=True

    def workflow(self,now):
        if self.failed: state='FAULT_RESTART_ROUND'
        elif self.started: state='RUNNING'
        elif self.goal_time is None: state='WAIT_GOAL'
        elif not self.odoms: state='WAIT_PLANT'
        else:
            odom=next(reversed(self.odoms.values()))
            ready=fresh(odom.header.stamp,now) and odom.header.stamp>=self.goal_time and self.guard_state=='PAUSED_READY'
            state='WAIT_INPUT' if not ready else 'READY'
            if self.auto_start and ready and not self.release_attempted:
                # Exactly one attempt; no retry can clear a true guard latch.
                self.release_attempted=True
                try:
                    rospy.wait_for_service('/pedsim_simulator/unpause_simulation',timeout=.2)
                    rospy.ServiceProxy('/pedsim_simulator/unpause_simulation',Empty)()
                    state='WAIT_PAIRED_OUTPUT'
                except (rospy.ROSException,rospy.ServiceException):self.failed=True;state='FAULT_RESTART_ROUND'
            elif self.release_attempted:state='WAIT_PAIRED_OUTPUT'
        if state!=self.workflow_state:
            self.workflow_state=state;self.status_pub.publish(String(state));rospy.loginfo('pedsim: %s',state)

    def now_checked(self):
        now=rospy.Time.now()
        if self.last_now is not None and now<self.last_now:self.failed=True
        self.last_now=now
        return now

    def store(self,queue,key,msg):
        if key in queue and queue[key]!=msg:self.failed=True;return
        queue[key]=msg
        if len(queue)>32:self.failed=True;queue.popitem(last=False)

    def on_odom(self,msg):
        with self.lock:
            now=self.now_checked();p=msg.pose.pose.position;q=msg.pose.pose.orientation
            if msg.header.frame_id!='world' or not fresh(msg.header.stamp,now) or not all(math.isfinite(x) for x in [p.x,p.y,p.z,q.x,q.y,q.z,q.w]):
                self.failed=True;return
            if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>1e-3:self.failed=True;return
            # Old odom is needed for exact input-stamp pairing, not latest pose.
            for key,old in list(self.odoms.items()):
                if not fresh(old.header.stamp,now):del self.odoms[key]
            self.store(self.odoms,msg.header.stamp.to_nsec(),msg)
            t=TransformStamped();t.header=msg.header;t.child_frame_id='pedsim_user'
            t.transform.translation.x=p.x;t.transform.translation.y=p.y;t.transform.rotation=q
            self.tf.sendTransform(t)
            self.match(now)

    def on_input(self,msg):
        with self.lock:
            now=self.now_checked()
            try:key=int(msg.child_frame_id)
            except ValueError:self.failed=True;return
            if key<=0 or msg.header.frame_id!='world' or not fresh(msg.header.stamp,now):self.failed=True;return
            self.store(self.inputs,key,msg);self.match(now)

    def on_agents(self,msg):
        with self.lock:
            now=self.now_checked()
            if msg.header.frame_id!='world' or not fresh(msg.header.stamp,now):self.failed=True;return
            try:pedestrian_values(msg)
            except ValueError:self.failed=True;return
            self.store(self.agents,msg.header.stamp.to_nsec(),msg);self.match(now)

    def match(self,now):
        if self.failed:return
        for key,msg in list(self.agents.items()):
            if not fresh(msg.header.stamp,now):self.failed=True;return
            evidence=self.inputs.get(key)
            if evidence is None:continue # normal cross-topic reordering, not dependency loss
            if not fresh(evidence.header.stamp,now):self.failed=True;return
            odom=self.odoms.get(evidence.header.stamp.to_nsec())
            if odom is None:continue
            p=odom.pose.pose.position;q=evidence.pose.pose.position
            if not all(math.isfinite(x) for x in [q.x,q.y,q.z]) or math.hypot(p.x-q.x,p.y-q.y)>1e-8:
                self.failed=True;return
            # Compare orientation sign-invariantly as TF may negate quaternion.
            a=odom.pose.pose.orientation;b=evidence.pose.pose.orientation
            av=[a.x,a.y,a.z,a.w];bv=[b.x,b.y,b.z,b.w]
            if not all(math.isfinite(x) for x in bv) or min(sum((x-y)**2 for x,y in zip(av,bv)),sum((x+y)**2 for x,y in zip(av,bv)))>1e-12:
                self.failed=True;return
            values=pedestrian_values(msg);out=DynamicObstacles();out.header=msg.header;out.num=len(values)
            if values:out.header=next(a.header for a in msg.agent_states if a.type!=2)
            for p,v in values:
                out.position.append(Vector3(p.x,p.y,p.z));out.velocity.append(Vector3(v.x,v.y,v.z));out.size.append(Vector3(.5,.5,1.7))
            self.pub.publish(out);self.started=True;self.last_pair=now
            del self.agents[key];del self.inputs[key]
        for key,msg in list(self.inputs.items()):
            if not fresh(msg.header.stamp,now):self.failed=True;return

    def watchdog(self,_):
        with self.lock:
            now=self.now_checked();self.match(now)
            if self.started and (not self.odoms or not fresh(next(reversed(self.odoms.values())).header.stamp,now) or not fresh(self.last_pair,now)):
                self.failed=True
            if hasattr(self,'auto_start'):self.workflow(now)
            if self.failed:rospy.logwarn_throttle(5.,'pedsim input evidence lost/invalid: restart bridge AND simulator; no empty fallback')

if __name__=='__main__':
    rospy.init_node('pedsim_single_bridge');Bridge();rospy.spin()
