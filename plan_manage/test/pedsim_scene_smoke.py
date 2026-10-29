#!/usr/bin/env python3
"""Real guarded pedsim editor + Qt smoke check on a private owned ROS master.

Requires the optional pinned overlay and test_social_pedestrian_panel target.
The cache is isolated in a temporary directory; no user scene memory is touched.
"""
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))
from pedsim_round import run_owned, stop_owned
from export_pedsim_blank import export_world
from cane_pedestrian import social_scene


def child(directory):
    import rospy
    from geometry_msgs.msg import PoseStamped, PointStamped
    from nav_msgs.msg import Odometry
    from onboard_detector.msg import DynamicObstacles
    from plan_manage.msg import SocialSceneState
    from plan_manage.srv import ControlSocialScene, ControlSocialSceneRequest
    from pedsim_srvs.srv import EditScene, EditSceneRequest
    from std_msgs.msg import String
    processes = []
    editor = None
    try:
        processes.append(subprocess.Popen(['roslaunch', '--wait', 'plan_manage', 'pedsim_single_source.launch',
            'scene_file:='+str(directory/'world.xml'), 'pedestrian_speeds:=[0.45,0.5]',
            'enable_scene_editor:=true', 'auto_start:=true'], start_new_session=True))
        rospy.init_node('social_scene_smoke', disable_signals=True)
        states=[];observations=[];statuses=[];guards=[];published_odoms=[]
        subscribers=[rospy.Subscriber('/pedsim_scene/state',SocialSceneState,states.append,queue_size=30),
            rospy.Subscriber('/onboard_detector/dynamic_obstacles_info',DynamicObstacles,observations.append,queue_size=30),
            rospy.Subscriber('/pedsim_single_bridge/status',String,lambda m:statuses.append(m.data),queue_size=30)]
        subscribers.append(rospy.Subscriber('/pedsim_simulator/guard_status',String,lambda m:guards.append(m.data),queue_size=30))
        odom_pub=rospy.Publisher('/sim_odom',Odometry,queue_size=1)
        goal_pub=rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1)
        start_pub=rospy.Publisher('/pedsim_scene/start_pose',PoseStamped,queue_size=1)
        ped_goal_pub=rospy.Publisher('/pedsim_scene/goal_pose',PoseStamped,queue_size=1)
        select_pub=rospy.Publisher('/pedsim_scene/select_point',PointStamped,queue_size=1)
        def odom(_):
            m=Odometry();m.header.frame_id='world';m.header.stamp=rospy.Time.now()
            m.pose.pose.position.x=10;m.pose.pose.position.y=10;m.pose.pose.orientation.w=1
            odom_pub.publish(m)
            published_odoms.append(m.header.stamp.to_sec())
        timer=rospy.Timer(rospy.Duration(.05),odom)
        def wait(predicate,description,timeout=10.):
            end=time.monotonic()+timeout
            while time.monotonic()<end:
                if predicate():return
                for p in processes:
                    if p.poll() is not None:raise RuntimeError('Process exited: '+str(p.args))
                time.sleep(.025)
            raise AssertionError(description+'; latest state='+str(states[-1] if states else None))
        def start_editor(remember=True):
            nonlocal editor
            before=states[-1].revision if states else 0
            editor=subprocess.Popen(['rosrun','plan_manage','pedsim_scene_editor.py','__name:=pedsim_scene',
                '_scene_description:='+str(directory/'world.json'),'_scene_context:=smoke-test',
                '_editor_state_file:='+str(directory/'memory.yaml'),
                '_remember_last:='+str(remember).lower()],start_new_session=True)
            processes.append(editor)
            wait(lambda:states and states[-1].ready and states[-1].revision>before,'editor startup')
        def stop_editor():
            stop_owned(editor);processes.remove(editor)
        def command(name,**kwargs):
            request=ControlSocialSceneRequest(command=name,expected_revision=states[-1].revision,**kwargs)
            response=rospy.ServiceProxy('/pedsim_scene/control',ControlSocialScene)(request)
            if not response.success:raise AssertionError(name+': '+response.message)
            # Drain the acknowledgement publication before issuing another command.
            time.sleep(.23)
            return response
        def load(people):
            scene=dict(format='cane_pedsim_scene_v1',frame_id='world',world='blank',start_on_goal=False,pedestrians=people)
            command('load',scenario_yaml=social_scene.dumps(scene))
        def person(identity,y,behavior='stop',speed=1.4):
            return dict(id=identity,start=[0.,y,0.],goal=[.8,y],desired_speed=speed,end_behavior=behavior)

        start_editor()
        wait(lambda:goal_pub.get_num_connections()>0,'goal connections')
        g=PoseStamped();g.header.frame_id='world';g.pose.position.x=9;g.pose.position.y=10;g.pose.orientation.w=1
        goal_pub.publish(g)
        wait(lambda:statuses and statuses[-1]=='RUNNING' and observations,'paired startup')
        assert len(states[-1].pedestrians)==2
        # rospack's workspace layout may differ; locate the catkin test binary.
        qt=Path(subprocess.check_output(['catkin','locate','--devel'],cwd=str(PACKAGE),text=True).strip())/'lib/plan_manage/test_social_pedestrian_panel'
        subprocess.run([str(qt)],check=True,timeout=35)
        print('PASS real Qt/pluginlib add/apply/pause/save/load',flush=True)

        # Exercise map tools through their actual ROS topics and keep robot TF unchanged.
        command('clear');command('select',selected_id=0)
        p=PoseStamped();p.header.frame_id='world';p.pose.position.x=-2.;p.pose.position.y=-4.
        p.pose.orientation.z=math.sin(math.pi/4);p.pose.orientation.w=math.cos(math.pi/4)
        start_pub.publish(p)
        wait(lambda:len(states[-1].pedestrians)==1,'map start placement')
        assert abs(states[-1].pedestrians[0].start.theta-math.pi/2)<1e-6
        p.pose.position.x=-1.;p.pose.position.y=-2.;ped_goal_pub.publish(p)
        wait(lambda:states[-1].pedestrians[0].goal.x==-1.,'map goal placement')
        point=PointStamped();point.header.frame_id='world';point.point.x=-2.;point.point.y=-4.
        select_pub.publish(point)
        wait(lambda:states[-1].selected_id!=0,'map selection')
        revision=states[-1].revision
        bad=rospy.ServiceProxy('/pedsim_scene/control',ControlSocialScene)(
            ControlSocialSceneRequest(command='clear',expected_revision=revision-1))
        assert not bad.success and states[-1].revision==revision
        invalid=EditSceneRequest(command='replace',expected_revision=revision)
        # Array mismatch must not mutate simulation or erase its guard.
        invalid.desired_speeds=[1.]
        response=rospy.ServiceProxy('/pedsim_simulator/edit_scene',EditScene)(invalid)
        assert not response.success and response.revision==revision

        load([person(1,0,'stop'),person(2,3,'return'),person(3,6,'repeat'),person(4,-6,'stop',0.)])
        command('start');time.sleep(.4);command('pause')
        wait(lambda:observations[-1].num==4 and all(math.hypot(v.x,v.y)<1e-9 for v in observations[-1].velocity),'zero observed pause speed')
        paused=[(p.x,p.y) for p in states[-1].poses];stamp=observations[-1].header.stamp
        time.sleep(.85)
        assert statuses[-1]=='RUNNING' and states[-1].ready
        assert observations[-1].header.stamp>stamp
        assert max(math.hypot(p.x-x,p.y-y) for p,(x,y) in zip(states[-1].poses,paused))<1e-8
        command('start');start_index=len(states)
        wait(lambda:states[-1].phases[:2]==['finished','finished'],'stop and return finish',12.)
        assert any(s.phases[1]=='returning' for s in states[start_index:])
        assert any(s.phases[2]=='returning' for s in states[start_index:])
        wait(lambda:any(s.phases[2]=='walking' for s in states[start_index:] if s.header.stamp>states[start_index].header.stamp+rospy.Duration(2.)), 'repeat turns back',7.)
        assert abs(states[-1].poses[0].x-.8)<.22 and abs(states[-1].poses[1].x)<.22
        assert abs(states[-1].poses[3].x)<1e-9
        command('pause');command('delete')
        wait(lambda:observations[-1].num==3,'delete reflected in observations')
        print('PASS map tools, invalid/stale edits, stop/return/repeat, pause heartbeat',flush=True)

        command('clear');time.sleep(.8)
        assert observations[-1].num==0 and statuses[-1]=='RUNNING', dict(
            guards=guards[-3:],odom_gaps=sorted((b-a for a,b in zip(published_odoms,published_odoms[1:])),reverse=True)[:4],
            now=rospy.Time.now().to_sec(),last_odom=published_odoms[-1],observation=observations[-1])
        cached=(directory/'memory.yaml').read_text();assert social_scene.loads(cached)['pedestrians']==[]
        stop_editor();start_editor();assert states[-1].pedestrians==[] and not states[-1].running
        stop_editor();start_editor(remember=False)
        assert len(states[-1].pedestrians)==2 and (directory/'memory.yaml').read_text()==cached
        stop_editor();(directory/'memory.yaml').write_text('invalid: [')
        start_editor();assert len(states[-1].pedestrians)==2
        print('PASS fresh empty observations, empty-scene memory, disabled memory, corrupt-cache fallback',flush=True)

        # A real lost robot input still latches; editor start/reset cannot clear it.
        timer.shutdown()
        wait(lambda:statuses[-1]=='FAULT_RESTART_ROUND' and not states[-1].ready,'true dependency loss')
        request=ControlSocialSceneRequest(command='start',expected_revision=states[-1].revision)
        assert not rospy.ServiceProxy('/pedsim_scene/control',ControlSocialScene)(request).success
        print('PASS true loss remains latched; editor cannot resume',flush=True)
        (directory/'results.json').write_text(json.dumps(dict(observations=len(observations),states=len(states),passed=True)))
        return 0
    finally:
        if 'guards' in locals():
            print('DIAGNOSTICS',dict(guards=guards[-2:],
                odom_max_gap=max((b-a for a,b in zip(published_odoms,published_odoms[1:])),default=0.),
                observed_max_gap=max(((b.header.stamp-a.header.stamp).to_sec() for a,b in zip(observations,observations[1:])),default=0.)),flush=True)
        for process in reversed(processes):stop_owned(process)


def main():
    os.environ['QT_QPA_PLATFORM']='offscreen'
    if len(sys.argv)==3 and sys.argv[1]=='--child':return child(Path(sys.argv[2]))
    with socket.socket() as sock:sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    os.environ['ROS_MASTER_URI']='http://127.0.0.1:%d'%port
    os.environ['ROS_HOSTNAME']='127.0.0.1';os.environ.pop('ROS_IP',None)
    directory=Path(tempfile.mkdtemp(prefix='cane-pedsim-stage4-test.'))
    def prepare():
        export_world(Path.home()/'cane-pedsim-ws/upstream/pedsim_simulator/scenarios/blank_world.xml',directory,'crowd_2')
    print('PRIVATE_MASTER',port,'EVIDENCE',directory,flush=True)
    return run_owned(port,prepare,lambda _:['/usr/bin/python3',str(Path(__file__).resolve()),'--child',str(directory)])


if __name__=='__main__':raise SystemExit(main())
