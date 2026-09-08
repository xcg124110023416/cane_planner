#!/usr/bin/env python3
import importlib.util
import sys
import unittest
import threading
from collections import OrderedDict
from unittest.mock import patch
from nav_msgs.msg import Odometry
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[1]/'scripts'
def load(name):
    spec=importlib.util.spec_from_file_location(name,ROOT/(name+'.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
export=load('export_pedsim_office')
bridge=load('pedsim_single_bridge')
import rospy
from pedsim_msgs.msg import AgentState,AgentStates
class Interfaces(unittest.TestCase):
    def test_boundaries(self):
        self.assertEqual(4,len(export.boundaries({(0,0)})))
        self.assertEqual(4,len(export.boundaries({(0,0),(1,0)})))
        self.assertEqual(8,len(export.boundaries({(0,0),(2,0)}))) # gap never joined
    def test_freshness(self):
        self.assertTrue(bridge.fresh(rospy.Time(10),rospy.Time(10.1)))
        self.assertFalse(bridge.fresh(rospy.Time(10),rospy.Time(11)))
        self.assertFalse(bridge.fresh(rospy.Time(12),rospy.Time(11)))
    def test_single_and_robot(self):
        msg=AgentStates();msg.header.frame_id='world';msg.header.stamp=rospy.Time(10)
        robot=AgentState();robot.type=2;msg.agent_states=[robot]
        self.assertEqual([],bridge.pedestrian_values(msg))
        person=AgentState();person.header=msg.header;msg.agent_states.append(person)
        self.assertEqual(1,len(bridge.pedestrian_values(msg)))
        msg.agent_states.append(person)
        with self.assertRaises(ValueError):bridge.pedestrian_values(msg)
    def test_bad_header_and_nonfinite(self):
        msg=AgentStates();msg.header.frame_id='world';msg.header.stamp=rospy.Time(10)
        p=AgentState();msg.agent_states=[p]
        with self.assertRaises(ValueError):bridge.pedestrian_values(msg)
        p.header=msg.header;p.pose.position.x=float('nan')
        with self.assertRaises(ValueError):bridge.pedestrian_values(msg)
class Pairing(unittest.TestCase):
    def test_actual_callbacks_reordering_motion_and_loss(self):
        b=bridge.Bridge.__new__(bridge.Bridge)
        b.lock=threading.Lock();b.odoms=OrderedDict();b.inputs=OrderedDict();b.agents=OrderedDict()
        b.started=False;b.failed=False;b.last_now=None;b.last_pair=None
        outputs=[]
        class Pub:
            def publish(self,msg):outputs.append(msg)
        class TF:
            def sendTransform(self,msg):pass
        b.pub=Pub();b.tf=TF()
        with patch.object(rospy.Time,'now',return_value=rospy.Time(10)) as now:
            for i in range(10):
                now.return_value=rospy.Time(10+i*.1)
                o=Odometry();o.header.frame_id='world';o.header.stamp=now.return_value;o.pose.pose.position.x=i*.12;o.pose.pose.orientation.w=1
                b.on_odom(o)
                a=AgentStates();a.header.frame_id='world';a.header.stamp=now.return_value
                person=AgentState();person.header=a.header;a.agent_states=[person]
                e=Odometry();e.header=o.header;e.pose=o.pose;e.child_frame_id=str(a.header.stamp.to_nsec())
                if i%2:b.on_input(e);b.watchdog(None);b.on_agents(a)
                else:b.on_agents(a);b.watchdog(None);b.on_input(e)
                self.assertFalse(b.failed);self.assertEqual(i+1,len(outputs))
            now.return_value=rospy.Time(12);b.watchdog(None);self.assertTrue(b.failed)
            o.header.stamp=now.return_value;b.on_odom(o)
            self.assertTrue(b.failed);self.assertEqual(10,len(outputs))

class AutoStart(unittest.TestCase):
    def test_first_goal_ready_only_and_failure_no_restart(self):
        b=bridge.Bridge.__new__(bridge.Bridge);b.auto_start=True;b.started=False;b.failed=False
        b.goal_time=None;b.guard_state='PAUSED_READY';b.release_attempted=False;b.workflow_state='';b.odoms=OrderedDict()
        states=[]
        class Pub:
            def publish(self,m):states.append(m.data)
        b.status_pub=Pub()
        with patch.object(rospy,'loginfo'),patch.object(rospy,'wait_for_service'),patch.object(rospy,'ServiceProxy') as service:
            b.workflow(rospy.Time(10));self.assertEqual('WAIT_GOAL',states[-1]);service.assert_not_called()
            b.goal_time=rospy.Time(10);b.workflow(rospy.Time(10));self.assertEqual('WAIT_PLANT',states[-1])
            o=Odometry();o.header.stamp=rospy.Time(9.9);b.odoms[1]=o;b.workflow(rospy.Time(10));service.assert_not_called()
            o.header.stamp=rospy.Time(10);b.workflow(rospy.Time(10));self.assertEqual(1,service.call_count)
            b.workflow(rospy.Time(10.1));self.assertEqual(1,service.call_count)
            b.started=True;b.workflow(rospy.Time(10.2));self.assertEqual('RUNNING',states[-1])
            b.failed=True;b.workflow(rospy.Time(11));self.assertEqual('FAULT_RESTART_ROUND',states[-1]);self.assertEqual(1,service.call_count)

if __name__=='__main__':unittest.main()
