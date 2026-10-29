#!/usr/bin/env python3
import importlib.util
import sys
import unittest
import threading
from collections import OrderedDict
from unittest.mock import patch, Mock
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
from pedsim_srvs.srv import EditSceneResponse
from geometry_msgs.msg import PoseStamped, Pose2D
editor_module=load('pedsim_scene_editor')
class Interfaces(unittest.TestCase):
    def test_boundaries(self):
        self.assertEqual(4,len(export.boundaries({(0,0)})))
        self.assertEqual(4,len(export.boundaries({(0,0),(1,0)})))
        self.assertEqual(8,len(export.boundaries({(0,0),(2,0)}))) # gap never joined
    def test_freshness(self):
        self.assertTrue(bridge.fresh(rospy.Time(10),rospy.Time(10.1)))
        self.assertFalse(bridge.fresh(rospy.Time(10),rospy.Time(11)))
        self.assertFalse(bridge.fresh(rospy.Time(12),rospy.Time(11)))
    def test_multi_robot_order_empty_and_limit(self):
        msg=AgentStates();msg.header.frame_id='world';msg.header.stamp=rospy.Time(10)
        robot=AgentState();robot.type=2;msg.agent_states=[robot]
        self.assertEqual([],bridge.pedestrian_values(msg))
        person=AgentState();person.header=msg.header;msg.agent_states.append(person)
        self.assertEqual(1,len(bridge.pedestrian_values(msg)))
        msg.agent_states.append(person)
        with self.assertRaises(ValueError):bridge.pedestrian_values(msg)
        msg.agent_states=[robot]
        for i in reversed(range(5)):
            person=AgentState();person.header=msg.header;person.id=i;person.pose.position.x=i
            msg.agent_states.append(person)
        self.assertEqual(list(range(5)),[p.x for p,v in bridge.pedestrian_values(msg)])
        person=AgentState();person.header=msg.header;person.id=5;msg.agent_states.append(person)
        with self.assertRaisesRegex(ValueError,'five'):bridge.pedestrian_values(msg)
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
        b.scene_revision=None;b.scene_cutoff=0;b.last_output=0
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
        b.editor_required=False;b.editor_ready=False
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

    def test_editor_must_acknowledge_before_release(self):
        b=bridge.Bridge.__new__(bridge.Bridge);b.auto_start=True;b.started=False;b.failed=False
        b.editor_required=True;b.editor_ready=False;b.goal_time=rospy.Time(10)
        b.guard_state='PAUSED_READY';b.release_attempted=False;b.workflow_state=''
        o=Odometry();o.header.stamp=rospy.Time(10);b.odoms=OrderedDict([(1,o)])
        b.status_pub=type('Pub',(),{'publish':lambda _,m:None})()
        with patch.object(rospy,'loginfo'),patch.object(rospy,'wait_for_service'),patch.object(rospy,'ServiceProxy') as service:
            b.workflow(rospy.Time(10));self.assertEqual('WAIT_EDITOR_READY',b.workflow_state);service.assert_not_called()
            b.editor_ready=True;b.workflow(rospy.Time(10));self.assertEqual(1,service.call_count)


class SceneVersions(unittest.TestCase):
    def setUp(self):
        self.b=bridge.Bridge.__new__(bridge.Bridge)
        b=self.b;b.lock=threading.Lock();b.odoms=OrderedDict();b.inputs=OrderedDict();b.agents=OrderedDict()
        b.started=False;b.failed=False;b.last_now=None;b.last_pair=None
        b.scene_revision=None;b.scene_cutoff=0;b.last_output=0
        self.outputs=[]
        b.pub=type('Pub',(),{'publish':lambda _,msg:self.outputs.append(msg)})()
        b.tf=type('TF',(),{'sendTransform':lambda _,msg:None})()
        self.now=patch.object(rospy.Time,'now',return_value=rospy.Time(10)).start()

    def tearDown(self):patch.stopall()

    def batch(self,t,revision,people=1):
        self.now.return_value=rospy.Time(t)
        o=Odometry();o.header.frame_id='world';o.header.stamp=rospy.Time(t);o.pose.pose.orientation.w=1
        self.b.on_odom(o)
        a=AgentStates();a.header=o.header
        for i in range(people):
            p=AgentState();p.header=a.header;p.id=i;a.agent_states.append(p)
        e=Odometry();e.header=o.header;e.pose=o.pose;e.child_frame_id='%d:%d'%(a.header.stamp.to_nsec(),revision)
        return a,e

    def test_new_agents_first_discards_old_evidence_and_late_agents(self):
        b=self.b
        old,old_e=self.batch(10,1);b.on_input(old_e)
        new,new_e=self.batch(10.1,2,0);b.on_agents(new);b.on_input(new_e)
        b.on_agents(old);b.on_input(old_e)
        self.assertFalse(b.failed);self.assertEqual(1,len(self.outputs));self.assertEqual(0,self.outputs[-1].num)
        # Late old packets remain ignorable, rather than masquerading as loss.
        self.now.return_value=rospy.Time(10.8);b.on_agents(old);b.on_input(old_e)
        self.assertFalse(b.failed)
        b.watchdog(None);self.assertTrue(b.failed)  # real missing heartbeat still latches

    def test_new_evidence_first_pause_and_empty_remain_fresh(self):
        b=self.b
        for i in range(12):
            a,e=self.batch(10+i*.1,1+i//3,0 if i>=6 else 1)
            b.on_input(e);b.on_agents(a);b.watchdog(None)
            self.assertFalse(b.failed)
        self.assertEqual(12,len(self.outputs));self.assertEqual(0,self.outputs[-1].num)

    def test_old_same_revision_cannot_replace_latest_output(self):
        b=self.b
        old,oe=self.batch(10,1)
        new,ne=self.batch(10.1,1);b.on_input(ne);b.on_agents(new)
        b.on_input(oe);b.on_agents(old)
        self.assertFalse(b.failed);self.assertEqual(1,len(self.outputs))

    def test_malformed_evidence_still_latches(self):
        a,e=self.batch(10,1);e.child_frame_id='100:broken';self.b.on_input(e)
        self.assertTrue(self.b.failed)

    def test_superseded_unpaired_input_cannot_expire_after_new_output(self):
        b=self.b
        old,old_e=self.batch(10,1);b.on_input(old_e)
        for i in range(1,9):
            new,new_e=self.batch(10+i*.1,1)
            b.on_input(new_e);b.on_agents(new)
            if i==2:b.on_agents(old)
            b.watchdog(None)
            self.assertFalse(b.failed)
        self.assertEqual(8,len(self.outputs))

    def test_simulation_clock_delivery_order_does_not_make_valid_batch_future(self):
        b=self.b;b.simulation_time=True
        agents,evidence=self.batch(9.99,1)
        agents.header=type(agents.header)(frame_id='world',stamp=rospy.Time(10))
        for person in agents.agent_states:person.header=agents.header
        evidence.child_frame_id='%d:1'%agents.header.stamp.to_nsec()
        b.on_input(evidence)
        # The agent and /clock messages travel on different connections.
        # Catching up the clock must precede the unchanged freshness check.
        self.now.side_effect=[rospy.Time(9.999),rospy.Time(10),rospy.Time(10)]
        with patch.object(bridge.time,'sleep'):
            b.on_agents(agents)
        self.assertFalse(b.failed);self.assertEqual(1,len(self.outputs))
        self.assertEqual(agents.header.stamp,self.outputs[-1].header.stamp)

    def test_simulation_future_stamp_still_faults_if_clock_never_catches_up(self):
        b=self.b;b.simulation_time=True
        agents,evidence=self.batch(9.99,1)
        agents.header=type(agents.header)(frame_id='world',stamp=rospy.Time(10))
        for person in agents.agent_states:person.header=agents.header
        self.now.return_value=rospy.Time(9.999)
        b.on_agents(agents)
        self.assertTrue(b.failed);self.assertEqual([],self.outputs)

    def test_simulation_delivery_wait_does_not_hide_real_clock_rollback(self):
        b=self.b;b.simulation_time=True
        agents,evidence=self.batch(10,1)
        self.now.return_value=rospy.Time(9.999)
        b.on_agents(agents)
        self.assertTrue(b.failed);self.assertEqual([],self.outputs)


class EditorAcknowledgements(unittest.TestCase):
    def setUp(self):
        self.editor=editor_module.Editor.__new__(editor_module.Editor)
        e=self.editor;e.revision=7;e.initialized=True;e.desynchronized=False;e.healthy=False
        e.scene=dict(format='cane_pedsim_scene_v1',frame_id='world',world='blank',start_on_goal=False,pedestrians=[])
        e.snapshot=None;e.selected=0

    def test_missing_snapshot_cannot_accept_external_scene_revision(self):
        e=self.editor;e.upstream=Mock(return_value=EditSceneResponse(success=True,revision=8))
        with self.assertRaisesRegex(ValueError,'outside'):e.call('state')
        self.assertTrue(e.desynchronized)
        e.upstream.return_value=EditSceneResponse(success=True,revision=7)
        with self.assertRaisesRegex(ValueError,'outside'):e.call('state')
        self.assertEqual(7,e.revision)

    def test_rejected_or_malformed_ack_does_not_replace_configuration(self):
        e=self.editor;previous=e.scene
        e.upstream=Mock(return_value=EditSceneResponse(success=False,message='stale revision',revision=8))
        with self.assertRaisesRegex(ValueError,'stale'):e.replace(previous)
        self.assertIs(previous,e.scene);self.assertEqual(7,e.revision)
        e.upstream.return_value=EditSceneResponse(success=True,revision=8,poses=[Pose2D()])
        with self.assertRaisesRegex(ValueError,'acknowledgement'):e.replace(previous)
        self.assertIs(previous,e.scene);self.assertEqual(7,e.revision)

    def test_map_pose_requires_world_planar_geometry(self):
        p=PoseStamped();p.header.frame_id='world';p.pose.orientation.w=1.
        self.assertEqual([0.,0.,0.],self.editor.world_pose(p))
        p.header.frame_id='map'
        with self.assertRaises(ValueError):self.editor.world_pose(p)
        p.header.frame_id='world';p.pose.position.z=.1
        with self.assertRaises(ValueError):self.editor.world_pose(p)
        p.pose.position.z=0.;p.pose.orientation.x=.1
        with self.assertRaises(ValueError):self.editor.world_pose(p)

if __name__=='__main__':unittest.main()
