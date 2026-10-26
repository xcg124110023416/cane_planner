#!/usr/bin/env python3
"""Verify actual simulator restarts, file loads, and cache isolation from user data."""
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

import rospy
import rostest
import yaml
from onboard_detector.msg import DynamicObstacles
from plan_manage.msg import PedestrianState
from plan_manage.srv import ControlPedestrian


class PedestrianPersistenceTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.cache = Path(self.directory.name) / "last.yaml"
        self.proc = None
        self.log = open(Path(self.directory.name) / "simulator.log", "w")
        rospy.set_param("/pedestrian_sim/frame_id", "world")
        rospy.set_param("/pedestrian_sim/scenario", "dwcg_crossing")
        rospy.set_param("/pedestrian_sim/editor_state_file", str(self.cache))
        rospy.set_param("/pedestrian_sim/remember_last", True)
        self.start_simulator()

    def start_simulator(self):
        script = Path(__file__).resolve().parents[1] / "scripts" / "pedestrian_sim.py"
        self.proc = subprocess.Popen([sys.executable, str(script), "__name:=pedestrian_sim"],
                                     stdout=self.log, stderr=self.log)
        rospy.wait_for_service("/pedestrian_sim/control", timeout=10)
        self.control = rospy.ServiceProxy("/pedestrian_sim/control", ControlPedestrian)

    def stop_simulator(self):
        if self.proc is not None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=5)
            self.proc = None

    def restart(self):
        self.stop_simulator()
        self.start_simulator()

    def state(self):
        return rospy.wait_for_message("/pedestrian_sim/state", PedestrianState, timeout=5)

    def wait_observation(self, predicate):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            obs = rospy.wait_for_message("/onboard_detector/dynamic_obstacles_info",
                                         DynamicObstacles, timeout=2)
            if predicate(obs):
                return obs
        self.fail("Timed out waiting for dynamic observation")

    def test_load_and_restart_preserve_applied_configuration(self):
        self.assertEqual(self.state().phase, "legacy_trigger")
        config = dict(mode="controlled", frame_id="world", start=[4, -3, 0], yaw=1.2,
                      speed=0.8, duration=3, size=[0.6, 0.7, 1.8],
                      end_behavior="repeat", start_on_goal=False)
        content = yaml.safe_dump({"saved_scene": [config]})
        response = self.control(command="load", scenario_yaml=content)
        self.assertTrue(response.success, response.message)
        state = self.state()
        self.assertEqual(state.phase, "waiting")
        self.assertAlmostEqual(state.speed, 0.8)
        self.assertEqual(state.start_pose.position.x, 4)
        self.assertEqual(state.size.y, 0.7)
        self.assertTrue(state.remember_last)
        self.assertTrue(self.cache.exists())
        saved = self.cache.read_text()
        exported = self.control(command="export")
        self.assertTrue(exported.success)
        self.assertEqual(yaml.safe_load(exported.scenario_yaml), yaml.safe_load(saved))
        self.assertTrue(self.control(command="start").success)
        self.wait_observation(lambda obs: obs.num == 1 and obs.position[0].x > 4.01)

        for bad in ("[broken", "a: [{}, {}]", yaml.safe_dump({"scene": [dict(config, frame_id="map")]}),
                    yaml.safe_dump({"scene": [dict(config, speed=-1)]})):
            response = self.control(command="load", scenario_yaml=bad)
            self.assertFalse(response.success)
            self.assertEqual(self.state().phase, "walking")
            self.assertEqual(self.cache.read_text(), saved)

        self.restart()
        state = self.state()
        self.assertEqual(state.phase, "waiting")
        self.assertEqual(state.end_behavior, "repeat")
        self.assertAlmostEqual(state.speed, 0.8)
        self.assertFalse(state.start_on_goal)
        self.assertEqual(state.start_pose.position.y, -3)
        self.wait_observation(lambda obs: obs.num == 1 and obs.position[0].x == 4
                              and obs.velocity[0].x == 0)

        self.assertTrue(self.control(command="clear").success)
        self.restart()
        self.assertFalse(self.state().placed)
        self.wait_observation(lambda obs: obs.num == 0)
        saved = self.cache.read_text()

        rospy.set_param("/pedestrian_sim/remember_last", False)
        self.restart()
        self.assertEqual(self.state().phase, "legacy_trigger")
        self.assertTrue(self.control(command="reset").success)
        self.assertEqual(self.cache.read_text(), saved)

        # A damaged cache falls back to the launch scene, without crashing startup.
        self.cache.write_text("[broken")
        rospy.set_param("/pedestrian_sim/remember_last", True)
        self.restart()
        self.assertEqual(self.state().phase, "legacy_trigger")

    def tearDown(self):
        self.stop_simulator()
        self.log.close()
        self.directory.cleanup()


if __name__ == "__main__":
    rospy.init_node("pedestrian_persistence_test")
    rostest.rosrun("plan_manage", "pedestrian_persistence", PedestrianPersistenceTest)
