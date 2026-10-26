#!/usr/bin/env python3
"""Exercise the actual simulator's control, TF, and fresh-observation contracts."""
import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from onboard_detector.msg import DynamicObstacles
from plan_manage.msg import PedestrianState
from plan_manage.srv import ControlPedestrian


class PedestrianEditorRosTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.state = None
        self.obs = None
        self.goals = 0
        self.state_sub = rospy.Subscriber("/pedestrian_sim/state", PedestrianState, self.on_state)
        self.obs_sub = rospy.Subscriber("/onboard_detector/dynamic_obstacles_info",
                                        DynamicObstacles, self.on_obs)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.on_goal)
        self.place = rospy.Publisher("/pedestrian_sim/initial_pose", PoseStamped, queue_size=1)
        self.goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        rospy.wait_for_service("/pedestrian_sim/control", timeout=10)
        self.control = rospy.ServiceProxy("/pedestrian_sim/control", ControlPedestrian)
        self.wait(lambda: self.state is not None and self.obs is not None
                  and self.place.get_num_connections() > 0 and self.goal.get_num_connections() > 0)

    def on_state(self, msg):
        with self.lock:
            self.state = msg

    def on_obs(self, msg):
        with self.lock:
            self.obs = msg

    def on_goal(self, msg):
        with self.lock:
            self.goals += 1

    def wait(self, predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                if predicate():
                    return
            time.sleep(0.02)
        self.fail("Timed out waiting for simulator state/observation")

    def command(self, command, **kwargs):
        response = self.control(command=command, **kwargs)
        self.assertTrue(response.success, response.message)

    def test_editor_workflow(self):
        self.assertEqual(self.obs.num, 0)  # Original trigger behavior before interaction.
        pose = PoseStamped()
        pose.header.frame_id = "editor_test_frame"
        pose.pose.position.x, pose.pose.position.y = 1, 2
        pose.pose.orientation.z = math.sin(math.pi / 4)
        pose.pose.orientation.w = math.cos(math.pi / 4)
        self.place.publish(pose)
        self.wait(lambda: self.state.phase == "waiting" and self.obs.num == 1)
        self.assertAlmostEqual(self.state.start_pose.position.x, 11)
        self.assertEqual(self.obs.header.frame_id, "world")
        self.assertEqual(self.goals, 0)  # Placing a person must not publish a robot goal.

        self.command("apply", speed=1, duration=1, end_behavior="return", start_on_goal=False)
        self.goal.publish(pose)
        self.wait(lambda: self.goals == 1)
        self.assertEqual(self.state.phase, "waiting")
        self.command("start")
        self.wait(lambda: self.obs.num == 1 and self.obs.position[0].y > 2.2)
        self.command("pause")
        self.wait(lambda: self.state.phase == "paused" and self.obs.velocity[0].y == 0)
        paused_y, stamp = self.obs.position[0].y, self.obs.header.stamp
        self.wait(lambda: (self.obs.header.stamp - stamp).to_sec() > 0.5)
        self.assertAlmostEqual(self.obs.position[0].y, paused_y)
        self.command("start")
        self.wait(lambda: self.obs.velocity[0].y < 0)
        self.wait(lambda: self.state.phase == "finished")
        self.wait(lambda: self.obs.velocity[0].y == 0 and abs(self.obs.position[0].y - 2) < 1e-8)
        self.assertEqual(self.obs.num, 1)

        self.command("apply", speed=0.4, duration=0.3, end_behavior="disappear", start_on_goal=True)
        self.goal.publish(pose)
        self.wait(lambda: self.state.phase == "walking")
        self.wait(lambda: self.state.phase == "finished" and self.obs.num == 0)
        self.command("reset")
        self.wait(lambda: self.state.phase == "waiting" and self.obs.num == 1)

        for kwargs in (dict(speed=-1, duration=1, end_behavior="stop"),
                       dict(speed=1, duration=0, end_behavior="stop"),
                       dict(speed=1, duration=1, end_behavior="typo")):
            response = self.control(command="apply", **kwargs)
            self.assertFalse(response.success)
        self.assertAlmostEqual(self.state.speed, 0.4)
        self.command("clear")
        self.wait(lambda: not self.state.placed and self.obs.num == 0)
        stamp = self.obs.header.stamp
        self.wait(lambda: (self.obs.header.stamp - stamp).to_sec() > 0.5)
        self.assertEqual(self.obs.num, 0)  # Clear keeps publishing fresh empty frames.
        self.place.publish(pose)
        self.wait(lambda: self.state.placed and self.obs.num == 1)


if __name__ == "__main__":
    rospy.init_node("pedestrian_editor_test")
    rostest.rosrun("plan_manage", "pedestrian_editor", PedestrianEditorRosTest)
