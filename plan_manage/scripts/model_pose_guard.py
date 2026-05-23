#!/usr/bin/env python3
"""Keep the Gazebo cane upright while preserving planar motion.

The real cane is supported by the user's hand. Gazebo contacts can still inject
roll/pitch into the model during sharp turns or collisions, so this node acts as
that hand support: it clamps roll and pitch to zero and keeps the nominal spawn
height, while leaving x, y, yaw, and planar twist untouched.
"""

import math

import rospy
import tf.transformations as tft
from gazebo_msgs.msg import ModelState, ModelStates


class ModelPoseGuard:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "smart_cane")
        self.z = rospy.get_param("~z", 0.03)
        self.roll_pitch_limit = rospy.get_param("~roll_pitch_limit", 0.01)
        self.z_limit = rospy.get_param("~z_limit", 0.01)
        self.latest = None

        self.pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.states_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.02), self.timer_callback)

    def states_callback(self, msg):
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            return
        self.latest = (msg.pose[idx], msg.twist[idx])

    def timer_callback(self, _event):
        if self.latest is None or rospy.is_shutdown():
            return

        pose, twist = self.latest
        q = pose.orientation
        roll, pitch, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])

        if (abs(roll) < self.roll_pitch_limit and
                abs(pitch) < self.roll_pitch_limit and
                abs(pose.position.z - self.z) < self.z_limit):
            return

        fixed_q = tft.quaternion_from_euler(0.0, 0.0, yaw)
        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = "world"
        state.pose.position.x = pose.position.x
        state.pose.position.y = pose.position.y
        state.pose.position.z = self.z
        state.pose.orientation.x = fixed_q[0]
        state.pose.orientation.y = fixed_q[1]
        state.pose.orientation.z = fixed_q[2]
        state.pose.orientation.w = fixed_q[3]
        state.twist = twist
        state.twist.linear.z = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        self.pub.publish(state)


if __name__ == "__main__":
    rospy.init_node("model_pose_guard")
    ModelPoseGuard()
    rospy.spin()
