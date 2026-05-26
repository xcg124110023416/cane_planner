#!/usr/bin/env python3

import math
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from onboard_detector.msg import DynamicObstacles
from std_srvs.srv import Empty


class FixedGoalPublisher:
    def __init__(self):
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.goal_x = rospy.get_param("~goal_x", 16.27)
        self.goal_y = rospy.get_param("~goal_y", 1.52)
        self.goal_z = rospy.get_param("~goal_z", 0.0)
        self.goal_yaw = rospy.get_param("~goal_yaw", 0.0)
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.start_time = rospy.get_param("~start_time", 10.0)
        self.publish_rate = rospy.get_param("~publish_rate", 10.0)
        self.publish_duration = rospy.get_param("~publish_duration", 1.0)
        self.reset_simulation = rospy.get_param("~reset_simulation", False)
        self.reset_service = rospy.get_param("~reset_service", "/gazebo/reset_simulation")
        self.reset_timeout = rospy.get_param("~reset_timeout", 10.0)
        self.unpause_simulation = rospy.get_param("~unpause_simulation", True)
        self.unpause_service = rospy.get_param("~unpause_service", "/gazebo/unpause_physics")
        self.unpause_timeout = rospy.get_param("~unpause_timeout", 10.0)
        self.wait_for_odom = rospy.get_param("~wait_for_odom", True)
        self.wait_for_pedestrians = rospy.get_param("~wait_for_pedestrians", True)
        self.allow_late = rospy.get_param("~allow_late", True)

        self.pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1, latch=True)

    def run(self):
        rospy.loginfo(
            "FixedGoalPublisher: goal=(%.2f, %.2f, yaw=%.2f) frame=%s start_time=%.2fs topic=%s",
            self.goal_x, self.goal_y, self.goal_yaw, self.frame_id, self.start_time, self.goal_topic)

        if self.reset_simulation:
            self.reset_gazebo_simulation()

        if self.unpause_simulation:
            self.unpause_gazebo_simulation()

        self.wait_until_clock_active()

        if self.wait_for_odom:
            rospy.loginfo("FixedGoalPublisher: waiting for /localization_odom")
            rospy.wait_for_message("/localization_odom", Odometry)

        if self.wait_for_pedestrians:
            rospy.loginfo("FixedGoalPublisher: waiting for /onboard_detector/dynamic_obstacles_info")
            rospy.wait_for_message("/onboard_detector/dynamic_obstacles_info", DynamicObstacles)

        self.wait_until_start_time()
        self.publish_goal()

    def reset_gazebo_simulation(self):
        rospy.loginfo("FixedGoalPublisher: waiting for %s", self.reset_service)
        rospy.wait_for_service(self.reset_service, timeout=self.reset_timeout)
        reset = rospy.ServiceProxy(self.reset_service, Empty)
        reset()
        # Use wall time here because simulated time may jump back to zero.
        time.sleep(0.5)
        rospy.loginfo("FixedGoalPublisher: Gazebo simulation reset")

    def unpause_gazebo_simulation(self):
        rospy.loginfo("FixedGoalPublisher: waiting for %s", self.unpause_service)
        rospy.wait_for_service(self.unpause_service, timeout=self.unpause_timeout)
        unpause = rospy.ServiceProxy(self.unpause_service, Empty)
        unpause()
        # Use wall time because simulated time may still be zero while physics starts.
        time.sleep(0.2)
        rospy.loginfo("FixedGoalPublisher: Gazebo physics unpaused")

    def wait_until_clock_active(self):
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() <= 0.0:
            rate.sleep()

    def wait_until_start_time(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            if now >= self.start_time:
                if now > self.start_time + 0.2:
                    msg = ("FixedGoalPublisher: current sim time %.2fs already passed start_time %.2fs" %
                           (now, self.start_time))
                    if not self.allow_late:
                        raise rospy.ROSException(msg)
                    rospy.logwarn("%s; publishing immediately", msg)
                return
            rate.sleep()

    def publish_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = self.goal_x
        msg.pose.position.y = self.goal_y
        msg.pose.position.z = self.goal_z
        half_yaw = 0.5 * self.goal_yaw
        msg.pose.orientation.z = math.sin(half_yaw)
        msg.pose.orientation.w = math.cos(half_yaw)

        rate = rospy.Rate(max(0.1, self.publish_rate))
        end_time = rospy.Time.now() + rospy.Duration(max(0.0, self.publish_duration))
        count = 0
        while not rospy.is_shutdown() and rospy.Time.now() <= end_time:
            msg.header.stamp = rospy.Time.now()
            self.pub.publish(msg)
            count += 1
            rate.sleep()

        rospy.loginfo("FixedGoalPublisher: published fixed goal %d times", count)


if __name__ == "__main__":
    rospy.init_node("publish_fixed_goal")
    FixedGoalPublisher().run()
