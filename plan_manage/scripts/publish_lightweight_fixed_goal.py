#!/usr/bin/env python3

import math

import rospkg
import rospy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class LightweightFixedGoalPublisher:
    def __init__(self):
        self.scenario = rospy.get_param("~scenario", "mixed")
        self.config_path = rospy.get_param("~config_path", self.default_config_path())
        scenario_cfg = self.load_scenario_config()
        start_cfg = scenario_cfg.get("start", {})
        goal_cfg = scenario_cfg.get("goal", {})

        self.start_topic = rospy.get_param("~start_topic", "/initialpose")
        self.start_x = rospy.get_param("~start_x", start_cfg.get("x", -2.8706040382385254))
        self.start_y = rospy.get_param("~start_y", start_cfg.get("y", 3.9538378715515137))
        self.start_z = rospy.get_param("~start_z", start_cfg.get("z", 0.0))
        self.start_yaw = rospy.get_param("~start_yaw", start_cfg.get("yaw", -1.592039942741394))
        self.start_cov_x = rospy.get_param("~start_cov_x", start_cfg.get("covariance_x", 0.25))
        self.start_cov_y = rospy.get_param("~start_cov_y", start_cfg.get("covariance_y", 0.25))
        self.start_cov_yaw = rospy.get_param(
            "~start_cov_yaw",
            start_cfg.get("covariance_yaw", 0.06853892326654787),
        )
        self.start_time = rospy.get_param("~start_time", 1.0)
        self.start_publish_rate = rospy.get_param("~start_publish_rate", 10.0)
        self.start_publish_duration = rospy.get_param("~start_publish_duration", 0.0)

        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.goal_x = rospy.get_param("~goal_x", goal_cfg.get("x", 4.004665374755859))
        self.goal_y = rospy.get_param("~goal_y", goal_cfg.get("y", -4.801575660705566))
        self.goal_z = rospy.get_param("~goal_z", goal_cfg.get("z", 0.0))
        self.goal_yaw = rospy.get_param("~goal_yaw", goal_cfg.get("yaw", 0.01213059201836586))
        self.frame_id = rospy.get_param("~frame_id", "world")

        self.goal_time = rospy.get_param("~goal_time", 3.0)
        self.goal_publish_rate = rospy.get_param("~goal_publish_rate", 10.0)
        self.goal_publish_duration = rospy.get_param("~goal_publish_duration", 0.0)
        self.republish_start_after_goal = rospy.get_param("~republish_start_after_goal", False)
        self.republish_start_delay = rospy.get_param("~republish_start_delay", 0.5)
        self.allow_late = rospy.get_param("~allow_late", True)

        self.wait_for_odom = rospy.get_param("~wait_for_odom", True)
        self.odom_topic = rospy.get_param("~odom_topic", "/simulation_generator/odom")
        self.wait_for_start_subscribers = rospy.get_param("~wait_for_start_subscribers", True)
        self.min_start_subscribers = rospy.get_param("~min_start_subscribers", 2)
        self.wait_for_start_odom = rospy.get_param("~wait_for_start_odom", True)
        self.start_odom_tolerance = rospy.get_param("~start_odom_tolerance", 0.15)
        self.start_odom_timeout = rospy.get_param("~start_odom_timeout", 3.0)
        self.wait_for_goal_subscribers = rospy.get_param("~wait_for_goal_subscribers", True)
        self.min_goal_subscribers = rospy.get_param("~min_goal_subscribers", 1)
        self.ready_timeout = rospy.get_param("~ready_timeout", 10.0)

        self.start_pub = rospy.Publisher(self.start_topic, PoseWithCovarianceStamped, queue_size=1, latch=True)
        self.pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1, latch=True)
        self.use_sim_time = rospy.get_param("/use_sim_time", False)
        self.base_time = None

    def default_config_path(self):
        package_path = rospkg.RosPack().get_path("plan_manage")
        return package_path + "/config/lightweight_fixed_goals.yaml"

    def load_scenario_config(self):
        try:
            with open(self.config_path, "r") as f:
                config = yaml.safe_load(f) or {}
        except Exception as exc:
            rospy.logwarn(
                "LightweightFixedGoalPublisher: failed to read %s: %s; using built-in defaults",
                self.config_path,
                exc,
            )
            return {}

        scenarios = config.get("scenarios", {})
        if self.scenario not in scenarios:
            rospy.logwarn(
                "LightweightFixedGoalPublisher: scenario '%s' not found in %s; available=%s; using built-in defaults",
                self.scenario,
                self.config_path,
                sorted(scenarios.keys()),
            )
            return {}
        return scenarios[self.scenario] or {}

    def run(self):
        rospy.loginfo(
            "LightweightFixedGoalPublisher: scenario=%s config=%s",
            self.scenario,
            self.config_path,
        )
        rospy.loginfo(
            "LightweightFixedGoalPublisher: start=(%.2f, %.2f, yaw=%.2f) at %.2fs topic=%s",
            self.start_x,
            self.start_y,
            self.start_yaw,
            self.start_time,
            self.start_topic,
        )
        rospy.loginfo(
            "LightweightFixedGoalPublisher: goal=(%.2f, %.2f, yaw=%.2f) at %.2fs frame=%s topic=%s",
            self.goal_x,
            self.goal_y,
            self.goal_yaw,
            self.goal_time,
            self.frame_id,
            self.goal_topic,
        )

        if self.use_sim_time:
            rospy.loginfo("LightweightFixedGoalPublisher: using absolute simulated time")
            self.wait_until_clock_active()
        else:
            rospy.loginfo("LightweightFixedGoalPublisher: using relative wall time")

        if self.wait_for_odom:
            rospy.loginfo("LightweightFixedGoalPublisher: waiting for %s", self.odom_topic)
            rospy.wait_for_message(self.odom_topic, Odometry, timeout=self.ready_timeout)

        if self.wait_for_start_subscribers:
            self.wait_until_subscribers(
                self.start_pub,
                self.min_start_subscribers,
                self.start_topic,
                "start",
            )

        if self.wait_for_goal_subscribers:
            self.wait_until_subscribers(
                self.pub,
                self.min_goal_subscribers,
                self.goal_topic,
                "goal",
            )

        self.base_time = rospy.Time.now()
        self.wait_until_time(self.start_time, "start_time")
        self.publish_start()
        if self.wait_for_start_odom:
            self.wait_until_odom_at_start()
        self.wait_until_time(self.goal_time, "goal_time")
        self.publish_goal()
        if self.republish_start_after_goal:
            self.sleep_relative(self.republish_start_delay)
            rospy.loginfo("LightweightFixedGoalPublisher: republishing fixed start after goal")
            self.publish_start()

    def wait_until_clock_active(self):
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() <= 0.0:
            rate.sleep()

    def wait_until_subscribers(self, pub, min_subscribers, topic, label):
        deadline = rospy.Time.now() + rospy.Duration(self.ready_timeout)
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            count = pub.get_num_connections()
            if count >= min_subscribers:
                rospy.loginfo(
                    "LightweightFixedGoalPublisher: %s subscribers ready on %s (%d/%d)",
                    label,
                    topic,
                    count,
                    min_subscribers,
                )
                return
            if rospy.Time.now() >= deadline:
                rospy.logwarn(
                    "LightweightFixedGoalPublisher: only %d/%d %s subscribers on %s after %.1fs; "
                    "continuing",
                    count,
                    min_subscribers,
                    label,
                    topic,
                    self.ready_timeout,
                )
                return
            rate.sleep()

    def wait_until_time(self, target_time, label):
        if not self.use_sim_time:
            target_time = self.base_time.to_sec() + target_time
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            if now >= target_time:
                if now > target_time + 0.2:
                    msg = (
                        "LightweightFixedGoalPublisher: current sim time %.2fs already passed "
                        "%s %.2fs" % (now, label, target_time)
                    )
                    if not self.allow_late:
                        raise rospy.ROSException(msg)
                    rospy.logwarn("%s; publishing immediately", msg)
                return
            rate.sleep()

    def sleep_relative(self, duration):
        if duration <= 0.0:
            return
        end_time = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            rate.sleep()

    def publish_start(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = self.start_x
        msg.pose.pose.position.y = self.start_y
        msg.pose.pose.position.z = self.start_z
        half_yaw = 0.5 * self.start_yaw
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)
        msg.pose.covariance[0] = self.start_cov_x
        msg.pose.covariance[7] = self.start_cov_y
        msg.pose.covariance[35] = self.start_cov_yaw

        count = self.publish_repeated(
            self.start_pub,
            msg,
            self.start_publish_rate,
            self.start_publish_duration,
        )
        rospy.loginfo("LightweightFixedGoalPublisher: published fixed start %d times", count)

    def wait_until_odom_at_start(self):
        deadline = rospy.Time.now() + rospy.Duration(self.start_odom_timeout)
        while not rospy.is_shutdown():
            try:
                odom = rospy.wait_for_message(self.odom_topic, Odometry, timeout=0.2)
            except rospy.ROSException:
                odom = None
            if odom is not None:
                dx = odom.pose.pose.position.x - self.start_x
                dy = odom.pose.pose.position.y - self.start_y
                dist = math.hypot(dx, dy)
                if dist <= self.start_odom_tolerance:
                    rospy.loginfo(
                        "LightweightFixedGoalPublisher: odom reached fixed start, dist=%.3fm",
                        dist,
                    )
                    return
            if rospy.Time.now() >= deadline:
                rospy.logwarn(
                    "LightweightFixedGoalPublisher: odom did not reach fixed start within %.1fs; "
                    "continuing",
                    self.start_odom_timeout,
                )
                return

    def publish_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = self.goal_x
        msg.pose.position.y = self.goal_y
        msg.pose.position.z = self.goal_z
        half_yaw = 0.5 * self.goal_yaw
        msg.pose.orientation.z = math.sin(half_yaw)
        msg.pose.orientation.w = math.cos(half_yaw)

        count = self.publish_repeated(
            self.pub,
            msg,
            self.goal_publish_rate,
            self.goal_publish_duration,
        )
        rospy.loginfo("LightweightFixedGoalPublisher: published fixed goal %d times", count)

    def publish_repeated(self, pub, msg, publish_rate, publish_duration):
        rate = rospy.Rate(max(0.1, publish_rate))
        end_time = rospy.Time.now() + rospy.Duration(max(0.0, publish_duration))
        count = 0
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        count += 1
        while not rospy.is_shutdown() and rospy.Time.now() <= end_time:
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            count += 1
            rate.sleep()
        return count


if __name__ == "__main__":
    rospy.init_node("publish_lightweight_fixed_goal")
    LightweightFixedGoalPublisher().run()
