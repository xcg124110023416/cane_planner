#!/usr/bin/env python3
"""Synthetic IMU aligned with Gazebo planar odometry.

FAST-LIO needs the yaw-rate used by the simulated LiDAR motion. The native
Gazebo IMU and model-state finite differences are noisy for the hand-supported
cane model, so this node derives a stable IMU stream from /odom.
"""
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class ImuFromOdom:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "imu_link")
        self.input_topic = rospy.get_param("~input_topic", "/odom")
        self.output_topic = rospy.get_param("~output_topic", "/imu")
        self.gravity = rospy.get_param("~gravity", 9.81)
        self.gyro_z_sign = rospy.get_param("~gyro_z_sign", 1.0)
        self.use_odom_accel = rospy.get_param("~use_odom_accel", False)

        self.pub = rospy.Publisher(self.output_topic, Imu, queue_size=100)
        self.last_stamp = None
        self.last_vx = 0.0
        self.last_vy = 0.0
        rospy.Subscriber(self.input_topic, Odometry, self.odom_callback, queue_size=50)

    def odom_callback(self, msg):
        if rospy.is_shutdown():
            return

        stamp = msg.header.stamp
        if stamp.is_zero():
            stamp = rospy.Time.now()

        ax = 0.0
        ay = 0.0
        if self.use_odom_accel and self.last_stamp is not None:
            dt = (stamp - self.last_stamp).to_sec()
            if dt > 1e-4:
                ax = (msg.twist.twist.linear.x - self.last_vx) / dt
                ay = (msg.twist.twist.linear.y - self.last_vy) / dt

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.frame_id
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity.x = msg.twist.twist.angular.x
        imu.angular_velocity.y = msg.twist.twist.angular.y
        imu.angular_velocity.z = self.gyro_z_sign * msg.twist.twist.angular.z
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = self.gravity

        imu.angular_velocity_covariance[0] = 1e-5
        imu.angular_velocity_covariance[4] = 1e-5
        imu.angular_velocity_covariance[8] = 1e-5
        imu.linear_acceleration_covariance[0] = 1e-3
        imu.linear_acceleration_covariance[4] = 1e-3
        imu.linear_acceleration_covariance[8] = 1e-3
        try:
            self.pub.publish(imu)
        except rospy.ROSException:
            # During roslaunch shutdown, rospy may close the publisher while an
            # already-queued odom callback is still running.
            if not rospy.is_shutdown():
                raise

        self.last_stamp = stamp
        self.last_vx = msg.twist.twist.linear.x
        self.last_vy = msg.twist.twist.linear.y


if __name__ == "__main__":
    rospy.init_node("imu_from_state")
    ImuFromOdom()
    rospy.spin()
