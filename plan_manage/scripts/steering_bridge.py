#!/usr/bin/env python3
"""Bridge keyboard /cmd_vel into the cane Gazebo drive interface.

linear.x is the virtual human push speed. angular.z is the steering yaw rate.
The Gazebo drive receives only forward velocity plus yaw rate, while the
steering joint is just a visual wheel angle.
"""
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class SteeringBridge:
    def __init__(self):
        self.steer_angle = 0.0
        self.forward_speed = 0.0
        self.yaw_rate = 0.0
        self.active = False
        self.sent_stop = True
        self.timeout = rospy.get_param("~timeout", 0.5)
        self.max_steer = rospy.get_param("~max_steer_angle", 0.9)

        self.cmd_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel_footprint", Twist, queue_size=10)
        self.steer_pub = rospy.Publisher(
            "/steering_joint_position_controller/command", Float64, queue_size=10)

        self.last_cmd_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.timer = rospy.Timer(rospy.Duration(0.02), self.control_loop)

    def cmd_callback(self, msg):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        if 0 < dt < 0.5:
            self.steer_angle += msg.angular.z * dt
            self.steer_angle = max(-self.max_steer, min(self.max_steer, self.steer_angle))
        self.last_time = now
        self.forward_speed = msg.linear.x
        self.yaw_rate = msg.angular.z
        self.last_cmd_time = now
        self.active = True
        self.sent_stop = False

    def control_loop(self, event):
        # Go silent after keyboard idle so MPC can own /cmd_vel_footprint.
        if not self.active:
            return
        if (rospy.Time.now() - self.last_cmd_time).to_sec() > self.timeout:
            self.forward_speed = 0.0
            self.yaw_rate = 0.0
            if not self.sent_stop:
                self.publish_command()
                self.sent_stop = True
            self.active = False
            return

        self.publish_command()

    def publish_command(self):
        cmd = Twist()
        cmd.linear.x = self.forward_speed
        cmd.linear.y = 0.0
        cmd.angular.z = self.yaw_rate
        self.cmd_pub.publish(cmd)
        self.steer_pub.publish(Float64(self.steer_angle))


if __name__ == "__main__":
    rospy.init_node("steering_bridge")
    SteeringBridge()
    rospy.spin()
