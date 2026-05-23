#!/usr/bin/env python3
"""Keyboard teleop bridge: /cmd_vel → /gazebo/set_model_state (no physics)."""
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
import tf.transformations as tft
import math

def main():
    rospy.init_node("cmd_vel_to_state")
    pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)

    rospy.wait_for_service("/gazebo/get_model_state")
    gm = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    resp = gm("smart_cane", "map")
    x = resp.pose.position.x
    y = resp.pose.position.y
    q = resp.pose.orientation
    yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    rospy.loginfo("Teleop init: pos=(%.2f,%.2f) yaw=%.2f", x, y, yaw)

    vx = vy = vw = 0.0
    last = rospy.Time.now()

    def cb(msg):
        nonlocal vx, vy, vw
        vx = msg.linear.x; vy = msg.linear.y; vw = msg.angular.z
    rospy.Subscriber("/cmd_vel", Twist, cb)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        dt = (now - last).to_sec()
        last = now
        if abs(vx) > 0.001 or abs(vy) > 0.001 or abs(vw) > 0.001:
            yaw += vw * dt
            x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
            y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
            s = ModelState()
            s.model_name = "smart_cane"
            s.pose.position.x = x; s.pose.position.y = y; s.pose.position.z = 0
            q = tft.quaternion_from_euler(0, 0, yaw)
            s.pose.orientation.x = q[0]; s.pose.orientation.y = q[1]
            s.pose.orientation.z = q[2]; s.pose.orientation.w = q[3]
            pub.publish(s)
        rate.sleep()

if __name__ == "__main__":
    main()
