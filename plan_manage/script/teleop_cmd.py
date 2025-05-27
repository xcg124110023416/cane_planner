#!/usr/bin/env python3
# -*- coding: utf-8 -*

import os
import sys
import tty
import termios
import roslib
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


path_ = Path()
mk = Marker()

def goal_callback(msg):
    mk.header.frame_id = "world"
    mk.type = Marker.POINTS
    mk.action = Marker.DELETE
    mk.id = 0

    mk.action = Marker.ADD
    mk.pose.orientation.x = 0.0
    mk.pose.orientation.y = 0.0
    mk.pose.orientation.z = 0.0
    mk.pose.orientation.w = 1.0
    mk.color.r = 1.0
    mk.color.g = 0.0
    mk.color.b = 0.0
    mk.color.a = 1
    mk.scale.x = 0.5
    mk.scale.y = 0.5
    mk.scale.z = 0.5
    pt = Point()
    rospy.loginfo("init success")
    
    path_.poses.clear()
    path_.poses.append(msg)
    pt.x = msg.pose.position.x
    pt.y = msg.pose.position.y
    pt.z = 0.0
    mk.points.clear()
    mk.points.append(pt)
    pub_vis.publish(mk)
    pub.publish(path_)
    rospy.loginfo("set end pos is [%lf,%lf]", pt.x, pt.y)


# 全局变量
sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback, queue_size=1)
pub = rospy.Publisher('/waypoint_generator/waypoints', Path, queue_size=15)
pub_vis = rospy.Publisher("waypoint_generator/vis", Marker, queue_size=10)




# def keyboardLoop():
    # 初始化
    # rospy.init_node('sim_teleop')
    # rate = rospy.Rate(rospy.get_param('~hz', 1))

    # end_x = 0.0
    # end_y = 0.0
    # mk.header.frame_id= "world"
    # mk.type = Marker.POINTS
    # mk.action = Marker.DELETE
    # mk.id = 0

    # mk.action = Marker.ADD
    # mk.pose.orientation.x = 0.0
    # mk.pose.orientation.y = 0.0
    # mk.pose.orientation.z = 0.0
    # mk.pose.orientation.w = 1.0
    # mk.color.r = 1.0
    # mk.color.g = 0.0
    # mk.color.b = 0.0
    # mk.color.a = 1
    # mk.scale.x = 0.5
    # mk.scale.y = 0.5
    # mk.scale.z = 0.5
    # pt = Point()
    # rospy.loginfo("init success")

    # 读取按键循环
    # while not rospy.is_shutdown():
        # fd = sys.stdin.fileno()
        # old_settings = termios.tcgetattr(fd)
        # # 不产生回显效果
        # old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
        # try:
        #     tty.setraw(fd)
        #     ch = sys.stdin.read(1)
        # finally:
        #     termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        # if ch == 'q':
        #     end_x += 1.0
        # elif ch == 'a':
        #     end_x -= 1.0
        # elif ch == 'w':
        #     end_y += 1.0
        # elif ch == 's':
        #     end_y -= 1.0
        # elif ch == 'p':
        #     exit()
        # elif ch == 'e':
        #     end_x = 0.0
        #     end_y = 0.0
        # elif ch == 'd':
        #     end_x = 5.0
        #     end_y = 13.0

        # 发送消息
        # path_.poses.clear()
        # goal_pose = PoseStamped()
        # cur_point = PoseStamped()
        # cur_point.pose.position.x = end_x
        # cur_point.pose.position.y = end_y
        # cur_point.pose.position.z = 0.0
        # cur_point.pose.orientation.w = 1.0
        # cur_point.pose.orientation.x = 0.0
        # cur_point.pose.orientation.y = 0.0
        # cur_point.pose.orientation.z = 0.0
        # path_.poses.append(goal_pose)
        # pt.x = goal_pose.pose.position.x
        # pt.y = goal_pose.pose.position.y
        # pt.z = 0.0
        # mk.points.clear()
        # mk.points.append(pt)
        # pub_vis.publish(mk)
        # pub.publish(path_)
        # rospy.loginfo("set end pos is [%lf,%lf]",pt.x,pt.y)
        


if __name__ == '__main__':
    try:
        # 初始化
        rospy.init_node('sim_teleop')
        rate = rospy.Rate(rospy.get_param('~hz', 1))
        # keyboardLoop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
