#!/usr/bin/env python3
# -*- coding: utf-8 -*
import os
import sys
import tty
import termios
import roslib
import rospy
from omniGKF_control.msg import omniGKFcmd
from select import select

# 全局变量
pub = rospy.Publisher('omniGKFcmd', omniGKFcmd, queue_size=10)

def getKey():
    
       tty.setraw(sys.stdin.fileno())
       rlist, _, _ = select([sys.stdin], [], [], 0.1)
       if rlist:
          key = sys.stdin.read(1)
          if key =='\x03':
            rospy.signal_shutdown('shutdown')
       else:
           key = ''
       return key
    
def keyboardLoop():
    # 初始化
    rospy.init_node('omniGKFcmd_publisher', anonymous=True)
    # rate = rospy.Rate(rospy.get_param('~hz', 1))
    rate = rospy.Rate(10)  # 10Hz
    vel = 0.0  # m/s
    pos = 0.0 # rad
    set_pos_zero = 0.0
    flag = True
    while flag:
        key = getKey()
        msg = omniGKFcmd()
        if key =='j':
            msg.gkf_state =True
            set_pos_zero = set_pos_zero + 0.1
            msg.set_pos_zero = set_pos_zero
        elif key == 'l':
            msg.gkf_state =True
            set_pos_zero = set_pos_zero - 0.1
            msg.set_pos_zero = set_pos_zero
        elif key == 'k':
            msg.gkf_state = True
            msg.set_zero = True
            pub.publish(msg)
            break
        else :
            msg.gkf_state = True
            msg.vel = 0.0
        # 发布消息
        pub.publish(msg)

        # 按照设定的频率等待
        rate.sleep()

    while not rospy.is_shutdown():
        key = getKey()
        msg = omniGKFcmd()
        msg.gkf_state = True

        if key == 'w':  # 前进
            vel = vel + 0.1
        elif key == 's':  # 后退
            msg.gkf_state = True
            vel = vel - 0.1
        elif key == 'a':  # 左转
            msg.gkf_state = True
            pos = pos + 0.1
        elif key == 'd':  # 右转
            msg.gkf_state = True
            pos = pos - 0.1

        msg.vel = vel
        msg.pos = pos

        # 发布消息
        pub.publish(msg)

        # 按照设定的频率等待
        rate.sleep()

    msg = omniGKFcmd()
    pub.publish(msg)

if __name__ == '__main__':
    keyboardLoop()
 
  