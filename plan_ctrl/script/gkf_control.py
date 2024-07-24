#!/usr/bin/env python3
# -*- coding: utf-8 -*

import os
import sys
import tty
import termios
import rospy
import serial

# 全局变量



def keyboardLoop():
    # 初始化
    rospy.init_node('gkf_control')
    ser_ = serial.Serial('/dev/feedback_controller',115200)
    if ser_.is_open:
        ser_.write('z000\n'.encode('utf-8'))
        rospy.loginfo("init success")
    
    pos = 0

    # 读取按键循环
    while not rospy.is_shutdown():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        # 不产生回显效果
        old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        if ch == 'a':
            rospy.loginfo("left")
            pos = pos + 800
        elif ch == 'd':
            rospy.loginfo("right")
            pos = pos - 800
        elif ch == 'a':
            rospy.loginfo("zero")
            pos = 0
        elif ch == 'p':
            ser_.close()
            exit()
        pos_str = 'z' + str(pos) + '\n'
        ser_.write(pos_str.encode('utf-8'))

if __name__ == '__main__':
    try:
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass
