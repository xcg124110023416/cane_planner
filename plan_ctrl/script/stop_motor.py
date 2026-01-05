#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import struct
import sys

def set_motor_stop(port='/dev/ttyACM0', baudrate=115200):
    """
    模拟 C++ 中的 Set(CMD_VEL, 0) 函数逻辑
    协议格式: [帧头(0x43), 命令码(0x01), 数据高8位, 数据低8位]
    """
    CMD_FRAME_HEADER = 0x43  # 'C'
    CMD_VEL = 0x01
    DATA = 0
    
    # 构造 4 字节的二进制帧: Header(B) + Cmd(B) + Data(h, 16位有符号整数, 大端序)
    # > 代表大端序, B 代表 unsigned char (1字节), h 代表 short (2字节)
    frame = struct.pack('>BBh', CMD_FRAME_HEADER, CMD_VEL, DATA)
    
    print(f"正在尝试连接串口: {port} ...")
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            ser.write(frame)
            print(f"成功发送停止指令 Set(CMD_VEL, 0)")
            print(f"发送的原始十六进制数据: {frame.hex().upper()}")
    except serial.SerialException as e:
        print(f"串口错误: {e}")
        print("提示: 请检查串口设备是否存在，或者是否有权限访问 (sudo chmod 666 /dev/ttyACM0)")
    except Exception as e:
        print(f"发生错误: {e}")

if __name__ == "__main__":
    # 默认使用 /dev/ttyACM0，也可以通过命令行参数指定
    target_port = '/dev/ttyACM0'
    if len(sys.argv) > 1:
        target_port = sys.argv[1]
    
    set_motor_stop(target_port)
