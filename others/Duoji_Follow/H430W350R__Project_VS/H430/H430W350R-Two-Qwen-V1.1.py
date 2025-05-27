#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
基于角度误差的闭环比例控制物体追踪示例

摄像头:Intel RealSense D455
舵机：两路 XH430-W350-R
思路:从imagepipeline拿到每帧centers→取第一个center,每帧根据目标像素误差计算角度误差，再按比例更新舵机角度。

双舵机测试脚本（各自独立 U2D2 → USB 口 + 不同波特率 + 同 ID=1)
- pan(水平) 舵机：   /dev/ttyUSB1 @ 1,000,000bps, ID=1
- tilt(垂直)舵机：  /dev/ttyUSB0 @  56,600bps, ID=1

"""

import sys, time            # main_rs(frame_callback) 每帧回调 centers
from imagepipeline import main_rs   # imagepipeline返回 cx, cy，
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# —————— 1. 配置区 ——————
# 串口 & 舵机 ID
PAN_PORT,  BAUD_PAN  = '/dev/ttyUSB1', 1_000_000
TILT_PORT, BAUD_TILT = '/dev/ttyUSB0', 57_600
PROTOCOL_VER         = 2.0
ID_PAN, ID_TILT      = 1, 1

# Control Table 地址 & 模式
ADDR_OP_MODE       = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POS      = 116
ADDR_PRESENT_POS   = 132
OPER_MODE_POS      = 3
TORQUE_ON, TORQUE_OFF = 1, 0

# 零点脉冲 & 初始角度
PAN_CENTER_TICK,   TILT_CENTER_TICK= 2053, 2031
pan_angle_current, tilt_angle_current = 0.0, 0.0  # “中心”对应 0°,跟踪状态

# 相机分辨率 & 视场角
IMAGE_WIDTH, IMAGE_HEIGHT = 1280, 720
H_FOV_DEG,  V_FOV_DEG     = 86.0, 57.0
# IMAGE_WIDTH, IMAGE_HEIGHT = 1280, 720
# H_FOV_DEG,  V_FOV_DEG     = 70.0, 40.0

# 比例增益（0 < Kp ≤ 1）
Kp_pan, Kp_tilt = 0.6, 0.6

# —————— 2. 初始化各自端口 ——————
def init_port(dev, baud):
    ph = PortHandler(dev);              #ph串口句柄
    pk = PacketHandler(PROTOCOL_VER)    #pk协议句柄
    if not ph.openPort():   
        sys.exit(f"无法打开串口 {dev}")
    if not ph.setBaudRate(baud): 
        sys.exit(f"无法设置波特 {baud} on {dev}")
    return ph, pk

ph_pan, pk_pan = None, None
ph_tilt, pk_tilt = None, None

def setup_all():
    global ph_pan, pk_pan, ph_tilt, pk_tilt
    ph_pan,  pk_pan  = init_port(PAN_PORT,  BAUD_PAN)
    ph_tilt, pk_tilt = init_port(TILT_PORT, BAUD_TILT)
    
    # 切位置模式 & 使能扭矩
    pk_pan.write1ByteTxRx(ph_pan, ID_PAN,   ADDR_OP_MODE,   OPER_MODE_POS)
    pk_pan.write1ByteTxRx(ph_tilt, ID_TILT,   ADDR_TORQUE_ENABLE, TORQUE_ON)
    pk_tilt.write1ByteTxRx(ph_pan, ID_PAN, ADDR_OP_MODE,  OPER_MODE_POS)
    pk_tilt.write1ByteTxRx(ph_tilt, ID_TILT, ADDR_TORQUE_ENABLE, TORQUE_ON)
    print("SET ALL OK!!")

def shutdown_all():
    # 关扭矩 & 关串口
    if pk_pan:  pk_pan.write1ByteTxRx(ph_pan,  ID_PAN,   ADDR_TORQUE_ENABLE, TORQUE_OFF)
    if pk_tilt: pk_tilt.write1ByteTxRx(ph_tilt, ID_TILT, ADDR_TORQUE_ENABLE, TORQUE_OFF)
    if ph_pan:  ph_pan.closePort()
    if ph_tilt: ph_tilt.closePort()

# —————— 3. 角度→脉冲映射 ——————
def deg2tick(angle_deg, center_tick):
    angle = max(-180.0, min(180.0, angle_deg))
    return int(center_tick + angle * 4096.0/360.0)

def pixel_to_error(cx, cy):
    """
    返回相对画面中心的角度误差 (pan_err, tilt_err)
    右偏 & 下偏 为正
    """
    dx = cx - IMAGE_WIDTH/2
    dy = cy - IMAGE_HEIGHT/2
    pan_err  = -dx / (IMAGE_WIDTH/2)  * (H_FOV_DEG/2)
    tilt_err = -dy/ (IMAGE_HEIGHT/2) * (V_FOV_DEG/2)
    return pan_err, tilt_err

# —————— 4. 回调 & 控制函数 ——————
def servo_update(centers):
    """
    这个函数会被 main_rs 在每帧调用。
    centers: list of (x,y)，我们取 centers[0]
    """
    global pan_angle_current, tilt_angle_current

    if not centers:
        return
    cx, cy = centers[0]

    # 1) 计算角度误差
    pan_err, tilt_err = pixel_to_error(cx, cy)

    # 2) P 控制：更新目标角度
    pan_angle_current  += Kp_pan  * pan_err
    tilt_angle_current += Kp_tilt * tilt_err

    # 3) 限幅
    pan_angle_current  = max(-180.0, min(180.0, pan_angle_current))
    tilt_angle_current = max(-180.0, min(180.0, tilt_angle_current))

    # 4) 映射 & 下发
    pan_tick  = deg2tick(pan_angle_current,  PAN_CENTER_TICK)
    tilt_tick = deg2tick(tilt_angle_current, TILT_CENTER_TICK)

    pk_pan.write4ByteTxRx(ph_pan,  ID_PAN,  ADDR_GOAL_POS, pan_tick)
    pk_tilt.write4ByteTxRx(ph_tilt, ID_TILT, ADDR_GOAL_POS, tilt_tick)

    # （可选）调试
    print(f"px({cx:.0f},{cy:.0f}) err°({pan_err:.2f},{tilt_err:.2f}) → ang°({pan_angle_current:.2f},{tilt_angle_current:.2f})")

# —————— 5. 入口 ——————
if __name__ == '__main__':
    setup_all()
    try:
        # 将 servo_update 作为回调传给 main_rs
        main_rs(frame_callback=servo_update)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown_all()