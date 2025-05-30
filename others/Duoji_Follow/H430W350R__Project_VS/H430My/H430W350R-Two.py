#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双舵机测试脚本（各自独立 U2D2 → USB 口 + 不同波特率 + 同 ID=1)
- pan(水平) 舵机：   /dev/ttyUSB1 @ 1,000,000bps, ID=1
- tilt(垂直)舵机：  /dev/ttyUSB0 @  56,600bps, ID=1
"""

import sys, time
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# —————— 1. 端口 & 波特 & ID ——————
PAN_PORT       = '/dev/ttyUSB0'
TILT_PORT      = '/dev/ttyUSB1'
BAUD_PAN       =  1_000_000        # pan 用 1Mbps
BAUD_TILT      =  57_600    # tilt 用 57600bps
PROTOCOL_VER   = 2.0

ID_PAN         = 1
ID_TILT        = 1

# —————— 2. 控制表地址 & 模式 ——————
ADDR_OP_MODE       = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POS   = 132

OPERATING_MODE_POS = 3
TORQUE_ON          = 1
TORQUE_OFF         = 0

# —————— 3. 映射 & 中心校准 ——————
# 两舵机都还是 0°→2048 中心映射，若有偏差在此处进行可手动改常量
# 物理零点脉冲
PAN_CENTER_TICK     = 1753
TILT_CENTER_TICK    = 2308

# —————— 角度→脉冲映射 ——————
def deg2tick(angle_deg , center_tick):
    angle = max(-180.0, min(180.0, angle_deg))
    return int(center_tick + angle * 4096.0 / 360.0)

# —————— 4. 初始化各自端口 ——————
def init_port(dev_name, baud):
    ph = PortHandler(dev_name)
    pk = PacketHandler(PROTOCOL_VER)
    if not ph.openPort():
        print(f"ERROR: 无法打开端口 {dev_name}")
        sys.exit(1)
    if not ph.setBaudRate(baud):
        print(f"ERROR: 无法设置波特率 {baud} on {dev_name}")
        sys.exit(1)
    return ph, pk

# —————— 5. Ping 验证 ——————
def ping_servo(ph, pk, dxl_id, label):
    _, comm, err = pk.ping(ph, dxl_id)
    if comm != COMM_SUCCESS:
        print(f"[{label}] ID={dxl_id} Ping 通信失败：{pk.getTxRxResult(comm)}")
        return False
    if err != 0:
        print(f"[{label}] ID={dxl_id} Ping 返回错误：{pk.getRxPacketError(err)}")
        return False
    print(f"[{label}] ID={dxl_id} Ping 成功")
    return True

# —————— 6. 模式切换 & 扭矩控制 ——————
def setup_servo(ph, pk, dxl_id):
    # 切位置控制模式
    res1, err1 = pk.write1ByteTxRx(ph, dxl_id, ADDR_OP_MODE, OPERATING_MODE_POS)
    # 使能扭矩
    res2, err2 = pk.write1ByteTxRx(ph, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ON)
    if res1 != COMM_SUCCESS:
        print(f"[ID={dxl_id}] 切模式失败：{pk.getTxRxResult(res1)}")
    if err1 != 0:
        print(f"[ID={dxl_id}] 切模式舵机错误：{pk.getRxPacketError(err1)}")
    if res2 != COMM_SUCCESS:
        print(f"[ID={dxl_id}] 使能扭矩失败：{pk.getTxRxResult(res2)}")
    if err2 != 0:
        print(f"[ID={dxl_id}] 扭矩舵机错误：{pk.getRxPacketError(err2)}")

def shutdown_servo(ph, pk, dxl_id):
    pk.write1ByteTxRx(ph, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_OFF)
    ph.closePort()

# —————— 7. 位置写入 & 读取 ——————
def write_goal(ph, pk, dxl_id, tick):
    comm, err = pk.write4ByteTxRx(ph, dxl_id, ADDR_GOAL_POSITION, tick)
    if comm != COMM_SUCCESS:
        print(f"[ID={dxl_id}] 写入 Goal 失败：{pk.getTxRxResult(comm)}")
    elif err != 0:
        print(f"[ID={dxl_id}] 写入 Goal 舵机错误：{pk.getRxPacketError(err)}")

def read_present(ph, pk, dxl_id):
    pos, comm, err = pk.read4ByteTxRx(ph, dxl_id, ADDR_PRESENT_POS)
    if comm != COMM_SUCCESS:
        print(f"[ID={dxl_id}] 读取 Present 失败：{pk.getTxRxResult(comm)}")
        return None
    if err != 0:
        print(f"[ID={dxl_id}] 读取 Present 舵机错误：{pk.getRxPacketError(err)}")
        return None
    return pos

# —————— 8. 主流程 ——————
if __name__ == '__main__':
    # 1. 分别初始化
    ph_pan, pk_pan   = init_port(PAN_PORT,  BAUD_PAN)
    ph_tilt, pk_tilt = init_port(TILT_PORT, BAUD_TILT)

    # 2. Ping 验证
    ok_pan  = ping_servo(ph_pan,   pk_pan,  ID_PAN,  "PAN")
    ok_tilt = ping_servo(ph_tilt,  pk_tilt, ID_TILT, "TILT")
    if not (ok_pan and ok_tilt):
        print("Ping 失败，请检查每个端口对应的 ID 和波特率。")
        ph_pan.closePort(); ph_tilt.closePort()
        sys.exit(1)

    # 3. 切模式 & 使能扭矩
    setup_servo(ph_pan,  pk_pan,  ID_PAN)
    setup_servo(ph_tilt, pk_tilt, ID_TILT)

    # 4. 测试动作：0°, 0°, -0°
    for angle in (0.0, 0.0, -0.0):
        pan_tick  = deg2tick(angle, PAN_CENTER_TICK)
        print(f"\n→ Pan → tick={pan_tick}  ")
        write_goal(ph_pan,  pk_pan,  ID_PAN,  pan_tick)
        time.sleep(1.0)
        print("  水平 回读=",  read_present(ph_pan,  pk_pan,  ID_PAN))

    for angle in (0.0, 30.0, -30.0):
        tilt_tick = deg2tick(angle, TILT_CENTER_TICK)
        print(f"\n→ Tilt → tick={tilt_tick} ")
        write_goal(ph_tilt, pk_tilt, ID_TILT, tilt_tick)
        time.sleep(1.0)
        print("  垂直 回读=", read_present(ph_tilt, pk_tilt, ID_TILT))


    # 5. 关闭
    shutdown_servo(ph_pan,  pk_pan,  ID_PAN)
    shutdown_servo(ph_tilt, pk_tilt, ID_TILT)