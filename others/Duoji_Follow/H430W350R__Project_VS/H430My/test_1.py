#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_servo_square.py

给舵机模拟一个绕画面中心做正方形轨迹的 (cx,cy) 输入，
每帧调用 servo_update,观察两个舵机是否按轨迹移动。
"""

import time
import math
import Image as st  # 改成你实际的舵机脚本名

def generate_square(cx0, cy0, size, steps_per_edge):
    """
    在画面中心 (cx0,cy0) 周围，生成一个边长 = 2*size 的正方形轨迹，
    每条边上插值 steps_per_edge 个点，总共 4*steps_per_edge 点。
    """
    corners = [
        (cx0 + size, cy0 + size),
        (cx0 + size, cy0 - size),
        (cx0 - size, cy0 - size),
        (cx0 - size, cy0 + size),
    ]
    pts = []
    for i in range(4):
        x1, y1 = corners[i]
        x2, y2 = corners[(i+1) % 4]
        for step in range(steps_per_edge):
            alpha = step / steps_per_edge
            pts.append((x1 * (1-alpha) + x2 * alpha,
                        y1 * (1-alpha) + y2 * alpha))
    return pts

def main():
    # 1) 初始化舵机
    st.setup_all()

    # # 2) 计算正方形轨迹参数
    cx0 = st.IMAGE_WIDTH  / 2
    cy0 = st.IMAGE_HEIGHT / 2
    size = min(st.IMAGE_WIDTH, st.IMAGE_HEIGHT) * 0.1  # 正方形半边长 = 20% 画面尺寸
    traj = generate_square(cx0, cy0, size, steps_per_edge=40)

    print("开始正方形轨迹测试，按 Ctrl-C 停止")
    try:
        while True:
            for (cx, cy) in traj:
                # 注意 servo_update 接受的是一个 list of centers
                st.servo_update([(cx, cy)])
                time.sleep(0.05) # 每 50ms 更新一次
    except KeyboardInterrupt:
        print("\n测试结束,关闭舵机")
    finally:
        st.shutdown_all()

if __name__ == '__main__':
    main()