#!/usr/bin/env python3
import os, sys
import math
import time
import threading
from typing import Dict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# 加载自定义 tracker
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from subscriber.rigid_tracker import RigidTracker

# === 全局缓存 ===
robot_position_cache = {}  # e.g. {'rigid1': {'x': ..., 'qx': ..., 'qw': ...}}

# === 获取当前位置 ===
def get_current_position(robot_name):
    if robot_name in robot_position_cache:
        rigid = robot_position_cache[robot_name]
        x = rigid["x"]
        y = rigid["z"]  # PhaseSpace 坐标 z 作为 y
        heading_y = rigid["heading_y"]
        # heading_y = quaternion_to_heading_y(rigid["qx"], rigid["qy"], rigid["qz"], rigid["qw"])
        print(f"✅ Current position of {robot_name} (from {robot_name}): x={x:.2f}, y={y:.2f}, heading_y={heading_y:2f}°")
        return x, y, heading_y
    else:
        print(f"⚠️ No position data for {robot_name}")
        return 0.0, 0.0, 0.0

# === 旋转对准目标方向 ===
def rotate_to_face_target(robot_id, publisher, target: Dict[str, float], angle_tolerance_deg=5):
    x_target, y_target = target["x"], target["y"]

    # 获取当前位姿并计算目标方向角
    x_now, y_now, _ = get_current_position(robot_id)
    dx = x_target - x_now
    dz = y_target - y_now

    # 机器人面朝 -Z，因此使用 atan2(dx, dz)
    target_angle = math.degrees(math.atan2(dx, dz)) % 360

    print(f"\n🎯 Target: ({x_target:.2f}, {y_target:.2f})")
    print(f"📌 Current: ({x_now:.2f}, {y_now:.2f})")
    print(f"🧮 Target degree: {target_angle:.2f}°")

    prev_error = None

    while True:
        _, _, heading_y_now = get_current_position(robot_id)

        # 计算误差（[-180, +180] 范围内的最小角度差）
        angle_error = (target_angle - heading_y_now + 180) % 360 - 180

        print(f"🔄 [调试] 当前heading_y={heading_y_now:.2f}°, 目标target={target_angle:.2f}°, 误差={angle_error:.2f}°")

        if abs(angle_error) < angle_tolerance_deg:
            print("🟢 目标朝向已达到，停止旋转")
            break

        new_direction = 1 if angle_error > 0 else -1

        # 判断是否需要调整方向
        if prev_error is None or abs(angle_error) < abs(prev_error):
            # 误差在缩小，继续转
            twist = Twist()
            if abs(angle_error) > 25:
                twist.angular.z = 0.5 * new_direction
            elif abs(angle_error) > 10:
                twist.angular.z = 0.3 * new_direction
            else:
                twist.angular.z = 0.15 * new_direction

            publisher.publish(twist)
            print(f"🌀 正在{'左' if new_direction == 1 else '右'}转，速度 = {twist.angular.z:.2f} rad/s")
        else:
            # 误差变大了，可能越转越偏
            print("🔁 误差变大，允许反向调整（下次切换方向）")

        prev_error = angle_error
        time.sleep(0.1)

    # 🛑 到达目标角度后停止
    publisher.publish(Twist())
    time.sleep(0.2)

# === 主程序入口 ===
def run(robot_name, target):
    rclpy.init()

    # 启动订阅 tracker
    node_tracker = RigidTracker(robot_position_cache, robot_name)
    tracker_thread = threading.Thread(target=rclpy.spin, args=(node_tracker,), daemon=True)
    tracker_thread.start()

    # 等待获取坐标数据
    print("⏳ Waiting for position data...")
    time.sleep(3)

    # 创建发布器
    node_pub = rclpy.create_node(f'{robot_name}_rotator')
    publisher = node_pub.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)

    # 执行旋转
    rotate_to_face_target(robot_name, publisher, target)

    # 退出
    time.sleep(1)
    node_tracker.destroy_node()
    node_pub.destroy_node()
    rclpy.shutdown()
    tracker_thread.join()

# === 启动测试 ===
if __name__ == "__main__":
    robot_name = "robot1"
    target = {"x": 1002, "y": 1415}  # 目标位置坐标（PhaseSpace）
    run(robot_name, target)
    print("🚀 Finished rotate-to-face-target test.")
