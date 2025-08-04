# { "action": "face", "parameters": { "target": "lucy" } }

#!/usr/bin/env python3
# face.py —— 让机器人朝向一个目标位置

import os, sys, time, math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading

# 加入项目根目录
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from config import semantic_locations
from phasespace.rigid_tracker import RigidTracker


# 全局缓存
robot_position_cache = {}
publisher_dict = {}


def get_current_position(robot_name, robot_position_cache):
    if robot_name in robot_position_cache:
        rigid = robot_position_cache[robot_name]
        x = rigid["x"]
        y = rigid["z"]
        heading_y = rigid["heading_y"]
        return x, y, heading_y
    else:
        print(f"⚠️ No position data for {robot_name}")
        return 0.0, 0.0, 0.0


def getRobotPositionCache(robot_id: str):
    node = RigidTracker(robot_position_cache, robot_id)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(f"⏳ Waiting for position data of {robot_id}...")
    for _ in range(30):
        if robot_id in robot_position_cache:
            print(f"✅ Got position data for {robot_id}")
            break
        time.sleep(0.2)
    else:
        print(f"❌ Timeout: No position data for {robot_id}")
        return None

    return node, spin_thread


def rotate_to_face_target(robot_id, publisher, target_position, robot_position_cache, angle_tolerance_deg=5):
    x_target, y_target = target_position["x"], target_position["y"]
    x_now, y_now, _ = get_current_position(robot_id, robot_position_cache)
    dx = x_target - x_now
    dz = y_target - y_now
    target_angle = math.degrees(math.atan2(dx, dz)) % 360

    print(f"\n🔄 ROTATE | target: ({x_target:.1f}, {y_target:.1f}) → {target_angle:.1f}°")

    while True:
        _, _, heading_y_now = get_current_position(robot_id, robot_position_cache)
        angle_error = (target_angle - heading_y_now + 180) % 360 - 180

        if abs(angle_error) < angle_tolerance_deg:
            print("✅ Face done.")
            break

        new_direction = 1 if angle_error > 0 else -1
        twist = Twist()
        if abs(angle_error) > 25:
            twist.angular.z = 0.5 * new_direction
        elif abs(angle_error) > 10:
            twist.angular.z = 0.3 * new_direction
        else:
            twist.angular.z = 0.15 * new_direction

        publisher.publish(twist)
        print(f"↪️ turning {'left' if new_direction==1 else 'right'} | heading_y: {heading_y_now:.1f} | target_angle: {target_angle:.1f} | angle_error: {angle_error:.1f}° | speed: {twist.angular.z:.2f}")
        time.sleep(0.1)

    publisher.publish(Twist())
    time.sleep(0.2)


def face_target(node: Node, robot_name: str, target_name: str):
    # 查目标坐标
    if target_name not in semantic_locations:
        print(f"❌ Target '{target_name}' not found in semantic_locations")
        return

    target_position = semantic_locations[target_name]
    if "x" not in target_position or "y" not in target_position:
        print(f"❌ Target '{target_name}' lacks 'x' and 'y'")
        return

    # 准备 publisher
    if robot_name not in publisher_dict:
        pub = node.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
        publisher_dict[robot_name] = (node, pub)
    else:
        _, pub = publisher_dict[robot_name]

    # 执行旋转
    rotate_to_face_target(robot_name, pub, target_position, robot_position_cache)


def main():
    rclpy.init()
    robot_name = "robot1"           # ← 换成你的机器人名称
    target_name = "robot2"            # ← 换成你想面向的目标名称

    # 获取位置缓存
    getRobotPositionCache(robot_name)

    # 初始化 ROS 节点
    node = rclpy.create_node(f'face_node_{robot_name}')

    # 开始 face 动作
    face_target(node, robot_name, target_name)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
