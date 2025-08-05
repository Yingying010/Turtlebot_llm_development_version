#!/usr/bin/env python3

# collect.py —— 独立导航 + 语音反馈 + 模拟收集
 
import os, sys, time, math
import rclpy
from rclpy.node import Node
from typing import Dict,List
from geometry_msgs.msg import Twist
import threading
# 添加项目根路径
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from config import semantic_locations
from ttsRepo.stream_tts import tts_manager
from phasespace.rigid_tracker import RigidTracker
 
# 全局缓存
robot_position_cache = {}
publisher_dict = {}
 
# === 获取机器人位姿 ===
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
 
 
# === 启动 PhaseSpace 追踪器 ===
def getRobotPositionCache(robot_id: str):
    node = RigidTracker(robot_position_cache, robot_id)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(f"⏳ Waiting for position data of {robot_id}...")
    for _ in range(30):
        if robot_id in robot_position_cache:
            print(f"✅ Got position data for {robot_id}.")
            print("📦 robot_position_cache =", robot_position_cache)
            break
        time.sleep(0.2)
    else:
        print(f"❌ Timeout: No position data for {robot_id}")
        return None  # or raise Exception()

    return node, spin_thread  # ⬅️ 可以返回节点句柄（可选）
 
 
# === 转向目标方向 ===
def rotate_to_face_target(robot_id, publisher, target: Dict[str, float], robot_position_cache, angle_tolerance_deg=5):
    x_target, y_target = target["x"], target["y"]
    x_now, y_now, _ = get_current_position(robot_id, robot_position_cache)
    dx = x_target - x_now
    dz = y_target - y_now
    target_angle = math.degrees(math.atan2(dx, dz)) % 360

    print(f"\n🔄 ROTATE | target: ({x_target:.1f}, {y_target:.1f}) → {target_angle:.1f}°")

    prev_error = None
    while True:
        _, _, heading_y_now = get_current_position(robot_id, robot_position_cache)
        angle_error = (target_angle - heading_y_now + 180) % 360 - 180

        if abs(angle_error) < angle_tolerance_deg:
            print("✅ ROTATE done.")
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
        prev_error = angle_error
        time.sleep(0.1)

    publisher.publish(Twist())
    time.sleep(0.2)

def rotate_to_final_heading(robot_name, publisher, heading_deg, robot_position_cache, angle_tolerance_deg=5):
    print(f"\n🎯 ROTATE TO HEADING: {heading_deg:.1f}°")
    while True:
        _, _, heading_y_now = get_current_position(robot_name, robot_position_cache)
        angle_error = (heading_deg - heading_y_now + 180) % 360 - 180

        if abs(angle_error) < angle_tolerance_deg:
            print("✅ Final heading aligned.")
            break

        direction = 1 if angle_error > 0 else -1
        twist = Twist()
        if abs(angle_error) > 25:
            twist.angular.z = 0.5 * direction
        elif abs(angle_error) > 10:
            twist.angular.z = 0.3 * direction
        else:
            twist.angular.z = 0.15 * direction

        publisher.publish(twist)
        print(f"↪️ adjusting to heading {heading_deg:.1f}°, current={heading_y_now:.1f}°, error={angle_error:.1f}°")
        time.sleep(0.1)

    publisher.publish(Twist())
    time.sleep(0.2)
 
 
# === 向目标前进 ===
def move_forward_until_reached(robot_name, publisher, target, robot_position_cache, tolerance=20, max_acceptable_angle_error=25):
    x_target, y_target = target["x"], target["y"]
    print(f"\n🚗 NEED TO MOVE → ({x_target:.1f}, {y_target:.1f})")

    while True:
        x_now, y_now, heading_y_now = get_current_position(robot_name, robot_position_cache)
        dx = x_target - x_now
        dz = y_target - y_now
        distance = math.hypot(dx, dz)

        if distance < tolerance:
            print("🎉 Reached target.")
            break

        # 计算目标方向角与误差
        target_angle = math.degrees(math.atan2(dx, dz)) % 360
        angle_error = (target_angle - heading_y_now + 180) % 360 - 180

        if abs(angle_error) > max_acceptable_angle_error:
            print(f"🔁 Too much angle error: {angle_error:.1f}°, rotating first...")
            rotate_to_face_target(robot_name, publisher, target, robot_position_cache)
            continue  # 旋转完成后重新进入循环

        # 可以前进
        twist = Twist()
        twist.linear.x = 0.1  # 小速度保证精度
        publisher.publish(twist)
        print(f"🚗 Moving | dist={distance:.2f} | heading={heading_y_now:.1f}°, target={target_angle:.1f}°, error={angle_error:.1f}°")

        time.sleep(0.2)
        publisher.publish(Twist())  # 停止一小段时间，避免积累误差
        time.sleep(0.1)


def navigate_to_position(node:Node, robot_name:str, target: Dict[str, float], robot_position_cache):
    x_target, y_target = target["x"], target["y"]
    x_now, y_now, _ = get_current_position(robot_name, robot_position_cache)

    dx = x_target - x_now
    dy = y_target - y_now
    distance = math.hypot(dx, dy)
    angle = math.atan2(dy, dx)

    print(f"\n🧭 NAVIGATE {robot_name} → ({x_target:.1f}, {y_target:.1f}) | dist={distance:.2f}")

    if robot_name not in publisher_dict:
        pub = node.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
        publisher_dict[robot_name] = (node, pub)
    else:
        _, pub = publisher_dict[robot_name]

    # ✅ Phase 1: rotate first
    rotate_to_face_target(robot_name, pub, target, robot_position_cache)

    # ✅ Phase 2: move straight
    move_forward_until_reached(robot_name, pub, target, robot_position_cache)

    # ✅ Phase 3: adjust to target heading if given
    if "heading_deg" in target:
        rotate_to_final_heading(robot_name, pub, target["heading_deg"], robot_position_cache)


    print(f"✅ {robot_name} navigation complete.")


def navigate_to_target(node, robot_name, target):
    getRobotPositionCache(robot_name)

    # === STEP 1: 检查是否是语义位置名（字符串） ===
    if isinstance(target, str):
        if target not in semantic_locations:
            print(f"❌ Error: target '{target}' not found in semantic_locations")
            return
        resolved_target = semantic_locations[target]
        print(f"🔍 Resolved semantic target '{target}' → {resolved_target}")
    else:
        resolved_target = target

    # === STEP 2: 执行导航 ===
    if isinstance(resolved_target, dict) and "x" in resolved_target and "y" in resolved_target:
        if "heading_deg" in resolved_target:
            print(f"📐 Target includes heading: {resolved_target['heading_deg']}°")
        navigate_to_position(node, robot_name, resolved_target, robot_position_cache)
    else:
        print(f"⚠️ Invalid resolved target: {resolved_target}")
 
 
# === 执行 deliver 行为 ===
def deliver_item(node: Node, robot_name: str, item: str, target):
    getRobotPositionCache(robot_name)
    # === 阶段 1：导航到目标位置 ===
    navigate_to_target(node, robot_name, target)
    print(f"🎯 Reached target location, collecting {item}...")
 
    # === 阶段 2：送达 ===
    print(f"🗣️ Speaking: I am delivering {item}")
    tts_manager.say(f"I am delivering {item}")
    time.sleep(3)  # 模拟送达时间
    tts_manager.say(f"{item} delivered successfully")



 
 
def main():
    rclpy.init()
    robot_name = "robot2"
    item = "candy"
    # target = "lucy"  # or {"x": ..., "y": ..., "heading_deg": ...}
    target = {"x": 500, "y": 500, "heading_deg": 90}
    getRobotPositionCache(robot_name)
    node = rclpy.create_node(f'collect_node_{robot_name}')
    deliver_item(node, robot_name, item, target)
    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == "__main__":

    main()

 