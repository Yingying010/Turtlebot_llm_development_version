#!/usr/bin/env python3

# collect.py —— 独立导航 + 语音反馈 + 模拟收集
 
import os, sys, time, math
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy._rclpy_pybind11 import InvalidHandle
from typing import Dict, List, Tuple, Optional
from geometry_msgs.msg import Twist
import threading
# 添加项目根路径
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from config import semantic_locations
from ttsRepo.stream_tts import tts_manager
from phasespace.rigid_tracker import RigidTracker
from rclpy.executors import MultiThreadedExecutor
 
# 全局缓存
robot_position_cache: Dict[str, Dict[str, float]] = {}
cache_lock = threading.Lock()
publisher_dict: Dict[Tuple[int, str], Publisher] = {}
 
# === 获取机器人位姿 ===
def get_current_position(robot_name: str) -> tuple:
    with cache_lock:
        rigid = robot_position_cache.get(robot_name)
        if rigid:
            x = rigid["x"]
            y = rigid["z"]
            heading_y = rigid["heading_y"]
            return x, y, heading_y
    print(f"⚠️ No position data for {robot_name}")
    return 0.0, 0.0, 0.0
 
 
# === 启动 PhaseSpace 追踪器 ===
def getRobotPositionCache(robot_id: str, executor: MultiThreadedExecutor) -> Optional[Node]:
    rigid_node = RigidTracker(
        position_cache=robot_position_cache,
        robot_name=robot_id,
        position_lock=cache_lock,  # 传入同一把锁，避免读写冲突
    )
    executor.add_node(rigid_node)
    print(f"⏳ Waiting for position data of {robot_id}...")
    for _ in range(50):  # ~10s
        with cache_lock:
            ok = robot_id in robot_position_cache
        if ok:
            print(f"✅ Got position data for {robot_id}.")
            return rigid_node
        time.sleep(0.2)
    print(f"❌ Timeout: No position data for {robot_id}")
    return None


def _ensure_pub(node: Node, robot_name: str) -> Publisher:
    key = (id(node), robot_name)
    pub = publisher_dict.get(key)
    if pub is None:
        pub = node.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
        publisher_dict[key] = pub
    return pub

def safe_publish_twist(node: Node, robot_name: str, twist: Twist):
    key = (id(node), robot_name)
    pub = _ensure_pub(node, robot_name)
    try:
        pub.publish(twist)
    except InvalidHandle:
        pub = node.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
        publisher_dict[key] = pub
        pub.publish(twist)
 
 
# === 转向目标方向 ===
def rotate_to_face_target(node: Node, robot_id: str, target: Dict[str, float],
                          angle_tolerance_deg: float = 5.0):
    x_target, y_target = target["x"], target["y"]
    x_now, y_now, _ = get_current_position(robot_id)
    dx = x_target - x_now
    dz = y_target - y_now
    target_angle = math.degrees(math.atan2(dx, dz)) % 360
 
    print(f"\n🔄 ROTATE | target: ({x_target:.1f}, {y_target:.1f}) → {target_angle:.1f}°")
 
    while True:
        _, _, heading_y_now = get_current_position(robot_id)
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
 
        safe_publish_twist(node, robot_id, twist)
        print(f"↪️ turning {'left' if new_direction==1 else 'right'} | heading_y: {heading_y_now:.1f} | "
              f"target_angle: {target_angle:.1f} | error: {angle_error:.1f}° | speed: {twist.angular.z:.2f}")

        time.sleep(0.1)
 
    safe_publish_twist(node, robot_id, Twist())
    time.sleep(0.2)
 

def rotate_to_final_heading(robot_name, publisher, heading_deg, angle_tolerance_deg=5):
    print(f"\n🎯 ROTATE TO HEADING: {heading_deg:.1f}°")
    while True:
        _, _, heading_y_now = get_current_position(robot_name)
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
def move_forward_until_reached(node: Node, robot_name: str, target: Dict[str, float],
                               tolerance: float = 20.0, max_acceptable_angle_error: float = 25.0):
    x_target, y_target = target["x"], target["y"]

    print(f"\n🚗 NEED TO MOVE → ({x_target:.1f}, {y_target:.1f})")

    while True:
        x_now, y_now, heading_y_now = get_current_position(robot_name)
        dx = x_target - x_now
        dz = y_target - y_now
        distance = math.hypot(dx, dz)

        if distance < tolerance:
            print("🎉 Reached target.")
            break
 
        # 目标方向角与误差
        target_angle = math.degrees(math.atan2(dx, dz)) % 360
        angle_error = (target_angle - heading_y_now + 180) % 360 - 180
 
        if abs(angle_error) > max_acceptable_angle_error:
            print(f"🔁 Too much angle error: {angle_error:.1f}°, rotating first...")
            rotate_to_face_target(node, robot_name, target)
            continue
 
        # 前进（小速度：精度更好）
        twist = Twist()
        twist.linear.x = 0.1
        safe_publish_twist(node, robot_name, twist)
        print(f"🚗 Moving | dist={distance:.2f} | heading={heading_y_now:.1f}°, target={target_angle:.1f}°, "
              f"error={angle_error:.1f}°")

        time.sleep(0.2)

        safe_publish_twist(node, robot_name, Twist())

        time.sleep(0.1)


def navigate_to_position(node: Node, robot_name: str, target: Dict[str, float]):
    x_target, y_target = target["x"], target["y"]
    x_now, y_now, _ = get_current_position(robot_name)
    dx = x_target - x_now
    dy = y_target - y_now

    distance = math.hypot(dx, dy)
 
    print(f"\n🧭 NAVIGATE {robot_name} → ({x_target:.1f}, {y_target:.1f}) | dist={distance:.2f}")
 
    # Phase 1: 先对准
    rotate_to_face_target(node, robot_name, target)
 
    # Phase 2: 直行
    move_forward_until_reached(node, robot_name, target)
 
    # Phase 3: 若给了目标朝向则调整
    if "heading_deg" in target:
        rotate_to_final_heading(node, robot_name, target["heading_deg"])
 
    print(f"✅ {robot_name} navigation complete.")


def navigate_to_target(node: Node, executor: MultiThreadedExecutor, robot_name: str, target):
    # 1) 确保位置跟踪节点已接入 executor 并数据就绪
    rigid_node = getRobotPositionCache(robot_name, executor)

    if rigid_node is None:
        print("❌ Abort navigation due to missing pose.")
        return
 
    # 2) 解析语义位置或直接使用坐标
    if isinstance(target, str):
        if target not in semantic_locations:
            print(f"❌ Error: target '{target}' not found in semantic_locations")
            return

        resolved_target = semantic_locations[target]
        print(f"🔍 Resolved semantic target '{target}' → {resolved_target}")
    else:
        resolved_target = target
 
    # 3) 执行导航
    if isinstance(resolved_target, dict) and "x" in resolved_target and "y" in resolved_target:
        if "heading_deg" in resolved_target:
            print(f"📐 Target includes heading: {resolved_target['heading_deg']}°")
        navigate_to_position(node, robot_name, resolved_target)

    else:
        print(f"⚠️ Invalid resolved target: {resolved_target}")
 
 
 
# === 执行 collect 行为 ===
def collect_item(node: Node, robot_name: str, item: str, target, executor):
    getRobotPositionCache(robot_name, executor)
    # === 阶段 1：导航到目标位置 ===
    navigate_to_target(node, executor, robot_name, target)
    print(f"🎯 Reached target location, collecting {item}...")
 
    # === 阶段 2：收集 ===
    print(f"🗣️ Speaking: I am collecting {item}")
    tts_manager.say(f"I am collecting {item}")
    time.sleep(3)  # 模拟收集时间
    tts_manager.say(f"{item} collected successfully")



 
 
def main():
    rclpy.init()
    robot_name = "robot2"
    item = "candy"
    # target = "lucy"  # 或者 {"x": ..., "y": ..., "heading_deg": ...}
    target = {"x": 200, "y": -500, "heading_deg": 90}
    getRobotPositionCache(robot_name)
    node = rclpy.create_node(f'collect_node_{robot_name}')
    collect_item(node, robot_name, item, target)
    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == "__main__":

    main()

 