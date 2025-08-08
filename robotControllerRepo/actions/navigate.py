#!/usr/bin/env python3
import os, sys, math, time, threading
from typing import Dict, List, Optional

# sys.path.append("/home/ubuntu2204/Desktop/yingying/ai-assistant_macos")
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from typing import Tuple
from rclpy.publisher import Publisher
from phasespace.rigid_tracker import RigidTracker
from config import semantic_locations
from rclpy._rclpy_pybind11 import InvalidHandle
 


# === 全局缓存 + 锁 ===
robot_position_cache: Dict[str, Dict[str, float]] = {}

publisher_dict: Dict[Tuple[int, str], Publisher] = {}
cache_lock = threading.Lock()


# === 将 RigidTracker 加入同一个 executor，并等待数据就绪 ===
def getRobotPositionCache(robot_id: str, executor: MultiThreadedExecutor) -> Optional[object]:
    # 不要在这里持锁！
    rigid_node = RigidTracker(
        position_cache=robot_position_cache,
        robot_name=robot_id,
        position_lock=cache_lock,        # ✅ 正确参数名
    )
    executor.add_node(rigid_node)        # ✅ 加入共享 executor
 
    print(f"⏳ Waiting for position data of {robot_id}...")
    for _ in range(50):  # ~10s
        with cache_lock:
            ok = robot_id in robot_position_cache
        if ok:
            print(f"✅ Got position data for {robot_id}. and {robot_position_cache}")
            return rigid_node
        time.sleep(0.2)
 
    print(f"❌ Timeout: No position data for {robot_id}")
    return None


def get_current_position(robot_name: str) -> tuple:
    with cache_lock:
        rigid = robot_position_cache.get(robot_name)
        if rigid:
            # 注意：PhaseSpace 坐标你之前用 x,z（y 轴朝上），这里保持一致
            x = rigid["x"]
            y = rigid["z"]
            heading_y = rigid["heading_y"]
            return x, y, heading_y
    print(f"⚠️ No position data for {robot_name}")
    return 0.0, 0.0, 0.0


def _ensure_pub(node: Node, robot_name: str) -> Publisher:
    key = (id(node), robot_name)
    pub = publisher_dict.get(key)
 
    # 若没有或句柄失效，创建/重建
    if pub is None:
        pub = node.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
        publisher_dict[key] = pub
        return pub
 
    # 轻量探测：对失效 publisher，下面 publish 会抛 InvalidHandle；我们捕获并重建
    try:
        # 做一次空速率 publish 探针不太优雅，所以交给调用处第一次 publish 捕获也行
        return pub
    except Exception:
        # 理论上不会在这触发，这里留兜底
        pub = node.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
        publisher_dict[key] = pub
        return pub
    
def safe_publish_twist(node: Node, robot_name: str, twist: Twist):
    pub = _ensure_pub(node, robot_name)
    try:
        pub.publish(twist)
    except InvalidHandle:
        # 句柄失效 → 重建后重试一次
        pub = node.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
        publisher_dict[(id(node), robot_name)] = pub
        pub.publish(twist)

def rotate_to_face_target(node, robot_id: str, publisher, target: Dict[str, float],
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

    publisher.publish(Twist())
    time.sleep(0.2)


def rotate_to_final_heading(node, robot_name: str, publisher, heading_deg: float,
                            angle_tolerance_deg: float = 5.0):
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

        safe_publish_twist(node, robot_name, twist)
        print(f"↪️ adjusting to heading {heading_deg:.1f}°, current={heading_y_now:.1f}°, error={angle_error:.1f}°")
        time.sleep(0.1)

    publisher.publish(Twist())
    time.sleep(0.2)


def move_forward_until_reached(node, robot_name: str, publisher, target: Dict[str, float],
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
            rotate_to_face_target(node, robot_name, publisher, target)
            continue

        # 前进
        twist = Twist()
        twist.linear.x = 0.1  # 小速度保证精度
        safe_publish_twist(node, robot_name, twist)
        print(f"🚗 Moving | dist={distance:.2f} | heading={heading_y_now:.1f}°, target={target_angle:.1f}°, "
              f"error={angle_error:.1f}°")

        time.sleep(0.2)
        publisher.publish(Twist())  # 脉冲式推进，减小累计误差
        time.sleep(0.1)


def navigate_to_position(node: Node, robot_name: str, target: Dict[str, float]):
    x_target, y_target = target["x"], target["y"]
    x_now, y_now, _ = get_current_position(robot_name)

    dx = x_target - x_now
    dy = y_target - y_now
    distance = math.hypot(dx, dy)

    print(f"\n🧭 NAVIGATE {robot_name} → ({x_target:.1f}, {y_target:.1f}) | dist={distance:.2f}")

    pub = _ensure_pub(node, robot_name)

    # Phase 1: 先对准
    rotate_to_face_target(node, robot_name, pub, target)

    # Phase 2: 直行
    move_forward_until_reached(node, robot_name, pub, target)

    # Phase 3: 若给了目标朝向则调整
    if "heading_deg" in target:
        rotate_to_final_heading(node, robot_name, pub, target["heading_deg"])

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


# （可选）清理：如果你在任务末尾想释放 RigidTracker
def detach_rigid_tracker(executor: MultiThreadedExecutor, rigid_node: Node):
    try:
        executor.remove_node(rigid_node)
    except Exception:
        pass
    try:
        rigid_node.destroy_node()
    except Exception:
        pass
