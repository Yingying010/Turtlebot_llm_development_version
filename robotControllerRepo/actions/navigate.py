#!/usr/bin/env python3

import os, sys, math, time, threading
from typing import Dict, Optional, Tuple
# 允许从项目根目录导入
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from rclpy.publisher import Publisher
from rclpy._rclpy_pybind11 import InvalidHandle
from phasespace.rigid_tracker import RigidTracker
from config import semantic_locations
from ttsRepo.stream_tts import tts_manager
from config import config
 
# === 全局缓存 + 锁 ===
robot_position_cache: Dict[str, Dict[str, float]] = {}
cache_lock = threading.Lock()
publisher_dict: Dict[Tuple[int, str], Publisher] = {}

DEFAULT_CP = {
    "priority_table": {},
    "default_priority": 5,
    "safe_yield_radius": 350.0,
    "resume_radius": 450.0,
    "yield_timeout_sec": 15.0,
    "probe_speed": 0.05,
}
cp = {**DEFAULT_CP, **(config.get("collision_params") or {})}

SAFE_YIELD_RADIUS = float(cp.get("safe_yield_radius"))
RESUME_RADIUS     = float(cp.get("resume_radius"))
YIELD_TIMEOUT_SEC = float(cp.get("yield_timeout_sec"))
PROBE_SPEED       = float(cp.get("probe_speed"))
PRIORITY_TABLE    = dict(cp.get("priority_table") or {})
DEFAULT_PRIORITY  = int(cp.get("default_priority"))

def get_priority(name: str) -> int:
    return PRIORITY_TABLE.get(name, DEFAULT_PRIORITY)
 
# === 将 RigidTracker 加入共享 executor，并等待数据就绪 ===

def getRobotPositionCache(robot_name: str, executor: MultiThreadedExecutor) -> Optional[Node]:
    rigid_node = RigidTracker(
        position_cache=robot_position_cache,
        robot_name=robot_name,
        position_lock=cache_lock,  # 传入同一把锁，避免读写冲突
    )
    executor.add_node(rigid_node)
    print(f"⏳ Waiting for position data of {robot_name}...")
    tts_manager.say(f"Initializing tracker for {robot_name}, waiting for position data.")
    for _ in range(50):  # ~10s
        with cache_lock:
            ok = robot_name in robot_position_cache
        if ok:
            print(f"✅ Got position data for {robot_name}.")
            tts_manager.say(f"Position data acquired for {robot_name}.")
            return rigid_node
        time.sleep(0.2)
    print(f"❌ Timeout: No position data for {robot_name}")
    # tts_manager.say(f"Can't get position data for {robot_name}. Please check the tracking system.")
    return None

def get_current_position(robot_name: str) -> tuple:
    with cache_lock:
        rigid = robot_position_cache.get(robot_name)
        if rigid:
            x = rigid["x"]
            y = rigid["z"]
            heading_y = rigid["heading_y"]
            return x, y, heading_y
    print(f"⚠️ No position data for {robot_name}")
    tts_manager.say(f"Can't get position data for {robot_name}. Please check the tracking system.")
    return 0.0, 0.0, 0.0
 
 
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
 
# === NEW: 工具函数：获取所有机器人位姿快照（避免锁外迭代）===
def snapshot_positions() -> Dict[str, Dict[str, float]]:
    with cache_lock:
        return {k: v.copy() for k, v in robot_position_cache.items()}

def distance_xy(a: Dict[str, float], b: Dict[str, float]) -> float:
    dx = a["x"] - b["x"]
    dz = a["z"] - b["z"]
    return math.hypot(dx, dz)

# === NEW: 查找最近的“冲突对象”及距离 ===
def find_nearest_robot(robot_name: str, radius: float) -> Optional[Tuple[str, float]]:
    poses = snapshot_positions()
    if robot_name not in poses:
        return None
    me = poses[robot_name]
    nearest = None
    best_d = float("inf")
    for other, pose in poses.items():
        if other == robot_name:
            continue
        d = distance_xy(me, pose)
        if d < radius and d < best_d:
            best_d = d
            nearest = other
    if nearest is None:
        return None
    return nearest, best_d

# === NEW: 让路等待：若自己优先级低且进入让路半径，就停车并等待到 RESUME_RADIUS 再恢复 ===
def yield_if_needed(node: Node, robot_name: str) -> bool:
    """返回 True 表示需要让路（已停车/等待），False 表示可继续行驶"""
    found = find_nearest_robot(robot_name, SAFE_YIELD_RADIUS)
    if not found:
        return False

    other, dist = found
    my_p = get_priority(robot_name)
    other_p = get_priority(other)

    # 仅当自己优先级更低时让路
    if my_p > other_p:
        # 停车一次
        safe_publish_twist(node, robot_name, Twist())
        print(f"🛑 {robot_name} 让路给 {other} | dist={dist:.1f} | priority {my_p}>{other_p}")
        try:
            tts_manager.say(f"{robot_name} yielding to {other}.")
        except Exception:
            pass

        # 等待直到拉开到 RESUME_RADIUS
        t0 = time.time()
        has_announced_timeout = False
        while True:
            found2 = find_nearest_robot(robot_name, RESUME_RADIUS)
            if not found2:
                print(f"✅ {robot_name} 让路结束，距离已恢复安全。")
                try:
                    tts_manager.say(f"{robot_name} resuming.")
                except Exception:
                    pass
                return True  # 本周期已让路，调用方应跳过本次直行输出
            # 超时兜底：低速探行，避免永久僵持（如优先级相近但对面卡住）
            if time.time() - t0 > YIELD_TIMEOUT_SEC:
                if not has_announced_timeout:
                    print(f"⏳ {robot_name} 让路超时，低速试探前进避免僵持。")
                    try:
                        tts_manager.say(f"{robot_name} probing forward slowly due to timeout.")
                    except Exception:
                        pass
                    has_announced_timeout = True
                twist = Twist()
                twist.linear.x = PROBE_SPEED
                safe_publish_twist(node, robot_name, twist)
                time.sleep(0.25)
                safe_publish_twist(node, robot_name, Twist())
            time.sleep(0.1)
    return False  # 不是自己让路




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
 
 
def rotate_to_final_heading(node: Node, robot_name: str, heading_deg: float,
                            angle_tolerance_deg: float = 5.0):
    if heading_deg is None:
        print("No final heading specified; skipping final rotate.")
        return True  # 不需要旋转，视为成功

    try:
        heading_deg = float(heading_deg)
    except (TypeError, ValueError):
        print(f"⚠️ Invalid heading_deg: {heading_deg}, skipping final rotate.")
        return True

    print(f"\ROTATE TO HEADING: {heading_deg:.1f}°")
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
 
    safe_publish_twist(node, robot_name, Twist())
    time.sleep(0.2)
 
 
def move_forward_until_reached(node: Node, robot_name: str, target: Dict[str, float],
                               tolerance: float = 20.0, max_acceptable_angle_error: float = 25.0, semantic_threshold = 0.0):
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
        
        
        if distance < semantic_threshold:
            print("🎉 Reached target.")
            break

        # === NEW: 先做避撞与让路判断（若需要让路，此次循环仅等待，不发布前进）===
        if yield_if_needed(node, robot_name):
            # 已让路并（可能）恢复，此次循环不前进，重新评估方向和距离
            time.sleep(0.05)
            continue
        
 
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
    move_forward_until_reached(node, robot_name, target, semantic_threshold=300.0)
 
    # Phase 3: 若给了目标朝向则调整
    if "heading_deg" in target:
        rotate_to_final_heading(node, robot_name, target["heading_deg"])
 
    print(f"✅ {robot_name} navigation complete.")
 
 
def navigate_to_target(node: Node, executor: MultiThreadedExecutor, robot_name: str, target):
    is_successful = False

    # 1) 确保位置跟踪节点已接入 executor 并数据就绪
    tracker_robot = getRobotPositionCache(robot_name, executor)
    if tracker_robot is None:
        print("❌ Abort navigation due to missing pose.")
        tts_manager.say(f"Can't get position data for {robot_name}. Please check the tracking system.")
        return is_successful
 
    # 2) 解析语义位置或直接使用坐标
    if isinstance(target, str):
        # firstly, try to find real position
        tracker_target = getRobotPositionCache(target, executor)
        if tracker_target:
            x, y, heading = get_current_position(target)
            resolved_target = {"x": x, "y": y, "heading_deg": heading}
            print(f"🔍 Resolved semantic target in tracking system '{target}' → {resolved_target}")
        else:
            if target in semantic_locations:
                resolved_target = semantic_locations[target]
                print(f"🔍 Resolved semantic target '{target}' → {resolved_target}")
            else:
                print(f"❌ Error: target '{target}' not found in semantic_locations")
                tts_manager.say(f"Can't get position data for {target}. Please check the tracking system or config file")

    else:
        resolved_target = target
 
    # 3) 执行导航
    if isinstance(resolved_target, dict) and "x" in resolved_target and "y" in resolved_target:
        if "heading_deg" in resolved_target:
            print(f"📐 Target includes heading: {resolved_target['heading_deg']}°")
        navigate_to_position(node, robot_name, resolved_target)
    else:
        print(f"⚠️ Invalid resolved target: {resolved_target}")
        return is_successful

    is_successful = True
    return is_successful
 