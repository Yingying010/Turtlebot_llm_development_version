#!/usr/bin/env python3

import os, sys, math, time, threading
from typing import Dict, Optional, Tuple, List
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
 
# === 全局缓存 + 锁 ===
robot_position_cache: Dict[str, Dict[str, float]] = {}
cache_lock = threading.Lock()
publisher_dict: Dict[Tuple[int, str], Publisher] = {}

# === 新增：分布式目标点管理 ===
# 记录每个目标点的占用情况: {target_key: [robot1, robot2, ...]}
target_occupation: Dict[str, List[str]] = {}
target_occupation_lock = threading.Lock()

# 记录每个机器人的分配位置: {robot_name: {"x": x, "y": y, "original_target": target_key}}
robot_assigned_positions: Dict[str, Dict] = {}


def get_target_key(target: Dict[str, float], tolerance: float = 10.0) -> str:
    """根据目标坐标生成唯一键值，用于识别相同目标点"""
    x = target.get("x", 0)
    y = target.get("y", 0)
    # 使用tolerance将相近的点归为同一个目标
    x_rounded = round(x / tolerance) * tolerance
    y_rounded = round(y / tolerance) * tolerance
    return f"{x_rounded:.1f},{y_rounded:.1f}"


def distribute_robots_around_target(target: Dict[str, float], robot_list: List[str], 
                                  radius: float = 200.0) -> Dict[str, Dict[str, float]]:
    """
    将多个机器人分布在目标点周围的圆形区域上
    
    Args:
        target: 原始目标点 {"x": x, "y": y, ...}
        robot_list: 需要分配位置的机器人列表
        radius: 分布半径
    
    Returns:
        Dict[robot_name, {"x": new_x, "y": new_y, "heading_deg": heading}]
    """
    center_x = target["x"]
    center_y = target["y"]
    num_robots = len(robot_list)
    
    distributed_positions = {}
    
    if num_robots == 1:
        # 只有一个机器人，直接去原始目标点
        distributed_positions[robot_list[0]] = target.copy()
    else:
        # 多个机器人，均匀分布在圆周上
        angle_step = 360.0 / num_robots
        
        for i, robot_name in enumerate(robot_list):
            angle_deg = i * angle_step
            angle_rad = math.radians(angle_deg)
            
            new_x = center_x + radius * math.cos(angle_rad)
            new_y = center_y + radius * math.sin(angle_rad)
            
            # 计算朝向目标中心的角度
            heading_to_center = math.degrees(math.atan2(center_x - new_x, center_y - new_y)) % 360
            
            distributed_positions[robot_name] = {
                "x": new_x,
                "y": new_y,
                "heading_deg": heading_to_center,
                "original_target": get_target_key(target)
            }
            
            print(f"🎯 分配位置给 {robot_name}: ({new_x:.1f}, {new_y:.1f}) 朝向 {heading_to_center:.1f}°")
    
    return distributed_positions


def register_robot_for_target(robot_name: str, target: Dict[str, float]) -> Dict[str, float]:
    """
    为机器人注册目标点，如果有多个机器人去同一个目标，则自动分配分布式位置
    
    Args:
        robot_name: 机器人名称
        target: 目标点坐标
    
    Returns:
        分配给该机器人的实际目标位置
    """
    target_key = get_target_key(target)
    
    with target_occupation_lock:
        # 检查是否已经有机器人在前往这个目标
        if target_key not in target_occupation:
            target_occupation[target_key] = []
        
        # 如果这个机器人还没有注册到这个目标
        if robot_name not in target_occupation[target_key]:
            target_occupation[target_key].append(robot_name)
            print(f"📝 注册 {robot_name} 到目标 {target_key}, 当前队列: {target_occupation[target_key]}")
        
        # 获取当前前往这个目标的所有机器人
        robots_for_target = target_occupation[target_key].copy()
    
    # 重新分配所有机器人的位置
    distributed_positions = distribute_robots_around_target(target, robots_for_target)
    
    # 更新全局分配记录
    with target_occupation_lock:
        for robot, pos in distributed_positions.items():
            robot_assigned_positions[robot] = pos
            print(f"🎯 更新 {robot} 的分配位置: ({pos['x']:.1f}, {pos['y']:.1f})")
    
    return distributed_positions[robot_name]


def unregister_robot_from_target(robot_name: str):
    """
    机器人到达目标后，从占用列表中移除
    """
    with target_occupation_lock:
        if robot_name in robot_assigned_positions:
            original_target = robot_assigned_positions[robot_name].get("original_target")
            if original_target and original_target in target_occupation:
                if robot_name in target_occupation[original_target]:
                    target_occupation[original_target].remove(robot_name)
                    print(f"✅ {robot_name} 已从目标 {original_target} 的占用列表中移除")
                
                # 如果没有机器人前往这个目标了，清空记录
                if not target_occupation[original_target]:
                    del target_occupation[original_target]
                    print(f"🗑️ 目标 {original_target} 的占用记录已清空")
            
            # 移除机器人的分配位置记录
            del robot_assigned_positions[robot_name]
            print(f"🗑️ 清除 {robot_name} 的分配位置记录")


# === 原有函数保持不变 ===

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

    print(f"\🔄 ROTATE TO HEADING: {heading_deg:.1f}°")
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
                               tolerance: float = 50.0, max_acceptable_angle_error: float = 25.0):
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
            resolved_target = {"x": x, "y": y, "heading": heading}
            print(f"🔍 Resolved semantic target in tracking system '{target}' → {resolved_target}")
        else:
            if target in semantic_locations:
                resolved_target = semantic_locations[target]
                print(f"🔍 Resolved semantic target '{target}' → {resolved_target}")
            else:
                print(f"❌ Error: target '{target}' not found in semantic_locations")
                tts_manager.say(f"Can't get position data for {target}. Please check the tracking system or config file")
                return is_successful
    else:
        resolved_target = target

    # 3) 新增：分布式目标点管理
    if isinstance(resolved_target, dict) and "x" in resolved_target and "y" in resolved_target:
        # 注册机器人到目标点，获取分配的实际位置
        actual_target = register_robot_for_target(robot_name, resolved_target)
        
        print(f"🎯 {robot_name} 分配到实际目标: ({actual_target['x']:.1f}, {actual_target['y']:.1f})")
        if "heading_deg" in actual_target:
            print(f"📐 Target includes heading: {actual_target['heading_deg']}°")
        
        # 执行导航到分配的位置
        navigate_to_position(node, robot_name, actual_target)
        
        # 导航完成后，从占用列表中移除
        unregister_robot_from_target(robot_name)
        
    else:
        print(f"⚠️ Invalid resolved target: {resolved_target}")
        return is_successful

    is_successful = True
    return is_successful


# === 新增：辅助函数用于调试和监控 ===

def get_target_occupation_status() -> Dict:
    """获取当前目标占用状态，用于调试"""
    with target_occupation_lock:
        return {
            "target_occupation": target_occupation.copy(),
            "robot_assigned_positions": robot_assigned_positions.copy()
        }


def print_occupation_status():
    """打印当前占用状态"""
    status = get_target_occupation_status()
    print("\n" + "="*50)
    print("🎯 目标占用状态:")
    for target_key, robots in status["target_occupation"].items():
        print(f"  {target_key}: {robots}")
    print("\n🤖 机器人分配位置:")
    for robot, pos in status["robot_assigned_positions"].items():
        print(f"  {robot}: ({pos['x']:.1f}, {pos['y']:.1f}) heading={pos.get('heading_deg', 'N/A'):.1f}°")
    print("="*50 + "\n")