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

# 全局状态缓存
robot_navigation_cache: Dict[str, Dict] = {}
navigation_cache_lock = threading.Lock()

# 配置参数 - 简化并修正
DISTRIBUTION_CONFIG = {
    "conflict_detection_radius": 300.0,  # 多远算同一目标(mm)
    "base_distribution_radius": 200.0,   # 基础分布半径(mm)
    "planning_wait_time": 0.3,          # 等待其他机器人广播的时间(秒)
}
 
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
    return None

def get_current_position(robot_name: str) -> tuple:
    with cache_lock:
        rigid = robot_position_cache.get(robot_name)
        if rigid:
            x = rigid["x"]
            y = rigid["z"]  # 注意：你的系统用z作为y坐标
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

# === 简化的目标点冲突检测 ===

def detect_target_conflict(robot_name: str, target: Dict) -> List[str]:
    """检测是否有其他机器人也要去相同目标区域"""
    conflicting_robots = []
    nearby_robots = get_nearby_robots(robot_name, detection_range=1000.0)
    
    for other_name, other_info in nearby_robots.items():
        other_target = other_info.get("target_pos", {})
        if not other_target:
            continue
            
        # 检查目标点距离
        distance_between_targets = math.hypot(
            target["x"] - other_target["x"],
            target["y"] - other_target["y"]
        )
        
        if distance_between_targets <= DISTRIBUTION_CONFIG["conflict_detection_radius"]:
            conflicting_robots.append(other_name)
            print(f"🎯 检测到 {other_name} 也要去附近区域")
    
    return conflicting_robots

def generate_distributed_position(robot_name: str, original_target: Dict, 
                                conflicting_robots: List[str]) -> Dict:
    """在目标点周围生成分布位置"""
    
    # 所有要去这个区域的机器人（包括自己）
    all_robots = [robot_name] + conflicting_robots
    all_robots.sort()  # 按名字排序，保证一致性
    
    my_index = all_robots.index(robot_name)
    total_robots = len(all_robots)
    
    base_x, base_y = original_target["x"], original_target["y"]
    
    if total_robots == 1:
        return original_target
    
    # 简化：固定半径分布
    distribution_radius = DISTRIBUTION_CONFIG["base_distribution_radius"]
    
    # 生成分布位置
    if total_robots == 2:
        # 两个机器人：一左一右
        offset_x = distribution_radius if my_index == 0 else -distribution_radius
        distributed_x = base_x + offset_x
        distributed_y = base_y
    else:
        # 多个机器人：圆形分布
        angle_step = 360.0 / total_robots
        my_angle = math.radians(angle_step * my_index)
        distributed_x = base_x + distribution_radius * math.cos(my_angle)
        distributed_y = base_y + distribution_radius * math.sin(my_angle)
    
    distributed_target = {
        "x": distributed_x,
        "y": distributed_y,
        "is_distributed": True,
        "original_target": original_target
    }
    
    print(f"📍 {robot_name} 分布位置: ({distributed_x:.1f}, {distributed_y:.1f}) "
          f"(第{my_index+1}/{total_robots}个)")
    
    return distributed_target

# === 简化的碰撞检测 ===

def broadcast_navigation_status(robot_name: str, current_pos: Tuple, target: Dict, status: str):
    """广播自己的导航状态"""
    x_now, y_now, heading_now = current_pos
    
    nav_info = {
        "current_pos": {"x": x_now, "y": y_now, "heading": heading_now},
        "target_pos": {"x": target["x"], "y": target["y"]},
        "status": status,
        "timestamp": time.time(),
        "robot_radius": 150.0,  # 安全半径（毫米）
    }
    
    with navigation_cache_lock:
        robot_navigation_cache[robot_name] = nav_info

def get_nearby_robots(robot_name: str, detection_range: float = 800.0) -> Dict[str, Dict]:
    """获取附近的机器人信息"""
    with cache_lock:
        my_pos = robot_position_cache.get(robot_name)
    if not my_pos:
        return {}
    
    nearby = {}
    with navigation_cache_lock:
        for other_name, other_info in robot_navigation_cache.items():
            if other_name == robot_name:
                continue
            
            other_pos = other_info["current_pos"]
            # 修正：统一使用相同的坐标系
            distance = math.hypot(
                my_pos["x"] - other_pos["x"], 
                my_pos["z"] - other_pos["y"]  # 注意坐标系转换
            )
            
            if distance <= detection_range:
                nearby[other_name] = other_info
    
    return nearby

def check_immediate_collision(robot_name: str, current_pos: Tuple) -> Optional[str]:
    """检查是否与其他机器人距离过近"""
    x_now, y_now, _ = current_pos
    nearby_robots = get_nearby_robots(robot_name, detection_range=400.0)
    
    for other_name, other_info in nearby_robots.items():
        other_pos = other_info["current_pos"]
        distance = math.hypot(
            x_now - other_pos["x"],
            y_now - other_pos["y"]
        )
        
        safe_distance = 200.0  # 200mm安全距离
        if distance < safe_distance:
            print(f"⚠️ 与 {other_name} 距离过近: {distance:.1f}mm")
            return other_name
    
    return None

def simple_collision_avoidance(robot_name: str, conflicting_robot: str) -> str:
    """简单的避让策略：按名字排序决定优先级"""
    if robot_name < conflicting_robot:
        print(f"🚀 {robot_name} 有优先权，继续前进")
        return "proceed"
    else:
        print(f"🤝 {robot_name} 让路给 {conflicting_robot}")
        return "yield"

def move_forward_with_collision_avoidance(node: Node, robot_name: str, target: Dict[str, float],
                                        tolerance: float = 20.0, max_acceptable_angle_error: float = 25.0):
    """带防碰撞的前进函数 - 简化版"""
    x_target, y_target = target["x"], target["y"]
    print(f"\n🚗 SAFE MOVE → ({x_target:.1f}, {y_target:.1f})")
    
    while True:
        x_now, y_now, heading_y_now = get_current_position(robot_name)
        current_pos = (x_now, y_now, heading_y_now)
        
        # 广播自己的状态
        broadcast_navigation_status(robot_name, current_pos, target, "moving")
        
        dx = x_target - x_now
        dz = y_target - y_now
        distance = math.hypot(dx, dz)

        if distance < tolerance:
            print("🎉 到达目标")
            broadcast_navigation_status(robot_name, current_pos, target, "arrived")
            break
        
        # 简化的碰撞检测：只检查距离过近
        conflicting_robot = check_immediate_collision(robot_name, current_pos)
        
        if conflicting_robot:
            avoidance_action = simple_collision_avoidance(robot_name, conflicting_robot)
            
            if avoidance_action == "yield":
                print(f"🛑 避让 {conflicting_robot}，暂停移动")
                broadcast_navigation_status(robot_name, current_pos, target, "yielding")
                safe_publish_twist(node, robot_name, Twist())
                time.sleep(1.0)  # 等待1秒
                continue
        
        # 正常前进逻辑（保持原有逻辑）
        target_angle = math.degrees(math.atan2(dx, dz)) % 360
        angle_error = (target_angle - heading_y_now + 180) % 360 - 180
        
        if abs(angle_error) > max_acceptable_angle_error:
            print(f"🔄 角度误差过大: {angle_error:.1f}°，先旋转")
            rotate_to_face_target(node, robot_name, target)
            continue
        
        # 前进
        twist = Twist()
        twist.linear.x = 0.1
        safe_publish_twist(node, robot_name, twist)
        print(f"🚗 前进中 | 距离={distance:.2f} | 朝向={heading_y_now:.1f}°")
        
        time.sleep(0.2)
        safe_publish_twist(node, robot_name, Twist())
        time.sleep(0.1)

# === 保持原有的旋转函数不变 ===
 
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
        return True

    try:
        heading_deg = float(heading_deg)
    except (TypeError, ValueError):
        print(f"⚠️ Invalid heading_deg: {heading_deg}, skipping final rotate.")
        return True

    print(f"\nROTATE TO HEADING: {heading_deg:.1f}°")
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

# === 简化的主导航函数 ===

def navigate_with_distribution(node: Node, robot_name: str, target: Dict[str, float]):
    """带分布策略的导航 - 简化版"""
    
    print(f"\n🎯 {robot_name} 导航到 ({target['x']:.1f}, {target['y']:.1f})")
    
    # 1. 广播目标意图
    current_pos = get_current_position(robot_name)
    broadcast_navigation_status(robot_name, current_pos, target, "planning")
    
    # 2. 短暂等待，让其他机器人广播
    time.sleep(DISTRIBUTION_CONFIG["planning_wait_time"])
    
    # 3. 检测冲突
    conflicting_robots = detect_target_conflict(robot_name, target)
    
    # 4. 决定最终目标
    if conflicting_robots:
        print(f"⚠️ 检测到目标冲突，与 {conflicting_robots} 机器人")
        tts_manager.say(f"Multiple robots heading to same area. Adjusting position.")
        final_target = generate_distributed_position(robot_name, target, conflicting_robots)
    else:
        print(f"✅ 无冲突，直接前往目标点")
        final_target = target
    
    # 5. 执行导航
    navigate_to_position_safe(node, robot_name, final_target)

def navigate_to_position_safe(node: Node, robot_name: str, target: Dict[str, float]):
    """安全导航函数"""
    x_target, y_target = target["x"], target["y"]
    x_now, y_now, _ = get_current_position(robot_name)
    distance = math.hypot(x_target - x_now, y_target - y_now)
    
    print(f"\n🧭 安全导航 {robot_name} → ({x_target:.1f}, {y_target:.1f}) | 距离={distance:.2f}")
    
    # Phase 1: 旋转对准
    rotate_to_face_target(node, robot_name, target)
    
    # Phase 2: 安全前进
    move_forward_with_collision_avoidance(node, robot_name, target)
    
    # Phase 3: 最终朝向
    if "heading_deg" in target:
        rotate_to_final_heading(node, robot_name, target["heading_deg"])
    
    print(f"✅ {robot_name} 安全导航完成")

def clean_navigation_cache(robot_name: str):
    """清理导航缓存"""
    with navigation_cache_lock:
        if robot_name in robot_navigation_cache:
            del robot_navigation_cache[robot_name]
            print(f"🧹 清理 {robot_name} 的导航缓存")

# === 主入口函数 ===
 
def navigate_to_target(node: Node, executor: MultiThreadedExecutor, robot_name: str, target):
    is_successful = False

    # 1) 确保位置跟踪节点已接入 executor 并数据就绪
    tracker_robot = getRobotPositionCache(robot_name, executor)
    if tracker_robot is None:
        print("❌ Abort navigation due to missing pose.")
        tts_manager.say(f"Can't get position data for {robot_name}. Please check the tracking system.")
        return is_successful
 
    # 2) 解析语义位置或直接使用坐标
    resolved_target = None
    if isinstance(target, str):
        # 首先尝试在跟踪系统中找到
        tracker_target = getRobotPositionCache(target, executor)
        if tracker_target:
            x, y, heading = get_current_position(target)
            resolved_target = {"x": x, "y": y, "heading": heading}
            print(f"🔍 在跟踪系统中找到目标 '{target}' → {resolved_target}")
        else:
            if target in semantic_locations:
                resolved_target = semantic_locations[target]
                print(f"🔍 在配置中找到目标 '{target}' → {resolved_target}")
            else:
                print(f"❌ Error: target '{target}' not found")
                tts_manager.say(f"Can't find target {target}. Please check the configuration.")
                return is_successful
    else:
        resolved_target = target
 
    # 3) 执行导航
    if isinstance(resolved_target, dict) and "x" in resolved_target and "y" in resolved_target:
        try:
            navigate_with_distribution(node, robot_name, resolved_target)
            is_successful = True
        except Exception as e:
            print(f"❌ Navigation failed: {e}")
            tts_manager.say("Navigation failed.")
        finally:
            clean_navigation_cache(robot_name)
    else:
        print(f"⚠️ Invalid target: {resolved_target}")

    return is_successful