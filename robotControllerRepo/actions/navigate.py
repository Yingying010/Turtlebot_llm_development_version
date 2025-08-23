#!/usr/bin/env python3

import os, sys, math, time, threading
from typing import Dict, Optional, Tuple, List
# 允许从项目根目录导入
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.publisher import Publisher
from rclpy._rclpy_pybind11 import InvalidHandle
import json
from phasespace.rigid_tracker import RigidTracker
import config
from ttsRepo.stream_tts import tts_manager

semantic_locations = config.get("semantic_locations")
 
# === 全局缓存 + 锁 ===
robot_position_cache: Dict[str, Dict[str, float]] = {}
cache_lock = threading.Lock()
publisher_dict: Dict[Tuple[int, str], Publisher] = {}

# === 新增：导航协商机制 ===
# 导航意图广播: {robot_name: {"target": target_dict, "timestamp": time, "status": "announcing/confirmed"}}
navigation_intentions: Dict[str, Dict] = {}
intentions_lock = threading.Lock()

# ROS消息发布器和订阅器
navigation_coordinator_pub: Optional[Publisher] = None
navigation_responses: Dict[str, List[Dict]] = {}  # {requesting_robot: [response1, response2, ...]}
responses_lock = threading.Lock()

# 当前机器人名称（需要在navigate_to_target中设置）
current_robot_name: str = ""


def get_current_robot_name() -> str:
    """获取当前机器人名称"""
    return current_robot_name


def set_current_robot_name(robot_name: str):
    """设置当前机器人名称"""
    global current_robot_name
    current_robot_name = robot_name


def setup_navigation_coordinator(node: Node):
    """初始化导航协调的ROS通信"""
    global navigation_coordinator_pub
    
    # 发布导航意图的topic
    navigation_coordinator_pub = node.create_publisher(
        String, '/navigation_coordination', 10
    )
    
    # 订阅其他机器人的导航意图
    node.create_subscription(
        String, '/navigation_coordination', 
        navigation_coordination_callback, 10
    )
    
    print("🤝 Navigation coordinator initialized")


def navigation_coordination_callback(msg):
    """处理其他机器人的导航协调消息"""
    try:
        data = json.loads(msg.data)
        msg_type = data.get("type")
        
        if msg_type == "navigation_request":
            handle_navigation_request(data)
        elif msg_type == "navigation_response":
            handle_navigation_response(data)
        elif msg_type == "navigation_confirmed":
            handle_navigation_confirmed(data)
            
    except json.JSONDecodeError:
        print(f"⚠️ Invalid navigation coordination message: {msg.data}")


def handle_navigation_request(data):
    """处理其他机器人的导航请求"""
    requesting_robot = data["robot_name"]
    target = data["target"]
    request_id = data["request_id"]
    
    # 检查所有机器人的导航意图（包括自己）
    with intentions_lock:
        all_intentions = navigation_intentions.copy()
    
    # 检查是否有冲突的目标
    conflicting_targets = []
    for robot_name, intention in all_intentions.items():
        if (robot_name != requesting_robot and 
            intention["status"] in ["announcing", "confirmed"] and 
            targets_are_same(intention["target"], target)):
            conflicting_targets.append({
                "robot_name": robot_name,
                "target": intention["target"],
                "timestamp": intention["timestamp"]
            })
    
    # 发送响应 - 注意：这里需要知道当前机器人的名称
    # 从全局变量或其他方式获取当前机器人名称
    current_robot_name = get_current_robot_name()
    
    response = {
        "type": "navigation_response",
        "request_id": request_id,
        "responding_robot": current_robot_name,
        "conflicts": conflicting_targets
    }
    
    if navigation_coordinator_pub:
        navigation_coordinator_pub.publish(String(data=json.dumps(response)))
        print(f"📤 回复导航请求给 {requesting_robot}: {len(conflicting_targets)} 个冲突")


def handle_navigation_response(data):
    """处理其他机器人对我们导航请求的响应"""
    request_id = data["request_id"]
    responding_robot = data["responding_robot"]
    conflicts = data["conflicts"]
    
    with responses_lock:
        if request_id not in navigation_responses:
            navigation_responses[request_id] = []
        navigation_responses[request_id].append({
            "robot": responding_robot,
            "conflicts": conflicts
        })
    
    print(f"📥 收到 {responding_robot} 的导航响应: {len(conflicts)} 个冲突")


def handle_navigation_confirmed(data):
    """处理其他机器人的导航确认"""
    robot_name = data["robot_name"]
    target = data["target"]
    
    with intentions_lock:
        navigation_intentions[robot_name] = {
            "target": target,
            "timestamp": time.time(),
            "status": "confirmed"
        }
    
    print(f"✅ {robot_name} 确认导航到 ({target['x']:.1f}, {target['y']:.1f})")


def targets_are_same(target1: Dict, target2: Dict, tolerance: float = 10.0) -> bool:
    """判断两个目标是否是同一个位置"""
    dx = abs(target1.get("x", 0) - target2.get("x", 0))
    dy = abs(target1.get("y", 0) - target2.get("y", 0))
    return dx <= tolerance and dy <= tolerance


def broadcast_navigation_request(robot_name: str, target: Dict[str, float], timeout: float = 3.0) -> List[Dict]:
    """
    广播导航意图，询问其他机器人是否有冲突
    
    Returns:
        List of conflicting robots and their targets
    """
    request_id = f"{robot_name}_{int(time.time() * 1000)}"
    
    # 清空之前的响应
    with responses_lock:
        navigation_responses[request_id] = []
    
    # 发布导航请求
    request = {
        "type": "navigation_request",
        "robot_name": robot_name,
        "target": target,
        "request_id": request_id,
        "timestamp": time.time()
    }
    
    if navigation_coordinator_pub:
        navigation_coordinator_pub.publish(String(data=json.dumps(request)))
        print(f"📢 {robot_name} 广播导航意图: 目标 ({target['x']:.1f}, {target['y']:.1f})")
    
    # 等待响应
    start_time = time.time()
    while time.time() - start_time < timeout:
        time.sleep(0.1)
    
    # 收集所有冲突
    all_conflicts = []
    with responses_lock:
        for response in navigation_responses.get(request_id, []):
            all_conflicts.extend(response["conflicts"])
    
    # 同时检查当前已记录的导航意图（防止消息丢失）
    with intentions_lock:
        current_intentions = navigation_intentions.copy()
    
    for other_robot, intention in current_intentions.items():
        if (other_robot != robot_name and 
            intention["status"] in ["announcing", "confirmed"] and 
            targets_are_same(intention["target"], target)):
            
            # 检查是否已经在冲突列表中
            already_exists = any(c["robot_name"] == other_robot for c in all_conflicts)
            if not already_exists:
                all_conflicts.append({
                    "robot_name": other_robot,
                    "target": intention["target"],
                    "timestamp": intention["timestamp"]
                })
                print(f"🔍 检测到本地记录的冲突: {other_robot}")
    
    print(f"📊 {robot_name} 收到 {len(all_conflicts)} 个冲突响应")
    return all_conflicts


def resolve_navigation_conflicts(robot_name: str, target: Dict[str, float], conflicts: List[Dict]) -> Dict[str, float]:
    """
    解决导航冲突，分配新的目标点
    
    Args:
        robot_name: 当前机器人名称
        target: 原始目标
        conflicts: 冲突的机器人列表
        
    Returns:
        分配给当前机器人的新目标点
    """
    if not conflicts:
        # 没有冲突，使用原始目标
        print(f"✅ {robot_name} 无冲突，使用原始目标")
        return target
    
    # 收集所有要去这个目标的机器人（包括自己）
    all_robots = [robot_name]
    for conflict in conflicts:
        if conflict["robot_name"] not in all_robots:
            all_robots.append(conflict["robot_name"])
    
    print(f"🎯 检测到 {len(all_robots)} 个机器人要去相同目标: {all_robots}")
    print(f"🔍 冲突详情:")
    for conflict in conflicts:
        print(f"  - {conflict['robot_name']}: ({conflict['target']['x']:.1f}, {conflict['target']['y']:.1f})")
    
    # 根据时间戳或名称排序，确保分配的一致性
    all_robots.sort()
    
    # 计算当前机器人在列表中的索引
    robot_index = all_robots.index(robot_name)
    print(f"📍 {robot_name} 在排序列表中的索引: {robot_index}")
    
    # 使用圆形分布算法
    center_x = target["x"]
    center_y = target["y"]
    radius = 200.0  # 分布半径
    num_robots = len(all_robots)
    
    if num_robots == 1:
        # 只有自己，使用原始目标
        new_target = target.copy()
        print(f"🎯 {robot_name} 独自导航，使用原始目标")
    else:
        # 分布在圆周上
        angle_step = 360.0 / num_robots
        angle_deg = robot_index * angle_step
        angle_rad = math.radians(angle_deg)
        
        new_x = center_x + radius * math.cos(angle_rad)
        new_y = center_y + radius * math.sin(angle_rad)
        
        # 计算朝向目标中心的角度
        heading_to_center = math.degrees(math.atan2(center_x - new_x, center_y - new_y)) % 360
        
        new_target = {
            "x": new_x,
            "y": new_y,
            "heading_deg": heading_to_center
        }
        
        print(f"🎯 {robot_name} 分布式分配:")
        print(f"  - 原始目标: ({center_x:.1f}, {center_y:.1f})")
        print(f"  - 分配角度: {angle_deg:.1f}°")
        print(f"  - 新目标: ({new_x:.1f}, {new_y:.1f})")
        print(f"  - 朝向角度: {heading_to_center:.1f}°")
    
    return new_target


def confirm_navigation_intent(robot_name: str, target: Dict[str, float]):
    """确认导航意图，通知其他机器人"""
    with intentions_lock:
        navigation_intentions[robot_name] = {
            "target": target,
            "timestamp": time.time(),
            "status": "confirmed"
        }
    
    # 广播确认消息
    confirmation = {
        "type": "navigation_confirmed",
        "robot_name": robot_name,
        "target": target,
        "timestamp": time.time()
    }
    
    if navigation_coordinator_pub:
        navigation_coordinator_pub.publish(String(data=json.dumps(confirmation)))
        print(f"✅ {robot_name} 确认导航意图")


def cleanup_navigation_intent(robot_name: str):
    """清理导航意图（导航完成后调用）"""
    with intentions_lock:
        if robot_name in navigation_intentions:
            del navigation_intentions[robot_name]
            print(f"🗑️ {robot_name} 清理导航意图")


# === 原有函数保持不变 ===

def getRobotPositionCache(robot_name: str, executor: MultiThreadedExecutor) -> Optional[Node]:
    rigid_node = RigidTracker(
        position_cache=robot_position_cache,
        robot_name=robot_name,
        position_lock=cache_lock,
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
        return True

    try:
        heading_deg = float(heading_deg)
    except (TypeError, ValueError):
        print(f"⚠️ Invalid heading_deg: {heading_deg}, skipping final rotate.")
        return True

    print(f"🔄 ROTATE TO HEADING: {heading_deg:.1f}°")
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
 
        target_angle = math.degrees(math.atan2(dx, dz)) % 360
        angle_error = (target_angle - heading_y_now + 180) % 360 - 180
 
        if abs(angle_error) > max_acceptable_angle_error:
            print(f"🔁 Too much angle error: {angle_error:.1f}°, rotating first...")
            rotate_to_face_target(node, robot_name, target)
            continue
 
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
 
    rotate_to_face_target(node, robot_name, target)
    move_forward_until_reached(node, robot_name, target, semantic_threshold=300.0)
 
    if "heading_deg" in target:
        rotate_to_final_heading(node, robot_name, target["heading_deg"])
 
    print(f"✅ {robot_name} navigation complete.")
 
 
def navigate_to_target(node: Node, executor: MultiThreadedExecutor, robot_name: str, target):
    """
    导航到目标，包含分布式协商机制
    """
    is_successful = False

    # 0) 设置当前机器人名称
    set_current_robot_name(robot_name)
    
    # 1) 初始化导航协调器
    setup_navigation_coordinator(node)
    
    # 2) 确保位置跟踪节点已接入 executor 并数据就绪
    tracker_robot = getRobotPositionCache(robot_name, executor)
    if tracker_robot is None:
        print("❌ Abort navigation due to missing pose.")
        tts_manager.say(f"Can't get position data for {robot_name}. Please check the tracking system.")
        return is_successful
 
    # 3) 解析语义位置或直接使用坐标
    if isinstance(target, str):
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

    # 4) 分布式协商导航
    if isinstance(resolved_target, dict) and "x" in resolved_target and "y" in resolved_target:
        
        # Step 1: 先记录自己的导航意图为"announcing"状态
        with intentions_lock:
            navigation_intentions[robot_name] = {
                "target": resolved_target,
                "timestamp": time.time(),
                "status": "announcing"
            }
        
        # Step 2: 广播导航意图，询问是否有冲突
        print(f"\n🤝 {robot_name} 开始导航协商...")
        conflicts = broadcast_navigation_request(robot_name, resolved_target)
        
        # Step 3: 根据冲突情况分配实际目标
        actual_target = resolve_navigation_conflicts(robot_name, resolved_target, conflicts)
        
        # Step 4: 确认导航意图
        confirm_navigation_intent(robot_name, actual_target)
        
        # Step 5: 执行导航
        print(f"🎯 {robot_name} 开始导航到分配目标: ({actual_target['x']:.1f}, {actual_target['y']:.1f})")
        if "heading_deg" in actual_target:
            print(f"📐 Target includes heading: {actual_target['heading_deg']}°")
        
        navigate_to_position(node, robot_name, actual_target)
        
        # Step 6: 导航完成，清理意图
        cleanup_navigation_intent(robot_name)
        
    else:
        print(f"⚠️ Invalid resolved target: {resolved_target}")
        return is_successful

    is_successful = True
    return is_successful

def navigate_to(node: Node, executor: MultiThreadedExecutor, robot_name: str, target):
    """
    轻量包装：让签名更简单，内部复用 navigate_to_target
    """
    return navigate_to_target(node, executor, robot_name, target)


# === 调试和监控函数 ===

def get_navigation_status() -> Dict:
    """获取当前导航协商状态"""
    with intentions_lock:
        return navigation_intentions.copy()


def print_navigation_status():
    """打印当前导航状态"""
    status = get_navigation_status()
    print("\n" + "="*50)
    print("🤝 导航协商状态:")
    for robot, intention in status.items():
        target = intention["target"]
        status_str = intention["status"]
        print(f"  {robot}: → ({target['x']:.1f}, {target['y']:.1f}) [{status_str}]")
    print("="*50 + "\n")


def main():
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init()
    node = rclpy.create_node("navigation_tester")
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # === 这里修改参数进行测试 ===
    robot_name = "robot1"
    # 目标点，可以用语义位置（"table"）或者直接用坐标
    target = {"x": 50.0, "y": 50.0, "heading": None}

    try:
        print(f"🚀 Starting navigation test for {robot_name}...")
        success = navigate_to(node, executor, robot_name, target)
        print(f"\n✅ Navigation success: {success}")
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()