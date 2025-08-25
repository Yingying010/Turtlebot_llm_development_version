#!/usr/bin/env python3

import os, sys, math, time, threading
from typing import Dict, Optional, Tuple, List
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.publisher import Publisher
from rclpy._rclpy_pybind11 import InvalidHandle
import json
import argparse
from phasespace.rigid_tracker import RigidTracker
import config
from loguru import logger
from ttsRepo.stream_tts import tts_manager
from robotControllerRepo.utils.coord_convert import resolve_object_position
from robotControllerRepo.actions.rotate import rotate_deg
from robotControllerRepo.actions.move import move


# Configuration
semantic_locations = config.get("semantic_locations")

# Global state management
robot_position_cache: Dict[str, Dict[str, float]] = {}
cache_lock = threading.Lock()
publisher_dict: Dict[Tuple[int, str], Publisher] = {}

# Navigation coordination state
navigation_intentions: Dict[str, Dict] = {}
intentions_lock = threading.Lock()
navigation_coordinator_pub: Optional[Publisher] = None
navigation_responses: Dict[str, List[Dict]] = {}
responses_lock = threading.Lock()
current_robot_name: str = ""

# ============================================================================
# CORE POSITION AND CONTROL FUNCTIONS
# ============================================================================

def get_current_robot_name() -> str:
    """Get current robot name for coordination"""
    return current_robot_name

def set_current_robot_name(robot_name: str):
    """Set current robot name for coordination"""
    global current_robot_name
    current_robot_name = robot_name

def getRobotPositionCache(name: str, executor: MultiThreadedExecutor) -> Optional[Node]:
    """Initialize position tracking for specified robot"""
    rigid_node = RigidTracker(
        position_cache=robot_position_cache,
        name=name,
        position_lock=cache_lock,
    )
    executor.add_node(rigid_node)
    print(f"[POSITION] Initializing tracker for {name}")
    tts_manager.say_sync(f"Initializing tracker for {name}, waiting for position data.")
    
    for _ in range(50):  # 10 second timeout
        with cache_lock:
            if name in robot_position_cache:
                print(f"[POSITION] Position data acquired for {name}")
                tts_manager.say_sync(f"Position data acquired for {name}.")
                return rigid_node
        time.sleep(0.2)
    
    print(f"[ERROR] Position tracking timeout for {name}")
    return None

def get_current_position(robot_name: str) -> Tuple[float, float, float]:
    """Get current position of specified robot"""
    with cache_lock:
        rigid = robot_position_cache.get(robot_name)
        if rigid:
            return rigid["x"], rigid["z"], rigid["heading_y"]
    
    print(f"[WARNING] No position data available for {robot_name}")
    tts_manager.say_sync(f"Can't get position data for {robot_name}. Please check the tracking system.")
    return 0.0, 0.0, 0.0

def get_position_with_polling(robot_name: str, timeout_sec=120.0, poll_interval=0.1):
    """尝试在 timeout_sec 秒内反复获取非 None 的位置"""
    start_time = time.time()
    while time.time() - start_time < timeout_sec:
        x, y, heading = get_current_position(robot_name)
        if None not in (x, y, heading):
            return x, y, heading
        time.sleep(poll_interval)
    logger.error(f"[POSITION] Timeout getting position for {robot_name}")
    return None, None, None

def _ensure_publisher(node: Node, robot_name: str) -> Publisher:
    """Ensure publisher exists for robot command velocity"""
    key = (id(node), robot_name)
    pub = publisher_dict.get(key)
    if pub is None:
        pub = node.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
        publisher_dict[key] = pub
    return pub

def safe_publish_twist(node: Node, robot_name: str, twist: Twist):
    """Safely publish twist command with error handling"""
    key = (id(node), robot_name)
    pub = _ensure_publisher(node, robot_name)
    try:
        pub.publish(twist)
    except InvalidHandle:
        pub = node.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
        publisher_dict[key] = pub
        pub.publish(twist)

# ============================================================================
# BASIC MOVEMENT PRIMITIVES
# ============================================================================

def rotate_to_face_target(node: Node, robot_id: str, target: Dict[str, float],
                          angle_tolerance_deg: float = 5.0):
    """Rotate robot to face target position using feedback control"""
    x_target, y_target = target["x"], target["y"]
    x_now, y_now, _ = get_position_with_polling(robot_id)
    dx = x_target - x_now
    dz = y_target - y_now
    target_angle = math.degrees(math.atan2(dx, dz)) % 360

    print(f"[ROTATE] Target: ({x_target:.1f}, {y_target:.1f}) -> {target_angle:.1f} degrees")

    while True:
        _, _, heading_y_now = get_position_with_polling(robot_id)
        angle_error = (target_angle - heading_y_now + 180) % 360 - 180

        if abs(angle_error) < angle_tolerance_deg:
            print("[ROTATE] Target orientation achieved")
            break

        direction = 1 if angle_error > 0 else -1
        twist = Twist()

        # Adaptive angular velocity based on error magnitude
        if abs(angle_error) > 25:
            twist.angular.z = 0.5 * direction
        elif abs(angle_error) > 10:
            twist.angular.z = 0.3 * direction
        else:
            twist.angular.z = 0.15 * direction

        safe_publish_twist(node, robot_id, twist)
        print(f"[ROTATE] Current: {heading_y_now:.1f}deg, Target: {target_angle:.1f}deg, "
              f"Error: {angle_error:.1f}deg, Speed: {twist.angular.z:.2f}rad/s")

        time.sleep(0.1)

    safe_publish_twist(node, robot_id, Twist())
    time.sleep(0.2)

def move_forward_until_reached(node: Node, robot_name: str, target: Dict[str, float],
                               tolerance: float = 10.0, max_acceptable_angle_error: float = 25.0, 
                               semantic_threshold: float = 0.0):
    """Move forward to target with continuous heading correction"""
    x_target, y_target = target["x"], target["y"]
    print(f"[MOVE] Approaching target ({x_target:.1f}, {y_target:.1f})")

    while True:
        x_now, y_now, heading_y_now = get_position_with_polling(robot_name)
        dx = x_target - x_now
        dz = y_target - y_now
        distance = math.hypot(dx, dz)

        if distance < tolerance or (semantic_threshold > 0 and distance < semantic_threshold):
            print("[MOVE] Target reached")
            break

        target_angle = math.degrees(math.atan2(dx, dz)) % 360
        angle_error = (target_angle - heading_y_now + 180) % 360 - 180

        if abs(angle_error) > max_acceptable_angle_error:
            print(f"[MOVE] Angle correction required: {angle_error:.1f} degrees")
            rotate_to_face_target(node, robot_name, target)
            continue

        twist = Twist()
        twist.linear.x = 0.1
        safe_publish_twist(node, robot_name, twist)
        print(f"[MOVE] Distance: {distance:.2f}mm, Heading: {heading_y_now:.1f}deg, "
              f"Target angle: {target_angle:.1f}deg, Error: {angle_error:.1f}deg")

        time.sleep(0.2)
        safe_publish_twist(node, robot_name, Twist())
        time.sleep(0.1)

# ============================================================================
# PRECISION MOVEMENT FUNCTIONS
# ============================================================================

def calculate_target_angle(current_pos: Tuple[float, float, float], 
                          target_pos: Dict[str, float]) -> float:
    """Calculate required heading angle from current position to target"""
    x_now, y_now, _ = current_pos
    x_target, y_target = target_pos["x"], target_pos["y"]
    dx = x_target - x_now
    dy = y_target - y_now
    return math.degrees(math.atan2(dx, dy)) % 360

def precision_rotate(node: Node, robot_name: str, target_angle_deg: float, 
                    angular_speed_deg_per_s: float = 10.0, tolerance_deg: float = 1.0):
    
    """Execute precision rotation using time-based control"""
    _, _, current_heading = get_position_with_polling(robot_name)
    angle_diff = (target_angle_deg - current_heading + 180) % 360 - 180

    if abs(angle_diff) <= tolerance_deg:
        print(f"[PRECISION_ROTATE] Already aligned: current={current_heading:.1f}deg, "
              f"target={target_angle_deg:.1f}deg, error={angle_diff:.1f}deg")
        return


    duration_sec = abs(angle_diff) / angular_speed_deg_per_s

    angular_speed_rad_per_s = math.radians(angular_speed_deg_per_s)
    
    twist = Twist()
    twist.angular.z = angular_speed_rad_per_s if angle_diff > 0 else -angular_speed_rad_per_s

    direction = "left" if angle_diff > 0 else "right"
    print(f"[PRECISION_ROTATE] Rotating {abs(angle_diff):.1f}deg {direction}, "
          f"duration: {duration_sec:.2f}s")

    safe_publish_twist(node, robot_name, twist)
    time.sleep(duration_sec)
    safe_publish_twist(node, robot_name, Twist())

    # Verify result
    _, _, final_heading = get_position_with_polling(robot_name)
    final_diff = (target_angle_deg - final_heading + 180) % 360 - 180
    print(f"[PRECISION_ROTATE] Final heading: {final_heading:.1f}deg, "
          f"residual error: {final_diff:.1f}deg")

def precision_approach(node: Node, robot_name: str, target: Dict[str, float], 
                      speed_m_per_s: float = 0.03):
    """Execute precision approach using calculated distance and time"""
    x_now, y_now, _ = get_position_with_polling(robot_name)
    x_target, y_target = target["x"], target["y"]

    dx = x_target - x_now
    dy = y_target - y_now
    distance_mm = math.hypot(dx, dy)
    duration_sec = distance_mm / (speed_m_per_s * 1000)

    print(f"[PRECISION_APPROACH] Distance: {distance_mm:.1f}mm, "
          f"duration: {duration_sec:.2f}s, speed: {speed_m_per_s:.3f}m/s")

    twist = Twist()
    twist.linear.x = speed_m_per_s
    safe_publish_twist(node, robot_name, twist)
    time.sleep(duration_sec)
    safe_publish_twist(node, robot_name, Twist())

    print("[PRECISION_APPROACH] Approach completed")

def multi_stage_heading_alignment(node: Node, robot_name: str, target_heading_deg: float):
    """Multi-stage heading alignment with increasing precision"""
    # 🔥 修复：检查 target_heading_deg 是否为 None
    if target_heading_deg is None:
        print("[HEADING_ALIGN] Target heading is None, skipping alignment")
        return
    
    # 🔥 修复：确保是数值类型
    try:
        target_heading_deg = float(target_heading_deg)
    except (TypeError, ValueError):
        print(f"[HEADING_ALIGN] Invalid target heading: {target_heading_deg}, skipping alignment")
        return
    
    print(f"[HEADING_ALIGN] Target heading: {target_heading_deg:.1f} degrees")
    
    # Stage 1: Coarse alignment
    precision_rotate(node, robot_name, target_heading_deg, 
                    angular_speed_deg_per_s=8.0, tolerance_deg=2.0)
    
    time.sleep(0.3)  # Allow stabilization
    
    # Check if fine adjustment needed
    _, _, current_heading = get_position_with_polling(robot_name)
    angle_error = abs((target_heading_deg - current_heading + 180) % 360 - 180)
    
    if angle_error > 1.0:
        print(f"[HEADING_ALIGN] Fine adjustment required, error: {angle_error:.1f} degrees")
        precision_rotate(node, robot_name, target_heading_deg,
                        angular_speed_deg_per_s=4.0, tolerance_deg=0.5)
        
        # Final verification
        time.sleep(0.2)
        _, _, final_heading = get_position_with_polling(robot_name)
        final_error = abs((target_heading_deg - final_heading + 180) % 360 - 180)
        print(f"[HEADING_ALIGN] Final heading: {final_heading:.1f}deg, error: {final_error:.1f}deg")
    else:
        print(f"[HEADING_ALIGN] Heading achieved within tolerance, error: {angle_error:.1f}deg")

# ============================================================================
# MAIN NAVIGATION FUNCTION
# ============================================================================

def navigate_to_position(node: Node, robot_name: str, target: Dict[str, float]):
    """Execute complete navigation to target position with multi-stage precision"""
    x_target, y_target = target["x"], target["y"]
    x_now, y_now, _ = get_position_with_polling(robot_name)
    initial_distance = math.hypot(x_target - x_now, y_target - y_now)

    print(f"[NAVIGATE] Robot: {robot_name}, Target: ({x_target:.1f}, {y_target:.1f}), "
          f"Distance: {initial_distance:.1f}mm")

    # Stage 1: Coarse navigation
    print("[NAVIGATE] Stage 1: Coarse navigation")
    rotate_to_face_target(node, robot_name, target)
    move_forward_until_reached(node, robot_name, target, semantic_threshold=300.0)

    # Stage 2: Precision adjustment
    print("[NAVIGATE] Stage 2: Precision adjustment")
    current_pos = get_position_with_polling(robot_name)
    target_angle = calculate_target_angle(current_pos, target)
    precision_rotate(node, robot_name, target_angle, angular_speed_deg_per_s=8.0)
    precision_approach(node, robot_name, target, speed_m_per_s=0.03)

    # Stage 3: Final heading alignment
        # 🔥 修复：检查 heading_deg 键是否存在且不为 None
    target_heading = target.get("heading_deg")
    if target_heading is not None:
        print("[NAVIGATE] Stage 3: Final heading alignment")
        multi_stage_heading_alignment(node, robot_name, target_heading)
    else:
        print("[NAVIGATE] Stage 3: Skipped (no target heading specified)")

    print(f"[NAVIGATE] Navigation completed for {robot_name}")

# ============================================================================
# NAVIGATION COORDINATION (DISTRIBUTED CONFLICT RESOLUTION)
# ============================================================================

def setup_navigation_coordinator(node: Node):
    """Initialize navigation coordination communication"""
    global navigation_coordinator_pub
    navigation_coordinator_pub = node.create_publisher(String, '/navigation_coordination', 10)
    node.create_subscription(String, '/navigation_coordination', 
                           navigation_coordination_callback, 10)
    print("[COORDINATION] Navigation coordinator initialized")

def navigation_coordination_callback(msg):
    """Handle navigation coordination messages"""
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
        print(f"[ERROR] Invalid coordination message: {msg.data}")

def handle_navigation_request(data):
    """Process navigation request from other robots"""
    requesting_robot = data["robot_name"]
    target = data["target"]
    request_id = data["request_id"]

    with intentions_lock:
        all_intentions = navigation_intentions.copy()

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

    current_robot = get_current_robot_name()
    response = {
        "type": "navigation_response",
        "request_id": request_id,
        "responding_robot": current_robot,
        "conflicts": conflicting_targets
    }

    if navigation_coordinator_pub:
        navigation_coordinator_pub.publish(String(data=json.dumps(response)))
        print(f"[COORDINATION] Response sent to {requesting_robot}: {len(conflicting_targets)} conflicts")

def handle_navigation_response(data):
    """Process navigation response from other robots"""
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

    print(f"[COORDINATION] Response received from {responding_robot}: {len(conflicts)} conflicts")

def handle_navigation_confirmed(data):
    """Process navigation confirmation from other robots"""
    robot_name = data["robot_name"]
    target = data["target"]

    with intentions_lock:
        navigation_intentions[robot_name] = {
            "target": target,
            "timestamp": time.time(),
            "status": "confirmed"
        }

    print(f"[COORDINATION] {robot_name} confirmed navigation to ({target['x']:.1f}, {target['y']:.1f})")

def targets_are_same(target1: Dict, target2: Dict, tolerance: float = 10.0) -> bool:
    """Check if two targets represent the same location"""
    dx = abs(target1.get("x", 0) - target2.get("x", 0))
    dy = abs(target1.get("y", 0) - target2.get("y", 0))
    return dx <= tolerance and dy <= tolerance

def broadcast_navigation_request(robot_name: str, target: Dict[str, float], 
                                timeout: float = 3.0) -> List[Dict]:
    """Broadcast navigation intent and collect conflict responses"""
    request_id = f"{robot_name}_{int(time.time() * 1000)}"

    with responses_lock:
        navigation_responses[request_id] = []

    request = {
        "type": "navigation_request",
        "robot_name": robot_name,
        "target": target,
        "request_id": request_id,
        "timestamp": time.time()
    }

    if navigation_coordinator_pub:
        navigation_coordinator_pub.publish(String(data=json.dumps(request)))
        print(f"[COORDINATION] Broadcasting navigation intent to ({target['x']:.1f}, {target['y']:.1f})")

    # Wait for responses
    start_time = time.time()
    while time.time() - start_time < timeout:
        time.sleep(0.1)

    # Collect all conflicts
    all_conflicts = []
    with responses_lock:
        for response in navigation_responses.get(request_id, []):
            all_conflicts.extend(response["conflicts"])

    # Check local intentions for consistency
    with intentions_lock:
        current_intentions = navigation_intentions.copy()

    for other_robot, intention in current_intentions.items():
        if (other_robot != robot_name and 
            intention["status"] in ["announcing", "confirmed"] and 
            targets_are_same(intention["target"], target)):
            
            if not any(c["robot_name"] == other_robot for c in all_conflicts):
                all_conflicts.append({
                    "robot_name": other_robot,
                    "target": intention["target"],
                    "timestamp": intention["timestamp"]
                })

    print(f"[COORDINATION] Collected {len(all_conflicts)} conflict responses")
    return all_conflicts

def resolve_navigation_conflicts(robot_name: str, target: Dict[str, float], 
                                conflicts: List[Dict]) -> Dict[str, float]:
    """Resolve navigation conflicts using distributed positioning"""
    if not conflicts:
        print(f"[CONFLICT_RESOLUTION] No conflicts detected for {robot_name}")
        return target

    all_robots = [robot_name]
    for conflict in conflicts:
        if conflict["robot_name"] not in all_robots:
            all_robots.append(conflict["robot_name"])

    print(f"[CONFLICT_RESOLUTION] {len(all_robots)} robots targeting same location: {all_robots}")

    # Deterministic ordering for consistent resolution
    all_robots.sort()
    robot_index = all_robots.index(robot_name)

    center_x, center_y = target["x"], target["y"]
    
    if len(all_robots) == 1:
        return target
    
    # Circular distribution algorithm
    radius = 200.0
    angle_step = 360.0 / len(all_robots)
    angle_deg = robot_index * angle_step
    angle_rad = math.radians(angle_deg)
    
    new_x = center_x + radius * math.cos(angle_rad)
    new_y = center_y + radius * math.sin(angle_rad)
    heading_to_center = math.degrees(math.atan2(center_x - new_x, center_y - new_y)) % 360
    
    new_target = {
        "x": new_x,
        "y": new_y,
        "heading_deg": heading_to_center
    }
    
    print(f"[CONFLICT_RESOLUTION] {robot_name} assigned position: "
          f"({new_x:.1f}, {new_y:.1f}), heading: {heading_to_center:.1f}deg")
    
    return new_target

def confirm_navigation_intent(robot_name: str, target: Dict[str, float]):
    """Confirm navigation intent to coordination system"""
    with intentions_lock:
        navigation_intentions[robot_name] = {
            "target": target,
            "timestamp": time.time(),
            "status": "confirmed"
        }

    confirmation = {
        "type": "navigation_confirmed",
        "robot_name": robot_name,
        "target": target,
        "timestamp": time.time()
    }

    if navigation_coordinator_pub:
        navigation_coordinator_pub.publish(String(data=json.dumps(confirmation)))
        print(f"[COORDINATION] {robot_name} navigation intent confirmed")

def cleanup_navigation_intent(robot_name: str):
    """Clean up navigation intent after completion"""
    with intentions_lock:
        if robot_name in navigation_intentions:
            del navigation_intentions[robot_name]
            print(f"[COORDINATION] {robot_name} navigation intent cleaned up")

# ============================================================================
# MAIN NAVIGATION INTERFACE
# ============================================================================

def navigate_to_target(node: Node, executor: MultiThreadedExecutor, robot_name: str, target):
    """Main navigation function with coordination and conflict resolution"""
    is_successful = False

    set_current_robot_name(robot_name)
    setup_navigation_coordinator(node)

    # Initialize position tracking
    tracker_robot = getRobotPositionCache(robot_name, executor)
    if tracker_robot is None:
        print(f"[ERROR] Position tracking failed for {robot_name}")
        tts_manager.say_sync(f"Can't get position data for {robot_name}. Please check the tracking system.")
        return is_successful

    # Resolve target
    if isinstance(target, str):
        tracker_target = getRobotPositionCache(target, executor)
        if tracker_target:
            x, y, heading = get_position_with_polling(target)
            resolved_target = {"x": x, "y": y, "heading": heading}
            print(f"[TARGET_RESOLUTION] Dynamic target '{target}' resolved to ({x:.1f}, {y:.1f})")
        else:
            if target in semantic_locations:
                resolved_target = semantic_locations[target]
                print(f"[TARGET_RESOLUTION] Semantic target '{target}' resolved")
            else:
                print(f"[ERROR] Target '{target}' not found")
                tts_manager.say_sync(f"Can't resolve target {target}")
                return is_successful
    else:
        resolved_target = target

    if not (isinstance(resolved_target, dict) and "x" in resolved_target and "y" in resolved_target):
        print(f"[ERROR] Invalid target format: {resolved_target}")
        return is_successful

    # Execute coordinated navigation
    print(f"[COORDINATION] Starting navigation coordination for {robot_name}")
    
    with intentions_lock:
        navigation_intentions[robot_name] = {
            "target": resolved_target,
            "timestamp": time.time(),
            "status": "announcing"
        }

    conflicts = broadcast_navigation_request(robot_name, resolved_target)
    actual_target = resolve_navigation_conflicts(robot_name, resolved_target, conflicts)
    confirm_navigation_intent(robot_name, actual_target)

    print(f"[NAVIGATION] Executing navigation to ({actual_target['x']:.1f}, {actual_target['y']:.1f})")
    navigate_to_position(node, robot_name, actual_target)
    cleanup_navigation_intent(robot_name)

    is_successful = True
    return is_successful

# ============================================================================
# Special navigation function for after follow
# ============================================================================
def navigate_to_object(node: Node, robot_name: str, item: str, executor):
    """
    after find the item
    """
    target_position = None
    used_key = None
    
    # 获取机器人当前位置
    robot_x, robot_y, robot_heading = get_position_with_polling(robot_name)
    robot_pos = {"x": robot_x, "y": robot_y, "heading_y": robot_heading}
    print(f"📍 Robot position: ({robot_x:.2f}, {robot_y:.2f}, {robot_heading:.1f}°)")
    
    # 尝试各个候选key
    print(f"🔑 Trying blackboard key: '{item}'")
    map_position = resolve_object_position(robot_name, item, robot_pos)
    if map_position:
        print(f"✅ Found in blackboard with key '{item}':")
        print(f"   📍 Map coordinates: ({map_position['x']:.2f}, {map_position['y']:.2f})")
        print(f"   📏 Estimated distance: {map_position.get('estimated_distance', 0):.2f}m")
        print(f"   🎯 Confidence: {map_position.get('confidence', 0):.2f}")
        target_position = map_position
        used_key = item
    
    # 如果黑板中没找到，尝试语义位置
    if not target_position:
        print(f"🔍 Not found in blackboard, checking semantic locations...")
        semantic_candidates = [item, f"{item}_location", f"{item}_pos"]
        
        for semantic_key in semantic_candidates:
            if semantic_key in semantic_locations:
                semantic_pos = semantic_locations[semantic_key]
                print(f"✅ Found in semantic locations with key '{semantic_key}':")
                print(f"   📍 Coordinates: ({semantic_pos['x']:.2f}, {semantic_pos['y']:.2f})")
                if 'heading_deg' in semantic_pos:
                    print(f"   🧭 Heading: {semantic_pos['heading_deg']}°")
                target_position = semantic_pos
                used_key = semantic_key
                break
    
    # 如果都没找到，报错
    if not target_position:
        print(f"❌ Could not find location for item '{item}'!")
        print(f"💡 Tried semantic keys: {[item, f'{item}_location', f'{item}_pos']}")
        print(f"💡 Available semantic locations: {list(semantic_locations.keys())}")
        tts_manager.say_sync(f"Sorry, I cannot find the location of {item}")
        return False
    
    print(f"🎯 Using location from: {used_key}")

    # === 阶段 1：导航到目标位置 ===
    print(f"\n🧭 Phase 1: Navigation to target")
    
    if isinstance(target_position, dict) and "estimated_distance" in target_position:
        # 🎯 使用coord_convert的结果，但仍需要调整朝向
        print(f"📏 Using coord_convert results")
        print(f"🎯 Target coordinates: ({target_position['x']:.2f}, {target_position['y']:.2f})")
        
        # 计算朝向（coord_convert给的是绝对坐标，需要相对当前位置计算角度）
        target_x, target_y = target_position["x"], target_position["y"]
        dx = target_x - robot_x
        dy = target_y - robot_y
        current_distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.degrees(math.atan2(dx, dy)) % 360
        angle_diff = (target_angle - robot_heading + 180) % 360 - 180
        
        print(f"📏 Current distance to target: {current_distance:.2f}m")
        print(f"🔄 Need to rotate: {angle_diff:.1f}°")
        
        # Step 1: 转向目标
        if abs(angle_diff) > 3:  # 3度容差
            print(f"🔄 Rotating {angle_diff:.1f}°...")
            rotate_success = rotate_deg(node, robot_name, angle_diff)
            if not rotate_success:
                print(f"❌ Rotation failed!")
                return False
        else:
            print(f"✅ Already facing target direction")
        
        # Step 2: 前进到目标
        print(f"🚶 Moving forward {current_distance:.2f}m...")
        is_successful = move(node, robot_name, "forward", current_distance, "meter")
    else:
        # 对于语义位置，使用完整导航
        is_successful = navigate_to_target(node, executor, robot_name, target_position)
    
    if not is_successful:
        print(f"❌ Navigation failed!")
        tts_manager.say_sync(f"Sorry, I could not reach the {item}")
        return False
        
    print(f"✅ Navigation completed! Reached target location")
    print(f"🎯 Ready to collect {item}...")

    return True


# ============================================================================
# UTILITY AND DEBUG FUNCTIONS
# ============================================================================

def get_navigation_status() -> Dict:
    """Get current navigation coordination status"""
    with intentions_lock:
        return navigation_intentions.copy()

def print_navigation_status():
    """Print current navigation status"""
    status = get_navigation_status()
    print("\n" + "="*50)
    print("NAVIGATION COORDINATION STATUS:")
    for robot, intention in status.items():
        target = intention["target"]
        status_str = intention["status"]
        print(f"  {robot}: -> ({target['x']:.1f}, {target['y']:.1f}) [{status_str}]")
    print("="*50 + "\n")

# ============================================================================
# TEST MAIN FUNCTION
# ============================================================================

def main():
    import rclpy
    import threading

    rclpy.init()
    node = rclpy.create_node("navigation_tester")
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    parser = argparse.ArgumentParser()
    parser.add_argument("--robot_id", required=True, choices=["robot1", "robot2"])
    args = parser.parse_args()
    
    test_target = {"x": 100.0, "y": 100.0, "heading_deg": 45.0}

    try:
        print(f"[TEST] Starting navigation test for {args.robot_id}")
        success = navigate_to_target(node, executor, args.robot_id, test_target)
        print(f"[TEST] Navigation result: {'SUCCESS' if success else 'FAILED'}")
    except KeyboardInterrupt:
        print("[TEST] Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()