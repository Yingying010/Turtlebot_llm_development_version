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

# 目标点占用缓存 - 新增
target_occupation: Dict[str, Dict] = {}  # {target_key: {robot: arrival_time, ...}}
target_lock = threading.Lock()

# 配置参数
DISTRIBUTION_CONFIG = {
   "conflict_detection_radius": 300.0,  # 多远算同一目标
   "base_distribution_radius": 200.0,   # 基础分布半径
   "planning_wait_time": 0.5,          # 等待其他机器人广播的时间
   "enable_dynamic_radius": True,       # 是否根据机器人数量调整半径
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

# === 新增：目标点管理功能 ===

def get_target_key(target: Dict) -> str:
   """生成目标点的唯一标识"""
   x, y = target["x"], target["y"]
   # 将坐标量化到网格，避免浮点误差
   grid_x = round(x / 50) * 50  # 50mm网格
   grid_y = round(y / 50) * 50
   return f"pos_{grid_x}_{grid_y}"

def detect_target_conflict(robot_name: str, target: Dict, 
                         conflict_radius: float = None) -> List[str]:
   """检测是否有其他机器人也要去相同目标区域"""
   if conflict_radius is None:
       conflict_radius = DISTRIBUTION_CONFIG["conflict_detection_radius"]
   
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
       
       if distance_between_targets <= conflict_radius:
           conflicting_robots.append(other_name)
           print(f"🎯 检测到 {other_name} 也要去附近区域")
   
   return conflicting_robots

def calculate_dynamic_radius(conflicting_robots: List[str], 
                          base_radius: float = None) -> float:
   """根据机器人数量动态调整分布半径"""
   if base_radius is None:
       base_radius = DISTRIBUTION_CONFIG["base_distribution_radius"]
   
   total_robots = len(conflicting_robots) + 1  # 包括自己
   
   if not DISTRIBUTION_CONFIG["enable_dynamic_radius"]:
       return base_radius
       
   if total_robots <= 2:
       return base_radius
   elif total_robots <= 4:
       return base_radius * 1.2
   elif total_robots <= 6:
       return base_radius * 1.5
   else:
       return base_radius * 2.0  # 太多机器人时增大半径

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
       # 只有自己，直接去目标点
       return original_target
   
   # 动态计算分布半径
   distribution_radius = calculate_dynamic_radius(conflicting_robots)
   
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
       "original_target": original_target,
       "distribution_radius": distribution_radius,
       "my_position_index": my_index,
       "total_robots": total_robots
   }
   
   print(f"📍 {robot_name} 分布位置: ({distributed_x:.1f}, {distributed_y:.1f}) "
         f"(第{my_index+1}/{total_robots}个，半径{distribution_radius:.1f}mm)")
   
   return distributed_target

def print_distribution_layout(robot_name: str, final_target: Dict):
   """打印分布布局信息"""
   if not final_target.get("is_distributed", False):
       print(f"📌 {robot_name}: 直接目标点")
       return
   
   original = final_target["original_target"]
   radius = final_target["distribution_radius"]
   count = final_target["total_robots"]
   
   print(f"""
   📊 {robot_name} 分布布局:
   - 原始目标: ({original['x']:.1f}, {original['y']:.1f})
   - 分布半径: {radius:.1f}mm
   - 机器人总数: {count}
   - 当前位置: ({final_target['x']:.1f}, {final_target['y']:.1f})
   """)

# === 碰撞检测和避让 ===

def broadcast_navigation_status(robot_name: str, current_pos: Tuple, target: Dict, status: str):
   """广播自己的导航状态"""
   x_now, y_now, heading_now = current_pos
   
   nav_info = {
       "current_pos": {"x": x_now, "y": y_now, "heading": heading_now},
       "target_pos": {"x": target["x"], "y": target["y"]},
       "status": status,  # "planning", "moving", "stopped", "yielding", "arrived", "distributed_arrived"
       "timestamp": time.time(),
       "robot_radius": 150.0,  # 安全半径（毫米）
       "speed": 0.1,
       "is_distributed": target.get("is_distributed", False)
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
           distance = math.hypot(
               my_pos["x"] - other_pos["x"], 
               my_pos["z"] - other_pos["y"]
           )
           
           if distance <= detection_range:
               nearby[other_name] = other_info
   
   return nearby

def predict_collision_simple(robot_name: str, current_pos: Tuple, target: Dict, 
                          look_ahead_seconds: float = 2.0) -> Optional[str]:
   """简单的碰撞预测：检查是否会与其他机器人相撞"""
   x_now, y_now, heading_now = current_pos
   x_target, y_target = target["x"], target["y"]
   
   # 我的移动方向和速度
   dx = x_target - x_now
   dy = y_target - y_now
   my_distance_to_target = math.hypot(dx, dy)
   
   if my_distance_to_target < 50:  # 已经很接近目标
       return None
   
   my_speed = 100  # mm/s，基于你的0.1 m/s
   my_direction = math.atan2(dx, dy)
   
   nearby_robots = get_nearby_robots(robot_name)
   
   for other_name, other_info in nearby_robots.items():
       other_pos = other_info["current_pos"]
       other_target = other_info["target_pos"]
       
       # 预测碰撞点
       for t in [0.5, 1.0, 1.5, 2.0]:  # 检查未来几个时间点
           # 我的预测位置
           my_future_x = x_now + my_speed * t * math.cos(my_direction) / 1000
           my_future_y = y_now + my_speed * t * math.sin(my_direction) / 1000
           
           # 对方的预测位置（假设也是直线运动）
           other_dx = other_target["x"] - other_pos["x"]
           other_dy = other_target["y"] - other_pos["y"]
           other_distance = math.hypot(other_dx, other_dy)
           
           if other_distance > 50:  # 对方还在移动
               other_speed = other_info.get("speed", 0.1) * 1000  # 转换为mm/s
               other_direction = math.atan2(other_dy, other_dx)
               
               other_future_x = other_pos["x"] + other_speed * t * math.cos(other_direction) / 1000
               other_future_y = other_pos["y"] + other_speed * t * math.sin(other_direction) / 1000
               
               # 检查距离
               predicted_distance = math.hypot(
                   my_future_x - other_future_x,
                   my_future_y - other_future_y
               )
               
               safe_distance = robot_navigation_cache[robot_name]["robot_radius"] + \
                             other_info["robot_radius"]
               
               if predicted_distance < safe_distance:
                   print(f"⚠️ 预测{t:.1f}秒后与{other_name}碰撞！距离: {predicted_distance:.1f}mm")
                   return other_name
   
   return None

def simple_collision_avoidance(robot_name: str, conflicting_robot: str) -> str:
   """简单的避让策略"""
   # 策略1：按机器人名字排序决定优先级（简单但确定性）
   if robot_name < conflicting_robot:
       print(f"🚀 {robot_name} 有优先权，继续前进")
       return "proceed"
   else:
       print(f"🤝 {robot_name} 让路给 {conflicting_robot}")
       return "yield"

def move_forward_with_collision_avoidance(node: Node, robot_name: str, target: Dict[str, float],
                                       tolerance: float = 20.0, max_acceptable_angle_error: float = 25.0):
   """带防碰撞的前进函数"""
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
           status = "distributed_arrived" if target.get("is_distributed", False) else "arrived"
           broadcast_navigation_status(robot_name, current_pos, target, status)
           break
       
       # 碰撞预测
       conflicting_robot = predict_collision_simple(robot_name, current_pos, target)
       
       if conflicting_robot:
           avoidance_action = simple_collision_avoidance(robot_name, conflicting_robot)
           
           if avoidance_action == "yield":
               print(f"🛑 避让 {conflicting_robot}，暂停移动")
               broadcast_navigation_status(robot_name, current_pos, target, "yielding")
               
               # 停止并等待
               safe_publish_twist(node, robot_name, Twist())
               time.sleep(0.5)
               continue
           else:
               print(f"🐌 检测到冲突但继续前进（有优先权）")
       
       # 正常前进逻辑
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

# === 旋转和移动函数 ===

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

# === 主要的导航函数 ===

def navigate_with_distribution(node: Node, robot_name: str, target: Dict[str, float]):
   """带分布策略的导航"""
   
   print(f"\n🎯 {robot_name} 导航到 ({target['x']:.1f}, {target['y']:.1f})")
   
   # 1. 广播自己的目标意图
   current_pos = get_current_position(robot_name)
   broadcast_navigation_status(robot_name, current_pos, target, "planning")
   
   # 等待一小段时间，让其他机器人也广播他们的意图
   time.sleep(DISTRIBUTION_CONFIG["planning_wait_time"])
   
   # 2. 检测冲突
   conflicting_robots = detect_target_conflict(robot_name, target)
   
   # 3. 决定最终目标位置
   if conflicting_robots:
       print(f"⚠️ 检测到目标冲突，与 {conflicting_robots} 机器人")
       tts_manager.say(f"Multiple robots detected heading to same area. Adjusting position.")
       
       # 生成分布位置
       final_target = generate_distributed_position(robot_name, target, conflicting_robots)
       print_distribution_layout(robot_name, final_target)
   else:
       print(f"✅ 无冲突，直接前往目标点")
       final_target = target
   
   # 4. 执行导航
   navigate_to_position_safe(node, robot_name, final_target)
   
   # 5. 到达后的状态更新和语音反馈
   current_pos = get_current_position(robot_name)
   if final_target.get("is_distributed", False):
       tts_manager.say("Arrived at distributed position around target area.")
       print(f"📍 {robot_name} 已到达目标区域的分布位置")
   else:
       tts_manager.say("Arrived at target position.")
       print(f"🎯 {robot_name} 已到达精确目标点")

def navigate_to_position_safe(node: Node, robot_name: str, target: Dict[str, float]):
   """安全导航函数"""
   x_target, y_target = target["x"], target["y"]
   x_now, y_now, _ = get_current_position(robot_name)
   distance = math.hypot(x_target - x_now, y_target - y_now)
   
   print(f"\n🧭 安全导航 {robot_name} → ({x_target:.1f}, {y_target:.1f}) | 距离={distance:.2f}")
   
   # 初始化导航状态
   current_pos = get_current_position(robot_name)
   broadcast_navigation_status(robot_name, current_pos, target, "starting")
   
   # Phase 1: 旋转对准
   rotate_to_face_target(node, robot_name, target)
   
   # Phase 2: 安全前进
   move_forward_with_collision_avoidance(node, robot_name, target)
   
   # Phase 3: 最终朝向
   if "heading_deg" in target:
       rotate_to_final_heading(node, robot_name, target["heading_deg"])
   
   print(f"✅ {robot_name} 安全导航完成")

def clean_navigation_cache(robot_name: str):
   """清理完成任务的机器人导航信息"""
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

   # 3) 执行带分布的智能导航
   if isinstance(resolved_target, dict) and "x" in resolved_target and "y" in resolved_target:
       if "heading_deg" in resolved_target:
           print(f"📐 Target includes heading: {resolved_target['heading_deg']}°")
       
       try:
           # 使用新的分布导航
           navigate_with_distribution(node, robot_name, resolved_target)
           is_successful = True
       except Exception as e:
           print(f"❌ Navigation failed: {e}")
           tts_manager.say("Navigation failed. Please check system status.")
       finally:
           # 清理导航缓存
           clean_navigation_cache(robot_name)
   else:
       print(f"⚠️ Invalid resolved target: {resolved_target}")
       return is_successful

   return is_successful