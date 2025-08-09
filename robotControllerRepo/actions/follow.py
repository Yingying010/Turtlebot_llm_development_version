#!/usr/bin/env python3

# follow.py —— 不改 rigid_tracker.py，单进程跟随
 
import os, sys, math, time, threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from typing import Dict, Tuple, Optional
from rclpy.publisher import Publisher
from rclpy._rclpy_pybind11 import InvalidHandle
from config import semantic_locations
 
# 工程根路径

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from ttsRepo.stream_tts import tts_manager
from phasespace.rigid_tracker import RigidTracker    # 原版，保持不动
 
# ------------------------------------------------------------------

robot_position_cache: Dict[str, Dict[str, float]] = {}
cache_lock = threading.Lock()
publisher_dict: Dict[Tuple[int, str], Publisher] = {}
 
# ---------- 工具 ----------

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


# --------- rotate and move

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
 
# ---------- 跟随线程 ----------
follow_dist = 200.0
hysteresis = 50.0

def follow_loop(ctrl_node: Node, follower: str, target: str):
    pub = _ensure_pub(ctrl_node, follower)
    tts_manager.say(f"{follower} is now following {target}")

    while rclpy.ok():
        fx, fz, _ = get_current_position(follower, robot_position_cache)
        tx, tz, _ = get_current_position(target, robot_position_cache)
        dx, dz = tx - fx, tz - fz
        dist = math.hypot(dx, dz)

        tgt_pos = {"x":tx, "y":tz}

        if dist > follow_dist+ hysteresis:
            rotate_to_face_target(follower,pub,tgt_pos,robot_position_cache)
            move_forward_until_reached(follower, pub, tgt_pos, robot_position_cache, tolerance=follow_dist+hysteresis)
        else:
            pub.publish(Twist())
            rotate_to_face_target(follower,pub,tgt_pos,robot_position_cache)
            time.sleep(0.2)

        print(f"👣 {follower}→{target}  dist={dist:.0f} mm")
        time.sleep(0.2)

# ------------------------------------------------------------------
def listner_whisper():
    Whisper_run()
 
# ------------------------------------------------------------------

def follow_run(follower:str, target:str, executor: MultiThreadedExecutor):
    # ➊ 生成两个 RigidTracker 节点（保持原版本）
    tracker_follower_node = getRobotPositionCache(follower, executor)
    tracker_target_node = getRobotPositionCache(target, executor)
 

    if tracker_follower_node is None:
        print("❌ Abort navigation of follower")
        return
    elif tracker_target_node is None:
        print("❌ Abort navigation of target")
        return
    
    follow_thread = threading.thread(follow_loop)
    listner_thread = threading.thread(listner)
    
 
# ------------------------------------------------------------------

if __name__ == "__main__":
    rclpy.init()
    follower, target = "robot2", "robot3"
    follow_run()

 