#!/usr/bin/env python3

# face.py —— 复用 follow 逻辑，实时坐标优先
 
import os, sys, math, threading, time

import rclpy

from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist

from typing import Dict
 
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from config import semantic_locations

from phasespace.rigid_tracker import RigidTracker
 
# ───── 全局缓存 ─────

robot_position_cache = {} 

pub_cache = {}
 
def ensure_pub(node: Node, name: str):
    return pub_cache.setdefault(name,
        node.create_publisher(Twist, f'/{name}/cmd_vel', 10))
 
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

def wait_for_pose(name: str,
                  cache: dict,
                  timeout: float = 2.0,
                  poll: float = 0.05):
    """
    等待 `timeout` 秒，直到 pose_cache 里出现 `name`。
    返回 (x, z, heading)；超时则返回 None
    """
    t0 = time.time()
    while time.time() - t0 < timeout:
        if name in cache:
            d = cache[name]
            return d["x"], d["z"], d["heading_y"]
        time.sleep(poll)
    return None

def ensure_tracker_with_wait(name: str,
                             pose_cache: dict,
                             executor: MultiThreadedExecutor,
                             timeout: float = 2.0):
    tracker = RigidTracker(pose_cache, name)
    executor.add_node(tracker)
 
    t0 = time.time()
    while time.time() - t0 < timeout:
        executor.spin_once(timeout_sec=0.1)      # ★ 关键：驱动回调
        if name in pose_cache:                   # 已收到第一帧
            d = pose_cache[name]
            return (d["x"], d["z"], d["heading_y"]), tracker
    # 超时
    return None, tracker
 
# ───── 旋转函数（与你原来一致，删 sleep-0-帧即可） ─────

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
 

# ───── 主流程 ─────
def face_run(robot: str, target: str):
    exec_ = MultiThreadedExecutor()
 
    # # ⓐ 自己的 Tracker ——无需等待
    # _, tracker_self = ensure_tracker_with_wait(robot, cache, exec_, 0)
 
    # # ⓑ 目标的 Tracker ——等待 2 秒
    # pose, tracker_target = ensure_tracker_with_wait(target, cache, exec_, 5.0)

    tracker_self = RigidTracker(robot_position_cache, robot)
    tracker_tgt = RigidTracker(robot_position_cache,target)
    exec_.add_node(tracker_self)
    exec_.add_node(tracker_tgt)
    

    # 后台 spin
    spin_thread = threading.Thread(target=exec_.spin, daemon=True)
    spin_thread.start()

    pose = wait_for_pose(target, robot_position_cache, 2.0)
 
    # ⓒ 确定目标坐标
    if pose is None:
        static = semantic_locations.get(target)
        if static and {"x", "y"} <= static.keys():
            tx, tz = static["x"], static["y"]
            print(f"⚠️ Using static pose for {target}")
        else:
            raise RuntimeError(
                f"No real-time data and no static position for '{target}'")
    else:
        tx, tz, _ = pose
        print(f"✅ Using real-time pose for {target}")
 
    # ⓓ 执行朝向
    node = rclpy.create_node("face_controller")
    pub  = ensure_pub(node, robot)
    rotate_to_face_target(robot, pub, {"x": tx, "y": tz}, robot_position_cache)
 
    # 清理
    node.destroy_node()
    exec_.shutdown()
    tracker_self.destroy_node(); 
    tracker_tgt.destroy_node()
    rclpy.shutdown()
 


if __name__ == "__main__":

    rclpy.init()

    face_run(robot="robot1", target="lucy")   # ← 改成你的机器人 / 目标

 