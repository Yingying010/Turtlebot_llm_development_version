#!/usr/bin/env python3

# follow.py —— Timer(20 Hz) 实时-navigate 跟随，不改 rigid_tracker.py
 
import os, sys, math, threading

from typing import Dict
 
import rclpy

from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
 
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from ttsRepo.stream_tts import tts_manager

from phasespace.rigid_tracker import RigidTracker     # 保持原版
 
# ───────── 参数 ─────────

FOLLOW_DIST_MM = 200.0      # 最近跟随距离

HYST_MM        = 50.0       # 迟滞

MAX_LIN        = 0.25       # m/s
 
ANG_FAST = 1.0              # rad/s

ANG_MED  = 0.6

ANG_SLOW = 0.3

ANG_FACE_TH = 5.0           # 近距离允许朝向误差

ANG_MOVE_TH = 25.0          # 直行前最大角误差
 
CTRL_DT = 0.05              # 控制周期 0.05 s → 20 Hz
 
# ───────── 全局缓存 ─────────

pose_cache = {}          # robot_name -> {x,z,heading_y,...}

pub_cache = {}
 
# ───────── 工具函数 ─────────

def ensure_pub(node: Node, name: str):
    return pub_cache.setdefault(name,
        node.create_publisher(Twist, f'/{name}/cmd_vel', 10))
 
def get_pose(name: str):
    if name in pose_cache:
        d = pose_cache[name]
        return d["x"], d["z"], d["heading_y"]
    return None
 
def ang_err(tgt_deg, cur_deg):
    return (tgt_deg - cur_deg + 180) % 360 - 180
 
# ───────── 低层动作 ─────────
def face_target(robot: str, pub, tx, tz):
    pose = get_pose(robot)
    if not pose: return
    x, z, yaw = pose
    dx, dz    = tx - x, tz - z
    tgt_yaw   = math.degrees(math.atan2(dx, dz)) % 360
    err       = ang_err(tgt_yaw, yaw)
    twist = Twist()

    if abs(err) < ANG_FACE_TH:
        pub.publish(Twist()); return         # 已对准

    if abs(err) > 25:
        twist.angular.z = ANG_FAST if err > 0 else -ANG_FAST
    elif abs(err) > 10:
        twist.angular.z = ANG_MED  if err > 0 else -ANG_MED
    else:
        twist.angular.z = ANG_SLOW if err > 0 else -ANG_SLOW
    pub.publish(twist)
 
def move_towards(robot: str, pub, tx, tz):
    pose = get_pose(robot)
    if not pose: return
    x, z, yaw = pose
    dx, dz    = tx - x, tz - z
    dist      = math.hypot(dx, dz)
    tgt_yaw = math.degrees(math.atan2(dx, dz)) % 360
    err     = ang_err(tgt_yaw, yaw)
    twist = Twist()

    # 若角度误差太大，先转头
    if abs(err) > ANG_MOVE_TH:
        face_target(robot, pub, tx, tz); return
 
    # 线速度按距离比例（0–MAX_LIN）
    v = min(MAX_LIN, 0.002 * (dist - FOLLOW_DIST_MM))
    twist.linear.x = max(0.05, v)            # 最小 0.05 m/s
    pub.publish(twist)
 
# ───────── 跟随逻辑 (Timer 回调) ─────────

def make_follow_step(node: Node, follower: str, target: str):
    pub = ensure_pub(node, follower)
    pose_f = get_pose(follower)
    pose_t = get_pose(target)

    if not pose_f or not pose_t:
        return                                # 坐标未就绪
    fx, fz, _ = pose_f
    tx, tz, _ = pose_t
    dist      = math.hypot(tx - fx, tz - fz)
    if dist > FOLLOW_DIST_MM + HYST_MM:
        move_towards(follower, pub, tx, tz)
    else:
        pub.publish(Twist())                  # 停止直行
        face_target(follower, pub, tx, tz)
 
# ───────── 运行包装 ─────────

def follow_run(follower: str, target: str):
    # ① 两个 RigidTracker 节点
    tracker_f = RigidTracker(pose_cache, follower)
    tracker_t = RigidTracker(pose_cache, target)

    # ② Executor 后台 spin
    executor = MultiThreadedExecutor()
    executor.add_node(tracker_f); executor.add_node(tracker_t)
    threading.Thread(target=executor.spin, daemon=True).start()
 
    # ③ 控制节点 + Timer
    ctrl_node = rclpy.create_node("follow_controller")
    ensure_pub(ctrl_node, follower)           # 提前创建 publisher
    tts_manager.say(f"{follower} is now following {target}")
    ctrl_node.create_timer(
        CTRL_DT, lambda: make_follow_step(ctrl_node, follower, target))
    try:
        rclpy.spin(ctrl_node)
    except KeyboardInterrupt:
        pass
 
    # 清理
    executor.shutdown()
    tracker_f.destroy_node(); tracker_t.destroy_node()
    ctrl_node.destroy_node()
    rclpy.shutdown()
 
# ───────── main ─────────

if __name__ == "__main__":
    rclpy.init()
    follow_run(follower="robot2", target="robot3")

 