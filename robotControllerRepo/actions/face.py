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

pose_cache = {}

pub_cache = {}
 
def ensure_pub(node: Node, name: str):

    return pub_cache.setdefault(name,

        node.create_publisher(Twist, f'/{name}/cmd_vel', 10))
 
def get_pose(name):

    d = pose_cache.get(name)

    return (d["x"], d["z"], d["heading_y"]) if d else None
 
# ───── 旋转函数（与你原来一致，删 sleep-0-帧即可） ─────

def rotate_to_face_target(robot, pub, tx, tz, tol=5):

    while True:

        pose = get_pose(robot)

        if not pose:

            time.sleep(0.05); continue

        x, z, yaw = pose

        tgt_yaw = math.degrees(math.atan2(tx-x, tz-z)) % 360

        err = (tgt_yaw - yaw + 180) % 360 - 180

        if abs(err) < tol:

            pub.publish(Twist()); break

        twist = Twist()

        twist.angular.z = 0.5 if abs(err) > 25 else 0.3 if abs(err) > 10 else 0.15

        if err < 0: twist.angular.z *= -1

        pub.publish(twist)

        time.sleep(0.05)
 
# ───── face 执行一次 ─────

def do_face(ctrl_node: Node, robot: str, target: str):

    # ① 实时优先

    tgt_pose = pose_cache.get(target)

    if tgt_pose:

        tx, tz = tgt_pose["x"], tgt_pose["z"]

    else:

        # 等 2 s 看看实时是否出现

        for _ in range(20):

            time.sleep(0.1)

            tgt_pose = pose_cache.get(target)

            if tgt_pose: break

        if tgt_pose:

            tx, tz = tgt_pose["x"], tgt_pose["z"]

        else:

            # 回退静态

            static = semantic_locations.get(target)

            if not static:

                ctrl_node.get_logger().error(f"No position for {target}")

                return

            tx, tz = static["x"], static["y"]
 
    pub = ensure_pub(ctrl_node, robot)

    rotate_to_face_target(robot, pub, tx, tz)

    ctrl_node.get_logger().info("✅ Face done.")
 
# ───── 主流程 ─────

def face_run(robot: str, target: str):

    # ① tracker + executor

    exec_ = MultiThreadedExecutor()

    for name in (robot, target):

        exec_.add_node(RigidTracker(pose_cache, name))

    threading.Thread(target=exec_.spin, daemon=True).start()
 
    # ② 控制节点一次性执行 face

    node = rclpy.create_node("face_controller")

    do_face(node, robot, target)
 
    # ③ 清理

    node.destroy_node()

    exec_.shutdown()

    rclpy.shutdown()
 
if __name__ == "__main__":

    rclpy.init()

    face_run(robot="robot1", target="robot2")   # ← 改成你的机器人 / 目标

 