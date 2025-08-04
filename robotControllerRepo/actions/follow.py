#!/usr/bin/env python3

# follow.py —— 不改 rigid_tracker.py，单进程跟随
 
import os, sys, math, time, threading

import rclpy

from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
 
# 工程根路径

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from ttsRepo.stream_tts import tts_manager

from phasespace.rigid_tracker import RigidTracker    # 原版，保持不动
 
# ------------------------------------------------------------------

cache = {}               # robot_name ➜ {"x","y","z","heading_y",...}

pub_cache = {}           # robot_name ➜ Publisher
 
# ---------- 工具 ----------

def get_pos(name: str):

    if name in cache:

        d = cache[name];   return d["x"], d["z"], d["heading_y"]

    print(f"⚠️ No pos for {name}"); return 0.0, 0.0, 0.0
 
def ensure_pub(node: Node, name: str):

    return pub_cache.setdefault(

        name,

        node.create_publisher(Twist, f'/{name}/cmd_vel', 10)

    )
 
# ---------- 跟随线程 ----------

def follow_loop(ctrl_node: Node, follower: str, target: str, dist_mm=200.0):

    pub = ensure_pub(ctrl_node, follower)

    tts_manager.say(f"{follower} is now following {target}")
 
    while rclpy.ok():

        fx, fz, fyaw = get_pos(follower)

        tx, tz, _    = get_pos(target)
 
        dx, dz = tx - fx, tz - fz

        D      = math.hypot(dx, dz)

        tgt_yaw = math.degrees(math.atan2(dx, dz)) % 360

        err     = (tgt_yaw - fyaw + 180) % 360 - 180
 
        twist = Twist()

        if abs(err) > 10:

            twist.angular.z = 0.3 * (1 if err > 0 else -1)

        if D > dist_mm:

            twist.linear.x = 0.15

        pub.publish(twist)
 
        print(f"👣 {follower}→{target}  dist={D:.0f} mm | yaw_err={err:.1f}° | v={twist.linear.x:.2f}")

        time.sleep(0.2)
 
# ------------------------------------------------------------------

def main():

    rclpy.init()
 
    follower, target = "robot2", "robot3"
 
    # ➊ 生成两个 RigidTracker 节点（保持原版本）

    tracker_f = RigidTracker(cache, follower)

    tracker_t = RigidTracker(cache, target)
 
    # ➋ MultiThreadedExecutor 统一 spin

    exec_ = MultiThreadedExecutor()

    exec_.add_node(tracker_f)

    exec_.add_node(tracker_t)
 
    spin_thread = threading.Thread(target=exec_.spin, daemon=True)

    spin_thread.start()
 
    # ➌ 控制节点 + 跟随逻辑

    ctrl_node = rclpy.create_node("follow_controller")

    threading.Thread(target=follow_loop,

                     args=(ctrl_node, follower, target),

                     daemon=True).start()
 
    try:

        rclpy.spin(ctrl_node)          # 主线程保持活跃，Ctrl-C 可退出

    except KeyboardInterrupt:

        pass
 
    # 清理

    exec_.shutdown()

    tracker_f.destroy_node()

    tracker_t.destroy_node()

    ctrl_node.destroy_node()

    rclpy.shutdown()
 
# ------------------------------------------------------------------

if __name__ == "__main__":

    main()

 