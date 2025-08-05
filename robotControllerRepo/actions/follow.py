#!/usr/bin/env python3

# follow.py —— 不改 rigid_tracker.py，单进程跟随
 
import os, sys, math, time, threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from typing import Dict
 
# 工程根路径

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from ttsRepo.stream_tts import tts_manager

from phasespace.rigid_tracker import RigidTracker    # 原版，保持不动
 
# ------------------------------------------------------------------

robot_position_cache = {}               # robot_name ➜ {"x","y","z","heading_y",...}

pub_cache = {}           # robot_name ➜ Publisher
 
# ---------- 工具 ----------

# def get_pos(name: str):

#     if name in cache:

#         d = cache[name];   return d["x"], d["z"], d["heading_y"]

#     print(f"⚠️ No pos for {name}"); return 0.0, 0.0, 0.0
 
def ensure_pub(node: Node, name: str):
    return pub_cache.setdefault(
        name,
        node.create_publisher(Twist, f'/{name}/cmd_vel', 10)
    )

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


# --------- rotate and move

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



def move_forward_until_reached(robot_name, publisher, target, robot_position_cache, tolerance=20, max_acceptable_angle_error=25):
    x_target, y_target = target["x"], target["y"]
    print(f"\n🚗 NEED TO MOVE → ({x_target:.1f}, {y_target:.1f})")

    while True:
        x_now, y_now, heading_y_now = get_current_position(robot_name, robot_position_cache)
        dx = x_target - x_now
        dz = y_target - y_now
        distance = math.hypot(dx, dz)

        if distance < tolerance:
            print("🎉 Reached target.")
            break

        # 计算目标方向角与误差
        target_angle = math.degrees(math.atan2(dx, dz)) % 360
        angle_error = (target_angle - heading_y_now + 180) % 360 - 180

        if abs(angle_error) > max_acceptable_angle_error:
            print(f"🔁 Too much angle error: {angle_error:.1f}°, rotating first...")
            rotate_to_face_target(robot_name, publisher, target, robot_position_cache)
            continue  # 旋转完成后重新进入循环

        # 可以前进
        twist = Twist()
        twist.linear.x = 0.1  # 小速度保证精度
        publisher.publish(twist)
        print(f"🚗 Moving | dist={distance:.2f} | heading={heading_y_now:.1f}°, target={target_angle:.1f}°, error={angle_error:.1f}°")

        time.sleep(0.2)
        publisher.publish(Twist())  # 停止一小段时间，避免积累误差
        time.sleep(0.1)
 
# ---------- 跟随线程 ----------
follow_dist = 200.0
hysteresis = 50.0

def follow_loop(ctrl_node: Node, follower: str, target: str):
    pub = ensure_pub(ctrl_node, follower)
    tts_manager.say(f"{follower} is now following {target}")

    while rclpy.ok():
        fx, fz, _ = get_current_position(follower, robot_position_cache)
        tx, tz, _ = get_current_position(target, robot_position_cache)
        dx, dz = tx - fx, tz - fz
        dist = math.hypot(dx, dz)

        target = {"x":tx, "y":tz}

        if dist > follow_dist+ hysteresis:
            rotate_to_face_target(follower,pub,target,robot_position_cache)
            move_forward_until_reached(follower, pub, target, robot_position_cache, tolerance=follow_dist+hysteresis)
        else:
            pub.publish(Twist())
            rotate_to_face_target(follower,pub,target,robot_position_cache)
            time.sleep(0.2)

        print(f"👣 {follower}→{target}  dist={dist:.0f} mm")
        time.sleep(0.2)
 
# ------------------------------------------------------------------

def follow_run(follower:str, target:str):
    # ➊ 生成两个 RigidTracker 节点（保持原版本）
    tracker_f = RigidTracker(robot_position_cache, follower)
    tracker_t = RigidTracker(robot_position_cache, target)
 
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
    rclpy.init()
    follower, target = "robot2", "robot3"
    follow_run()

 