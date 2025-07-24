import os, sys
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time
from typing import Dict
import threading
from subscriber.rigid_tracker import RigidTracker

# === 全局缓存 ===
robot_position_cache = {}  # e.g., {'rigid1': {'x': .., 'y': .., 'z': .., 'qx': .., ...}}
publisher_dict = {}        # e.g., {'turtlebot1': (node, publisher)}

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

    
def rotate_to_face_target(robot_id, publisher, target: Dict[str, float], angle_tolerance_deg=5):
    x_target, y_target = target["x"], target["y"]
    x_now, y_now, _ = get_current_position(robot_id)
    dx = x_target - x_now
    dz = y_target - y_now
    target_angle = math.degrees(math.atan2(dx, dz)) % 360

    print(f"\n🔄 ROTATE | target: ({x_target:.1f}, {y_target:.1f}) → {target_angle:.1f}°")

    prev_error = None
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

        publisher.publish(twist)
        print(f"↪️ turning {'left' if new_direction==1 else 'right'} | heading_y: {heading_y_now:.1f} | target_angle: {target_angle:.1f} | angle_error: {angle_error:.1f}° | speed: {twist.angular.z:.2f}")
        prev_error = angle_error
        time.sleep(0.1)

    publisher.publish(Twist())
    time.sleep(0.2)

    
def move_forward_until_reached(robot_name, publisher, target, tolerance=20, max_acceptable_angle_error=25):
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

        # 计算目标方向角与误差
        target_angle = math.degrees(math.atan2(dx, dz)) % 360
        angle_error = (target_angle - heading_y_now + 180) % 360 - 180

        if abs(angle_error) > max_acceptable_angle_error:
            print(f"🔁 Too much angle error: {angle_error:.1f}°, rotating first...")
            rotate_to_face_target(robot_name, publisher, target)
            continue  # 旋转完成后重新进入循环

        # 可以前进
        twist = Twist()
        twist.linear.x = 0.1  # 小速度保证精度
        publisher.publish(twist)
        print(f"🚗 Moving | dist={distance:.2f} | heading={heading_y_now:.1f}°, target={target_angle:.1f}°, error={angle_error:.1f}°")

        time.sleep(0.2)
        publisher.publish(Twist())  # 停止一小段时间，避免积累误差
        time.sleep(0.1)



def navigate_to_position(robot_name, target: Dict[str, float], robot_position_cache):
    x_target, y_target = target["x"], target["y"]
    x_now, y_now, _ = get_current_position(robot_name, robot_position_cache)

    dx = x_target - x_now
    dy = y_target - y_now
    distance = math.hypot(dx, dy)
    angle = math.atan2(dy, dx)

    print(f"\n🧭 NAVIGATE {robot_name} → ({x_target:.1f}, {y_target:.1f}) | dist={distance:.2f}")

    if robot_name not in publisher_dict:
        node = rclpy.create_node(f'{robot_name}_navigator')
        pub = node.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
        publisher_dict[robot_name] = (node, pub)
    else:
        _, pub = publisher_dict[robot_name]

    # ✅ Phase 1: rotate first
    rotate_to_face_target(robot_name, pub, target)

    # ✅ Phase 2: move straight
    move_forward_until_reached(robot_name, pub, target)

    print(f"✅ {robot_name} navigation complete.")


def navigate_to_target(robot_name, target, robot_position_cache):
    if isinstance(target, dict) and "x" in target and "y" in target:
        navigate_to_position(robot_name, target, robot_position_cache)
    else:
        print(f"⚠️ Invalid target: {target}")

def run(robot_name, target):
    rclpy.init()
    node = RigidTracker(robot_position_cache, robot_name)

    executor_thread = threading.Thread(target=rclpy.spin, args=(node,))
    executor_thread.start()

    print("⏳ Waiting for position data...")
    for i in range(30):  # 最多等 6 秒
        if robot_name in robot_position_cache:
            print("✅ Position data received.")
            break
        time.sleep(0.2)
    else:
        print("❌ Timeout: No position data.")
        return

    navigate_to_target(robot_name, target, robot_position_cache)

    time.sleep(2)
    node.destroy_node()
    rclpy.shutdown()
    executor_thread.join()

if __name__ == "__main__":
    robot_name = "robot1"
    target = {"x": -1135, "y": -1253}  # PhaseSpace 的坐标（x,z）
    run(robot_name, target)
    print("🚀 Finished test navigation.")
