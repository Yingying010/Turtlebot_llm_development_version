# actions/move.py
import time
from geometry_msgs.msg import Twist
from rclpy.node import Node

def move(node: Node, robot_id, direction, value, unit):
    is_successful = False
    print(f"🚗 {robot_id} moving {direction} for {value} {unit}")
    publisher = node.create_publisher(Twist, f'/{robot_id}/cmd_vel', 10)

    twist = Twist()
    speed = 0.2  # m/s

    if direction == "forward":
        twist.linear.x = speed
    elif direction == "backward":
        twist.linear.x = -speed
    else:
        print(f"⚠️ Unknown direction: {direction}")
        return  # ❌ 不调用 rclpy.shutdown()

    if unit == "seconds":
        duration = value
    elif unit == "meters":
        duration = value / abs(speed)
    else:
        print(f"⚠️ Unknown unit: {unit}")
        return  # ❌ 不调用 rclpy.shutdown()

    print(f"🚗 {robot_id} moving {direction} for {duration:.2f} seconds...")

    time.sleep(0.2)  # 等待 publisher 初始化

    try:
        start = time.time()
        while time.time() - start < duration:
            publisher.publish(twist)
            time.sleep(0.01)  # 提高平滑度

        publisher.publish(Twist())  # stop
        print(f"✅ {robot_id} finished moving.")
        is_successful = True
        return is_successful
    except Exception:
        return is_successful
