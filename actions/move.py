# actions/move.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

def move_linear(robot_id, direction, value, unit):
    print(f"🚗 {robot_id} moving {direction} for {value} {unit}")
    node = rclpy.create_node(f'{robot_id}_mover')
    publisher = node.create_publisher(Twist, f'/{robot_id}/cmd_vel', 10)

    twist = Twist()
    speed = 0.2  # m/s

    if direction == "forward":
        twist.linear.x = speed
    elif direction == "backward":
        twist.linear.x = -speed
    else:
        print(f"⚠️ Unknown direction: {direction}")
        rclpy.shutdown()
        return

    # duration
    if unit == "seconds":
        duration = value
    elif unit == "meters":
        duration = value / abs(speed)
    else:
        print(f"⚠️ Unknown unit: {unit}")
        rclpy.shutdown()
        return

    print(f"🚗 {robot_id} moving {direction} for {duration:.2f} seconds...")

    start = time.time()
    while time.time() - start < duration:
        publisher.publish(twist)
        time.sleep(0.1)

    publisher.publish(Twist())  # stop
    node.destroy_node()
    print(f"✅ {robot_id} finished moving.")
