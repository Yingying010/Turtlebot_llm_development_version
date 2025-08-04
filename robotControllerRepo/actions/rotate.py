# actions/rotate.py

import time
import math
from geometry_msgs.msg import Twist
from rclpy.node import Node

def rotate(node: Node, robot_id: str, direction: str, value: float, unit: str, target: str = "self"):
    print(f"🔁 {robot_id} turning {direction} {value} {unit} around {target}")

    pub = node.create_publisher(Twist, f"/{robot_id}/cmd_vel", 10)
    angular_speed = math.radians(30)  # 30°/s = 0.523 rad/s
    twist = Twist()

    if direction == "left":
        twist.angular.z = angular_speed
    elif direction == "right":
        twist.angular.z = -angular_speed
    else:
        print(f"⚠️ Unknown direction: {direction}")
        return

    if unit == "degrees":
        angle_rad = math.radians(value)
        duration = angle_rad / abs(angular_speed)
        print(f"{robot_id} turning {direction} for {value}° ({duration:.2f}s) around {target}")
 
    elif unit == "seconds":
        duration = value
        print(f"{robot_id} turning {direction} for {duration:.2f}s around {target}")
    else:
        print(f"⚠️ Unsupported unit: {unit}")
        return

    time.sleep(0.2)

    start_time = time.time()
    while time.time() - start_time < duration:
        pub.publish(twist)
        time.sleep(0.01)

    pub.publish(Twist())  # 停止
    print(f"✅ {robot_id} finished rotating.")
