# actions/rotate.py

import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

def rotate(robot_id, direction, value, unit, target="self"):
    print(f"🔁 {robot_id} turning {direction} {value}{unit} around {target}")
    node = Node(f"{robot_id}_rotator")
    pub = node.create_publisher(Twist, f"/{robot_id}/cmd_vel", 10)

    twist = Twist()
    angular_speed = math.radians(30)  # 默认角速度：30°/s → 0.5236 rad/s

    if direction == "left":
        twist.angular.z = angular_speed
    elif direction == "right":
        twist.angular.z = -angular_speed
    else:
        print(f"⚠️ Unknown direction: {direction}")
        node.destroy_node()
        return

    # 🔄 支持单位：degrees 或 seconds
    if unit == "degrees":
        angle_rad = math.radians(value)
        duration = angle_rad / abs(angular_speed)
        print(f"🔁 {robot_id} turning {direction} for {value}° ({duration:.2f}s) around {target}")

    elif unit == "seconds":
        duration = value
        print(f"⏱️ {robot_id} turning {direction} for {duration:.2f}s around {target}")

    else:
        print(f"⚠️ Unsupported unit: {unit}")
        node.destroy_node()
        return

    # 执行旋转
    start_time = time.time()
    while time.time() - start_time < duration:
        pub.publish(twist)
        time.sleep(0.1)

    pub.publish(Twist())  # 停止
    node.destroy_node()
    print(f"✅ {robot_id} finished rotating.")
