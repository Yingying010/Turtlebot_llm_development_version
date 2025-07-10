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
    angular_speed = math.radians(30)  # 角速度，单位 rad/s（默认设置为30°/s）

    if direction == "left":
        twist.angular.z = angular_speed
    elif direction == "right":
        twist.angular.z = -angular_speed
    else:
        print(f"⚠️ Unknown direction: {direction}")
        node.destroy_node()
        return

    # 角度转为时间（单位统一为 degrees → radians → 秒）
    if unit == "degrees":
        angle_rad = math.radians(value)
        duration = angle_rad / abs(angular_speed)
    else:
        print(f"⚠️ Unsupported unit: {unit}")
        node.destroy_node()
        return

    print(f"🔁 {robot_id} turning {direction} for {value}° ({duration:.2f}s) around {target}")

    start_time = time.time()
    while time.time() - start_time < duration:
        pub.publish(twist)
        time.sleep(0.1)

    pub.publish(Twist())  # 停止
    node.destroy_node()
    print(f"✅ {robot_id} finished rotating.")
