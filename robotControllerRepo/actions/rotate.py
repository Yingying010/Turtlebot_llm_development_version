# actions/rotate.py

import time
import math
from geometry_msgs.msg import Twist
from rclpy.node import Node

def rotate(node: Node, robot_id: str, direction: str, value: float, unit: str, target: str = "self"):
    is_successful = False
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
        return is_successful

    if unit == "degree":
        angle_rad = math.radians(value)
        duration = angle_rad / abs(angular_speed)
        print(f"{robot_id} turning {direction} for {value}° ({duration:.2f}s) around {target}")
 
    elif unit == "second":
        duration = value
        print(f"{robot_id} turning {direction} for {duration:.2f}s around {target}")
    else:
        print(f"⚠️ Unsupported unit: {unit}")
        return is_successful

    time.sleep(0.2)

    start_time = time.time()
    try:
        while time.time() - start_time < duration:
            pub.publish(twist)
            time.sleep(0.01)

        pub.publish(Twist())  # 停止
        print(f"✅ {robot_id} finished rotating.")
        is_successful = True
        return is_successful
    except Exception:
        return is_successful
    

def rotate_deg(node: Node, robot_id: str, deg: float):
    """
    轻量包装：按角度旋转（正=左转，负=右转），单位 degree
    """
    direction = "left" if deg >= 0 else "right"
    return rotate(node, robot_id, direction, abs(deg), "degree", target="self")
