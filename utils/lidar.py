#!/usr/bin/env python3

import math

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist
 
SAFE_DISTANCE = 1.5   # 小于这个距离就转向

TURN_SPEED    = 0.3

FORWARD_SPEED = 0.05

COOLDOWN_TICKS = 50   # 冷却计数（执行一次转向后，延迟若干回调再检测）
 
class LidarAvoider(Node):

    def __init__(self):

        super().__init__('lidar_avoider')

        self.pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)

        self.sub = self.create_subscription(LaserScan, '/robot1/scan', self.cb, 10)

        self.cooldown = 0
 
    def front_distance(self, msg: LaserScan):

        n = len(msg.ranges)

        if n == 0: return None

        mid = n // 2

        d = msg.ranges[mid]

        return d if math.isfinite(d) else None
 
    def cb(self, msg: LaserScan):

        d = self.front_distance(msg)

        if d is None:

            self.get_logger().warn('前方测距无效')

            return

        self.get_logger().info(f'前方测距 = {d:.3f} m')
 
        if self.cooldown > 0:

            self.cooldown -= 1

            return
 
        cmd = Twist()

        if d < SAFE_DISTANCE:

            cmd.angular.z = TURN_SPEED

            self.cooldown = COOLDOWN_TICKS

        else:

            cmd.linear.x = FORWARD_SPEED

        self.pub.publish(cmd)
 
def main():

    rclpy.init()

    node = LidarAvoider()

    try:

        rclpy.spin(node)

    finally:

        node.destroy_node()

        rclpy.shutdown()
 
if __name__ == '__main__':

    main()

 