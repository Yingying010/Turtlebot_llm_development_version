#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('multi_robot_velocity_publisher')

        # 创建两个publisher
        self.publisher_robot1 = self.create_publisher(
            Twist,
            '/robot1/cmd_vel',
            10
        )
        self.publisher_robot2 = self.create_publisher(
            Twist,
            '/robot2/cmd_vel',
            10
        )

        # 设置定时器，每0.1秒发布一次
        self.timer = self.create_timer(0.1, self.publish_velocity)

    def publish_velocity(self):
        # Robot1: forward
        twist1 = Twist()
        twist1.linear.x = 0.01
        twist1.angular.z = 0.0

        # Robot2: backward
        twist2 = Twist()
        twist2.linear.x = -0.01
        twist2.angular.z = 0.0

        self.publisher_robot1.publish(twist1)
        self.publisher_robot2.publish(twist2)

        self.get_logger().info('Published to robot1: forward 0.01 m/s')
        self.get_logger().info('Published to robot2: backward 0.01 m/s')


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
