#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommTestReceiver(Node):
    def __init__(self):
        super().__init__('ros2_comm_test_receiver')
        self.sub = self.create_subscription(String, '/robot_status', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f"📥 Received: {msg.data[:100]}...")

def main(args=None):
    rclpy.init(args=args)
    node = CommTestReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
