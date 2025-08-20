#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class CommTestSender(Node):
    def __init__(self):
        super().__init__('ros2_comm_test_sender')
        self.pub = self.create_publisher(String, '/robot_status', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = {
            "kind": "state",
            "robot": "robotA",
            "status": "ready",
            "sync_group": 9999,
            "ts": time.time()
        }
        out = String()
        out.data = json.dumps(msg)
        self.pub.publish(out)
        self.get_logger().info(f"📤 Sent test message #{self.count} to /robot_status")
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = CommTestSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
