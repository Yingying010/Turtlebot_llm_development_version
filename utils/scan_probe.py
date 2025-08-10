#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
 
class ScanProbe(Node):
    def __init__(self):
        super().__init__('scan_probe')
        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.count = 0
        self.create_subscription(LaserScan, '/robot1/scan', self.cb, qos)
        self.get_logger().info("Listening to /robot1/scan with BEST_EFFORT QoS...")
 
    def cb(self, msg: LaserScan):
        self.count += 1
        front_min = min(msg.ranges) if msg.ranges else float('inf')
        self.get_logger().info(f"[{self.count}] front_min={front_min:.3f} m, total_beams={len(msg.ranges)}")
 
def main():
    rclpy.init()
    node = ScanProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()