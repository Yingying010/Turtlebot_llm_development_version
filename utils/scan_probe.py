import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
class LaserScanSubscriber(Node):
    def __init__(self):
        super().__init__('laser_scan_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
    def listener_callback(self, msg):
        # 打印出激光扫描数据的第一个距离值
        self.get_logger().info('Received LaserScan data: "%s"' % str(msg.ranges[0]))
def main(args=None):
    rclpy.init(args=args)
    laser_scan_subscriber = LaserScanSubscriber()
    rclpy.spin(laser_scan_subscriber)
    rclpy.shutdown()
if __name__ == '__main__':
    main()