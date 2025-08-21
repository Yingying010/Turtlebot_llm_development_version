#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class ObstacleAvoidanceTest(Node):
    def __init__(self, robot_name="robot1"):
        super().__init__('obstacle_avoidance_test')
        
        self.robot_name = robot_name
        
        # 订阅雷达数据
        self.laser_sub = self.create_subscription(
            LaserScan,
            f'/{robot_name}/scan',
            self.laser_callback,
            10
        )
        
        # 发布速度命令
        self.cmd_pub = self.create_publisher(
            Twist,
            f'/{robot_name}/cmd_vel',
            10
        )
        
        # 避障参数
        self.obstacle_threshold = 0.5  # 50cm
        self.emergency_stop = 0.3      # 30cm紧急停止
        
        self.get_logger().info(f'避障测试节点启动 - 机器人: {robot_name}')

    def laser_callback(self, msg: LaserScan):
        """处理雷达数据并进行避障"""
        
        # 获取有效的距离数据
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[(ranges >= msg.range_min) & (ranges <= msg.range_max)]
        
        if len(valid_ranges) == 0:
            self.get_logger().warn("没有有效的雷达数据")
            return
        
        # 检查前方区域（前方90度范围）
        total_points = len(ranges)
        front_start = total_points * 3 // 8  # 前方45度左侧
        front_end = total_points * 5 // 8    # 前方45度右侧
        front_ranges = ranges[front_start:front_end]
        
        # 过滤无效值
        front_valid = front_ranges[(front_ranges >= msg.range_min) & (front_ranges <= msg.range_max)]
        
        if len(front_valid) == 0:
            self.get_logger().warn("前方没有有效雷达数据")
            return
        
        # 找到最近的障碍物
        min_distance = np.min(front_valid)
        
        # 创建速度命令
        twist = Twist()
        
        if min_distance < self.emergency_stop:
            # 紧急停止
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().warn(f"紧急停止！前方障碍物距离: {min_distance:.2f}m")
            
        elif min_distance < self.obstacle_threshold:
            # 避障 - 停止前进，开始转向
            twist.linear.x = 0.0
            twist.angular.z = 0.3  # 左转
            self.get_logger().info(f"避障中... 障碍物距离: {min_distance:.2f}m，正在转向")
            
        else:
            # 安全区域 - 直行
            twist.linear.x = 0.1  # 慢速前进
            twist.angular.z = 0.0
            self.get_logger().info(f"前方安全，距离: {min_distance:.2f}m，继续前进")
        
        # 发布速度命令
        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    
    # 可以通过命令行参数指定机器人名称
    import sys
    robot_name = sys.argv[1] if len(sys.argv) > 1 else "robot1"
    
    node = ObstacleAvoidanceTest(robot_name)
    
    print(f"开始测试 {robot_name} 的避障功能...")
    print("机器人会自动前进，遇到障碍物时会停止并转向")
    print("按 Ctrl+C 停止测试")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("测试结束")
    finally:
        # 停止机器人
        twist = Twist()
        node.cmd_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()