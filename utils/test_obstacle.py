#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import os
import math

class LidarDebugger(Node):
    def __init__(self, robot_name="robot1"):
        super().__init__('lidar_debugger')
        
        self.robot_name = robot_name
        
        # 订阅雷达数据
        self.laser_sub = self.create_subscription(
            LaserScan,
            f'/{robot_name}/scan',
            self.laser_callback,
            10
        )
        
        self.last_print_time = time.time()
        self.print_interval = 1.0  # 每秒打印一次
        
        print(f"=== 雷达调试工具启动 - 机器人: {robot_name} ===")
        print("实时显示雷达数据，按 Ctrl+C 停止")
        print("-" * 60)

    def laser_callback(self, msg: LaserScan):
        """处理并显示雷达数据"""
        
        current_time = time.time()
        if current_time - self.last_print_time < self.print_interval:
            return
        self.last_print_time = current_time
        
        # 清屏（可选）
        os.system('clear' if os.name == 'posix' else 'cls')
        
        print(f"=== 雷达数据调试 - {self.robot_name} ===")
        print(f"时间戳: {current_time:.2f}")
        print(f"Frame ID: {msg.header.frame_id}")
        print("-" * 50)
        
        # 基本信息
        print(f"角度范围: {math.degrees(msg.angle_min):.1f}° 到 {math.degrees(msg.angle_max):.1f}°")
        print(f"角度增量: {math.degrees(msg.angle_increment):.2f}°")
        print(f"距离范围: {msg.range_min:.2f}m 到 {msg.range_max:.2f}m")
        print(f"数据点数: {len(msg.ranges)}")
        print("-" * 50)
        
        # 转换为numpy数组方便处理
        ranges = np.array(msg.ranges)
        
        # 统计有效数据
        valid_mask = (ranges >= msg.range_min) & (ranges <= msg.range_max) & np.isfinite(ranges)
        valid_ranges = ranges[valid_mask]
        
        print(f"有效数据点: {len(valid_ranges)}/{len(ranges)} ({len(valid_ranges)/len(ranges)*100:.1f}%)")
        
        if len(valid_ranges) > 0:
            print(f"最近距离: {np.min(valid_ranges):.3f}m")
            print(f"最远距离: {np.max(valid_ranges):.3f}m")
            print(f"平均距离: {np.mean(valid_ranges):.3f}m")
        else:
            print("❌ 没有有效的距离数据！")
            return
        
        print("-" * 50)
        
        # 分区域分析（前方、左侧、右侧、后方）
        total_points = len(ranges)
        
        # 定义区域（以机器人为中心，前方为0度）
        regions = {
            "前方": (total_points * 7//16, total_points * 9//16),    # 前方 45度范围
            "右前": (total_points * 5//16, total_points * 7//16),    # 右前
            "右侧": (total_points * 3//16, total_points * 5//16),    # 右侧
            "右后": (total_points * 1//16, total_points * 3//16),    # 右后
            "后方": (0, total_points * 1//16),                       # 后方
            "左后": (total_points * 15//16, total_points),           # 左后
            "左侧": (total_points * 13//16, total_points * 15//16),  # 左侧
            "左前": (total_points * 11//16, total_points * 13//16),  # 左前
        }
        
        for region_name, (start, end) in regions.items():
            region_ranges = ranges[start:end]
            region_valid = region_ranges[
                (region_ranges >= msg.range_min) & 
                (region_ranges <= msg.range_max) & 
                np.isfinite(region_ranges)
            ]
            
            if len(region_valid) > 0:
                min_dist = np.min(region_valid)
                avg_dist = np.mean(region_valid)
                
                # 用颜色和符号表示距离
                if min_dist < 0.3:
                    status = "🔴 危险"
                elif min_dist < 0.5:
                    status = "🟡 注意"
                elif min_dist < 1.0:
                    status = "🟢 安全"
                else:
                    status = "⚪ 远距"
                
                print(f"{region_name}: 最近{min_dist:.2f}m, 平均{avg_dist:.2f}m {status}")
            else:
                print(f"{region_name}: 无有效数据")
        
        print("-" * 50)
        
        # 简单的障碍物警告
        front_ranges = ranges[total_points * 7//16:total_points * 9//16]
        front_valid = front_ranges[
            (front_ranges >= msg.range_min) & 
            (front_ranges <= msg.range_max) & 
            np.isfinite(front_ranges)
        ]
        
        if len(front_valid) > 0:
            front_min = np.min(front_valid)
            if front_min < 0.3:
                print("⚠️  前方有紧急障碍物！建议立即停止！")
            elif front_min < 0.5:
                print("⚠️  前方有障碍物，建议避让")
            else:
                print("✅ 前方安全")
        
        print("=" * 60)

def main():
    import math
    
    rclpy.init()
    
    # 可以通过命令行参数指定机器人名称
    import sys
    robot_name = sys.argv[1] if len(sys.argv) > 1 else "robot1"
    
    node = LidarDebugger(robot_name)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n调试结束")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()