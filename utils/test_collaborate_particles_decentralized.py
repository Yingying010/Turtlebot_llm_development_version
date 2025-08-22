#!/usr/bin/env python3
"""
分布式同步运动控制系统
支持多台机器人的时间戳同步控制
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import threading
import time
import json
import argparse
import sys
from typing import Dict, Optional
from dataclasses import dataclass
from datetime import datetime

class DebugLogger:
    """调试信息输出类"""
    
    @staticmethod
    def get_timestamp():
        """获取格式化的时间戳"""
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]
    
    @staticmethod
    def log_command_sent(robot_id: str, direction: str, speed: float, start_time: float):
        """记录发送的指令"""
        start_formatted = datetime.fromtimestamp(start_time).strftime("%H:%M:%S.%f")[:-3]
        print(f"📤 [{DebugLogger.get_timestamp()}] SENT → {robot_id} | {direction} | {speed:.3f}m/s | start_at:{start_formatted}")
    
    @staticmethod
    def log_command_received(robot_id: str, direction: str, speed: float, wait_time: float):
        """记录接收的指令"""
        print(f"📨 [{DebugLogger.get_timestamp()}] RECV → {robot_id} | {direction} | {speed:.3f}m/s | wait:{wait_time:.3f}s")
    
    @staticmethod
    def log_movement_start(robot_id: str, direction: str, speed: float, duration: float):
        """记录运动开始"""
        print(f"🚀 [{DebugLogger.get_timestamp()}] START → {robot_id} | {direction} | {speed:.3f}m/s | {duration:.1f}s")
    
    @staticmethod
    def log_movement_stop(robot_id: str, reason: str = "completed"):
        """记录运动停止"""
        print(f"🛑 [{DebugLogger.get_timestamp()}] STOP → {robot_id} | {reason}")
    
    @staticmethod
    def log_emergency_stop():
        """记录紧急停止"""
        print(f"🚨 [{DebugLogger.get_timestamp()}] EMERGENCY_STOP → ALL_ROBOTS")
    
    @staticmethod
    def log_status(robot_id: str, status: str):
        """记录状态变化"""
        print(f"📊 [{DebugLogger.get_timestamp()}] STATUS → {robot_id} | {status}")
    
    @staticmethod
    def log_sync_info(delay: float, robots: list):
        """记录同步信息"""
        robot_list = ", ".join(robots)
        execute_time = datetime.fromtimestamp(time.time() + delay).strftime("%H:%M:%S.%f")[:-3]
        print(f"⏰ [{DebugLogger.get_timestamp()}] SYNC → delay:{delay:.3f}s | execute_at:{execute_time} | robots:[{robot_list}]")
    
    @staticmethod
    def log_timing_info(robot_id: str, expected_time: float, actual_time: float):
        """记录时间精度信息"""
        time_diff = actual_time - expected_time
        status = "✅ ON_TIME" if abs(time_diff) < 0.1 else "⚠️ LATE" if time_diff > 0 else "⚠️ EARLY"
        print(f"⏱️ [{DebugLogger.get_timestamp()}] TIMING → {robot_id} | diff:{time_diff:.3f}s | {status}")
    
    @staticmethod
    def log_network_info(robot_id: str, sent_time: float, received_time: float):
        """记录网络延迟信息"""
        network_delay = received_time - sent_time
        print(f"🌐 [{DebugLogger.get_timestamp()}] NETWORK → {robot_id} | delay:{network_delay:.3f}s")

@dataclass
class MovementCommand:
    """运动指令数据类"""
    robot_id: str
    direction: str
    speed: float
    start_timestamp: float
    duration: float
    command_id: str

class SynchronizedMovementCoordinator(Node):
    """主控节点 - 负责协调和发送运动指令"""
    
    def __init__(self):
        super().__init__('movement_coordinator')
        
        # 配置可靠的QoS
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        # 发布运动指令
        self.command_publisher = self.create_publisher(
            String, 
            '/robot_movement_commands', 
            qos_profile
        )
        
        # 订阅状态反馈
        self.status_subscriber = self.create_subscription(
            String,
            '/robot_status_feedback',
            self.status_callback,
            qos_profile
        )
        
        self.robot_status = {}  # 记录各机器人状态
        self.command_counter = 0
        
        self.get_logger().info("🎯 Movement Coordinator initialized")
    
    def status_callback(self, msg):
        """处理机器人状态反馈"""
        try:
            status_data = json.loads(msg.data)
            robot_id = status_data.get('robot_id')
            self.robot_status[robot_id] = status_data
            
            self.get_logger().info(
                f"📡 Status from {robot_id}: {status_data.get('status', 'unknown')}"
            )
        except json.JSONDecodeError:
            self.get_logger().warning("❌ Invalid status message received")
    
    def send_synchronized_command(self, commands: list, delay_seconds: float = 3.0):
        """
        发送同步运动指令
        
        Args:
            commands: 运动指令列表 [{'robot_id': 'robot1', 'direction': 'forward', 'speed': 0.1, 'duration': 5.0}, ...]
            delay_seconds: 延迟执行时间（秒），用于同步
        """
        # 计算统一的开始时间戳（当前时间 + 延迟）
        start_timestamp = time.time() + delay_seconds
        self.command_counter += 1
        
        # 记录同步信息
        robot_ids = [cmd['robot_id'] for cmd in commands]
        DebugLogger.log_sync_info(delay_seconds, robot_ids)
        
        self.get_logger().info(f"📋 Preparing synchronized movement commands...")
        self.get_logger().info(f"⏰ Scheduled start time: {delay_seconds} seconds from now")
        
        for cmd_data in commands:
            command = MovementCommand(
                robot_id=cmd_data['robot_id'],
                direction=cmd_data['direction'],
                speed=cmd_data['speed'],
                start_timestamp=start_timestamp,
                duration=cmd_data.get('duration', 10.0),
                command_id=f"cmd_{self.command_counter}_{cmd_data['robot_id']}"
            )
            
            # 将指令序列化并发布
            command_msg = String()
            command_msg.data = json.dumps({
                'type': 'movement_command',
                'robot_id': command.robot_id,
                'direction': command.direction,
                'speed': command.speed,
                'start_timestamp': command.start_timestamp,
                'duration': command.duration,
                'command_id': command.command_id,
                'sent_at': time.time()
            })
            
            self.command_publisher.publish(command_msg)
            
            # 记录发送的指令
            DebugLogger.log_command_sent(
                command.robot_id, 
                command.direction, 
                command.speed, 
                command.start_timestamp
            )
            
            self.get_logger().info(
                f"📤 Sent command to {command.robot_id}: {command.direction} at {command.speed}m/s"
            )
        
        self.get_logger().info(f"✅ All commands sent. Robots will start in {delay_seconds} seconds")
    
    def stop_all_robots(self):
        """紧急停止所有机器人"""
        DebugLogger.log_emergency_stop()
        
        stop_msg = String()
        stop_msg.data = json.dumps({
            'type': 'emergency_stop',
            'timestamp': time.time()
        })
        self.command_publisher.publish(stop_msg)
        self.get_logger().info("🚨 Emergency stop command sent to all robots")


class SynchronizedRobotClient(Node):
    """客户端节点 - 每台机器人运行一个实例"""
    
    def __init__(self, robot_id: str):
        super().__init__(f'robot_client_{robot_id}')
        self.robot_id = robot_id
        
        # 配置可靠的QoS
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        # 订阅运动指令
        self.command_subscriber = self.create_subscription(
            String,
            '/robot_movement_commands',
            self.command_callback,
            qos_profile
        )
        
        # 发布状态反馈
        self.status_publisher = self.create_publisher(
            String,
            '/robot_status_feedback',
            qos_profile
        )
        
        # 运动控制发布器
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            f'/{robot_id}/cmd_vel',
            10
        )
        
        # 状态管理
        self.current_movement = None
        self.stop_flag = threading.Event()
        self.movement_thread = None
        
        # 定期发送心跳
        self.heartbeat_timer = self.create_timer(5.0, self.send_heartbeat)
        
        self.get_logger().info(f"🤖 Robot client {robot_id} initialized")
        self.send_status("ready")
    
    def command_callback(self, msg):
        """处理收到的运动指令"""
        try:
            command_data = json.loads(msg.data)
            command_type = command_data.get('type')
            
            # 记录网络延迟
            if 'sent_at' in command_data:
                DebugLogger.log_network_info(
                    self.robot_id, 
                    command_data['sent_at'], 
                    time.time()
                )
            
            if command_type == 'movement_command':
                # 检查是否是给自己的指令
                if command_data.get('robot_id') == self.robot_id:
                    self.handle_movement_command(command_data)
            
            elif command_type == 'emergency_stop':
                self.emergency_stop()
                
        except json.JSONDecodeError:
            self.get_logger().warning("❌ Invalid command message received")
    
    def handle_movement_command(self, command_data):
        """处理运动指令"""
        start_timestamp = command_data['start_timestamp']
        current_time = time.time()
        
        # 计算等待时间
        wait_time = start_timestamp - current_time
        
        # 记录接收的指令
        DebugLogger.log_command_received(
            self.robot_id,
            command_data['direction'],
            command_data['speed'],
            wait_time
        )
        
        if wait_time < -1.0:  # 指令已过期超过1秒
            self.get_logger().warning(
                f"⚠️ Command expired by {abs(wait_time):.2f} seconds, ignoring"
            )
            return
        
        # 停止当前运动
        self.stop_current_movement()
        
        # 记录新指令
        self.current_movement = command_data
        
        self.get_logger().info(
            f"📨 Received command: {command_data['direction']} at {command_data['speed']}m/s"
        )
        self.get_logger().info(f"⏱️ Will start in {max(0, wait_time):.2f} seconds")
        
        # 启动运动线程
        self.movement_thread = threading.Thread(
            target=self.execute_movement,
            args=(command_data, max(0, wait_time))
        )
        self.movement_thread.start()
    
    def execute_movement(self, command_data, wait_time):
        """执行运动指令"""
        try:
            # 等待到指定时间
            if wait_time > 0:
                time.sleep(wait_time)
            
            # 记录实际执行时间精度
            actual_start_time = time.time()
            expected_start_time = command_data['start_timestamp']
            DebugLogger.log_timing_info(self.robot_id, expected_start_time, actual_start_time)
            
            # 检查是否被停止
            if self.stop_flag.is_set():
                return
            
            direction = command_data['direction']
            speed = command_data['speed']
            duration = command_data['duration']
            
            # 记录运动开始
            DebugLogger.log_movement_start(self.robot_id, direction, speed, duration)
            
            # 准备运动指令
            twist = Twist()
            if direction == "forward":
                twist.linear.x = abs(speed)
            elif direction == "backward":
                twist.linear.x = -abs(speed)
            elif direction == "left":
                twist.angular.z = abs(speed)
            elif direction == "right":
                twist.angular.z = -abs(speed)
            else:
                self.get_logger().error(f"❌ Unknown direction: {direction}")
                return
            
            self.get_logger().info(f"🚀 Starting {direction} movement at {speed}m/s for {duration}s")
            self.send_status(f"moving_{direction}")
            
            # 执行运动
            start_time = time.time()
            loop_count = 0
            while time.time() - start_time < duration and not self.stop_flag.is_set():
                self.cmd_vel_publisher.publish(twist)
                loop_count += 1
                # 每秒输出一次运动状态
                if loop_count % 10 == 0:  # 10Hz -> 每秒一次
                    elapsed = time.time() - start_time
                    remaining = duration - elapsed
                    print(f"🔄 [{DebugLogger.get_timestamp()}] MOVING → {self.robot_id} | {direction} | {speed:.3f}m/s | remaining:{remaining:.1f}s")
                time.sleep(0.1)  # 10Hz
            
            # 停止运动
            self.cmd_vel_publisher.publish(Twist())
            
            if not self.stop_flag.is_set():
                DebugLogger.log_movement_stop(self.robot_id, "completed")
                self.get_logger().info(f"✅ Movement completed")
                self.send_status("completed")
            else:
                DebugLogger.log_movement_stop(self.robot_id, "interrupted")
                self.get_logger().info(f"🛑 Movement stopped")
                self.send_status("stopped")
            
        except Exception as e:
            DebugLogger.log_movement_stop(self.robot_id, f"error: {e}")
            self.get_logger().error(f"❌ Movement execution failed: {e}")
            self.send_status("error")
        finally:
            self.current_movement = None
    
    def stop_current_movement(self):
        """停止当前运动"""
        if self.movement_thread and self.movement_thread.is_alive():
            self.stop_flag.set()
            self.movement_thread.join(timeout=1.0)
            self.stop_flag.clear()
        
        # 确保机器人停止
        self.cmd_vel_publisher.publish(Twist())
    
    def emergency_stop(self):
        """紧急停止"""
        DebugLogger.log_movement_stop(self.robot_id, "emergency_stop")
        self.get_logger().info("🚨 Emergency stop received!")
        self.stop_current_movement()
        self.send_status("emergency_stopped")
    
    def send_status(self, status: str):
        """发送状态反馈"""
        # 记录状态变化（排除心跳以减少噪音）
        if status != "heartbeat":
            DebugLogger.log_status(self.robot_id, status)
            
        status_msg = String()
        status_msg.data = json.dumps({
            'robot_id': self.robot_id,
            'status': status,
            'timestamp': time.time(),
            'current_command': self.current_movement.get('command_id') if self.current_movement else None
        })
        self.status_publisher.publish(status_msg)
    
    def send_heartbeat(self):
        """发送心跳信号"""
        self.send_status("heartbeat")


def run_coordinator():
    """运行主控节点"""
    rclpy.init()
    
    try:
        coordinator = SynchronizedMovementCoordinator()
        
        print("🎯 Synchronized Movement Coordinator started")
        print("Commands:")
        print("  1 - Forward sync test")
        print("  2 - Opposite directions test")
        print("  3 - Rotation test")
        print("  s - Emergency stop")
        print("  q - Quit")
        
        def input_thread():
            while rclpy.ok():
                try:
                    user_input = input("\nEnter command: ").strip().lower()
                    
                    if user_input == '1':
                        # 同向运动测试
                        commands = [
                            {'robot_id': 'robot1', 'direction': 'forward', 'speed': 0.1, 'duration': 5.0},
                            {'robot_id': 'robot2', 'direction': 'forward', 'speed': 0.1, 'duration': 5.0}
                        ]
                        coordinator.send_synchronized_command(commands)
                    
                    elif user_input == '2':
                        # 反向运动测试
                        commands = [
                            {'robot_id': 'robot1', 'direction': 'forward', 'speed': 0.1, 'duration': 5.0},
                            {'robot_id': 'robot2', 'direction': 'backward', 'speed': 0.1, 'duration': 5.0}
                        ]
                        coordinator.send_synchronized_command(commands)
                    
                    elif user_input == '3':
                        # 旋转测试
                        commands = [
                            {'robot_id': 'robot1', 'direction': 'left', 'speed': 0.5, 'duration': 3.0},
                            {'robot_id': 'robot2', 'direction': 'right', 'speed': 0.5, 'duration': 3.0}
                        ]
                        coordinator.send_synchronized_command(commands)
                    
                    elif user_input == 's':
                        coordinator.stop_all_robots()
                    
                    elif user_input == 'q':
                        break
                        
                except KeyboardInterrupt:
                    break
        
        # 启动输入线程
        input_thread_handle = threading.Thread(target=input_thread, daemon=True)
        input_thread_handle.start()
        
        # 运行ROS节点
        rclpy.spin(coordinator)
        
    except KeyboardInterrupt:
        print("\n🛑 Coordinator shutting down...")
    finally:
        coordinator.destroy_node()
        rclpy.shutdown()


def run_client(robot_id: str):
    """运行客户端节点"""
    rclpy.init()
    
    try:
        client = SynchronizedRobotClient(robot_id)
        
        print(f"🤖 Robot client {robot_id} started")
        print("Waiting for commands from coordinator...")
        print("Press Ctrl+C to stop")
        
        rclpy.spin(client)
        
    except KeyboardInterrupt:
        print(f"\n🛑 Robot {robot_id} shutting down...")
    finally:
        client.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Distributed Synchronized Robot Controller')
    parser.add_argument('mode', choices=['coordinator', 'client'], 
                       help='Run as coordinator or client')
    parser.add_argument('--robot-id', type=str, default='robot1',
                       help='Robot ID for client mode (default: robot1)')
    
    args = parser.parse_args()
    
    if args.mode == 'coordinator':
        run_coordinator()
    elif args.mode == 'client':
        run_client(args.robot_id)


if __name__ == '__main__':
    main()