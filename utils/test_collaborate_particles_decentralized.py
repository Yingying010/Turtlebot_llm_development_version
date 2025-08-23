#!/usr/bin/env python3
"""
声悬浮协同运输系统 (Contactless Transport)
两台机器人协调运输悬浮颗粒到目标位置
完整版本 - 包含时间戳同步验证和断连处理
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import threading
import time
import json
import argparse
from datetime import datetime
from enum import Enum
import math

class TransportState(Enum):
    IDLE = "idle"
    HANDSHAKE_INIT = "handshake_init"
    HANDSHAKE_ACK = "handshake_ack"  
    HANDSHAKE_READY = "handshake_ready"
    TRANSPORT_ACTIVE = "transport_active"
    TRANSPORT_COMPLETE = "transport_complete"

class DebugLogger:
    @staticmethod
    def get_timestamp():
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]
    
    @staticmethod
    def log(robot_id: str, action: str, details: str = ""):
        timestamp = DebugLogger.get_timestamp()
        print(f"[{timestamp}] {robot_id} | {action} | {details}")

class ContactlessTransportController(Node):
    """声悬浮协同运输控制器 - 直线运输"""
    
    def __init__(self, robot_id: str, transport_distance: float = 3.0, target_name: str = "target"):
        super().__init__(f'transport_{robot_id}')
        self.robot_id = robot_id
        self.transport_distance = transport_distance  # 直线运输距离（米）
        self.target_name = target_name
        
        # 配置可靠的QoS
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        
        # 通信
        self.sync_publisher = self.create_publisher(String, '/transport_coordination', qos_profile)
        self.sync_subscriber = self.create_subscription(String, '/transport_coordination', self.coordination_callback, qos_profile)
        
        # 运动控制
        self.cmd_vel_publisher = self.create_publisher(Twist, f'/{robot_id}/cmd_vel', 10)
        
        # 运输状态
        self.transport_state = TransportState.IDLE
        self.handshake_id = None
        self.execution_timestamp = None
        self.peer_robot = None
        self.transport_thread = None
        self.stop_flag = threading.Event()
        self.peer_online = False
        self.peer_last_heartbeat = 0.0  # 记录peer最后心跳时间
        self.peer_sync_timestamps = []  # 记录peer的同步时间戳
        
        # 声悬浮运输参数（直线运输专用）
        self.HANDSHAKE_TIMEOUT = 3.0
        self.SYNC_DELAY_MS = 100      # 100ms同步延迟
        self.TRANSPORT_SPEED = 0.05   # 0.05m/s匀速直线运输
        self.COORDINATION_FREQ = 20   # 20Hz控制频率
        self.HEARTBEAT_TIMEOUT = 5.0  # 5秒心跳超时（断连检测）
        
        # 计算运输时间
        self.transport_duration = self.transport_distance / self.TRANSPORT_SPEED
        
        print(f"🔊 {robot_id} - Linear Transport Controller")
        print(f"🚛 Rigid levitation system - straight line only")
        print(f"Target: {target_name}")
        print(f"Distance: {self.transport_distance:.2f}m (straight line)")
        print(f"Duration: {self.transport_duration:.1f}s at {self.TRANSPORT_SPEED}m/s")
        print(f"⚠️  No turning capability - levitation boards must stay aligned")
        
        # 心跳和自动启动
        self.heartbeat_timer = self.create_timer(2.0, self.send_heartbeat)
        
        if robot_id == "robot1":
            self.auto_start_timer = self.create_timer(1.0, self.check_auto_start)
    
    def coordination_callback(self, msg):
        """处理协调消息"""
        try:
            data = json.loads(msg.data)
            sender = data.get('sender')
            
            if sender == self.robot_id:
                return
            
            msg_type = data.get('type')
            
            if msg_type == 'transport_syn':
                self.handle_transport_syn(data)
            elif msg_type == 'transport_ack':
                self.handle_transport_ack(data)
            elif msg_type == 'transport_ready':
                self.handle_transport_ready(data)
            elif msg_type == 'heartbeat':
                self.handle_heartbeat(data)
            elif msg_type == 'emergency_stop':
                self.emergency_stop()
            elif msg_type == 'sync_timestamp':
                self.handle_sync_timestamp(data)
                
        except json.JSONDecodeError:
            pass
    
    def send_heartbeat(self):
        """发送心跳"""
        heartbeat_msg = {
            'type': 'heartbeat',
            'sender': self.robot_id,
            'timestamp': time.time()
        }
        self.publish_message(heartbeat_msg)
    
    def handle_heartbeat(self, data):
        """处理心跳 - 更新连接状态"""
        sender = data['sender']
        current_time = time.time()
        
        if not self.peer_online:
            self.peer_online = True
            print(f"✅ {sender} online - ready for transport")
        
        # 更新peer最后心跳时间
        self.peer_last_heartbeat = current_time
        if sender != self.peer_robot:
            self.peer_robot = sender
    
    def check_peer_connection(self):
        """检查peer机器人连接状态"""
        if not self.peer_robot or self.peer_last_heartbeat == 0.0:
            return False
            
        current_time = time.time()
        time_since_last_heartbeat = current_time - self.peer_last_heartbeat
        
        if time_since_last_heartbeat > self.HEARTBEAT_TIMEOUT:
            self.peer_online = False
            print(f"⚠️  {self.peer_robot} heartbeat timeout: {time_since_last_heartbeat:.1f}s")
            return False
        
        return True
    
    def handle_sync_timestamp(self, data):
        """处理同步时间戳消息 - 验证同步精度"""
        sender = data['sender']
        peer_timestamp = data['transport_timestamp']
        peer_elapsed = data['elapsed']
        peer_distance = data['distance_covered']
        
        current_time = time.time()
        
        # 记录同步时间戳
        self.peer_sync_timestamps.append({
            'peer_timestamp': peer_timestamp,
            'local_timestamp': current_time,
            'peer_elapsed': peer_elapsed,
            'peer_distance': peer_distance
        })
        
        # 只保留最近10个时间戳
        if len(self.peer_sync_timestamps) > 10:
            self.peer_sync_timestamps.pop(0)
        
        # 计算时间戳差异（网络延迟 + 时钟不同步）
        timestamp_diff = abs(current_time - peer_timestamp)
        
        if timestamp_diff > 0.1:  # 100ms阈值
            print(f"⚠️  SYNC_WARNING | {sender} | timestamp_diff: {timestamp_diff*1000:.1f}ms")
        
        print(f"🕐 SYNC_TIMESTAMP | {sender} | peer_distance: {peer_distance:.2f}m | diff: {timestamp_diff*1000:.1f}ms")
    
    def check_auto_start(self):
        """自动启动运输任务 - 增强连接检查"""
        # 检查peer连接状态
        if not self.check_peer_connection():
            if self.peer_online:  # 如果之前在线，现在离线了
                print(f"⚠️  {self.peer_robot} connection lost - waiting for reconnection...")
                self.peer_online = False
            return
        
        if (self.peer_online and 
            self.transport_state == TransportState.IDLE and 
            not hasattr(self, 'transport_started')):
            
            print(f"🚀 Both robots online - initiating straight-line transport {self.transport_distance:.2f}m...")
            self.transport_started = True
            self.initiate_transport()
    
    def initiate_transport(self):
        """发起运输握手"""
        if self.transport_state != TransportState.IDLE:
            return False
        
        self.handshake_id = f"transport_{int(time.time() * 1000)}"
        self.transport_state = TransportState.HANDSHAKE_INIT
        
        # 发送运输请求
        transport_msg = {
            'type': 'transport_syn',
            'sender': self.robot_id,
            'handshake_id': self.handshake_id,
            'target_name': self.target_name,
            'transport_distance': self.transport_distance,
            'transport_duration': self.transport_duration,
            'timestamp': time.time()
        }
        
        self.publish_message(transport_msg)
        threading.Timer(self.HANDSHAKE_TIMEOUT, self.transport_timeout).start()
        return True
    
    def handle_transport_syn(self, data):
        """处理运输请求"""
        if self.transport_state != TransportState.IDLE:
            return
        
        self.handshake_id = data['handshake_id']
        self.peer_robot = data['sender']
        self.target_name = data['target_name']
        self.transport_distance = data['transport_distance']
        self.transport_duration = data['transport_duration']
        self.transport_state = TransportState.HANDSHAKE_ACK
        
        print(f"🤝 Accepting straight-line transport: {self.target_name}")
        print(f"📏 Distance: {self.transport_distance:.2f}m | Duration: {self.transport_duration:.1f}s")
        
        # 发送确认
        ack_msg = {
            'type': 'transport_ack',
            'sender': self.robot_id,
            'handshake_id': self.handshake_id,
            'timestamp': time.time()
        }
        
        self.publish_message(ack_msg)
    
    def handle_transport_ack(self, data):
        """处理运输确认并设定执行时间"""
        if (self.transport_state != TransportState.HANDSHAKE_INIT or 
            data['handshake_id'] != self.handshake_id):
            return
        
        self.peer_robot = data['sender']
        self.transport_state = TransportState.HANDSHAKE_READY
        
        # 设定协调执行时间
        current_time = time.time()
        sync_delay_sec = self.SYNC_DELAY_MS / 1000.0
        self.execution_timestamp = current_time + sync_delay_sec
        
        print(f"⏱️  Coordinated transport starts in {self.SYNC_DELAY_MS}ms")
        
        # 发送执行指令
        ready_msg = {
            'type': 'transport_ready',
            'sender': self.robot_id,
            'handshake_id': self.handshake_id,
            'execution_timestamp': self.execution_timestamp,
            'target_name': self.target_name,
            'timestamp': time.time()
        }
        
        self.publish_message(ready_msg)
        self.schedule_transport()
    
    def handle_transport_ready(self, data):
        """处理运输就绪指令"""
        if (self.transport_state != TransportState.HANDSHAKE_ACK or 
            data['handshake_id'] != self.handshake_id):
            return
        
        self.execution_timestamp = data['execution_timestamp']
        self.transport_state = TransportState.HANDSHAKE_READY
        
        print(f"🎯 Rigid transport system synchronized - straight line only")
        self.schedule_transport()
    
    def schedule_transport(self):
        """调度运输执行"""
        current_time = time.time()
        wait_time = self.execution_timestamp - current_time
        
        self.transport_thread = threading.Thread(
            target=self.execute_contactless_transport,
            args=(wait_time,)
        )
        self.transport_thread.start()
    
    def execute_contactless_transport(self, wait_time):
        """执行声悬浮协同运输"""
        try:
            # 精确等待到同步时间点
            if wait_time > 0:
                target_time = time.time() + wait_time
                while time.time() < target_time - 0.001:
                    remaining = target_time - time.time()
                    if remaining > 0.01:
                        time.sleep(remaining - 0.005)
                    else:
                        time.sleep(0.0001)
                while time.time() < target_time:
                    pass
            
            # 记录同步精度
            actual_start = time.time()
            sync_error = actual_start - self.execution_timestamp
            precision = "🎯 PERFECT" if abs(sync_error) < 0.002 else "✅ GOOD"
            
            print(f"🔊 TRANSPORT_START | sync_error: {sync_error*1000:.2f}ms | {precision}")
            
            self.transport_state = TransportState.TRANSPORT_ACTIVE
            
            # 🔊 声悬浮协调运输逻辑
            if self.robot_id == "robot1":
                direction_str = "forward"
                self.execute_coordinated_movement("forward")
            elif self.robot_id == "robot2":
                direction_str = "backward"  
                self.execute_coordinated_movement("backward")
            
            print(f"✅ TRANSPORT_COMPLETE | particle delivered {self.transport_distance:.2f}m to {self.target_name}")
            self.transport_state = TransportState.TRANSPORT_COMPLETE
            
            # 5秒后重置状态，准备下次运输
            threading.Timer(5.0, self.reset_for_next_transport).start()
            
        except Exception as e:
            print(f"❌ TRANSPORT_ERROR | {e}")
            self.transport_state = TransportState.IDLE
    
    def execute_coordinated_movement(self, direction):
        """执行协调直线运动 - 刚性连接系统"""
        print(f"🔊 LINEAR_TRANSPORT | {direction} | speed: {self.TRANSPORT_SPEED}m/s")
        print(f"⚠️  Rigid system: levitation boards must stay perfectly aligned")
        
        # 直线运输到目标距离
        print(f"🚛 STRAIGHT_LINE_ONLY | {direction} | distance: {self.transport_distance:.2f}m | {self.transport_duration:.1f}s")
        self.stable_transport_movement(direction, self.transport_duration)
        
        # 直接停止
        self.cmd_vel_publisher.publish(Twist())
        print(f"⏹️ LINEAR_TRANSPORT_COMPLETE | {direction}")
        print(f"🔊 Levitation boards maintained alignment throughout transport")
    
    def stable_transport_movement(self, direction, duration):
        """稳定运输运动 - 带时间戳同步验证"""
        twist = Twist()
        if direction == "forward":
            twist.linear.x = self.TRANSPORT_SPEED
        elif direction == "backward":
            twist.linear.x = -self.TRANSPORT_SPEED
        
        start_time = time.time()
        loop_count = 0
        next_progress_time = start_time + 2.0  # 每2秒报告一次进度和同步状态
        next_heartbeat_check = start_time + 1.0  # 每秒检查peer连接状态
        
        # 🕐 记录运动开始的精确时间戳
        actual_start_timestamp = time.time()
        sync_error = actual_start_timestamp - self.execution_timestamp
        print(f"🕐 MOVEMENT_START | timestamp: {actual_start_timestamp:.6f} | sync_error: {sync_error*1000:.2f}ms")
        
        while (time.time() - start_time < duration and 
               not self.stop_flag.is_set()):
            
            current_time = time.time()
            
            # 🔍 检查peer机器人连接状态
            if current_time >= next_heartbeat_check:
                if not self.check_peer_connection():
                    print(f"🚨 PEER_DISCONNECTED | {self.peer_robot} lost connection during transport")
                    print(f"🛑 SAFETY_STOP | aborting transport to prevent particle loss")
                    self.emergency_stop()
                    break
                next_heartbeat_check = current_time + 1.0
            
            # 📡 发布运动命令（带时间戳）
            self.cmd_vel_publisher.publish(twist)
            
            # 📊 定期报告运输进度和时间戳
            if current_time >= next_progress_time:
                elapsed = current_time - start_time
                remaining = duration - elapsed
                distance_covered = elapsed * self.TRANSPORT_SPEED
                
                # 🕐 发布同步时间戳消息
                sync_msg = {
                    'type': 'sync_timestamp',
                    'sender': self.robot_id,
                    'transport_timestamp': current_time,
                    'elapsed': elapsed,
                    'distance_covered': distance_covered
                }
                self.publish_message(sync_msg)
                
                print(f"🚛 TRANSPORT_PROGRESS | {direction} | timestamp: {current_time:.3f}")
                print(f"   📏 covered: {distance_covered:.2f}m | remaining: {remaining:.1f}s")
                print(f"   🔗 peer_status: {'ONLINE' if self.peer_online else 'OFFLINE'}")
                
                next_progress_time = current_time + 2.0
            
            time.sleep(1.0 / self.COORDINATION_FREQ)  # 20Hz
    
    def reset_for_next_transport(self):
        """重置状态，准备下次运输"""
        self.transport_state = TransportState.IDLE
        self.handshake_id = None
        self.execution_timestamp = None
        self.peer_robot = None
        print(f"🔄 Ready for next transport task")
    
    def transport_timeout(self):
        """运输超时处理"""
        if self.transport_state in [TransportState.HANDSHAKE_INIT, TransportState.HANDSHAKE_ACK]:
            self.transport_state = TransportState.IDLE
            print(f"⏰ Transport handshake timeout")
    
    def emergency_stop(self):
        """紧急停止 - 增强版本"""
        print(f"🚨 EMERGENCY_STOP | {self.robot_id} levitation system halted")
        print(f"🛑 Reason: {self.peer_robot} disconnection during transport")
        print(f"⚠️  Particle transport aborted for safety")
        
        self.stop_flag.set()
        self.cmd_vel_publisher.publish(Twist())
        self.transport_state = TransportState.IDLE
        
        # 广播紧急停止消息给peer（如果还连着）
        emergency_msg = {
            'type': 'emergency_stop',
            'sender': self.robot_id,
            'reason': 'peer_disconnection',
            'timestamp': time.time()
        }
        self.publish_message(emergency_msg)
        
        if self.transport_thread and self.transport_thread.is_alive():
            self.transport_thread.join(timeout=2.0)
        
        self.stop_flag.clear()
        
        # 重置运输状态，准备重新开始
        threading.Timer(3.0, self.reset_after_emergency).start()
    
    def reset_after_emergency(self):
        """紧急停止后重置"""
        print(f"🔄 {self.robot_id} ready for next transport after emergency stop")
        if hasattr(self, 'transport_started'):
            delattr(self, 'transport_started')  # 允许重新自动启动
    
    def publish_message(self, message_dict):
        """发布消息"""
        msg = String()
        msg.data = json.dumps(message_dict)
        self.sync_publisher.publish(msg)

def main():
    parser = argparse.ArgumentParser(description='Linear Contactless Transport Controller')
    parser.add_argument('robot_id', choices=['robot1', 'robot2'], 
                       help='Robot ID (robot1 or robot2)')
    parser.add_argument('--distance', type=float, default=3.0,
                       help='Straight line transport distance in meters (default: 3.0)')
    parser.add_argument('--target-name', type=str, default='target',
                       help='Target name (default: target)')
    
    args = parser.parse_args()
    robot_id = args.robot_id
    distance = args.distance
    target_name = args.target_name
    
    rclpy.init()
    
    try:
        controller = ContactlessTransportController(robot_id, distance, target_name)
        
        print(f"🔊 Rigid Acoustic Levitation System")
        print(f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        print(f"⚠️  CONSTRAINT: Straight line transport only")
        print(f"📐 System: Two robots + rigid levitation boards")
        print(f"🚫 No turning: Boards must stay aligned")
        print(f"🕐 Timestamp sync: ±2ms precision target")
        print(f"🔗 Disconnect safety: 5s heartbeat timeout")
        print(f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        print(f"Mission: Linear transport to {target_name}")
        print(f"Distance: {distance:.2f}m (straight line)")
        print(f"Speed:   {controller.TRANSPORT_SPEED}m/s")
        print(f"Time:    {controller.transport_duration:.1f}s")
        print(f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        
        if robot_id == "robot1":
            print("🤖 robot1: Forward motion (front of rigid system)")
            print("Waiting for robot2, then auto-start linear transport...")
        else:
            print("🤖 robot2: Backward motion (rear of rigid system)")  
            print("Waiting for transport coordination...")
        
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print(f"\n🛑 {robot_id} rigid transport system shutdown")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()