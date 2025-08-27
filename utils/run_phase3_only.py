#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase3 Only: Distributed Synchronized Transport with Three-Way Handshake
使用标准三次握手协议确保可靠同步

三次握手流程:
1. SYN: robot1 → robot2 "我想开始运输"
2. SYN-ACK: robot2 → robot1 "我也准备好了，确认开始"  
3. ACK: robot1 → robot2 "收到确认，开始执行"

运行方式:
  robot1 (initiator):
    python3 phase3_only_transport.py --robot_id robot1 --distance 1000
  robot2 (responder):
    python3 phase3_only_transport.py --robot_id robot2 --distance 1000
"""

import math
import time
import json
import argparse
import threading
from loguru import logger
from enum import Enum

import os, sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# ===== 运输参数 =====
SPEED = 0.02  # 移动速度 m/s
HANDSHAKE_TIMEOUT_SEC = 30.0    # 三次握手总超时时间
TRANSPORT_DELAY_SEC = 2.0       # GO信号后延迟启动时间

# ===== 三次握手消息类型 =====
class HandshakeMsg(Enum):
    SYN = "syn"           # 第1步：发起方请求开始
    SYN_ACK = "syn_ack"   # 第2步：接收方确认并请求同步
    ACK = "ack"           # 第3步：发起方最终确认
    GO = "go"             # 第4步：开始执行信号
    ABORT = "abort"       # 中止信号

class HandshakeState(Enum):
    INIT = "init"
    SYN_SENT = "syn_sent"      # 已发送SYN，等待SYN_ACK
    SYN_RECEIVED = "syn_recv"  # 已收到SYN，准备发送SYN_ACK  
    SYN_ACK_SENT = "syn_ack_sent"  # 已发送SYN_ACK，等待ACK
    ACK_RECEIVED = "ack_recv"  # 已收到ACK，握手完成
    ESTABLISHED = "established" # 连接建立，准备传输
    CLOSED = "closed"

# ===== 直线运动控制 =====
class Motion:
    def __init__(self, node: Node, robot_ns: str):
        self.node = node
        self.pub = node.create_publisher(Twist, f'/{robot_ns}/cmd_vel', 10)

    def stop(self):
        """停止运动"""
        self.pub.publish(Twist())

    def drive_constant(self, forward: bool, dist_mm: float, speed_mps: float):
        """
        匀速直线运动
        :param forward: True=前进, False=后退
        :param dist_mm: 移动距离，单位：毫米（mm）
        :param speed_mps: 移动速度，单位：米每秒（m/s）
        """
        dist_m = dist_mm / 1000.0  # 转换为米
        duration = dist_m / max(speed_mps, 1e-6)

        twist = Twist()
        twist.linear.x = speed_mps if forward else -speed_mps

        logger.info(f"🚗 {'Forward' if forward else 'Backward'} motion: {dist_mm}mm in {duration:.2f}s at {speed_mps}m/s")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.pub.publish(twist)
            time.sleep(0.02)  # 50Hz发布频率
        
        self.stop()
        logger.info("✅ Motion completed")

# ===== 三次握手运输管理器 =====
class ThreeWayHandshakeTransport:
    """
    实现标准三次握手协议的分布式同步运输
    """
    def __init__(self, node: Node, robot_id: str, distance_mm: float):
        self.node = node
        self.robot_id = robot_id
        self.peer_id = "robot1" if robot_id == "robot2" else "robot2"
        self.distance_mm = distance_mm
        
        # robot1是发起方(initiator)，robot2是响应方(responder)
        self.is_initiator = (robot_id == "robot1")
        self.is_leader = self.is_initiator  # 发起方也是运动领导者(前进)
        
        self.completed = False
        self.success = False
        self.aborted = False

        # 握手状态
        self.state = HandshakeState.INIT
        self.state_lock = threading.Lock()

        # 通信设置
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub = node.create_publisher(String, "/three_way_handshake", qos)
        self.sub = node.create_subscription(String, "/three_way_handshake", self.on_message, qos)

        # 握手同步事件
        self.syn_ack_received = threading.Event()  # 收到SYN_ACK
        self.ack_received = threading.Event()      # 收到ACK
        self.go_received = threading.Event()       # 收到GO信号
        self.abort_received = threading.Event()    # 收到ABORT

        # 运输开始时间
        self.start_time = None

        # 运动控制
        self.motion = Motion(node, robot_id)

        # 序列号（防重放）
        self.seq_num = int(time.time() * 1000) % 10000
        self.peer_seq_num = None

        logger.info(f"🤖 {robot_id} initialized as {'INITIATOR (leader/forward)' if self.is_initiator else 'RESPONDER (follower/backward)'}")

        # 启动工作线程
        self.worker = threading.Thread(target=self._execute_handshake_transport, daemon=True)
        self.worker.start()

    def _set_state(self, new_state: HandshakeState):
        """线程安全的状态更新"""
        with self.state_lock:
            old_state = self.state
            self.state = new_state
            logger.info(f"🔄 State: {old_state.value} → {new_state.value}")

    def _publish_message(self, msg_type: HandshakeMsg, **kwargs):
        """发布握手消息"""
        msg_data = {
            "type": msg_type.value,
            "from": self.robot_id,
            "to": self.peer_id,
            "seq": self.seq_num,
            "timestamp": time.time(),
            **kwargs
        }
        msg = String()
        msg.data = json.dumps(msg_data)
        self.pub.publish(msg)
        logger.info(f"📤 Sent {msg_type.value} (seq={self.seq_num}) to {self.peer_id}")

    def _abort(self, reason: str):
        """中止握手和运输"""
        if not self.aborted:
            self.aborted = True
            self._publish_message(HandshakeMsg.ABORT, reason=reason)
            logger.error(f"🚨 ABORT: {reason}")
            self._set_state(HandshakeState.CLOSED)
        
        self.motion.stop()
        # 设置所有事件，避免死等
        self.syn_ack_received.set()
        self.ack_received.set() 
        self.go_received.set()
        self.abort_received.set()

    def on_message(self, msg: String):
        """处理接收到的握手消息"""
        try:
            data = json.loads(msg.data)
        except Exception as e:
            logger.warning(f"⚠️ Invalid message: {e}")
            return

        msg_type_str = data.get("type")
        from_robot = data.get("from")
        to_robot = data.get("to")
        seq_num = data.get("seq")

        # 过滤消息
        if from_robot == self.robot_id:  # 忽略自己的消息
            return
        if to_robot != self.robot_id and msg_type_str != HandshakeMsg.ABORT.value:  # 忽略不是发给自己的消息
            return

        try:
            msg_type = HandshakeMsg(msg_type_str)
        except ValueError:
            logger.warning(f"⚠️ Unknown message type: {msg_type_str}")
            return

        logger.info(f"📥 Received {msg_type.value} from {from_robot} (seq={seq_num})")

        # 处理不同类型的消息
        if msg_type == HandshakeMsg.SYN:
            self._handle_syn(from_robot, seq_num, data)
        elif msg_type == HandshakeMsg.SYN_ACK:
            self._handle_syn_ack(from_robot, seq_num, data)
        elif msg_type == HandshakeMsg.ACK:
            self._handle_ack(from_robot, seq_num, data)
        elif msg_type == HandshakeMsg.GO:
            self._handle_go(from_robot, data)
        elif msg_type == HandshakeMsg.ABORT:
            self._handle_abort(from_robot, data)

    def _handle_syn(self, from_robot: str, seq_num: int, data: dict):
        """处理SYN消息（第1步）"""
        if self.state == HandshakeState.INIT:
            self.peer_seq_num = seq_num
            logger.info(f"✅ SYN received, sending SYN_ACK")
            
            self._set_state(HandshakeState.SYN_RECEIVED)
            # 发送SYN_ACK响应
            self._publish_message(HandshakeMsg.SYN_ACK, 
                                 ack_seq=seq_num, 
                                 distance=self.distance_mm)
            self._set_state(HandshakeState.SYN_ACK_SENT)

    def _handle_syn_ack(self, from_robot: str, seq_num: int, data: dict):
        """处理SYN_ACK消息（第2步）"""
        ack_seq = data.get("ack_seq")
        if (self.state == HandshakeState.SYN_SENT and 
            ack_seq == self.seq_num):
            
            self.peer_seq_num = seq_num
            logger.info(f"✅ SYN_ACK received with correct ack_seq={ack_seq}")
            self.syn_ack_received.set()

    def _handle_ack(self, from_robot: str, seq_num: int, data: dict):
        """处理ACK消息（第3步）"""
        ack_seq = data.get("ack_seq")
        if (self.state == HandshakeState.SYN_ACK_SENT and 
            ack_seq == self.seq_num):
            
            logger.info(f"✅ ACK received with correct ack_seq={ack_seq}")
            self._set_state(HandshakeState.ACK_RECEIVED)
            self.ack_received.set()

    def _handle_go(self, from_robot: str, data: dict):
        """处理GO信号（第4步）"""
        self.start_time = data.get("start_time")
        self.distance_mm = data.get("distance", self.distance_mm)
        logger.info(f"🚦 GO signal received, start_time: {self.start_time}")
        self._set_state(HandshakeState.ESTABLISHED)
        self.go_received.set()

    def _handle_abort(self, from_robot: str, data: dict):
        """处理ABORT信号"""
        reason = data.get("reason", "Unknown")
        logger.warning(f"🚨 Received ABORT from {from_robot}: {reason}")
        self.aborted = True
        self._set_state(HandshakeState.CLOSED)
        self.motion.stop()
        self.abort_received.set()

    def _execute_handshake_transport(self):
        """执行完整的三次握手+同步运输流程"""
        try:
            logger.info("🎯 Starting Three-Way Handshake Transport Protocol")

            # 执行三次握手
            if not self._perform_three_way_handshake():
                return

            if self.aborted:
                return

            # 握手成功，开始同步运输
            self._perform_synchronized_transport()

        except Exception as e:
            logger.error(f"💥 Error during handshake transport: {e}")
            self._abort(f"Execution error: {e}")
        finally:
            self.completed = True
            logger.info("🏁 Three-way handshake transport finished")

    def _perform_three_way_handshake(self) -> bool:
        """执行标准三次握手协议"""
        start_time = time.time()
        
        if self.is_initiator:
            # === 发起方流程 ===
            logger.info("🤝 INITIATOR: Starting three-way handshake...")
            
            # 第1步：发送SYN
            logger.info("📤 Step 1/3: Sending SYN...")
            self._set_state(HandshakeState.SYN_SENT)
            self._publish_message(HandshakeMsg.SYN, distance=self.distance_mm)

            # 等待SYN_ACK（第2步响应）
            logger.info("⏳ Step 2/3: Waiting for SYN_ACK...")
            if not self._wait_with_timeout(self.syn_ack_received, 
                                         HANDSHAKE_TIMEOUT_SEC - (time.time() - start_time),
                                         "SYN_ACK"):
                return False

            # 第3步：发送ACK确认
            logger.info("📤 Step 3/3: Sending ACK...")
            self._publish_message(HandshakeMsg.ACK, ack_seq=self.peer_seq_num)
            self._set_state(HandshakeState.ESTABLISHED)

        else:
            # === 响应方流程 ===  
            logger.info("🤝 RESPONDER: Waiting for handshake initiation...")
            
            # 等待ACK（第3步确认）
            logger.info("⏳ Waiting for final ACK...")
            if not self._wait_with_timeout(self.ack_received,
                                         HANDSHAKE_TIMEOUT_SEC - (time.time() - start_time), 
                                         "ACK"):
                return False
                
            self._set_state(HandshakeState.ESTABLISHED)

        logger.info("🎉 Three-way handshake completed successfully!")
        return True

    def _perform_synchronized_transport(self):
        """执行同步运输"""
        if self.is_initiator:
            # 发起方发送GO信号
            self.start_time = time.time() + TRANSPORT_DELAY_SEC
            logger.info(f"🚦 INITIATOR: Sending GO signal, start_time: {self.start_time}")
            self._publish_message(HandshakeMsg.GO, 
                                start_time=self.start_time, 
                                distance=self.distance_mm)
        else:
            # 响应方等待GO信号
            logger.info("⏳ RESPONDER: Waiting for GO signal...")
            if not self._wait_with_timeout(self.go_received, 10.0, "GO signal"):
                return

        if self.aborted:
            return

        # 精确时间同步等待
        if self.start_time:
            while time.time() < self.start_time:
                remaining = self.start_time - time.time()
                if remaining <= 0:
                    break
                sleep_time = 0.0005 if remaining < 0.002 else min(0.01, remaining / 2)
                time.sleep(sleep_time)

        # 开始同步运输
        forward_motion = self.is_leader  # initiator前进，responder后退
        
        logger.info(f"🚀 Starting synchronized transport!")
        logger.info(f"   Role: {'LEADER (forward)' if forward_motion else 'FOLLOWER (backward)'}")
        logger.info(f"   Distance: {self.distance_mm}mm")
        logger.info(f"   Speed: {SPEED}m/s")
        
        # 执行运动
        self.motion.drive_constant(forward_motion, self.distance_mm, SPEED)

        logger.info("🎉 Synchronized transport completed successfully!")
        self.success = True

    def _wait_with_timeout(self, event: threading.Event, timeout: float, event_name: str) -> bool:
        """带超时的事件等待"""
        if timeout <= 0:
            self._abort(f"Overall handshake timeout")
            return False
            
        if not event.wait(timeout=timeout):
            self._abort(f"Timeout waiting for {event_name}")
            return False
        
        if self.aborted:
            return False
            
        return True

    def wait_for_completion(self, timeout: float = 120.0) -> bool:
        """等待运输完成"""
        start_time = time.time()
        while not self.completed and time.time() - start_time < timeout:
            time.sleep(0.1)
        return self.success

# ===== 主程序入口 =====
def main():
    parser = argparse.ArgumentParser(description="Three-Way Handshake Transport")
    parser.add_argument("--robot_id", required=True, choices=["robot1", "robot2"], 
                       help="Robot identifier (robot1=initiator, robot2=responder)")
    parser.add_argument("--distance", type=float, default=1000.0, 
                       help="Transport distance in mm (default: 1000mm)")
    parser.add_argument("--speed", type=float, default=0.05,
                       help="Transport speed in m/s (default: 0.05)")
    
    args = parser.parse_args()

    # 更新全局速度参数
    global SPEED
    SPEED = args.speed

    logger.info(f"🎬 Starting Three-Way Handshake Transport for {args.robot_id}")
    logger.info(f"   Role: {'INITIATOR (SYN sender)' if args.robot_id == 'robot1' else 'RESPONDER (SYN receiver)'}")
    logger.info(f"   Distance: {args.distance}mm")
    logger.info(f"   Speed: {SPEED}m/s")

    # 初始化ROS
    rclpy.init()
    node = rclpy.create_node(f"handshake_transport_{args.robot_id}")
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # 创建运输管理器
    transport_manager = ThreeWayHandshakeTransport(node, args.robot_id, args.distance)

    try:
        # 在后台运行executor
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        # 等待运输完成
        logger.info("⏳ Waiting for handshake transport completion...")
        success = transport_manager.wait_for_completion(timeout=180.0)

        if success:
            logger.info("🎉 Three-way handshake transport completed successfully!")
            print("✅ SUCCESS: Handshake transport completed")
        else:
            logger.error("❌ Three-way handshake transport failed or timed out")
            print("❌ FAILED: Handshake transport failed")

    except KeyboardInterrupt:
        logger.info("🛑 Interrupted by user")
        transport_manager._abort("User interrupt")
    finally:
        transport_manager.motion.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()