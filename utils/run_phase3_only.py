#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fixed Phase3: Three-Way Handshake Transport with SYN Retry
修复了SYN重试机制，解决启动顺序问题

运行方式:
  robot1: python3 fixed_phase3_transport.py --robot_id robot1 --distance 1000
  robot2: python3 fixed_phase3_transport.py --robot_id robot2 --distance 1000
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

# ===== 参数配置 =====
SPEED = 0.05  # 移动速度 m/s
HANDSHAKE_TIMEOUT_SEC = 60.0    # 总超时时间
SYN_RETRY_INTERVAL = 1.0        # SYN重试间隔
TRANSPORT_DELAY_SEC = 3.0       # GO信号后延迟启动时间

# ===== 消息类型 =====
class MsgType(Enum):
    SYN = "syn"
    SYN_ACK = "syn_ack" 
    ACK = "ack"
    GO = "go"
    ABORT = "abort"

class State(Enum):
    INIT = "init"
    SYN_SENT = "syn_sent"
    SYN_ACK_SENT = "syn_ack_sent"
    ESTABLISHED = "established"
    CLOSED = "closed"

# ===== 运动控制 =====
class Motion:
    def __init__(self, node: Node, robot_ns: str):
        self.node = node
        self.pub = node.create_publisher(Twist, f'/{robot_ns}/cmd_vel', 10)

    def stop(self):
        self.pub.publish(Twist())

    def drive_constant(self, forward: bool, dist_mm: float, speed_mps: float):
        """匀速直线运动"""
        dist_m = dist_mm / 1000.0
        duration = dist_m / max(speed_mps, 1e-6)

        twist = Twist()
        twist.linear.x = speed_mps if forward else -speed_mps

        logger.info(f"{'Forward' if forward else 'Backward'}: {dist_mm}mm in {duration:.2f}s at {speed_mps}m/s")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.pub.publish(twist)
            time.sleep(0.02)
        
        self.stop()
        logger.info("Motion completed")

# ===== 三次握手运输管理器 =====
class FixedHandshakeTransport:
    def __init__(self, node: Node, robot_id: str, distance_mm: float):
        self.node = node
        self.robot_id = robot_id
        self.peer_id = "robot1" if robot_id == "robot2" else "robot2"
        self.distance_mm = distance_mm
        
        self.is_initiator = (robot_id == "robot1")
        self.completed = False
        self.success = False
        self.aborted = False
        
        # 状态管理
        self.state = State.INIT
        self.state_lock = threading.Lock()
        
        # 序列号
        self.seq_num = int(time.time() * 1000) % 10000
        self.peer_seq_num = None
        
        # 启动时间
        self.start_time = None
        
        # 通信
        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub = node.create_publisher(String, "/handshake_fixed", qos)
        self.sub = node.create_subscription(String, "/handshake_fixed", self.on_message, qos)
        
        # 同步事件
        self.syn_ack_received = threading.Event()
        self.ack_received = threading.Event()
        self.go_received = threading.Event()
        self.abort_received = threading.Event()
        
        # 运动控制
        self.motion = Motion(node, robot_id)
        
        logger.info(f"{robot_id} - {'INITIATOR' if self.is_initiator else 'RESPONDER'}")
        
        # 启动工作线程
        self.worker = threading.Thread(target=self._main_worker, daemon=True)
        self.worker.start()

    def _set_state(self, new_state: State):
        with self.state_lock:
            old_state = self.state
            self.state = new_state
            logger.info(f"{self.robot_id}: {old_state.value} → {new_state.value}")

    def _publish(self, msg_type: MsgType, **data):
        msg_data = {
            "type": msg_type.value,
            "from": self.robot_id,
            "to": self.peer_id,
            "seq": self.seq_num,
            "timestamp": time.time(),
            **data
        }
        
        msg = String()
        msg.data = json.dumps(msg_data)
        self.pub.publish(msg)
        
        logger.info(f"{self.robot_id}: Sent {msg_type.value} (seq={self.seq_num})")

    def _abort(self, reason: str):
        if not self.aborted:
            self.aborted = True
            self._publish(MsgType.ABORT, reason=reason)
            logger.error(f"{self.robot_id}: ABORT - {reason}")
            self._set_state(State.CLOSED)
        
        self.motion.stop()
        # 唤醒所有等待的线程
        self.syn_ack_received.set()
        self.ack_received.set()
        self.go_received.set()
        self.abort_received.set()

    def on_message(self, msg: String):
        try:
            data = json.loads(msg.data)
        except:
            return
        
        msg_type_str = data.get("type")
        from_robot = data.get("from")
        to_robot = data.get("to")
        seq = data.get("seq")
        
        # 过滤消息
        if from_robot == self.robot_id:
            return
        if to_robot != self.robot_id and msg_type_str != MsgType.ABORT.value:
            return
        
        try:
            msg_type = MsgType(msg_type_str)
        except ValueError:
            return
        
        logger.info(f"{self.robot_id}: Received {msg_type.value} from {from_robot} (seq={seq})")
        
        # 处理消息
        if msg_type == MsgType.SYN:
            self._handle_syn(seq, data)
        elif msg_type == MsgType.SYN_ACK:
            self._handle_syn_ack(seq, data)
        elif msg_type == MsgType.ACK:
            self._handle_ack(seq, data)
        elif msg_type == MsgType.GO:
            self._handle_go(data)
        elif msg_type == MsgType.ABORT:
            self._handle_abort(data)

    def _handle_syn(self, seq: int, data: dict):
        """处理SYN消息"""
        if self.state in [State.INIT, State.SYN_ACK_SENT]:
            self.peer_seq_num = seq
            logger.info(f"{self.robot_id}: SYN received, sending SYN_ACK")
            
            # 发送SYN_ACK
            self._publish(MsgType.SYN_ACK, ack_seq=seq, distance=self.distance_mm)
            self._set_state(State.SYN_ACK_SENT)

    def _handle_syn_ack(self, seq: int, data: dict):
        """处理SYN_ACK消息"""
        ack_seq = data.get("ack_seq")
        if self.state == State.SYN_SENT and ack_seq == self.seq_num:
            self.peer_seq_num = seq
            logger.info(f"{self.robot_id}: SYN_ACK received (ack_seq={ack_seq})")
            self.syn_ack_received.set()

    def _handle_ack(self, seq: int, data: dict):
        """处理ACK消息"""
        ack_seq = data.get("ack_seq")
        if self.state == State.SYN_ACK_SENT and ack_seq == self.seq_num:
            logger.info(f"{self.robot_id}: ACK received (ack_seq={ack_seq})")
            self._set_state(State.ESTABLISHED)
            self.ack_received.set()

    def _handle_go(self, data: dict):
        """处理GO消息"""
        self.start_time = data.get("start_time")
        logger.info(f"🚦 {self.robot_id}: GO received, start_time={self.start_time}")
        self.go_received.set()

    def _handle_abort(self, data: dict):
        """处理ABORT消息"""
        reason = data.get("reason", "Unknown")
        logger.warning(f"{self.robot_id}: Received ABORT - {reason}")
        self.aborted = True
        self.motion.stop()
        self._set_state(State.CLOSED)
        self.abort_received.set()

    def _main_worker(self):
        """主工作流程"""
        try:
            logger.info(f"{self.robot_id}: Starting handshake transport")
            
            # 执行三次握手
            if not self._do_handshake():
                return
            
            if self.aborted:
                return
            
            # 执行同步运输
            self._do_transport()
            
        except Exception as e:
            logger.error(f"{self.robot_id}: Worker error - {e}")
            self._abort(f"Worker error: {e}")
        finally:
            self.completed = True
            logger.info(f"{self.robot_id}: Worker finished")

    def _do_handshake(self) -> bool:
        """执行三次握手"""
        start_time = time.time()
        
        if self.is_initiator:
            logger.info(f"{self.robot_id}: INITIATOR - Starting handshake")
            
            # 第1步：重复发送SYN
            self._set_state(State.SYN_SENT)
            last_syn_time = 0
            
            logger.info(f"{self.robot_id}: Sending SYN with retries...")
            
            while not self.syn_ack_received.is_set() and not self.aborted:
                current_time = time.time()
                
                # 超时检查
                if current_time - start_time > HANDSHAKE_TIMEOUT_SEC:
                    self._abort("Handshake timeout")
                    return False
                
                # 重发SYN
                if current_time - last_syn_time >= SYN_RETRY_INTERVAL:
                    self._publish(MsgType.SYN, distance=self.distance_mm)
                    elapsed = current_time - start_time
                    logger.info(f"{self.robot_id}: SYN retry (elapsed: {elapsed:.1f}s)")
                    last_syn_time = current_time
                
                # 等待SYN_ACK
                if self.syn_ack_received.wait(timeout=0.2):
                    break
            
            if self.aborted:
                return False
            
            logger.info(f"{self.robot_id}: SYN_ACK received!")
            
            # 第3步：发送ACK
            logger.info(f"{self.robot_id}: Sending ACK")
            self._publish(MsgType.ACK, ack_seq=self.peer_seq_num)
            self._set_state(State.ESTABLISHED)
            
        else:
            logger.info(f"{self.robot_id}: RESPONDER - Waiting for handshake")
            
            # 等待ACK (SYN和SYN_ACK在消息处理中自动完成)
            logger.info(f"{self.robot_id}: Waiting for ACK...")
            
            timeout_remaining = HANDSHAKE_TIMEOUT_SEC - (time.time() - start_time)
            if not self.ack_received.wait(timeout=max(1.0, timeout_remaining)):
                self._abort("Waiting ACK timeout")
                return False
            
            if self.aborted:
                return False
        
        logger.info(f"{self.robot_id}: Handshake completed!")
        return True

    def _do_transport(self):
        """执行同步运输"""
        if self.is_initiator:
            # 发送GO信号
            self.start_time = time.time() + TRANSPORT_DELAY_SEC
            logger.info(f"{self.robot_id}: Sending GO signal")
            logger.info(f"Start time: {self.start_time}")
            
            self._publish(MsgType.GO, start_time=self.start_time, distance=self.distance_mm)
            logger.info(f"GO signal sent, starting in {TRANSPORT_DELAY_SEC} seconds")
            
        else:
            # 等待GO信号
            logger.info(f"{self.robot_id}: Waiting for GO signal...")
            if not self.go_received.wait(timeout=15.0):
                self._abort("GO signal timeout")
                return
        
        if self.aborted:
            return
        
        # 等待启动时间
        logger.info(f"{self.robot_id}: Waiting for start time...")
        while time.time() < self.start_time:
            remaining = self.start_time - time.time()
            if remaining <= 0:
                break
            sleep_time = 0.001 if remaining < 0.01 else min(0.01, remaining / 2)
            time.sleep(sleep_time)
        
        # 开始运输
        forward = self.is_initiator  # initiator前进，responder后退
        
        logger.info(f"{self.robot_id}: Starting transport!")
        logger.info(f"Direction: {'FORWARD' if forward else 'BACKWARD'}")
        logger.info(f"Distance: {self.distance_mm}mm")
        logger.info(f"Speed: {SPEED}m/s")
        
        # 执行运动
        self.motion.drive_constant(forward, self.distance_mm, SPEED)
        
        logger.info(f"{self.robot_id}: Transport completed!")
        self.success = True

    def wait_for_completion(self, timeout: float = 120.0) -> bool:
        """等待完成"""
        start_time = time.time()
        while not self.completed and time.time() - start_time < timeout:
            time.sleep(0.1)
        return self.success

def main():
    parser = argparse.ArgumentParser(description="Fixed Phase3 Handshake Transport")
    parser.add_argument("--robot_id", required=True, choices=["robot1", "robot2"])
    parser.add_argument("--distance", type=float, default=1000.0)
    parser.add_argument("--speed", type=float, default=0.05)
    
    args = parser.parse_args()

    global SPEED
    SPEED = args.speed

    logger.info(f"Starting FIXED Handshake Transport")
    logger.info(f"Robot: {args.robot_id} ({'INITIATOR' if args.robot_id == 'robot1' else 'RESPONDER'})")
    logger.info(f"Distance: {args.distance}mm")
    logger.info(f"Speed: {SPEED}m/s")

    # 初始化ROS
    rclpy.init()
    node = rclpy.create_node(f"fixed_handshake_{args.robot_id}")
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # 创建运输管理器
    transport = FixedHandshakeTransport(node, args.robot_id, args.distance)

    try:
        # 后台运行executor
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        # 等待完成
        logger.info("Waiting for completion...")
        success = transport.wait_for_completion(timeout=180.0)

        if success:
            logger.info("FIXED handshake transport SUCCESS!")
            print("SUCCESS")
        else:
            logger.error("FIXED handshake transport FAILED!")
            print("FAILED")

    except KeyboardInterrupt:
        logger.info("Interrupted")
        transport._abort("User interrupt")
    finally:
        transport.motion.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()