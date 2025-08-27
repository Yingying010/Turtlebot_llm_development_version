#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase3 Only: Distributed Synchronized Transport
直接执行分布式同步运输，跳过队形规划和导航阶段

运行方式:
  robot1:
    python3 phase3_only_transport.py --robot_id robot1 --distance 1000
  robot2:
    python3 phase3_only_transport.py --robot_id robot2 --distance 1000
"""

import math
import time
import json
import argparse
import threading
from loguru import logger

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
WAIT_READY_TIMEOUT_SEC = 30.0   # 等对方 READY 的超时
WAIT_GO_TIMEOUT_SEC = 30.0      # follower 等 GO 的超时

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

# ===== 协调消息类型 =====
MSG_READY = "ready"
MSG_GO = "go" 
MSG_ABORT = "abort"

# ===== Phase3 运输管理器 =====
class Phase3TransportManager:
    """
    仅执行Phase3的分布式同步运输
    """
    def __init__(self, node: Node, robot_id: str, distance_mm: float):
        self.node = node
        self.robot_id = robot_id
        self.peer_id = "robot1" if robot_id == "robot2" else "robot2"
        self.distance_mm = distance_mm
        
        # robot1是领导者(前进)，robot2是跟随者(后退)
        self.is_leader = (robot_id == "robot1")
        
        self.completed = False
        self.success = False
        self.aborted = False

        # 通信设置
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub = node.create_publisher(String, "/phase3_transport", qos)
        self.sub = node.create_subscription(String, "/phase3_transport", self.on_message, qos)

        # 同步事件
        self.peer_ready_event = threading.Event()
        self.go_event = threading.Event()
        self.abort_event = threading.Event()

        # 运输开始时间
        self.start_time = None

        # 运动控制
        self.motion = Motion(node, robot_id)

        logger.info(f"🤖 {robot_id} initialized as {'LEADER (forward)' if self.is_leader else 'FOLLOWER (backward)'}")

        # 启动工作线程
        self.worker = threading.Thread(target=self._execute_phase3, daemon=True)
        self.worker.start()

    def _publish_message(self, msg_type: str, **kwargs):
        """发布协调消息"""
        msg_data = {
            "type": msg_type,
            "from": self.robot_id,
            "timestamp": time.time(),
            **kwargs
        }
        msg = String()
        msg.data = json.dumps(msg_data)
        self.pub.publish(msg)
        logger.debug(f"📤 Sent {msg_type}: {kwargs}")

    def _abort(self, reason: str):
        """中止运输"""
        if not self.aborted:
            self.aborted = True
            self._publish_message(MSG_ABORT, reason=reason)
            logger.error(f"🚨 ABORT: {reason}")
        
        self.motion.stop()
        self.abort_event.set()
        self.peer_ready_event.set()
        self.go_event.set()

    def on_message(self, msg: String):
        """处理接收到的协调消息"""
        try:
            data = json.loads(msg.data)
        except Exception as e:
            logger.warning(f"⚠️ Invalid message: {e}")
            return

        msg_type = data.get("type")
        from_robot = data.get("from")

        # 忽略自己发送的消息
        if from_robot == self.robot_id:
            return

        logger.debug(f"📥 Received {msg_type} from {from_robot}")

        if msg_type == MSG_READY and from_robot == self.peer_id:
            logger.info(f"✅ {from_robot} is ready")
            self.peer_ready_event.set()

        elif msg_type == MSG_GO and from_robot == self.peer_id:
            self.start_time = data.get("start_time")
            logger.info(f"🚦 GO signal received, start_time: {self.start_time}")
            self.go_event.set()

        elif msg_type == MSG_ABORT:
            reason = data.get("reason", "Unknown")
            logger.warning(f"🚨 Received ABORT from {from_robot}: {reason}")
            self.aborted = True
            self.abort_event.set()
            self.motion.stop()

    def _execute_phase3(self):
        """执行Phase3分布式同步运输"""
        try:
            logger.info("🎯 Starting Phase3: Distributed Synchronized Transport")

            # Step 1: 声明准备就绪
            logger.info("📢 Broadcasting READY signal")
            self._publish_message(MSG_READY)

            # Step 2: 等待对方准备就绪
            logger.info(f"⏳ Waiting for {self.peer_id} to be ready...")
            if not self.peer_ready_event.wait(timeout=WAIT_READY_TIMEOUT_SEC):
                self._abort(f"Timeout waiting for {self.peer_id} READY signal")
                return

            if self.aborted:
                return

            logger.info("✅ Both robots are ready")

            # Step 3: 同步启动
            if self.is_leader:
                # 领导者发送GO信号
                self.start_time = time.time() + 2.0  # 2秒后开始
                logger.info(f"🚦 Leader sending GO signal, start_time: {self.start_time}")
                self._publish_message(MSG_GO, start_time=self.start_time, distance=self.distance_mm)
            else:
                # 跟随者等待GO信号
                logger.info("⏳ Follower waiting for GO signal...")
                if not self.go_event.wait(timeout=WAIT_GO_TIMEOUT_SEC):
                    self._abort("Timeout waiting for GO signal")
                    return

            if self.aborted:
                return

            # Step 4: 精确同步等待
            if self.start_time:
                while time.time() < self.start_time:
                    remaining = self.start_time - time.time()
                    if remaining <= 0:
                        break
                    # 最后2ms用忙等，其余时间轻睡眠
                    sleep_time = 0.0005 if remaining < 0.002 else min(0.01, remaining / 2)
                    time.sleep(sleep_time)

            # Step 5: 开始同步运输
            forward_motion = self.is_leader  # leader前进，follower后退
            
            logger.info(f"🚀 Starting synchronized transport!")
            logger.info(f"   Direction: {'FORWARD' if forward_motion else 'BACKWARD'}")
            logger.info(f"   Distance: {self.distance_mm}mm")
            logger.info(f"   Speed: {SPEED}m/s")

            # 执行运动
            self.motion.drive_constant(forward_motion, self.distance_mm, SPEED)

            logger.info("🎉 Phase3 transport completed successfully!")
            self.success = True

        except Exception as e:
            logger.error(f"💥 Error during Phase3 execution: {e}")
            self._abort(f"Execution error: {e}")
        finally:
            self.completed = True
            logger.info("🏁 Phase3 execution finished")

    def wait_for_completion(self, timeout: float = 120.0) -> bool:
        """等待运输完成"""
        start_time = time.time()
        while not self.completed and time.time() - start_time < timeout:
            time.sleep(0.1)
        return self.success

# ===== 主程序入口 =====
def main():
    parser = argparse.ArgumentParser(description="Phase3 Only Transport")
    parser.add_argument("--robot_id", required=True, choices=["robot1", "robot2"], 
                       help="Robot identifier")
    parser.add_argument("--distance", type=float, default=1000.0, 
                       help="Transport distance in mm (default: 1000mm)")
    parser.add_argument("--speed", type=float, default=0.05,
                       help="Transport speed in m/s (default: 0.05)")
    
    args = parser.parse_args()

    # 更新全局速度参数
    global SPEED
    SPEED = args.speed

    logger.info(f"🎬 Starting Phase3-only transport for {args.robot_id}")
    logger.info(f"   Distance: {args.distance}mm")
    logger.info(f"   Speed: {SPEED}m/s")

    # 初始化ROS
    rclpy.init()
    node = rclpy.create_node(f"phase3_transport_{args.robot_id}")
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # 创建运输管理器
    transport_manager = Phase3TransportManager(node, args.robot_id, args.distance)

    try:
        # 在后台运行executor
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        # 等待运输完成
        logger.info("⏳ Waiting for transport completion...")
        success = transport_manager.wait_for_completion(timeout=180.0)

        if success:
            logger.info("🎉 Phase3 transport completed successfully!")
            print("✅ SUCCESS: Transport completed")
        else:
            logger.error("❌ Phase3 transport failed or timed out")
            print("❌ FAILED: Transport failed")

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