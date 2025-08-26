#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Collaborative Movement - Phase 3 Only
让两个机器人协调前进的简化版本

使用方法:
  robot1: python3 collaborative_move.py --robot_id robot1
  robot2: python3 collaborative_move.py --robot_id robot2
"""

import math
import time
import json
import argparse
import threading
from loguru import logger

import os, sys
# 添加项目根目录到Python路径
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(PROJECT_ROOT)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from std_msgs.msg import String

# 修复导入路径
try:
    from ttsRepo.stream_tts import tts_manager
except ImportError:
    # 如果找不到tts模块，创建一个简单的替代品
    class DummyTTS:
        def say_sync(self, text):
            print(f"[TTS] {text}")
    tts_manager = DummyTTS()

# ===== 运动参数 =====
SPEED = 0.02        # 运动速度 (m/s)
DISTANCE = 1000     # 运动距离 (mm)

# 超时配置（秒）
WAIT_READY_TIMEOUT_SEC = 30.0   # 等对方 READY 的超时
WAIT_GO_TIMEOUT_SEC = 30.0      # follower 等 GO 的超时

# ===== 简单直线匀速控制 =====
class Motion:
    def __init__(self, node: Node, robot_ns: str):
        self.node = node
        self.pub = node.create_publisher(Twist, f'/{robot_ns}/cmd_vel', 10)

    def stop(self):
        """停止机器人"""
        self.pub.publish(Twist())

    def drive_constant(self, forward: bool, dist_mm: float, speed_mps: float):
        """
        匀速直线运动
        :param forward: True=前进，False=后退
        :param dist_mm: 移动距离，单位：毫米（mm）
        :param speed_mps: 移动速度，单位：米每秒（m/s）
        """
        dist_m = dist_mm / 1000.0  # 转换为米
        duration = dist_m / max(speed_mps, 1e-6)

        twist = Twist()
        twist.linear.x = speed_mps if forward else -speed_mps

        start_time = time.time()
        logger.info(f"开始运动: 方向={'前进' if forward else '后退'}, 距离={dist_mm}mm, 速度={speed_mps}m/s, 预计用时={duration:.2f}s")
        
        while time.time() - start_time < duration:
            self.pub.publish(twist)
            time.sleep(0.02)  # 50Hz 控制频率
        
        self.stop()
        logger.info("运动完成")

# ===== 协调协议消息类型 =====
MSG_READY = "ready"
MSG_GO = "go"
MSG_ABORT = "abort"

class CollaborativeMove:
    """
    协作运动管理器 - 只包含Phase3的同步运动部分
    """
    def __init__(self, node: Node, executor: MultiThreadedExecutor, robot_id: str, 
                 distance: float = DISTANCE, speed: float = SPEED):
        self.node = node
        self.executor = executor
        self.robot_id = robot_id
        self.peer_id = "robot1" if robot_id == "robot2" else "robot2"
        self.is_robot1 = (robot_id == "robot1")

        # 完成状态
        self.completed = False
        self.success = False
        self.aborted = False

        # 通信设置
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub = node.create_publisher(String, "/move_coordination", qos)
        self.sub = node.create_subscription(String, "/move_coordination", self.on_message, qos)

        # 同步事件
        self.peer_ready_event = threading.Event()
        self.go_event = threading.Event()
        self.abort_event = threading.Event()

        # 运动控制
        self.motion = Motion(node, robot_id)
        
        # 运动参数
        self.start_at = None
        self.distance = distance  # 使用传入的参数
        self.speed = speed        # 使用传入的参数

        logger.info(f"[{robot_id}] 协作运动管理器初始化完成")
        # tts_manager.say_sync(f"{robot_id} ready for collaborative movement")

        # 启动工作线程
        self.worker = threading.Thread(target=self._run_collaborative_move, daemon=True)
        self.worker.start()

    def _publish_message(self, msg_type: str, payload: dict = None):
        """发布协调消息"""
        if payload is None:
            payload = {}
        
        msg = String()
        msg.data = json.dumps({
            "type": msg_type,
            "robot": self.robot_id,
            "timestamp": time.time(),
            **payload
        })
        self.pub.publish(msg)
        logger.debug(f"[{self.robot_id}] 发送消息: {msg_type}")

    def _abort(self, reason: str):
        """中止协作运动"""
        if not self.aborted:
            self.aborted = True
            self._publish_message(MSG_ABORT, {"reason": reason})
            logger.error(f"[{self.robot_id}] 中止运动: {reason}")
            # tts_manager.say_sync(f"Movement aborted: {reason}")
        
        self.motion.stop()
        self.abort_event.set()
        self.peer_ready_event.set()
        self.go_event.set()

    def on_message(self, msg: String):
        """处理协调消息"""
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        msg_type = data.get("type")
        robot = data.get("robot")

        # 忽略自己发送的消息
        if robot == self.robot_id:
            return

        if msg_type == MSG_READY and robot == self.peer_id:
            logger.info(f"[{self.robot_id}] 收到 {self.peer_id} 的 READY 信号")
            self.peer_ready_event.set()

        elif msg_type == MSG_GO:
            self.start_at = float(data.get("start_at", time.time()))
            if "distance" in data:
                self.distance = float(data["distance"])
            if "speed" in data:
                self.speed = float(data["speed"])
            
            logger.info(f"[{self.robot_id}] 收到 GO 信号，开始时间: {self.start_at}")
            self.go_event.set()

        elif msg_type == MSG_ABORT:
            reason = data.get("reason", "未知原因")
            logger.warning(f"[{self.robot_id}] 收到中止信号: {reason}")
            self.aborted = True
            self.abort_event.set()
            self.motion.stop()
            self.peer_ready_event.set()
            self.go_event.set()

    def _run_collaborative_move(self):
        """执行协作运动的主流程"""
        try:
            # 步骤1: 广播 READY 并等待对方 READY
            logger.info(f"[{self.robot_id}] 广播 READY 信号")
            # tts_manager.say_sync("Broadcasting ready signal")
            self._publish_message(MSG_READY)

            logger.info(f"[{self.robot_id}] 等待 {self.peer_id} 的 READY 信号...")
            if not self.peer_ready_event.wait(timeout=WAIT_READY_TIMEOUT_SEC):
                self._abort(f"等待 {self.peer_id} READY 信号超时 (>{WAIT_READY_TIMEOUT_SEC}s)")
                return

            if self.aborted:
                return

            # 步骤2: 协调开始时间 - 改进版本，预留更多准备时间
            if self.is_robot1:
                # robot1 作为协调者，发送 GO 信号
                self.start_at = time.time() + 3.0  # 预留3秒准备时间，确保同步
                logger.info(f"[{self.robot_id}] 发送 GO 信号")
                logger.info(f"[{self.robot_id}] 当前时间: {time.time():.3f}")
                logger.info(f"[{self.robot_id}] 计划开始时间: {self.start_at:.3f}")
                logger.info(f"[{self.robot_id}] 倒计时: {self.start_at - time.time():.3f} 秒")
                
                self._publish_message(MSG_GO, {
                    "start_at": self.start_at,
                    "distance": self.distance,
                    "speed": self.speed
                })
                # tts_manager.say_sync("Sending go signal, movement will start in 3 seconds")
            else:
                # robot2 等待 GO 信号
                logger.info(f"[{self.robot_id}] 等待 GO 信号...")
                # tts_manager.say_sync("Waiting for go signal")
                if not self.go_event.wait(timeout=WAIT_GO_TIMEOUT_SEC):
                    self._abort(f"等待 GO 信号超时 (>{WAIT_GO_TIMEOUT_SEC}s)")
                    return
                
                logger.info(f"[{self.robot_id}] 收到 GO 信号")
                logger.info(f"[{self.robot_id}] 当前时间: {time.time():.3f}")
                logger.info(f"[{self.robot_id}] 计划开始时间: {self.start_at:.3f}")
                logger.info(f"[{self.robot_id}] 倒计时: {self.start_at - time.time():.3f} 秒")

            if self.aborted:
                return

            # 步骤3: 精确时间同步等待 - 改进版本
            logger.info(f"[{self.robot_id}] 开始精确时间同步等待...")
            countdown_shown = False
            
            while True:
                current_time = time.time()
                remain = self.start_at - current_time
                
                if remain <= 0:
                    logger.info(f"[{self.robot_id}] ⏰ 时间到！开始运动！")
                    break
                
                # 显示倒计时（每秒显示一次）
                if remain <= 3.0 and not countdown_shown:
                    logger.info(f"[{self.robot_id}] ⏱️ 倒计时: {remain:.1f} 秒")
                    if remain <= 1.0:
                        countdown_shown = True
                
                # 最后5ms用忙等，其余时间轻睡
                if remain < 0.005:
                    time.sleep(0.0001)  # 100微秒精度
                elif remain < 0.1:
                    time.sleep(0.001)   # 1毫秒精度
                else:
                    time.sleep(0.01)    # 10毫秒精度

            # 步骤4: 同步开始运动 - 添加更详细的日志
            forward = self.is_robot1  # 两个机器人都前进
            
            # 记录实际开始时间
            actual_start_time = time.time()
            planned_start_time = self.start_at
            sync_error = actual_start_time - planned_start_time
            
            logger.info(f"[{self.robot_id}] 🚀 开始协作运动！")
            logger.info(f"[{self.robot_id}] 计划开始时间: {planned_start_time:.6f}")
            logger.info(f"[{self.robot_id}] 实际开始时间: {actual_start_time:.6f}")
            logger.info(f"[{self.robot_id}] 同步误差: {sync_error*1000:.2f} ms")
            logger.info(f"[{self.robot_id}] 运动参数: 方向={'前进' if forward else '后退'}, 距离={self.distance}mm, 速度={self.speed}m/s")
            
            # tts_manager.say_sync("Starting synchronized movement now")
            
            self.motion.drive_constant(forward, self.distance, self.speed)
            
            # 记录完成时间
            finish_time = time.time()
            duration = finish_time - actual_start_time
            
            logger.info(f"[{self.robot_id}] ✅ 协作运动完成！")
            logger.info(f"[{self.robot_id}] 运动用时: {duration:.2f} 秒")
            logger.info(f"[{self.robot_id}] 完成时间: {finish_time:.6f}")
            # tts_manager.say_sync("Collaborative movement completed successfully")
            
            self.success = True

        except Exception as e:
            logger.error(f"[{self.robot_id}] 协作运动出错: {e}")
            self._abort(f"运动出错: {e}")
        finally:
            self.completed = True
            self.motion.stop()

    def wait_for_completion(self, timeout: float = None) -> bool:
        """等待协作运动完成"""
        if timeout:
            self.worker.join(timeout)
        else:
            self.worker.join()
        return self.success

# ========== 主入口 ==========
def main():
    parser = argparse.ArgumentParser(description="协作运动测试")
    parser.add_argument("--robot_id", required=True, choices=["robot1", "robot2"], 
                       help="机器人ID")
    parser.add_argument("--distance", type=float, default=DISTANCE, 
                       help="运动距离(mm)")
    parser.add_argument("--speed", type=float, default=SPEED, 
                       help="运动速度(m/s)")
    args = parser.parse_args()

    # 使用局部变量而不是修改全局变量
    distance = args.distance
    speed = args.speed

    logger.info(f"启动协作运动: robot_id={args.robot_id}, distance={distance}mm, speed={speed}m/s")

    try:
        rclpy.init()
        node = Node(f"collaborative_move_{args.robot_id}")
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        # 创建协作运动管理器，传入自定义参数
        move_manager = CollaborativeMove(node, executor, args.robot_id, distance=distance, speed=speed)

        # 启动executor
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        # 等待协作运动完成
        logger.info("等待协作运动完成...")
        success = move_manager.wait_for_completion(timeout=120)  # 最多等待2分钟

        if success:
            logger.info("🎉 协作运动成功完成!")
        else:
            logger.error("❌ 协作运动失败或超时")

    except KeyboardInterrupt:
        logger.info("用户中断")
    except Exception as e:
        logger.error(f"程序出错: {e}")
    finally:
        try:
            if 'move_manager' in locals():
                move_manager.motion.stop()
            if 'node' in locals():
                node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            logger.warning(f"清理资源时出错: {e}")

if __name__ == "__main__":
    main()