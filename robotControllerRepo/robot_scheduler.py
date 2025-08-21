#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, sys
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)
import json, time, threading, traceback
from collections import defaultdict
from typing import Dict, Any, Optional, Set
from ttsRepo.stream_tts import tts_manager
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from loguru import logger
from textwrap import dedent

# === 你项目里的依赖 ===
from robotControllerRepo.robot_controller import execute_action
import config

import datetime

def now():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

# =========================
# 三次握手管理器
# =========================
class ThreeWayHandshakeManager:
    """管理sequential任务的三次握手确认机制"""
    def __init__(self, robot_name: str, publisher):
        self.robot_name = robot_name
        self.publisher = publisher
        
        # 握手状态跟踪
        self.sent_finished: Dict[int, float] = {}  # seq -> timestamp
        self.received_acks: Dict[int, Set[str]] = {}  # seq -> {robot_names}
        self.sent_ack_acks: Dict[int, Set[str]] = {}  # seq -> {robot_names}
        self.completed_sequences: Set[int] = set()  # 已完成握手的sequences
        
        # 锁保护
        self.lock = threading.Lock()
        
    def announce_finished(self, sequence: int):
        """第一步：宣布完成 (SYN)"""
        with self.lock:
            self.sent_finished[sequence] = time.time()
            
        msg = {
            "kind": "finished",
            "robot": self.robot_name,
            "sequence": sequence,
            "ts": time.time()
        }
        self.publisher.publish(String(data=json.dumps(msg)))
        logger.info(f"📤 {self.robot_name} → SYN: finished seq={sequence}")
        
    def send_ack(self, to_robot: str, sequence: int):
        """第二步：发送确认 (ACK)"""
        msg = {
            "kind": "ack_finished",
            "robot": self.robot_name,
            "to": to_robot,
            "sequence": sequence,
            "ts": time.time()
        }
        self.publisher.publish(String(data=json.dumps(msg)))
        logger.info(f"🤝 {self.robot_name} → ACK: to {to_robot} seq={sequence}")
        
    def send_ack_ack(self, to_robot: str, sequence: int):
        """第三步：确认收到确认 (ACK-ACK)"""
        with self.lock:
            if sequence not in self.sent_ack_acks:
                self.sent_ack_acks[sequence] = set()
            self.sent_ack_acks[sequence].add(to_robot)
            
        msg = {
            "kind": "ack_ack_finished",
            "robot": self.robot_name,
            "to": to_robot,
            "sequence": sequence,
            "ts": time.time()
        }
        self.publisher.publish(String(data=json.dumps(msg)))
        logger.info(f"✅ {self.robot_name} → ACK-ACK: to {to_robot} seq={sequence}")
        
    def handle_received_finished(self, from_robot: str, sequence: int):
        """处理收到的finished消息，自动发送ACK"""
        logger.info(f"📨 {self.robot_name} received SYN: {from_robot} finished seq={sequence}")
        self.send_ack(from_robot, sequence)
        
        # 对于其他机器人完成的sequence，收到SYN即可认为该sequence完成
        with self.lock:
            self.completed_sequences.add(sequence)
        
    def handle_received_ack(self, from_robot: str, sequence: int) -> bool:
        """处理收到的ACK，自动发送ACK-ACK，返回握手是否完成"""
        logger.info(f"📨 {self.robot_name} received ACK from {from_robot} seq={sequence}")
        
        with self.lock:
            if sequence not in self.received_acks:
                self.received_acks[sequence] = set()
            self.received_acks[sequence].add(from_robot)
            
        # 自动发送ACK-ACK
        self.send_ack_ack(from_robot, sequence)
        
        # 检查握手是否完成（简化：收到至少一个ACK就认为完成）
        with self.lock:
            if len(self.received_acks[sequence]) >= 1:
                self.completed_sequences.add(sequence)
                return True
        return False
        
    def handle_received_ack_ack(self, from_robot: str, sequence: int):
        """处理收到的ACK-ACK"""
        logger.info(f"📨 {self.robot_name} received ACK-ACK from {from_robot} seq={sequence}")
        
    def is_sequence_complete(self, sequence: int) -> bool:
        """检查指定sequence是否已完成握手"""
        with self.lock:
            return sequence in self.completed_sequences
        
    def wait_for_sequence_completion(self, sequence: int, timeout: float = 30.0) -> bool:
        """等待指定sequence完成握手"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.is_sequence_complete(sequence):
                return True
            time.sleep(0.1)
        return False

# =========================
# 改进的Sequential管理器
# =========================
class ImprovedSequentialManager:
    def __init__(self, node: Node, robot_name: str, all_robots: Set[str]):
        self.node = node
        self.robot_name = robot_name
        self.all_robots = all_robots
        
        # QoS设置
        qos_profile = QoSProfile(depth=50)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        # 发布器和订阅器
        self.pub = node.create_publisher(String, '/robot_status', qos_profile)
        self.sub = node.create_subscription(String, '/robot_status', self.status_callback, qos_profile)
        
        # 三次握手管理器
        self.handshake_mgr = ThreeWayHandshakeManager(robot_name, self.pub)
        
        # 序列等待事件
        self.sequence_events: Dict[int, threading.Event] = {}
        
        logger.info(f"🤖 ImprovedSequentialManager initialized for {robot_name}")
        
    def status_callback(self, msg):
        """处理收到的状态消息"""
        try:
            data = json.loads(msg.data)
            kind = data.get("kind")
            robot = data.get("robot")
            sequence = data.get("sequence")
            to_robot = data.get("to")

            # 忽略自己的消息
            if robot == self.robot_name:
                return
                
            if kind == "finished" and sequence is not None:
                self.handshake_mgr.handle_received_finished(robot, sequence)
                # 触发等待该sequence的事件
                if sequence in self.sequence_events:
                    logger.info(f"🚦 {self.robot_name} signaling completion for seq={sequence}")
                    self.sequence_events[sequence].set()
                
            elif kind == "ack_finished" and to_robot == self.robot_name and sequence is not None:
                is_complete = self.handshake_mgr.handle_received_ack(robot, sequence)
                if is_complete and sequence in self.sequence_events:
                    logger.info(f"🎊 {self.robot_name} handshake complete for seq={sequence}")
                    self.sequence_events[sequence].set()
                
            elif kind == "ack_ack_finished" and to_robot == self.robot_name and sequence is not None:
                self.handshake_mgr.handle_received_ack_ack(robot, sequence)

        except Exception as e:
            logger.warning(f"Parse error in status_callback: {e}")

    def wait_for_previous_sequence(self, sequence: int, prev_sequence: int, prev_owner: str, timeout: float = 60.0) -> bool:
        """等待前序sequence完成"""
        if prev_owner == self.robot_name:
            logger.info(f"📝 {self.robot_name}: Previous seq={prev_sequence} is mine, no waiting needed")
            return True
            
        # 检查是否已经完成
        if self.handshake_mgr.is_sequence_complete(prev_sequence):
            logger.info(f"✅ {self.robot_name}: Previous seq={prev_sequence} already completed")
            return True
            
        logger.info(f"⏳ {self.robot_name} waiting for seq={prev_sequence} from {prev_owner}")
        
        # 创建等待事件
        if prev_sequence not in self.sequence_events:
            self.sequence_events[prev_sequence] = threading.Event()
        
        # 等待完成
        success = self.sequence_events[prev_sequence].wait(timeout)
        
        if success:
            logger.info(f"🎉 {self.robot_name}: seq={prev_sequence} completed, can start seq={sequence}")
        else:
            logger.warning(f"❌ {self.robot_name}: Timeout waiting for seq={prev_sequence}")
            
        return success

    def execute_sequence_with_handshake(self, task: Dict[str, Any], executor) -> bool:
        """执行sequence任务并进行三次握手"""
        sequence = task.get("sequence")
        task_id = task.get("task_id")
        
        logger.info(f"🎬 {self.robot_name} starting sequence {sequence}")
        
        # 执行任务
        success = execute_action(self.node, executor, task)
        
        if success:
            logger.info(f"✅ {self.robot_name} completed execution for seq={sequence}")
            
            # 启动三次握手
            self.handshake_mgr.announce_finished(sequence)
            
            # 等待握手完成
            handshake_success = self.handshake_mgr.wait_for_sequence_completion(sequence, timeout=10.0)
            
            if handshake_success:
                logger.info(f"🎊 {self.robot_name} handshake confirmed for seq={sequence}")
            else:
                logger.warning(f"⚠️ {self.robot_name} handshake timeout for seq={sequence}")
                
        return success

# =========================
# 同步管理器（保持原有实现）
# =========================
class RobotSyncManager:
    def __init__(self, node: Node, robot_name: str, sync_group: int, target_count: int, timeout: float = 60.0):
        self.node = node
        self.robot_name = robot_name
        self.sync_group = sync_group
        self.target_count = target_count
        self.timeout = timeout

        self.ready_set: Set[str] = set()
        self.ack_set: Set[str] = set()
        self.prev_ready_set: Set[str] = set()

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        
        self.pub = node.create_publisher(String, '/robot_status', qos_profile)
        self.sub = node.create_subscription(String, '/robot_status', self.status_callback, qos_profile)

        self.ready_timer = node.create_timer(1.0, self.publish_ready)
        self.sync_start_time = time.time()
        self.sync_timeout_timer = node.create_timer(0.5, self.check_timeout)

        self.sync_complete_event = threading.Event()
        logger.info(f"{self.robot_name} waiting for ALL ack_ready...")

    def publish_ready(self):
        msg = {
            "kind": "ready",
            "robot": self.robot_name,
            "sync_group": self.sync_group,
            "ts": time.time()
        }
        self.pub.publish(String(data=json.dumps(msg)))
        logger.info(f"PUBLISH {self.robot_name} → ready")

    def publish_ack(self, to_robot):
        msg = {
            "kind": "ack_ready",
            "robot": self.robot_name,
            "to": to_robot,
            "sync_group": self.sync_group,
            "ts": time.time()
        }
        self.pub.publish(String(data=json.dumps(msg)))
        logger.info(f"{self.robot_name} send ack_ready to {to_robot}")

    def status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("sync_group") != self.sync_group:
                return
            kind = data.get("kind")
            robot = data.get("robot")

            if kind == "ready":
                if robot not in self.ready_set:
                    self.ready_set.add(robot)
                    self.publish_ack(robot)
            elif kind == "ack_ready" and data.get("to") == self.robot_name:
                if robot not in self.ack_set:
                    self.ack_set.add(robot)

            self.print_ready_set()
            self.check_sync_complete()

        except Exception as e:
            logger.warning(f"Parse error: {e}")

    def print_ready_set(self):
        if self.ready_set != self.prev_ready_set:
            logger.info(f"{self.robot_name} sees ready_set = {sorted(self.ready_set)}")
            self.prev_ready_set = set(self.ready_set)

    def check_sync_complete(self):
        if len(self.ready_set) >= self.target_count and len(self.ack_set) >= self.target_count:
            if not self.sync_complete_event.is_set():
                logger.info(f"✅ All robots OK! {self.robot_name} can proceed.")
                self.sync_complete_event.set()
                self.ready_timer.cancel()
                self.sync_timeout_timer.cancel()

    def check_timeout(self):
        if self.sync_complete_event.is_set():
            return
        elapsed = time.time() - self.sync_start_time
        if elapsed > self.timeout:
            logger.warning(f"⏱️ Timeout! {self.robot_name} failed to sync in time.")
            self.ready_timer.cancel()
            self.sync_timeout_timer.cancel()
            self.sync_complete_event.set()

    def wait_for_sync(self):
        self.sync_complete_event.wait()
        return len(self.ready_set) >= self.target_count and len(self.ack_set) >= self.target_count

# =========================
# 工具函数
# =========================
def validate_local_plan(task_data: Dict[str, Any]):
    """校验任务计划"""
    seen_seq_owner: Dict[int, str] = {}
    for robot, tasks in task_data.get("robots", {}).items():
        seen_seq_in_robot = set()
        for t in tasks:
            s = t.get("sequence")
            sg = t.get("sync_group")

            if s is not None and sg is not None:
                raise ValueError(f"Task cannot have both sequence and sync_group: {t}")

            if s is not None:
                if s in seen_seq_in_robot:
                    raise ValueError(f"Duplicate sequence={s} in robot '{robot}'")
                seen_seq_in_robot.add(s)

                if s in seen_seq_owner and seen_seq_owner[s] != robot:
                    raise ValueError(
                        f"Duplicate global sequence={s} found on '{seen_seq_owner[s]}' and '{robot}'"
                    )
                seen_seq_owner[s] = robot

def build_prev_stage_map(task_data: Dict[str, Any]) -> Dict[int, Optional[int]]:
    """构建前驱阶段映射"""
    stages = sorted({
        t["sequence"]
        for tasks in task_data.get("robots", {}).values()
        for t in tasks
        if t.get("sequence") is not None
    })
    prev = {}
    for i, s in enumerate(stages):
        prev[s] = stages[i - 1] if i > 0 else None
    return prev

def build_seq_owner_map(task_data: Dict[str, Any]) -> Dict[int, str]:
    """构建序列所有者映射"""
    owner = {}
    for robot, tasks in task_data.get("robots", {}).items():
        for t in tasks:
            s = t.get("sequence")
            if s is not None:
                owner[int(s)] = robot
    return owner

def count_robots_per_sync_key(task_data: Dict[str, Any]) -> Dict[tuple, int]:
    """计算同步组机器人数量"""
    out: Dict[tuple, int] = {}
    sg_map = defaultdict(set)
    for robot, tasks in task_data.get("robots", {}).items():
        for t in tasks:
            sg = t.get("sync_group")
            if sg is not None:
                sg = int(sg)
                sg_map[(None, sg)].add(robot)
    out.update({k: len(v) for k, v in sg_map.items()})
    return out

def ensure_task_ids(task_data: Dict[str, Any]):
    """确保每个任务都有ID"""
    for robot, tasks in task_data.get("robots", {}).items():
        for i, t in enumerate(tasks):
            seq = t.get("sequence")
            sg = t.get("sync_group")
            tag = f"seq{seq}" if seq is not None else (f"sg{sg}" if sg is not None else "local")
            t["task_id"] = f"{robot}-{tag}-{i}"

# =========================
# 改进的调度器主逻辑
# =========================
def run_improved_scheduler_for_robot(node: Node, robot_name: str, task_data: Dict[str, Any],
                                    executor, prev_stage: Dict[int, Optional[int]],
                                    seq_owner_map: Dict[int, str]):
    """使用三次握手机制的改进调度器"""
    logger.info(f"🤖 Robot `{robot_name}` starting IMPROVED task scheduler with three-way handshake")

    is_successful_overall = True
    tasks = task_data["robots"].get(robot_name, [])
    all_robots = set(task_data["robots"].keys())
    target_counts = count_robots_per_sync_key(task_data)

    # 创建sequential管理器
    seq_mgr = ImprovedSequentialManager(node, robot_name, all_robots)
    
    # 给ROS一些时间建立连接
    time.sleep(2.0)

    try:
        for task in tasks:
            task["robot"] = robot_name
            tid = task.get("task_id")
            seq = task.get("sequence")
            sg = task.get("sync_group")
            
            if sg is not None:
                sg = int(sg)

            # A) Sequential execution with three-way handshake
            if seq is not None and sg is None:
                prev_seq = prev_stage.get(seq)
                
                # 等待前序任务
                if prev_seq is not None:
                    prev_owner = seq_owner_map.get(prev_seq)
                    if prev_owner and prev_owner != robot_name:
                        tts_manager.say_sync("I'm waiting for the other robots to finish")
                        success = seq_mgr.wait_for_previous_sequence(seq, prev_seq, prev_owner)
                        if not success:
                            logger.warning(f"❌ {robot_name} failed to wait for seq={prev_seq}")
                            is_successful_overall = False
                            continue
                        tts_manager.say_sync(f"sequence {prev_seq} is finished, I'm going to start the mission")

                # 执行任务并进行三次握手
                success = seq_mgr.execute_sequence_with_handshake(task, executor)
                if not success:
                    is_successful_overall = False
                continue

            # B) Synchronous execution (unchanged)
            elif sg is not None and seq is None:
                sync_group = sg
                target_count = target_counts.get((None, sync_group), 2)
                sync_manager = RobotSyncManager(node, robot_name, sync_group, target_count)

                success = sync_manager.wait_for_sync()
                if not success:
                    logger.warning(f"❌ {robot_name} sync failed for group {sync_group}")
                    tts_manager.say_sync("synchronization failed or timeout, skipping this task")
                    is_successful_overall = False
                    continue

                logger.info(f"✅ {robot_name} sync success → executing task {tid}")
                success = execute_action(node, executor, task)
                if not success:
                    is_successful_overall = False
                continue

            # C) Local execution (unchanged)
            else:
                success = execute_action(node, executor, task)
                if not success:
                    is_successful_overall = False

        if is_successful_overall:
            logger.info(f"🎯 All tasks completed successfully for `{robot_name}`!")
        else:
            logger.warning(f"⚠️ Some tasks failed for `{robot_name}`")
            
        return is_successful_overall

    except Exception as e:
        logger.exception(f"⚠️ Failed to execute tasks for {robot_name}: {e}")
        return False

# =========================
# 入口函数
# =========================
def run(task_data: Dict[str, Any]):
    """改进的调度器入口"""
    validate_local_plan(task_data)

    robot_id = config.get("robot_id")
    is_successful = False

    # 初始化ROS
    try:
        rclpy.init()
    except RuntimeError:
        pass

    # 创建节点和执行器
    ros_node = rclpy.create_node(f"improved_scheduler_{robot_id}")
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)

    # 启动executor
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        ensure_task_ids(task_data)
        prev_stage = build_prev_stage_map(task_data)
        seq_owner_map = build_seq_owner_map(task_data)
        
        is_successful = run_improved_scheduler_for_robot(
            ros_node, robot_id, task_data, executor, prev_stage, seq_owner_map
        )
        
    except Exception as e:
        logger.exception(f"⚠️ Scheduler failed: {e}")
    finally:
        logger.info(f"🛑 Shutting down improved scheduler for {robot_id}")
        try:
            executor.remove_node(ros_node)
        except Exception:
            pass
        executor.shutdown()
        ros_node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1)

    return is_successful

# =========================
# 测试入口
# =========================
if __name__ == "__main__":
    test_data = {
        "robots": {
            "robot1": [
                {"action": "move", "parameters": {"direction": "forward", "value": 3, "unit": "seconds"}, "sequence": 0},
                {"action": "turn", "parameters": {"direction": "left", "value": 90, "unit": "degrees"}, "sequence": 2}
            ],
            "robot2": [
                {"action": "navigate", "parameters": {"target": "table"}, "sequence": 1},
                {"action": "collect", "parameters": {"item": "box", "target": "table"}, "sequence": 3}
            ]
        }
    }
    run(test_data)