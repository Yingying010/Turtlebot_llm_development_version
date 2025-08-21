#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Enhanced Robot Scheduler with Professional Logging
Academic Multi-Robot Coordination System
"""

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

# Import project dependencies
from robotControllerRepo.robot_controller import execute_action
import config

import datetime

def now():
    """Get current timestamp in millisecond precision"""
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

# =========================
# Three-Way Handshake Protocol Manager
# =========================
class ThreeWayHandshakeManager:
    """
    Manages three-way handshake confirmation protocol for sequential task coordination.
    Implements SYN-ACK-ACK pattern for reliable distributed synchronization.
    """
    def __init__(self, robot_name: str, publisher):
        self.robot_name = robot_name
        self.publisher = publisher
        
        # Handshake state tracking
        self.sent_finished: Dict[int, float] = {}  # sequence -> timestamp
        self.received_acks: Dict[int, Set[str]] = {}  # sequence -> robot_names
        self.sent_ack_acks: Dict[int, Set[str]] = {}  # sequence -> robot_names
        self.completed_sequences: Set[int] = set()  # completed handshake sequences
        
        # Thread-safe operations
        self.lock = threading.Lock()
        
        logger.info(f"[HANDSHAKE] Initialized three-way handshake manager for robot: {self.robot_name}")
        
    def announce_finished(self, sequence: int):
        """Phase 1: Announce task completion (SYN)"""
        with self.lock:
            self.sent_finished[sequence] = time.time()
            
        msg = {
            "kind": "finished",
            "robot": self.robot_name,
            "sequence": sequence,
            "timestamp": time.time()
        }
        self.publisher.publish(String(data=json.dumps(msg)))
        logger.info(f"[HANDSHAKE] {self.robot_name} → SYN: Task completion announced for sequence={sequence}")
        
    def send_ack(self, to_robot: str, sequence: int):
        """Phase 2: Send acknowledgment (ACK)"""
        msg = {
            "kind": "ack_finished",
            "robot": self.robot_name,
            "to": to_robot,
            "sequence": sequence,
            "timestamp": time.time()
        }
        self.publisher.publish(String(data=json.dumps(msg)))
        logger.info(f"[HANDSHAKE] {self.robot_name} → ACK: Acknowledging completion to {to_robot} for sequence={sequence}")
        
    def send_ack_ack(self, to_robot: str, sequence: int):
        """Phase 3: Confirm acknowledgment receipt (ACK-ACK)"""
        with self.lock:
            if sequence not in self.sent_ack_acks:
                self.sent_ack_acks[sequence] = set()
            self.sent_ack_acks[sequence].add(to_robot)
            
        msg = {
            "kind": "ack_ack_finished",
            "robot": self.robot_name,
            "to": to_robot,
            "sequence": sequence,
            "timestamp": time.time()
        }
        self.publisher.publish(String(data=json.dumps(msg)))
        logger.info(f"[HANDSHAKE] {self.robot_name} → ACK-ACK: Confirming acknowledgment to {to_robot} for sequence={sequence}")
        
    def handle_received_finished(self, from_robot: str, sequence: int):
        """Process received completion announcement and auto-respond with ACK"""
        logger.info(f"[HANDSHAKE] {self.robot_name} ← SYN: Received completion notification from {from_robot} for sequence={sequence}")
        self.send_ack(from_robot, sequence)
        
        # Mark sequence as completed for external robot completions
        with self.lock:
            self.completed_sequences.add(sequence)
        
    def handle_received_ack(self, from_robot: str, sequence: int) -> bool:
        """Process received ACK and auto-respond with ACK-ACK"""
        logger.info(f"[HANDSHAKE] {self.robot_name} ← ACK: Received acknowledgment from {from_robot} for sequence={sequence}")
        
        with self.lock:
            if sequence not in self.received_acks:
                self.received_acks[sequence] = set()
            self.received_acks[sequence].add(from_robot)
            
        # Auto-send ACK-ACK
        self.send_ack_ack(from_robot, sequence)
        
        # Check handshake completion (simplified: one ACK sufficient)
        with self.lock:
            if len(self.received_acks[sequence]) >= 1:
                self.completed_sequences.add(sequence)
                logger.info(f"[HANDSHAKE] {self.robot_name}: Three-way handshake completed for sequence={sequence}")
                return True
        return False
        
    def handle_received_ack_ack(self, from_robot: str, sequence: int):
        """Process received ACK-ACK confirmation"""
        logger.info(f"[HANDSHAKE] {self.robot_name} ← ACK-ACK: Received final confirmation from {from_robot} for sequence={sequence}")
        
    def is_sequence_complete(self, sequence: int) -> bool:
        """Check if specified sequence handshake is complete"""
        with self.lock:
            return sequence in self.completed_sequences
        
    def wait_for_sequence_completion(self, sequence: int, timeout: float = 30.0) -> bool:
        """Wait for specified sequence handshake completion with timeout"""
        start_time = time.time()
        logger.debug(f"[HANDSHAKE] {self.robot_name}: Waiting for handshake completion of sequence={sequence} (timeout={timeout}s)")
        
        while time.time() - start_time < timeout:
            if self.is_sequence_complete(sequence):
                elapsed = time.time() - start_time
                logger.info(f"[HANDSHAKE] {self.robot_name}: Handshake completion confirmed for sequence={sequence} (elapsed={elapsed:.2f}s)")
                return True
            time.sleep(0.1)
            
        logger.warning(f"[HANDSHAKE] {self.robot_name}: Handshake timeout for sequence={sequence} after {timeout}s")
        return False

# =========================
# Enhanced Sequential Task Manager
# =========================
class ImprovedSequentialManager:
    """
    Advanced sequential task coordination manager implementing distributed consensus
    through three-way handshake protocol for reliable multi-robot orchestration.
    """
    def __init__(self, node: Node, robot_name: str, all_robots: Set[str]):
        self.node = node
        self.robot_name = robot_name
        self.all_robots = all_robots
        
        # Configure QoS for reliable communication
        qos_profile = QoSProfile(depth=50)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        # Initialize ROS communication
        self.pub = node.create_publisher(String, '/robot_status', qos_profile)
        self.sub = node.create_subscription(String, '/robot_status', self.status_callback, qos_profile)
        
        # Initialize handshake protocol manager
        self.handshake_mgr = ThreeWayHandshakeManager(robot_name, self.pub)
        
        # Sequence synchronization events
        self.sequence_events: Dict[int, threading.Event] = {}
        
        logger.info(f"[SEQUENTIAL] Sequential task manager initialized for robot: {robot_name}")
        logger.debug(f"[SEQUENTIAL] Participating robots: {sorted(all_robots)}")
        
    def status_callback(self, msg):
        """Process incoming status messages from other robots"""
        try:
            data = json.loads(msg.data)
            kind = data.get("kind")
            robot = data.get("robot")
            sequence = data.get("sequence")
            to_robot = data.get("to")

            # Ignore self-originated messages
            if robot == self.robot_name:
                return
                
            logger.debug(f"[SEQUENTIAL] {self.robot_name}: Received {kind} message from {robot}")
                
            if kind == "finished" and sequence is not None:
                self.handshake_mgr.handle_received_finished(robot, sequence)
                # Signal waiting threads for this sequence
                if sequence in self.sequence_events:
                    logger.debug(f"[SEQUENTIAL] {self.robot_name}: Signaling completion event for sequence={sequence}")
                    self.sequence_events[sequence].set()
                
            elif kind == "ack_finished" and to_robot == self.robot_name and sequence is not None:
                is_complete = self.handshake_mgr.handle_received_ack(robot, sequence)
                if is_complete and sequence in self.sequence_events:
                    logger.debug(f"[SEQUENTIAL] {self.robot_name}: Handshake completed, signaling sequence={sequence}")
                    self.sequence_events[sequence].set()
                
            elif kind == "ack_ack_finished" and to_robot == self.robot_name and sequence is not None:
                self.handshake_mgr.handle_received_ack_ack(robot, sequence)

        except json.JSONDecodeError as e:
            logger.error(f"[SEQUENTIAL] {self.robot_name}: JSON decode error in status callback: {e}")
        except Exception as e:
            logger.error(f"[SEQUENTIAL] {self.robot_name}: Unexpected error in status callback: {e}")

    def wait_for_previous_sequence(self, sequence: int, prev_sequence: int, prev_owner: str, timeout: float = 60.0) -> bool:
        """Wait for prerequisite sequence completion from specified robot"""
        if prev_owner == self.robot_name:
            logger.debug(f"[SEQUENTIAL] {self.robot_name}: Previous sequence={prev_sequence} owned by self, proceeding immediately")
            return True
            
        # Check if already completed
        if self.handshake_mgr.is_sequence_complete(prev_sequence):
            logger.info(f"[SEQUENTIAL] {self.robot_name}: Prerequisite sequence={prev_sequence} already completed by {prev_owner}")
            return True
            
        logger.info(f"[SEQUENTIAL] {self.robot_name}: Waiting for prerequisite sequence={prev_sequence} from {prev_owner} (timeout={timeout}s)")
        
        # Create synchronization event if needed
        if prev_sequence not in self.sequence_events:
            self.sequence_events[prev_sequence] = threading.Event()
        
        # Wait for completion signal
        start_time = time.time()
        success = self.sequence_events[prev_sequence].wait(timeout)
        elapsed_time = time.time() - start_time
        
        if success:
            logger.info(f"[SEQUENTIAL] {self.robot_name}: Prerequisite sequence={prev_sequence} completed by {prev_owner} (waited {elapsed_time:.2f}s)")
        else:
            logger.error(f"[SEQUENTIAL] {self.robot_name}: Timeout waiting for sequence={prev_sequence} from {prev_owner} after {timeout}s")
            
        return success

    def execute_sequence_with_handshake(self, task: Dict[str, Any], executor, 
                                        next_sequence_owner: Optional[str] = None) -> bool:
        """Execute sequential task with conditional handshake protocol"""
        sequence = task.get("sequence")
        task_id = task.get("task_id")
        action = task.get("action")
        
        logger.info(f"[EXECUTION] {self.robot_name}: Starting sequential task execution")
        logger.info(f"[EXECUTION] Task details - ID: {task_id}, Action: {action}, Sequence: {sequence}")
        
        # Execute the actual task
        start_time = time.time()
        success = execute_action(self.node, executor, task)
        execution_time = time.time() - start_time
        
        if success:
            logger.info(f"[EXECUTION] {self.robot_name}: Task execution completed successfully (duration: {execution_time:.2f}s)")
            
            # 🔍 关键优化：只在跨机器人sequential任务时进行握手
            if next_sequence_owner and next_sequence_owner != self.robot_name:
                logger.info(f"[EXECUTION] {self.robot_name}: Next sequence owner is {next_sequence_owner}, initiating handshake")
                
                # Initiate handshake protocol
                self.handshake_mgr.announce_finished(sequence)
                
                # Wait for handshake completion
                handshake_start = time.time()
                handshake_success = self.handshake_mgr.wait_for_sequence_completion(sequence, timeout=10.0)
                handshake_time = time.time() - handshake_start
                
                if handshake_success:
                    logger.info(f"[EXECUTION] {self.robot_name}: Handshake protocol completed successfully for sequence={sequence} (duration: {handshake_time:.2f}s)")
                else:
                    logger.warning(f"[EXECUTION] {self.robot_name}: Handshake protocol timeout for sequence={sequence}")
                    
            else:
                logger.info(f"[EXECUTION] {self.robot_name}: Next sequence is local or final, skipping handshake")
                
        else:
            logger.error(f"[EXECUTION] {self.robot_name}: Task execution failed for sequence={sequence}")
            
        return success
    

    def build_next_sequence_owner_map(task_data: Dict[str, Any]) -> Dict[int, Optional[str]]:
        """Build mapping of next sequence ownership for handshake optimization"""
        sequences = sorted({
            task["sequence"]
            for tasks in task_data.get("robots", {}).values()
            for task in tasks
            if task.get("sequence") is not None
        })
        
        # Build sequence to owner mapping
        seq_to_owner = {}
        for robot, tasks in task_data.get("robots", {}).items():
            for task in tasks:
                sequence = task.get("sequence")
                if sequence is not None:
                    seq_to_owner[sequence] = robot
        
        # Build next owner mapping
        next_owner_map = {}
        for i, seq in enumerate(sequences):
            next_seq = sequences[i + 1] if i + 1 < len(sequences) else None
            next_owner_map[seq] = seq_to_owner.get(next_seq) if next_seq is not None else None
        
        logger.debug(f"[ANALYSIS] Next sequence owner map: {next_owner_map}")
        return next_owner_map

# =========================
# Synchronous Coordination Manager
# =========================
class RobotSyncManager:
    """
    Manages synchronous task coordination across multiple robots using
    distributed consensus protocol with ready-acknowledge handshaking.
    """
    def __init__(self, node: Node, robot_name: str, sync_group: int, target_count: int, timeout: float = 60.0):
        self.node = node
        self.robot_name = robot_name
        self.sync_group = sync_group
        self.target_count = target_count
        self.timeout = timeout

        # Synchronization state tracking
        self.ready_set: Set[str] = set()
        self.ack_set: Set[str] = set()
        self.prev_ready_set: Set[str] = set()

        # Configure reliable QoS
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        
        # Initialize ROS communication
        self.pub = node.create_publisher(String, '/robot_status', qos_profile)
        self.sub = node.create_subscription(String, '/robot_status', self.status_callback, qos_profile)

        # Setup periodic ready announcements and timeout monitoring
        self.ready_timer = node.create_timer(1.0, self.publish_ready)
        self.sync_start_time = time.time()
        self.sync_timeout_timer = node.create_timer(0.5, self.check_timeout)

        # Synchronization completion event
        self.sync_complete_event = threading.Event()
        
        logger.info(f"[SYNC] Synchronization manager initialized for robot: {self.robot_name}")
        logger.info(f"[SYNC] Sync group: {sync_group}, Target participants: {target_count}, Timeout: {timeout}s")

    def publish_ready(self):
        """Broadcast readiness signal to synchronization group"""
        msg = {
            "kind": "ready",
            "robot": self.robot_name,
            "sync_group": self.sync_group,
            "timestamp": time.time()
        }
        self.pub.publish(String(data=json.dumps(msg)))
        logger.debug(f"[SYNC] {self.robot_name}: Broadcasting readiness signal for group {self.sync_group}")

    def publish_ack(self, to_robot):
        """Send acknowledgment to specific robot"""
        msg = {
            "kind": "ack_ready",
            "robot": self.robot_name,
            "to": to_robot,
            "sync_group": self.sync_group,
            "timestamp": time.time()
        }
        self.pub.publish(String(data=json.dumps(msg)))
        logger.debug(f"[SYNC] {self.robot_name}: Sending acknowledgment to {to_robot} for group {self.sync_group}")

    def status_callback(self, msg):
        """Process synchronization messages from other robots"""
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
                    logger.debug(f"[SYNC] {self.robot_name}: Registered ready signal from {robot}")
                    
            elif kind == "ack_ready" and data.get("to") == self.robot_name:
                if robot not in self.ack_set:
                    self.ack_set.add(robot)
                    logger.debug(f"[SYNC] {self.robot_name}: Received acknowledgment from {robot}")

            self.update_ready_set_status()
            self.check_sync_complete()

        except json.JSONDecodeError as e:
            logger.error(f"[SYNC] {self.robot_name}: JSON decode error: {e}")
        except Exception as e:
            logger.error(f"[SYNC] {self.robot_name}: Unexpected error in sync callback: {e}")

    def update_ready_set_status(self):
        """Log changes in ready set for monitoring"""
        if self.ready_set != self.prev_ready_set:
            logger.info(f"[SYNC] {self.robot_name}: Updated ready participants: {sorted(self.ready_set)} ({len(self.ready_set)}/{self.target_count})")
            self.prev_ready_set = set(self.ready_set)

    def check_sync_complete(self):
        """Check if synchronization requirements are met"""
        ready_threshold_met = len(self.ready_set) >= self.target_count
        ack_threshold_met = len(self.ack_set) >= self.target_count
        
        if ready_threshold_met and ack_threshold_met:
            if not self.sync_complete_event.is_set():
                elapsed = time.time() - self.sync_start_time
                logger.info(f"[SYNC] {self.robot_name}: Synchronization achieved for group {self.sync_group} (duration: {elapsed:.2f}s)")
                logger.info(f"[SYNC] Final participants - Ready: {sorted(self.ready_set)}, Acknowledged: {sorted(self.ack_set)}")
                
                self.sync_complete_event.set()
                self.ready_timer.cancel()
                self.sync_timeout_timer.cancel()

    def check_timeout(self):
        """Monitor for synchronization timeout"""
        if self.sync_complete_event.is_set():
            return
            
        elapsed = time.time() - self.sync_start_time
        if elapsed > self.timeout:
            logger.error(f"[SYNC] {self.robot_name}: Synchronization timeout for group {self.sync_group} after {self.timeout}s")
            logger.error(f"[SYNC] Timeout state - Ready: {sorted(self.ready_set)}, Acknowledged: {sorted(self.ack_set)}")
            
            self.ready_timer.cancel()
            self.sync_timeout_timer.cancel()
            self.sync_complete_event.set()

    def wait_for_sync(self):
        """Wait for synchronization completion and return success status"""
        logger.info(f"[SYNC] {self.robot_name}: Entering synchronization wait for group {self.sync_group}")
        self.sync_complete_event.wait()
        
        success = len(self.ready_set) >= self.target_count and len(self.ack_set) >= self.target_count
        if success:
            logger.info(f"[SYNC] {self.robot_name}: Synchronization successful for group {self.sync_group}")
        else:
            logger.warning(f"[SYNC] {self.robot_name}: Synchronization failed for group {self.sync_group}")
            
        return success

# =========================
# Task Plan Validation and Analysis
# =========================
def validate_local_plan(task_data: Dict[str, Any]):
    """Validate task plan for consistency and correctness"""
    logger.info("[VALIDATION] Starting task plan validation")
    
    seen_seq_owner: Dict[int, str] = {}
    total_tasks = 0
    
    for robot, tasks in task_data.get("robots", {}).items():
        logger.debug(f"[VALIDATION] Validating tasks for robot: {robot} ({len(tasks)} tasks)")
        seen_seq_in_robot = set()
        
        for task_idx, task in enumerate(tasks):
            total_tasks += 1
            sequence = task.get("sequence")
            sync_group = task.get("sync_group")
            action = task.get("action")

            # Check for conflicting coordination mechanisms
            if sequence is not None and sync_group is not None:
                raise ValueError(f"[VALIDATION] Task {task_idx} for {robot} has both sequence and sync_group: {task}")

            # Validate sequence uniqueness within robot
            if sequence is not None:
                if sequence in seen_seq_in_robot:
                    raise ValueError(f"[VALIDATION] Duplicate sequence={sequence} in robot '{robot}'")
                seen_seq_in_robot.add(sequence)

                # Validate global sequence uniqueness
                if sequence in seen_seq_owner and seen_seq_owner[sequence] != robot:
                    raise ValueError(f"[VALIDATION] Duplicate global sequence={sequence} found on '{seen_seq_owner[sequence]}' and '{robot}'")
                seen_seq_owner[sequence] = robot
                
            logger.debug(f"[VALIDATION] Task {task_idx}: {action} - Valid")
    
    logger.info(f"[VALIDATION] Task plan validation completed successfully")
    logger.info(f"[VALIDATION] Total tasks: {total_tasks}, Robots: {len(task_data.get('robots', {}))}")

def build_prev_stage_map(task_data: Dict[str, Any]) -> Dict[int, Optional[int]]:
    """Build dependency mapping for sequential stages"""
    sequences = sorted({
        task["sequence"]
        for tasks in task_data.get("robots", {}).values()
        for task in tasks
        if task.get("sequence") is not None
    })
    
    dependency_map = {}
    for i, seq in enumerate(sequences):
        dependency_map[seq] = sequences[i - 1] if i > 0 else None
    
    logger.debug(f"[ANALYSIS] Sequential dependency map: {dependency_map}")
    return dependency_map

def build_seq_owner_map(task_data: Dict[str, Any]) -> Dict[int, str]:
    """Build mapping of sequence ownership"""
    owner_map = {}
    for robot, tasks in task_data.get("robots", {}).items():
        for task in tasks:
            sequence = task.get("sequence")
            if sequence is not None:
                owner_map[int(sequence)] = robot
    
    logger.debug(f"[ANALYSIS] Sequence ownership map: {owner_map}")
    return owner_map

def count_robots_per_sync_key(task_data: Dict[str, Any]) -> Dict[tuple, int]:
    """Analyze synchronization group participation"""
    sync_groups = defaultdict(set)
    
    for robot, tasks in task_data.get("robots", {}).items():
        for task in tasks:
            sync_group = task.get("sync_group")
            if sync_group is not None:
                sync_groups[(None, int(sync_group))].add(robot)
    
    participation_counts = {key: len(robots) for key, robots in sync_groups.items()}
    
    for (_, group), count in participation_counts.items():
        logger.debug(f"[ANALYSIS] Sync group {group}: {count} participating robots")
    
    return participation_counts

def ensure_task_ids(task_data: Dict[str, Any]):
    """Generate unique task identifiers for tracking"""
    logger.debug("[PREPROCESSING] Generating task identifiers")
    
    task_count = 0
    for robot, tasks in task_data.get("robots", {}).items():
        for task_idx, task in enumerate(tasks):
            sequence = task.get("sequence")
            sync_group = task.get("sync_group")
            
            if sequence is not None:
                coord_tag = f"seq{sequence}"
            elif sync_group is not None:
                coord_tag = f"sync{sync_group}"
            else:
                coord_tag = "local"
                
            task["task_id"] = f"{robot}-{coord_tag}-{task_idx}"
            task_count += 1
            
    logger.info(f"[PREPROCESSING] Generated {task_count} unique task identifiers")

# =========================
# Enhanced Scheduler Main Logic
# =========================
def run_improved_scheduler_for_robot(node: Node, robot_name: str, task_data: Dict[str, Any],
                                    executor, prev_stage: Dict[int, Optional[int]],
                                    seq_owner_map: Dict[int, str]):
    """Execute improved task scheduling with professional coordination protocols"""
    
    logger.info(f"[SCHEDULER] Initializing enhanced task scheduler for robot: {robot_name}")
    
    execution_success = True
    tasks = task_data["robots"].get(robot_name, [])
    all_robots = set(task_data["robots"].keys())
    sync_participation = count_robots_per_sync_key(task_data)

    logger.info(f"[SCHEDULER] Task execution plan - {len(tasks)} tasks assigned to {robot_name}")
    logger.info(f"[SCHEDULER] Multi-robot system - Total participants: {sorted(all_robots)}")

    # Initialize sequential coordination manager
    sequential_manager = ImprovedSequentialManager(node, robot_name, all_robots)
    
    # Allow time for ROS network establishment
    logger.debug("[SCHEDULER] Waiting for ROS communication initialization")
    time.sleep(2.0)

    next_owner_map = ImprovedSequentialManager.build_next_sequence_owner_map(task_data)

    try:
        for task_idx, task in enumerate(tasks):
            task["robot"] = robot_name
            task_id = task.get("task_id")
            sequence = task.get("sequence")
            sync_group = task.get("sync_group")
            action = task.get("action")
            
            logger.info(f"[SCHEDULER] Processing task {task_idx + 1}/{len(tasks)}: {task_id}")
            logger.info(f"[SCHEDULER] Task configuration - Action: {action}, Sequence: {sequence}, Sync Group: {sync_group}")
            
            if sync_group is not None:
                sync_group = int(sync_group)

            # Sequential execution with three-way handshake
            if sequence is not None and sync_group is None:
                logger.info(f"[SCHEDULER] Executing sequential task with handshake protocol")
                
                prev_sequence = prev_stage.get(sequence)
                
                # Handle sequential dependencies
                if prev_sequence is not None:
                    prev_owner = seq_owner_map.get(prev_sequence)
                    if prev_owner and prev_owner != robot_name:
                        logger.info(f"[SCHEDULER] Waiting for prerequisite sequence {prev_sequence} from {prev_owner}")
                        tts_manager.say_sync("Waiting for prerequisite task completion from other robots")
                        
                        wait_success = sequential_manager.wait_for_previous_sequence(sequence, prev_sequence, prev_owner)
                        if not wait_success:
                            logger.error(f"[SCHEDULER] Failed to receive prerequisite sequence {prev_sequence} completion")
                            execution_success = False
                            continue

                # Execute with handshake confirmation
                next_owner = next_owner_map.get(sequence)
                task_success = sequential_manager.execute_sequence_with_handshake(
                    task, executor, next_owner
                )
                if not task_success:
                    logger.error(f"[SCHEDULER] Sequential task execution failed: {task_id}")
                    execution_success = False
                continue

            # Synchronous execution with consensus protocol
            elif sync_group is not None and sequence is None:
                logger.info(f"[SCHEDULER] Executing synchronous task with consensus protocol")
                
                target_participants = sync_participation.get((None, sync_group), 2)
                sync_manager = RobotSyncManager(node, robot_name, sync_group, target_participants)

                sync_success = sync_manager.wait_for_sync()
                if not sync_success:
                    logger.error(f"[SCHEDULER] Synchronization failed for group {sync_group}")
                    tts_manager.say_sync("Synchronization protocol failed or timeout occurred")
                    execution_success = False
                    return False

                logger.info(f"[SCHEDULER] Synchronization successful, executing task: {task_id}")
                task_success = execute_action(node, executor, task)
                if not task_success:
                    logger.error(f"[SCHEDULER] Synchronous task execution failed: {task_id}")
                    execution_success = False
                continue

            # Independent local execution
            else:
                logger.info(f"[SCHEDULER] Executing independent local task")
                task_success = execute_action(node, executor, task)
                if not task_success:
                    logger.error(f"[SCHEDULER] Local task execution failed: {task_id}")
                    execution_success = False

        # Final execution summary
        if execution_success:
            logger.info(f"[SCHEDULER] Task execution completed successfully for robot: {robot_name}")
            logger.info(f"[SCHEDULER] All {len(tasks)} tasks executed without errors")
        else:
            logger.warning(f"[SCHEDULER] Task execution completed with errors for robot: {robot_name}")
            
        return execution_success

    except Exception as e:
        logger.exception(f"[SCHEDULER] Critical error during task execution for {robot_name}: {e}")
        return False

# =========================
# Main Entry Point
# =========================
def run(task_data: Dict[str, Any]):
    """Enhanced multi-robot task scheduler entry point"""
    
    logger.info("[SYSTEM] Starting enhanced multi-robot coordination system")
    
    # Validate task plan
    try:
        validate_local_plan(task_data)
    except Exception as e:
        logger.error(f"[SYSTEM] Task plan validation failed: {e}")
        return False

    robot_id = config.get("robot_id")
    execution_result = False

    logger.info(f"[SYSTEM] Initializing scheduler for robot: {robot_id}")

    # Initialize ROS environment
    try:
        rclpy.init()
        logger.info("[SYSTEM] ROS environment initialized successfully")
    except RuntimeError as e:
        logger.warning(f"[SYSTEM] ROS already initialized: {e}")

    # Create ROS node and executor
    try:
        ros_node = rclpy.create_node(f"enhanced_scheduler_{robot_id}")
        executor = MultiThreadedExecutor()
        executor.add_node(ros_node)
        
        logger.info(f"[SYSTEM] ROS node created: enhanced_scheduler_{robot_id}")
        logger.info("[SYSTEM] Multi-threaded executor configured")

        # Start executor in background thread
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        logger.info("[SYSTEM] ROS executor thread started")

        try:
            # Preprocess task data
            ensure_task_ids(task_data)
            dependency_map = build_prev_stage_map(task_data)
            ownership_map = build_seq_owner_map(task_data)
            
            logger.info("[SYSTEM] Task preprocessing completed")
            logger.info("[SYSTEM] Beginning coordinated task execution")
            
            # Execute scheduler
            execution_result = run_improved_scheduler_for_robot(
                ros_node, robot_id, task_data, executor, dependency_map, ownership_map
            )
            
            if execution_result:
                logger.info(f"[SYSTEM] Multi-robot coordination completed successfully for {robot_id}")
            else:
                logger.error(f"[SYSTEM] Multi-robot coordination failed for {robot_id}")
            
        except Exception as e:
            logger.exception(f"[SYSTEM] Scheduler execution error: {e}")
            execution_result = False
            
        finally:
            # Cleanup ROS resources
            logger.info(f"[SYSTEM] Initiating cleanup for robot: {robot_id}")
            try:
                executor.remove_node(ros_node)
                logger.debug("[SYSTEM] Node removed from executor")
            except Exception as cleanup_error:
                logger.warning(f"[SYSTEM] Node cleanup warning: {cleanup_error}")
                
            executor.shutdown()
            ros_node.destroy_node()
            logger.debug("[SYSTEM] ROS node destroyed")
            
            try:
                rclpy.shutdown()
                logger.debug("[SYSTEM] ROS environment shutdown")
            except Exception as shutdown_error:
                logger.warning(f"[SYSTEM] ROS shutdown warning: {shutdown_error}")
                
            # Wait for executor thread completion
            executor_thread.join(timeout=1)
            if executor_thread.is_alive():
                logger.warning("[SYSTEM] Executor thread did not terminate gracefully")
            else:
                logger.debug("[SYSTEM] Executor thread terminated successfully")

    except Exception as e:
        logger.exception(f"[SYSTEM] Critical system initialization error: {e}")
        execution_result = False

    logger.info(f"[SYSTEM] Enhanced multi-robot coordination system shutdown complete")
    logger.info(f"[SYSTEM] Final execution status: {'SUCCESS' if execution_result else 'FAILURE'}")
    
    return execution_result

# =========================
# Test Entry Point
# =========================
if __name__ == "__main__":
    """Test harness for enhanced multi-robot coordination system"""
    
    logger.info("[TEST] Initializing test scenario for enhanced coordination system")
    
    test_scenario = {
        "robots": {
            "robot1": [
                {
                    "action": "move", 
                    "parameters": {"direction": "forward", "value": 3, "unit": "seconds"}, 
                    "sequence": 0
                },
                {
                    "action": "turn", 
                    "parameters": {"direction": "left", "value": 90, "unit": "degrees"}, 
                    "sequence": 2
                }
            ],
            "robot2": [
                {
                    "action": "navigate", 
                    "parameters": {"target": "table"}, 
                    "sequence": 1
                },
                {
                    "action": "collect", 
                    "parameters": {"item": "box", "target": "table"}, 
                    "sequence": 3
                }
            ]
        }
    }
    
    logger.info("[TEST] Test scenario configuration:")
    logger.info(f"[TEST] - Participating robots: {list(test_scenario['robots'].keys())}")
    logger.info(f"[TEST] - Total tasks: {sum(len(tasks) for tasks in test_scenario['robots'].values())}")
    
    test_result = run(test_scenario)
    
    if test_result:
        logger.info("[TEST] Test scenario completed successfully")
    else:
        logger.error("[TEST] Test scenario failed")
        
    exit(0 if test_result else 1)