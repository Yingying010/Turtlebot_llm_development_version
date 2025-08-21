#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import json, time, threading
from typing import Set, Dict, Optional, List, Any
import datetime

def now():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

# 预设测试数据
TEST_DATA_1 = {
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

class ThreeWayHandshakeManager:
    """管理三次握手确认机制"""
    def __init__(self, robot_name: str, publisher):
        self.robot_name = robot_name
        self.publisher = publisher
        
        # 握手状态跟踪
        self.sent_finished: Dict[int, float] = {}  # seq -> timestamp
        self.received_acks: Dict[int, Set[str]] = {}  # seq -> {robot_names}
        self.sent_ack_acks: Dict[int, Set[str]] = {}  # seq -> {robot_names}
        
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
        print(f"{now()} | 📤 {self.robot_name} → SYN: finished seq={sequence}")
        
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
        print(f"{now()} | 🤝 {self.robot_name} → ACK: to {to_robot} seq={sequence}")
        
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
        print(f"{now()} | ✅ {self.robot_name} → ACK-ACK: to {to_robot} seq={sequence}")
        
    def handle_ack(self, from_robot: str, sequence: int) -> bool:
        """处理收到的ACK，返回是否完成握手"""
        with self.lock:
            if sequence not in self.received_acks:
                self.received_acks[sequence] = set()
            self.received_acks[sequence].add(from_robot)
            
        print(f"{now()} | 🎯 {self.robot_name} received ACK from {from_robot} seq={sequence}")
        
        # 自动发送ACK-ACK
        self.send_ack_ack(from_robot, sequence)
        return True
        
    def handle_ack_ack(self, from_robot: str, sequence: int):
        """处理收到的ACK-ACK"""
        print(f"{now()} | 🎉 {self.robot_name} received ACK-ACK from {from_robot} seq={sequence}")
        
    def is_handshake_complete(self, sequence: int, expected_robots: Set[str]) -> bool:
        """检查三次握手是否完成"""
        with self.lock:
            received = self.received_acks.get(sequence, set())
            ack_acked = self.sent_ack_acks.get(sequence, set())
            
            # 需要收到所有期待机器人的ACK，并发送了所有ACK-ACK
            return received >= expected_robots and ack_acked >= expected_robots

class SequentialRobotWithHandshake(Node):
    def __init__(self, robot_name: str, test_data: Dict[str, Any]):
        super().__init__(f"seq_robot_{robot_name}")
        self.robot_name = robot_name
        self.test_data = test_data
        self.tasks = test_data["robots"].get(robot_name, [])
        
        # Build dependencies
        self.prev_stage_map = self.build_prev_stage_map()
        self.seq_owner_map = self.build_seq_owner_map()
        self.all_robots = set(test_data["robots"].keys())
        
        # Create QoS profile for reliable communication
        qos_profile = QoSProfile(depth=50)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        # ROS communication
        self.pub = self.create_publisher(String, '/robot_status', qos_profile)
        self.sub = self.create_subscription(String, '/robot_status', self.status_callback, qos_profile)
        
        # 三次握手管理器
        self.handshake_mgr = ThreeWayHandshakeManager(robot_name, self.pub)
        
        # 状态跟踪
        self.completed_sequences: Set[int] = set()
        self.sequence_events: Dict[int, threading.Event] = {}
        self.running = True
        
        # 超时保护
        self.timeout_duration = 120.0
        self.start_time = time.time()
        self.timeout_timer = self.create_timer(2.0, self.check_timeout)
        
        self.get_logger().info(f"[INIT] Robot `{robot_name}` with {len(self.tasks)} tasks")
        print(f"{now()} | 🤖 {robot_name} initialized with sequences: {[t.get('sequence') for t in self.tasks]}")
        print(f"{now()} | 📊 Dependencies: {self.prev_stage_map}")
        print(f"{now()} | 👥 Owners: {self.seq_owner_map}")
        print(f"{now()} | 🌐 All robots: {sorted(self.all_robots)}")
        
        # 延迟启动
        self.startup_timer = self.create_timer(3.0, self.delayed_start)

    def delayed_start(self):
        """延迟启动"""
        self.startup_timer.cancel()
        print(f"{now()} | 🚀 {self.robot_name} starting execution with three-way handshake")
        threading.Thread(target=self.run_all_tasks, daemon=True).start()

    def build_prev_stage_map(self) -> Dict[int, Optional[int]]:
        """构建前驱阶段映射"""
        stages = sorted({
            t["sequence"]
            for tasks in self.test_data.get("robots", {}).values()
            for t in tasks
            if t.get("sequence") is not None
        })
        prev = {}
        for i, s in enumerate(stages):
            prev[s] = stages[i - 1] if i > 0 else None
        return prev

    def build_seq_owner_map(self) -> Dict[int, str]:
        """构建序列所有者映射"""
        owner = {}
        for robot, tasks in self.test_data.get("robots", {}).items():
            for t in tasks:
                s = t.get("sequence")
                if s is not None:
                    owner[int(s)] = robot
        return owner

    def check_timeout(self):
        """超时检查"""
        if not self.running:
            self.timeout_timer.cancel()
            return
            
        elapsed = time.time() - self.start_time
        if elapsed > self.timeout_duration:
            print(f"{now()} | ⏱️ Timeout! {self.robot_name} failed to complete in time.")
            self.timeout_timer.cancel()
            rclpy.shutdown()

    def status_callback(self, msg):
        """处理收到的状态消息"""
        try:
            data = json.loads(msg.data)
            kind = data.get("kind")
            robot = data.get("robot")
            sequence = data.get("sequence")
            to_robot = data.get("to")

            if kind == "finished" and robot != self.robot_name:
                print(f"{now()} | 📨 {self.robot_name} received SYN: {robot} finished seq={sequence}")
                # 发送ACK确认
                self.handshake_mgr.send_ack(robot, sequence)
                
                # 对于其他机器人完成的sequence，我们收到SYN就认为握手开始
                # 这里我们可以立即标记为已完成，因为我们已经发送了ACK
                self.completed_sequences.add(sequence)
                if sequence in self.sequence_events:
                    print(f"{now()} | 🚦 {self.robot_name} signaling completion for seq={sequence} after SYN")
                    self.sequence_events[sequence].set()
                
            elif kind == "ack_finished" and to_robot == self.robot_name:
                print(f"{now()} | 📨 {self.robot_name} received ACK from {robot} seq={sequence}")
                # 处理ACK并发送ACK-ACK
                self.handshake_mgr.handle_ack(robot, sequence)
                
                # 检查握手是否完成（对于自己发起的sequence）
                expected_robots = self.all_robots - {self.robot_name}
                if self.handshake_mgr.is_handshake_complete(sequence, expected_robots):
                    print(f"{now()} | 🎊 {self.robot_name} three-way handshake complete for seq={sequence}")
                    self.completed_sequences.add(sequence)
                    if sequence in self.sequence_events:
                        self.sequence_events[sequence].set()
                
            elif kind == "ack_ack_finished" and to_robot == self.robot_name:
                print(f"{now()} | 📨 {self.robot_name} received ACK-ACK from {robot} seq={sequence}")
                self.handshake_mgr.handle_ack_ack(robot, sequence)

        except Exception as e:
            self.get_logger().warn(f"Parse error: {e}")

    def wait_for_previous_sequence(self, sequence: int) -> bool:
        """等待前序任务完成（通过三次握手确认）"""
        prev_seq = self.prev_stage_map.get(sequence)
        if prev_seq is None:
            print(f"{now()} | ✅ {self.robot_name}: No dependency for seq={sequence}")
            return True
            
        prev_owner = self.seq_owner_map.get(prev_seq)
        if prev_owner == self.robot_name:
            print(f"{now()} | 📝 {self.robot_name}: Previous seq={prev_seq} is mine, no waiting needed")
            return True

        # 检查是否已经完成（可能在启动前就收到了消息）
        if prev_seq in self.completed_sequences:
            print(f"{now()} | ✅ {self.robot_name}: Previous seq={prev_seq} already completed with handshake")
            return True
            
        print(f"{now()} | ⏳ {self.robot_name} waiting for seq={prev_seq} from {prev_owner} (with handshake)")
        print(f"{now()} | 📊 Current completed sequences: {sorted(self.completed_sequences)}")
        
        # 创建等待事件
        if prev_seq not in self.sequence_events:
            self.sequence_events[prev_seq] = threading.Event()
        
        # 再次检查，可能在创建事件期间已经完成
        if prev_seq in self.completed_sequences:
            print(f"{now()} | ✅ {self.robot_name}: seq={prev_seq} completed while setting up wait")
            return True
        
        # 等待握手完成，给更多时间让消息传播
        print(f"{now()} | 🕐 {self.robot_name} starting wait for seq={prev_seq}")
        success = self.sequence_events[prev_seq].wait(timeout=30.0)
        
        if success:
            print(f"{now()} | 🎉 {self.robot_name}: seq={prev_seq} handshake completed, can start seq={sequence}")
        else:
            print(f"{now()} | ❌ {self.robot_name}: Timeout waiting for seq={prev_seq} handshake")
            print(f"{now()} | 📊 Final completed sequences: {sorted(self.completed_sequences)}")
            
        return success

    def execute_task(self, task: Dict[str, Any]) -> bool:
        """执行单个任务"""
        action = task.get("action", "unknown")
        params = task.get("parameters", {})
        sequence = task.get("sequence")
        
        print(f"{now()} | 🎯 {self.robot_name} executing {action} (seq={sequence})")
        
        # 模拟任务执行
        if action == "move":
            duration = min(params.get("value", 2), 2)
            print(f"{now()} | 🚗 {self.robot_name} moving {params.get('direction')} for {duration}s")
            time.sleep(duration)
        elif action == "turn":
            print(f"{now()} | 🔄 {self.robot_name} turning {params.get('direction')} {params.get('value', 90)}°")
            time.sleep(1.0)
        elif action == "navigate":
            target = params.get("target", "unknown")
            print(f"{now()} | 🧭 {self.robot_name} navigating to {target}")
            time.sleep(1.5)
        elif action == "collect":
            item = params.get("item", "unknown")
            target = params.get("target", "unknown")
            print(f"{now()} | 📦 {self.robot_name} collecting {item} from {target}")
            time.sleep(1.5)
        else:
            time.sleep(1.0)
            
        print(f"{now()} | ✅ {self.robot_name} completed {action} (seq={sequence})")
        return True

    def run_all_tasks(self):
        """运行所有任务"""
        print(f"{now()} | 🚀 {self.robot_name} starting task execution")
        
        for task in self.tasks:
            if not self.running:
                break
                
            sequence = task.get("sequence")
            
            # 等待依赖
            if not self.wait_for_previous_sequence(sequence):
                print(f"{now()} | ❌ {self.robot_name} failed to wait for dependencies")
                continue
                
            # 执行任务
            print(f"{now()} | 🎬 {self.robot_name} starting seq={sequence}")
            success = self.execute_task(task)
            
            if success:
                # 启动三次握手
                print(f"{now()} | 🤝 {self.robot_name} starting handshake for seq={sequence}")
                self.handshake_mgr.announce_finished(sequence)
                
                # 等待握手完成（对自己的任务，需要等其他机器人确认）
                expected_robots = self.all_robots - {self.robot_name}
                timeout_count = 0
                while not self.handshake_mgr.is_handshake_complete(sequence, expected_robots):
                    time.sleep(0.5)
                    timeout_count += 1
                    if timeout_count > 20:  # 10秒超时
                        print(f"{now()} | ⚠️ {self.robot_name} handshake timeout for seq={sequence}")
                        break
                        
                if timeout_count <= 20:
                    print(f"{now()} | ✅ {self.robot_name} handshake confirmed for seq={sequence}")
                    self.completed_sequences.add(sequence)
                
            else:
                print(f"{now()} | ❌ {self.robot_name} failed to execute task")
                
        print(f"{now()} | 🏁 {self.robot_name} completed all tasks!")
        self.running = False

def main():
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python3 test_handshake.py <robot_name>")
        print("Available robots: robot1, robot2")
        print("\nExample:")
        print("  Terminal 1: python3 test_handshake.py robot1")
        print("  Terminal 2: python3 test_handshake.py robot2")
        print("\n🤝 This version includes THREE-WAY HANDSHAKE:")
        print("  1. SYN: Robot announces 'finished'")
        print("  2. ACK: Other robots acknowledge")  
        print("  3. ACK-ACK: Original robot confirms receipt")
        return

    robot_name = sys.argv[1]
    test_data = TEST_DATA_1
    
    if robot_name not in test_data["robots"]:
        print(f"❌ Robot '{robot_name}' not found in test data")
        return
    
    print(f"🤖 Starting {robot_name} with THREE-WAY HANDSHAKE")
    print(f"⚠️  Start both robots within 10 seconds!")
    
    rclpy.init()
    node = SequentialRobotWithHandshake(robot_name, test_data)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n{now()} | 🛑 {robot_name} interrupted by user")
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()