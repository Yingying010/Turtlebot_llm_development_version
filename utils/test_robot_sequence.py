#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys
import time
import threading
from typing import Dict
from datetime import datetime

def now():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]

# 机器人任务映射（可根据需要修改）
TASK_SEQUENCE = [
    ("robot1", 0),
    ("robot2", 1),
    ("robot1", 2),
    ("robot2", 3),
]

class SequenceMultiTaskNode(Node):
    def __init__(self, robot_name):
        super().__init__(f"seq_multi_{robot_name}")
        self.robot_name = robot_name
        self.pub = self.create_publisher(String, "/stage_status", 10)
        self.sub = self.create_subscription(String, "/stage_status", self.status_callback, 10)

        self.stage_status: Dict[int, str] = {}  # {stage_idx: "finished"}
        self.lock = threading.Lock()

        self.my_stages = [s for r, s in TASK_SEQUENCE if r == robot_name]
        self.stage_events: Dict[int, threading.Event] = {s: threading.Event() for s in self.my_stages}

        self.get_logger().info(f"[INIT] {robot_name} responsible for stages: {self.my_stages}")

        threading.Thread(target=self.run_all_stages, daemon=True).start()

    def status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            stage = data["stage"]
            status = data["status"]
            with self.lock:
                self.stage_status[stage] = status
                # 触发下一个阶段的等待者
                for s in self.my_stages:
                    if stage == s - 1 and status == "finished":
                        self.stage_events[s].set()
        except Exception as e:
            self.get_logger().warn(f"Parse error: {e}")

    def publish_status(self, stage: int, status: str):
        msg = {
            "robot": self.robot_name,
            "stage": stage,
            "status": status,
            "ts": time.time()
        }
        self.pub.publish(String(data=json.dumps(msg)))
        print(f"{now()} | 🛰️ {self.robot_name} → Stage {stage} → {status}")

    def run_all_stages(self):
        for stage in self.my_stages:
            if stage > 0:
                print(f"{now()} | {self.robot_name} waiting for Stage {stage - 1} to finish...")
                self.stage_events[stage].wait()

            self.publish_status(stage, "running")
            time.sleep(3)  # 模拟任务执行
            self.publish_status(stage, "finished")
            print(f"{now()} | ✅ {self.robot_name} completed Stage {stage}")

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 test_robot_sequence_multi.py <robot_name>")
        return
    robot_name = sys.argv[1]
    rclpy.init()
    node = SequenceMultiTaskNode(robot_name)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
