#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json, time, threading
from typing import Dict, Set
import datetime

def now():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

class FullSyncNode(Node):
    def __init__(self, robot_name, sync_group, target_count):
        super().__init__(f"sync_task_{robot_name}")
        self.robot_name = robot_name
        self.sync_group = int(sync_group)
        self.target_count = int(target_count)

        self.ready_set: Set[str] = set()
        self.ack_set: Set[str] = set()
        self.prev_ready_set: Set[str] = set()

        self.pub = self.create_publisher(String, '/robot_status', 10)
        self.sub = self.create_subscription(String, '/robot_status', self.status_callback, 10)

        self.ready_timer = self.create_timer(1.0, self.publish_ready)
        self.sync_event = threading.Event()
        self.task_thread_started = False

        self.get_logger().info(f"[INIT] Robot `{robot_name}` in sync_group={sync_group}, target={target_count}")
        print(f"{now()} | {robot_name} waiting for ALL ack_ready...")

    def publish_ready(self):
        msg = {
            "kind": "ready",
            "robot": self.robot_name,
            "sync_group": self.sync_group,
            "ts": time.time()
        }
        self.pub.publish(String(data=json.dumps(msg)))
        print(f"{now()} | 📤 PUBLISH {self.robot_name} → ready")

    def publish_ack(self, to_robot):
        msg = {
            "kind": "ack_ready",
            "robot": self.robot_name,
            "to": to_robot,
            "sync_group": self.sync_group,
            "ts": time.time()
        }
        self.pub.publish(String(data=json.dumps(msg)))
        print(f"{now()} | 📤 {self.robot_name} send ack_ready to {to_robot}")

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
            self.get_logger().warn(f"Parse error: {e}")

    def print_ready_set(self):
        if self.ready_set != self.prev_ready_set:
            print(f"{now()} | {self.robot_name} sees ready_set = {sorted(self.ready_set)}")
            self.prev_ready_set = set(self.ready_set)

    def check_sync_complete(self):
        if len(self.ready_set) >= self.target_count and len(self.ack_set) >= self.target_count and not self.task_thread_started:
            self.task_thread_started = True
            print(f"{now()} | ✅ All robots OK! {self.robot_name} starting task...")
            threading.Thread(target=self.run_task, daemon=True).start()

    def run_task(self):
        self.start_time = time.time()
        print(f"{now()} | 🚀 {self.robot_name} running task at {self.start_time:.6f}")
        time.sleep(3)
        print(f"{now()} | ✅ {self.robot_name} finished task at {time.time():.6f}")

def main():
    import sys
    if len(sys.argv) < 4:
        print("Usage: python3 test_robot_sync.py <robot_name> <sync_group> <target_count>")
        return
    robot_name, sync_group, target_count = sys.argv[1], sys.argv[2], sys.argv[3]
    rclpy.init()
    node = FullSyncNode(robot_name, sync_group, target_count)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
