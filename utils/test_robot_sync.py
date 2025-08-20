#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json, time, threading
from typing import Dict
import datetime

def now():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

class FullSyncNode(Node):
    def __init__(self, robot_name, sync_group, target_count):
        super().__init__(f"sync_task_{robot_name}")
        self.robot_name = robot_name
        self.sync_group = int(sync_group)
        self.target_count = int(target_count)
        self.status_cache: Dict[str, str] = {}  # {robot: status}
        self.pub = self.create_publisher(String, '/robot_status', 10)
        self.sub = self.create_subscription(String, '/robot_status', self.status_callback, 10)
        self.go_sub = self.create_subscription(String, '/robot_go', self.go_callback, 10)

        self.ready_timer = self.create_timer(1.0, self.publish_ready)
        self.sync_event = threading.Event()
        self.go_event = threading.Event()
        self.start_at = None  # 时间戳

        self.get_logger().info(f"[INIT] Robot `{robot_name}` in sync_group={sync_group}, target={target_count}")

    def publish_ready(self):
        self.publish_status("ready")
        self.check_sync_status()

    def publish_status(self, status):
        msg = {
            "kind": "state",
            "robot": self.robot_name,
            "sync_group": self.sync_group,
            "status": status,
            "ts": time.time()
        }
        self.pub.publish(String(data=json.dumps(msg)))
        print(f"{now()} | 📤 PUBLISH {self.robot_name} → {status}")

    def status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("kind") != "state":
                return
            robot = data["robot"]
            status = data["status"]
            sg = data["sync_group"]
            if sg != self.sync_group:
                return
            self.status_cache[robot] = status
            self.check_sync_status()
        except Exception as e:
            self.get_logger().warn(f"Parse error: {e}")

    def check_sync_status(self):
        ready_count = sum(1 for s in self.status_cache.values() if s == "ready")
        remaining = self.target_count - ready_count
        print(f"{now()} | sync_group_id={self.sync_group} [ need={self.target_count}, ready={ready_count}, remaining={remaining} ]")

        if ready_count >= self.target_count and not self.sync_event.is_set():
            self.sync_event.set()
            self.ready_timer.cancel()
            if self.robot_name == "robot1":
                self.publish_sync_go()

    def publish_sync_go(self, delay=0.3):
        self.start_at = time.time() + delay
        msg = {
            "kind": "go",
            "sync_group": self.sync_group,
            "start_at": self.start_at,
            "ts": time.time()
        }
        self.pub.publish(String(data=json.dumps(msg)))
        print(f"{now()} | 🚦 {self.robot_name} publish GO: start_at = {self.start_at:.3f}")

    def go_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("kind") != "go":
                return
            sg = data["sync_group"]
            if sg != self.sync_group:
                return
            self.start_at = data["start_at"]
            self.go_event.set()
        except Exception as e:
            self.get_logger().warn(f"Parse go error: {e}")

    def run_task(self):
        print(f"{now()} | ⏳ {self.robot_name} waiting for GO...")
        self.go_event.wait()
        delay = self.start_at - time.time()
        if delay > 0:
            time.sleep(delay)
        print(f"{now()} | 🚀 All robots GO! {self.robot_name} starting task...")
        self.publish_status("running")
        time.sleep(3)  # 模拟执行耗时
        self.publish_status("finished")
        print(f"{now()} | ✅ {self.robot_name} finished task.")

def main():
    import sys
    if len(sys.argv) < 4:
        print("Usage: python3 test_full_sync_task.py <robot_name> <sync_group> <target_count>")
        return
    robot_name, sync_group, target_count = sys.argv[1], sys.argv[2], sys.argv[3]
    rclpy.init()
    node = FullSyncNode(robot_name, sync_group, target_count)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
