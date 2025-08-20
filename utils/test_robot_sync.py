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

        self.pub = self.create_publisher(String, '/robot_status', 10)
        self.sub = self.create_subscription(String, '/robot_status', self.status_callback, 10)

        self.go_event = threading.Event()

        self.ready_timer = self.create_timer(1.0, self.send_ready)
        self.ack_sent = False

        self.get_logger().info(f"[INIT] Robot `{robot_name}` in sync_group={sync_group}, target={target_count}")

    def send_ready(self):
        self.publish_status("ready")

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

            if status == "ready":
                self.ready_set.add(robot)
                print(f"{now()} | 🤝 {self.robot_name} sees ready_set = {self.ready_set}")
                if len(self.ready_set) == self.target_count and not self.ack_sent:
                    self.ack_sent = True
                    self.publish_status("ack_ready")
                    self.ack_set.add(self.robot_name)
                    print(f"{now()} | ✅ {self.robot_name} sees all ready, sending ack_ready")

            elif status == "ack_ready":
                self.ack_set.add(robot)
                print(f"{now()} | 📬 {self.robot_name} got ack_ready from {robot}, ack_set = {self.ack_set}")
                if len(self.ack_set) == self.target_count and not self.go_event.is_set():
                    self.go_event.set()

        except Exception as e:
            self.get_logger().warn(f"Parse error: {e}")

    def run_task(self):
        print(f"{now()} | ⏳ {self.robot_name} waiting for ALL ack_ready...")
        self.go_event.wait()
        print(f"{now()} | 🚀 All robots GO! {self.robot_name} starting task...")
        self.publish_status("running")
        time.sleep(3)  # 模拟任务
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

    # 开新线程等待 ack 完成后执行任务
    threading.Thread(target=node.run_task, daemon=True).start()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
