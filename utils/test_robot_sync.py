#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import json
import time

class SyncTestNode(Node):
    def __init__(self, robot_name):
        super().__init__(f"sync_tester_{robot_name}")
        self.robot_name = robot_name
        self.pub = self.create_publisher(String, '/robot_status', 10)
        self.sub = self.create_subscription(String, '/robot_status', self.status_callback, 10)
        self.status_cache = {}  # {robot_name: status}
        self.event_ready = threading.Event()
        self.sync_group = 0  # 固定使用 sg=0

        self.timer = self.create_timer(0.5, self.send_ready_once)

    def send_ready_once(self):
        self.publish_status("ready")
        self.timer.cancel()  # 只发一次 ready
        self.get_logger().info("📤 Sent 'ready'")

        # 等对方 ready
        threading.Thread(target=self.wait_and_run, daemon=True).start()

    def status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("kind") != "state":
                return
            robot = data.get("robot")
            status = data.get("status")
            sg = data.get("sync_group")
            if sg != self.sync_group or robot == self.robot_name:
                return
            self.status_cache[robot] = status
            self.get_logger().info(f"📥 Got status: {robot} → {status}")
            if status == "ready":
                self.event_ready.set()
        except Exception as e:
            self.get_logger().warn(f"Parse failed: {e}")

    def wait_and_run(self):
        self.get_logger().info("⏳ Waiting for other robot to be 'ready'...")
        ok = self.event_ready.wait(timeout=5)
        if not ok:
            self.get_logger().warn("⚠️ Timeout waiting for other robot")
        else:
            self.get_logger().info("✅ Both robots are ready")

        self.publish_status("running")
        self.get_logger().info("🚀 Running task...")
        time.sleep(1.0)
        self.publish_status("finished")
        self.get_logger().info("✅ Finished task.")

    def publish_status(self, status):
        msg = {
            "kind": "state",
            "robot": self.robot_name,
            "sync_group": self.sync_group,
            "status": status,
            "ts": time.time()
        }
        out = String()
        out.data = json.dumps(msg)
        self.pub.publish(out)

def main():
    import sys
    if len(sys.argv) < 2:
        print("Usage: python3 test_robot_sync.py <robot_name>")
        return

    robot_name = sys.argv[1]
    rclpy.init()
    node = SyncTestNode(robot_name)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
