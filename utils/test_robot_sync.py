import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys
import time
import threading
from collections import defaultdict

# === 状态缓存 ===
status_cache = defaultdict(dict)  # (sg) -> {robot_name: status}
target_counts = {}  # (sg) -> need count
cache_lock = threading.Lock()

class BarrierTester(Node):
    def __init__(self, robot_name: str, sync_group: int):
        super().__init__('barrier_tester_' + robot_name)
        self.robot = robot_name
        self.sg = sync_group
        self.pub = self.create_publisher(String, '/robot_status', 10)
        self.sub = self.create_subscription(String, '/robot_status', self.status_callback, 10)
        self.timer = self.create_timer(1.0, self.publish_ready)

        # 初始化目标计数（简单起见：认为是2台机器）
        target_counts[self.sg] = 2

        print(f"[INIT] Robot `{self.robot}` in sync_group={self.sg}, target=2")

    def publish_ready(self):
        msg = {
            "kind": "state",
            "robot": self.robot,
            "status": "ready",
            "sync_group": self.sg,
            "ts": time.time()
        }
        self.pub.publish(String(data=json.dumps(msg)))
        print(f"[PUBLISH] {self.robot} → ready")

    def status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("kind") != "state":
                return

            robot = data.get("robot")
            sg = data.get("sync_group")
            status = data.get("status")

            if sg != self.sg:
                return

            with cache_lock:
                status_cache[sg][robot] = status
                need = target_counts.get(sg, 0)
                ready = sum(1 for s in status_cache[sg].values() if s == "ready")
                remaining = max(need - ready, 0)

            print(f"📊 sg={sg} → need={need}, ready={ready}, remaining={remaining}")

        except Exception as e:
            print(f"[ERROR] {e}")

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 test_barrier_minimal.py <robot_name> <sync_group>")
        return

    robot_name = sys.argv[1]
    sync_group = int(sys.argv[2])

    rclpy.init()
    node = BarrierTester(robot_name, sync_group)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("[EXIT] Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
