# ros_lifecycle.py
 
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import time
 
class ROSManager:
    def __init__(self):
        self.executor = None
        self.spin_thread = None
        self.nodes = []
        self.initialized = False
 
    def start(self):
        if self.initialized:
            return
 
        rclpy.init()
        self.executor = MultiThreadedExecutor()
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        self.initialized = True
        print("✅ ROSManager started")
 
    def add_node(self, node: Node):
        self.executor.add_node(node)
        self.nodes.append(node)
 
    def shutdown(self):
        print("🛑 ROSManager shutting down...")
        for node in self.nodes:
            if node is not None and node.context.ok():
                print(f"🧹 Destroying node: {node.get_name()}")
                node.destroy_node()
 
        rclpy.shutdown()
        self.initialized = False
        print("✅ ROSManager shutdown complete")
 
    def wait(self, delay=0.5):
        """Optional helper to wait before shutdown."""
        time.sleep(delay)