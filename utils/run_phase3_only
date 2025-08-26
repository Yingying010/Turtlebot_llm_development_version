#!/usr/bin/env python3
# 只运行 Phase 3 的运输逻辑
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import time
import threading
import json
import argparse
from loguru import logger


SPEED = 0.05  # m/s
PATH_LEN = 1000  # mm
WAIT_READY_TIMEOUT_SEC = 120
WAIT_GO_TIMEOUT_SEC = 120


class Motion:
    def __init__(self, node: Node, robot_ns: str):
        self.pub = node.create_publisher(Twist, f'/{robot_ns}/cmd_vel', 10)

    def stop(self):
        self.pub.publish(Twist())

    def drive_constant(self, forward: bool, dist_mm: float, speed_mps: float):
        dist_m = dist_mm / 1000.0
        dur = dist_m / speed_mps
        tw = Twist()
        tw.linear.x = speed_mps if forward else -speed_mps
        t0 = time.time()
        while time.time() - t0 < dur:
            self.pub.publish(tw)
            time.sleep(0.02)
        self.stop()


class Phase3Runner:
    def __init__(self, node: Node, robot_id: str):
        self.node = node
        self.robot_id = robot_id
        self.peer_id = "robot1" if robot_id == "robot2" else "robot2"
        self.is_r1 = (robot_id == "robot1")

        self.start_at = None
        self.path_len = PATH_LEN

        self.motion = Motion(node, robot_id)

        self.pub = node.create_publisher(String, "/transport_coordination", 10)
        self.sub = node.create_subscription(String, "/transport_coordination", self.on_msg, 10)

        self.peer_ready_event = threading.Event()
        self.go_event = threading.Event()

    def publish(self, typ, payload):
        msg = String()
        msg.data = json.dumps({"type": typ, **payload, "ts": time.time()})
        self.pub.publish(msg)

    def on_msg(self, msg: String):
        try:
            data = json.loads(msg.data)
            typ = data.get("type")
            who = data.get("who")
            if typ == "ready" and who == self.peer_id:
                self.peer_ready_event.set()
            elif typ == "go":
                self.start_at = float(data["start_at"])
                self.go_event.set()
        except Exception as e:
            logger.warning(f"Failed to parse coordination msg: {e}")

    def run(self):
        self.publish("ready", {"who": self.robot_id})
        self.node.get_logger().info(f"[{self.robot_id}] sent READY, waiting for peer")

        if not self.peer_ready_event.wait(timeout=WAIT_READY_TIMEOUT_SEC):
            logger.error("Peer READY timeout")
            return

        self.node.get_logger().info(f"[{self.robot_id}] both READY")

        if self.is_r1:
            self.start_at = time.time() + 1.0
            self.publish("go", {
                "who": self.robot_id,
                "start_at": self.start_at,
                "dist": self.path_len,
                "speed": SPEED
            })
        else:
            if not self.go_event.wait(timeout=WAIT_GO_TIMEOUT_SEC):
                logger.error("GO wait timeout")
                return

        # 等待到绝对时间点
        while True:
            remain = self.start_at - time.time()
            if remain <= 0:
                break
            time.sleep(0.001 if remain < 0.01 else 0.01)

        self.node.get_logger().info(f"[{self.robot_id}] Start driving!")
        self.motion.drive_constant(forward=self.is_r1, dist_mm=self.path_len, speed_mps=SPEED)
        logger.info("Phase 3 transport done.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot_id", required=True, choices=["robot1", "robot2"])
    args = parser.parse_args()

    rclpy.init()
    node = Node(f"{args.robot_id}_phase3_runner")
    runner = Phase3Runner(node, args.robot_id)

    runner.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
