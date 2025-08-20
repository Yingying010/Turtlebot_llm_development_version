import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import threading
from typing import List, Dict, Set
from datetime import datetime
import sys

def now():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]

class RobotSequenceNode(Node):
    def __init__(self, robot_name: str, stages: List[int], stage_owners: Dict[int, str]):
        super().__init__(f"seq_multi_{robot_name}")
        self.robot_name = robot_name
        self.stages = stages
        self.stage_owners = stage_owners
        self.current_stage_idx = 0

        self.pub = self.create_publisher(String, '/robot_status', 10)
        self.sub = self.create_subscription(String, '/robot_status', self.status_callback, 10)

        self.finished_ack: Dict[int, Set[str]] = {}
        self.waiting_ack_from: Dict[int, Set[str]] = {}
        self.running = True

        self.get_logger().info(f"[INIT] {robot_name} responsible for stages: {self.stages}")
        threading.Thread(target=self.run_sequence, daemon=True).start()

    def run_sequence(self):
        while self.current_stage_idx < len(self.stages):
            stage = self.stages[self.current_stage_idx]

            # 等待上一个阶段完成
            if (stage - 1) in self.stage_owners:
                prev_robot = self.stage_owners[stage - 1]
                if prev_robot != self.robot_name:
                    print(f"{now()} | {self.robot_name} waiting for Stage {stage - 1} to finish...")
                    self.wait_for_stage(stage - 1)

            # 执行当前任务
            print(f"{now()} | ✅ {self.robot_name} ➔ Stage {stage} ➔ running")
            time.sleep(2)
            print(f"{now()} | ✅ {self.robot_name} ➔ Stage {stage} ➔ finished")

            # 广播完成并等待 ack
            next_robots = {r for s, r in self.stage_owners.items() if s == stage + 1}
            self.waiting_ack_from[stage] = set(next_robots)
            self.finished_ack[stage] = set()

            self.broadcast_finished_until_ack(stage, next_robots)
            self.current_stage_idx += 1

        print(f"{now()} | 🚀 {self.robot_name} completed all assigned stages.")
        self.running = False

    def wait_for_stage(self, stage: int):
        done = threading.Event()

        def watcher():
            while self.robot_name not in self.finished_ack.get(stage, set()):
                time.sleep(0.1)
            done.set()

        threading.Thread(target=watcher, daemon=True).start()
        done.wait()

    def broadcast_finished_until_ack(self, stage: int, target_robots: Set[str]):
        start_time = time.time()

        def loop():
            while self.running:
                elapsed = time.time() - start_time
                if elapsed > 60:
                    print(f"{now()} | ⏱️ Timeout: {self.robot_name} stage {stage} ack not received in time. Exiting.")
                    rclpy.shutdown()
                    return

                pending = self.waiting_ack_from[stage] - self.finished_ack[stage]
                if not pending:
                    break

                msg = {
                    "kind": "finished",
                    "robot": self.robot_name,
                    "stage": stage,
                    "ts": time.time()
                }
                self.pub.publish(String(data=json.dumps(msg)))
                print(f"{now()} | {self.robot_name} broadcast stage {stage} ➔ finished to {pending}")
                time.sleep(1.0)

        threading.Thread(target=loop, daemon=True).start()

    def status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            kind = data.get("kind")
            from_robot = data.get("robot")
            stage = data.get("stage")
            to = data.get("to")

            if kind == "finished":
                for s in self.stages:
                    if s - 1 == stage and self.stage_owners.get(stage) == from_robot:
                        # 我负责 s，它依赖 stage = s-1，由对方完成
                        ack = {
                            "kind": "ack_finished",
                            "robot": self.robot_name,
                            "to": from_robot,
                            "stage": stage,
                            "ts": time.time()
                        }
                        self.pub.publish(String(data=json.dumps(ack)))
                        print(f"{now()} | {self.robot_name} ✉️ send ack_finished to {from_robot} for stage {stage}")
                        self.finished_ack.setdefault(stage, set()).add(self.robot_name)

            elif kind == "ack_finished" and to == self.robot_name:
                self.finished_ack.setdefault(stage, set()).add(from_robot)
                print(f"{now()} | {self.robot_name} received ack_finished from {from_robot} for stage {stage}")

        except Exception as e:
            self.get_logger().warn(f"[ParseError] {e}")

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 test_robot_sequence.py <robot_name>")
        return

    robot_name = sys.argv[1]
    stage_owners = {
        0: "robot1",
        1: "robot2",
        2: "robot1",
        3: "robot2"
    }
    stages = [s for s, r in stage_owners.items() if r == robot_name]

    rclpy.init()
    node = RobotSequenceNode(robot_name, stages, stage_owners)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
