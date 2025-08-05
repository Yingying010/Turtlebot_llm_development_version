import os, sys, json, time, traceback, threading
from collections import defaultdict
from typing import Dict, Any

PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)

import rclpy
from std_msgs.msg import String
from loguru import logger
from textwrap import dedent

from robotControllerRepo.robot_controller import execute_action
from utils.ros_lifecycle import ROSManager
from config import config

# ── 全局缓存 ───────────────────────────────────────────────
status_cache = defaultdict(lambda: defaultdict(str))   # {(seq,sync):{robot:status}}
status_pub = None

# ── 工具函数 ───────────────────────────────────────────────
def count_robots_per_sync_key(task_data: Dict[str, Any]) -> Dict[tuple, int]:
    counter = defaultdict(set)
    for robot, tasks in task_data["robots"].items():
        for t in tasks:
            sg, seq = t.get("sync_group"), t.get("sequence")
            if sg is not None and seq is not None:
                counter[(seq, sg)].add(robot)
    return {k: len(v) for k, v in counter.items()}

def status_rank(s: str) -> int:
    return {"ready": 1, "running": 2, "finished": 3}.get(s, 0)

# ── ROS 回调 ───────────────────────────────────────────────
def status_callback(msg):
    try:
        data = json.loads(msg.data)
        key = (data["sequence"], data["sync_group"])
        status_cache[key][data["robot"]] = data["status"]
    except Exception as e:
        print(f"⚠️ status_callback error: {e}")

# ── 等待同步 ───────────────────────────────────────────────
def wait_for_all_status(sync_key: tuple, need_cnt: int, expect: str):
    while True:
        ready = [r for r, s in status_cache[sync_key].items()
                 if status_rank(s) >= status_rank(expect)]
        print(f"🔍 Waiting {expect}: {len(ready)}/{need_cnt} in {sync_key}")
        if len(ready) >= need_cnt:
            break
        time.sleep(0.2)

# ── 周期发布 ───────────────────────────────────────────────
def start_periodic_publisher(robot, task_id, seq, sg, st, stop_evt):
    def loop():
        msg = json.dumps({"robot": robot, "task_id": task_id,
                          "sequence": seq, "sync_group": sg, "status": st})
        while not stop_evt.is_set() and rclpy.ok():
            status_pub.publish(String(data=msg))
            time.sleep(0.5)
    th = threading.Thread(target=loop, daemon=True)
    th.start();  return th

# ── Task 调度核心 ──────────────────────────────────────────
def run_scheduler_for_robot(node, robot, data, targets, executor) -> bool:
    tasks = sorted(data["robots"].get(robot, []), key=lambda x: x["sequence"])
    try:
        for t in tasks:
            t["robot"] = robot
            seq, sg = t["sequence"], t.get("sync_group")
            key = (seq, sg)
            if sg:
                ev = threading.Event()
                start_periodic_publisher(robot, t["task_id"], seq, sg, "ready", ev)
                wait_for_all_status(key, targets[key], "ready")
                ev.set()
            execute_action(node, t, executor)
            if sg:
                ev = threading.Event()
                start_periodic_publisher(robot, t["task_id"], seq, sg, "finished", ev)
                wait_for_all_status(key, targets[key], "finished")
                ev.set()
            time.sleep(0.3)
        print(f"🎯 `{robot}` all tasks done")
        return True
    except Exception:
        logger.warning(f"⚠️ Failed:\n{traceback.format_exc()}")
        return False

# ── 顶层入口 ───────────────────────────────────────────────
def run(task_data: Dict[str, Any]) -> bool:
    robot_id = config.get("robot_id")
    ros = ROSManager();  ros.start()

    node = rclpy.create_node(f"status_node_{robot_id}")
    ros.add_node(node)

    global status_pub
    status_pub = node.create_publisher(String, "/robot_status", 10)
    node.create_subscription(String, "/robot_status", status_callback, 10)

    targets = count_robots_per_sync_key(task_data)
    try:
        return run_scheduler_for_robot(node, robot_id, task_data, targets, ros.executor)
    finally:
        time.sleep(1.0)
        ros.shutdown()

# ── 方便本地测试 ───────────────────────────────────────────
if __name__ == "__main__":
    raw = dedent("""
    {
      "robots": {
        "robot1": [
          {"task_id": "t0","action":"move","parameters":{"direction":"forward","value":2,"unit":"meters"},"sync_group":null,"sequence":1},
          {"task_id": "t1","action":"turn","parameters":{"direction":"left","value":90,"unit":"degrees"},"sync_group":null,"sequence":2}
        ]
      }
    }""")
    run(json.loads(raw))
