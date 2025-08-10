#!/usr/bin/env python3
import os, sys, json, time, threading, traceback
from collections import defaultdict
from typing import Dict, Any
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from robotControllerRepo.robot_controller import execute_action
from config import config
from loguru import logger
from textwrap import dedent

# === 全局变量 ===
ros_node = None
status_pub = None
status_cache = defaultdict(lambda: defaultdict(str))  # {(seq, sync_group): {robot: status}}
target_counts: Dict[tuple, int] = {}

# 心跳
HEARTBEAT_INTERVAL = 2.0
_heartbeat_stop = threading.Event()
_heartbeat_thread: threading.Thread | None = None

# 事件 & 锁
status_events = defaultdict(threading.Event)
cache_lock = threading.Lock()

def status_rank(s: str) -> int:
    return {"ready": 1, "running": 2, "finished": 3}.get(s, 0)

# === 提取任务中每个 sync_group 的机器人数量 ===
def count_robots_per_sync_key(task_data: Dict[str, Any]) -> Dict[tuple, int]:
    count_map = defaultdict(set)
    for robot, tasks in task_data["robots"].items():
        for task in tasks:
            sg = task.get("sync_group")
            seq = task.get("sequence")
            if sg is not None and seq is not None:
                count_map[(seq, sg)].add(robot)
    return {k: len(v) for k, v in count_map.items()}

# === 安全发布 ===
def _safe_publish(payload: dict):
    if status_pub is None or not rclpy.ok():
        return
    try:
        status_pub.publish(String(data=json.dumps(payload)))
    except Exception:
        logger.exception("publish failed")

# === 只在状态变化时发一次（ready→running→finished）===
def publish_state(robot: str, task_id: str, sequence: int, sync_group: str | None, status: str):
    msg = {
        "kind": "state",
        "robot": robot,
        "task_id": task_id,
        "sequence": sequence,
        "sync_group": sync_group,
        "status": status,
        "ts": time.time()
    }
    _safe_publish(msg)
    print(f"[{robot}] 📣 State: {status} @ ({sequence}, {sync_group})")

# === 低频心跳（仅活性检测，不参与 barrier 判断）===
def start_heartbeat(robot: str):
    global _heartbeat_thread
    if _heartbeat_thread and _heartbeat_thread.is_alive():
        return
    def loop():
        while not _heartbeat_stop.is_set():
            _safe_publish({
                "kind": "heartbeat",
                "robot": robot,
                "ts": time.time()
            })
            time.sleep(HEARTBEAT_INTERVAL)
    _heartbeat_thread = threading.Thread(target=loop, daemon=True)
    _heartbeat_thread.start()

def stop_heartbeat():
    _heartbeat_stop.set()
    if _heartbeat_thread:
        _heartbeat_thread.join(timeout=1)

# === 状态订阅回调 ===
def status_callback(msg: String):
    try:
        data = json.loads(msg.data)
        kind = data.get("kind", "state")
        if kind == "heartbeat":
            # 心跳不计入 barrier；可按需记录 last_seen
            print(f"💓 HB from {data.get('robot')} at {data.get('ts')}")
            return

        key = (data["sequence"], data["sync_group"])
        robot = data["robot"]
        status = data["status"]

        with cache_lock:
            status_cache[key][robot] = status

            reached_ready    = sum(status_rank(s) >= status_rank("ready")    for s in status_cache[key].values())
            reached_finished = sum(status_rank(s) >= status_rank("finished") for s in status_cache[key].values())

            if reached_ready    >= target_counts.get(key, 0):
                status_events[(key, "ready")].set()
            if reached_finished >= target_counts.get(key, 0):
                status_events[(key, "finished")].set()
            print(f"📥 {robot} → {status} @ {key}  "
                  f"(need {target_counts.get(key,0)}, ready={reached_ready}, fin={reached_finished})")
    except Exception:
        print(f"⚠️ status_callback error: \n{traceback.format_exc()}")

# === 等待所有机器人 status 为某个值 ===
def wait_for_all_status(sync_key: tuple, expected: str, timeout=60):
    ev = status_events[(sync_key, expected)]
    ok = ev.wait(timeout)
    if not ok:
        raise TimeoutError(f"{sync_key} 等待 {expected} 超时")
    ev.clear()

# === 执行任务调度（变更触发：ready/run/finished 各发一次）===
def run_scheduler_for_robot(node, robot_name: str, task_data: Dict[str, Any], executor):
    print(f"\n🤖 Robot `{robot_name}` starting task scheduler...\n")

    is_successful_overall = False
    robot_tasks = task_data["robots"].get(robot_name, [])
    robot_tasks.sort(key=lambda x: x["sequence"])

    try:
        for task in robot_tasks:
            task["robot"] = robot_name
            seq = task["sequence"]
            sync_group = task.get("sync_group")
            sync_key = (seq, sync_group)
            task_id = task["task_id"]

            if sync_group is not None:
                # 所有人就绪屏障：先各自发 ready，然后等待 all ready
                publish_state(robot_name, task_id, seq, sync_group, "ready")
                wait_for_all_status(sync_key, "ready")
                print(f"[{robot_name}] ✅ All ready → executing task {task_id}")
                # 开始执行前，发 running
                publish_state(robot_name, task_id, seq, sync_group, "running")

            # 执行动作
            ok = execute_action(node, executor, task)
            is_successful_overall = ok or is_successful_overall

            if sync_group is not None:
                # 执行完成，发 finished，然后等待 all finished
                publish_state(robot_name, task_id, seq, sync_group, "finished")
                wait_for_all_status(sync_key, "finished")
                print(f"[{robot_name}] 🎉 All finished for {sync_key}")

        if is_successful_overall:
            print(f"🎯 All tasks completed for `{robot_name}`!")
        return is_successful_overall
    except Exception:
        logger.warning(f"⚠️ Failed to execute tasks:\n{traceback.format_exc()}")
        return False

def shutdown_node(node: Node) -> bool:
    if node is None or not rclpy.ok():
        return False
    try:
        # 更安全：如果被 executor 管理，应该先从外部 remove
        node.get_logger().info("🛑 Safe shutdown")
        node.destroy_node()
        return True
    except Exception:
        node.get_logger().exception(f"❌ Node destruction failed: \n{traceback.format_exc()}")
        return False

def run(task_data: Dict[str, Any]):
    global ros_node, status_pub, target_counts
    robot_id = config.get("robot_id")
    is_successful = False

    # 1) 先统计同步组规模（避免回调早来时还没 target_counts）
    target_counts = count_robots_per_sync_key(task_data)
    print("🎯 target_counts =", target_counts)

    # 2) init（多次调用保护）
    try:
        rclpy.init()
    except RuntimeError:
        pass

    # 3) node & executor
    ros_node = rclpy.create_node(f"status_node_{robot_id}")
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)

    # 4) QoS：RELIABLE + TRANSIENT_LOCAL（关键）
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability  = DurabilityPolicy.TRANSIENT_LOCAL

    # 5) pub/sub
    status_pub = ros_node.create_publisher(String, "/robot_status", qos)
    ros_node.create_subscription(String, "/robot_status", status_callback, qos)

    # 6) spin + 心跳
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    start_heartbeat(robot_id)

    try:
        is_successful = run_scheduler_for_robot(ros_node, robot_id, task_data, executor)
        return is_successful
    except Exception:
        logger.exception(f"⚠️ Failed to schedule tasks :\n{traceback.format_exc()}")
        return is_successful
    finally:
        print(f"🛑 Shutting down ROS node for {robot_id}")
        stop_heartbeat()
        try:
            executor.remove_node(ros_node)
        except Exception:
            pass
        executor.shutdown()
        shutdown_node(ros_node)
        rclpy.shutdown()
        spin_thread.join(timeout=1)

if __name__ == "__main__":
    raw_response = dedent("""
    {
      "robots": {
        "robot1": [
          {"task_id": "t0", "action": "move", "parameters": {"direction": "forward", "value": 2, "unit": "meters"}, "sync_group": "A", "sequence": 1},
          {"task_id": "t1", "action": "turn", "parameters": {"direction": "left", "value": 90, "unit": "degrees"}, "sync_group": "A", "sequence": 2}
        ]
      }
    }
    """)
    task_data = json.loads(raw_response)
    run(task_data)
