import os, sys
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)
 
import json
import time
from collections import defaultdict
from typing import Dict, Any
import traceback
from std_msgs.msg import String
from loguru import logger
from textwrap import dedent
from robotControllerRepo.robot_controller import execute_action
from config import config
from utils.ros_lifecycle import ROSManager
import threading
import rclpy

 
# === 全局变量 ===
ros_node = None
status_pub = None
status_cache = defaultdict(lambda: defaultdict(str))  # {(seq, sync_group): {robot: status}}
 
 
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
 
 
# === 发布任务状态 ===
def publish_status(robot: str, task_id: str, sequence: int, sync_group: str, status: str):
    msg = {
        "robot": robot,
        "task_id": task_id,
        "sequence": sequence,
        "sync_group": sync_group,
        "status": status
    }
    status_pub.publish(String(data=json.dumps(msg)))
    print(f"[{robot}] 📣 Published status: {status} for {sequence}-{sync_group}")
 
 
def start_periodic_status_publisher(robot, task_id, seq, sync_group, status, stop_event):
    def publish_loop():
        msg = {
            "robot": robot,
            "task_id": task_id,
            "sequence": seq,
            "sync_group": sync_group,
            "status": status
        }
        while not stop_event.is_set():
            try:
                status_pub.publish(String(data=json.dumps(msg)))
                print(f"[{robot}] ⏱️ Periodic status: {status} for {seq}-{sync_group}")
            except Exception as e:
                print(f"[{robot}] ⚠️ Failed to publish: {e}")
                break
            time.sleep(0.5)
 
    t = threading.Thread(target=publish_loop, daemon=True)
    t.start()
    return t
 
 
# === 状态订阅回调 ===
def status_callback(msg):
    try:
        data = json.loads(msg.data)
        key = (data["sequence"], data["sync_group"])
        robot = data["robot"]
        status = data["status"]
        status_cache[key][robot] = status
        print(f"📥 Received status from {robot}: {status} in {key}")
    except Exception as e:
        print(f"⚠️ status_callback error: {e}")
 
 
def status_rank(s: str) -> int:
    return {"ready": 1, "running": 2, "finished": 3}.get(s, 0)
 
 
# === 等待所有机器人 status 为某个值 ===
def wait_for_all_status(sync_key: tuple, target_count: int, expected_status: str):
    while True:
        current_statuses = status_cache[sync_key]
        matching = [r for r, s in current_statuses.items() if status_rank(s) >= status_rank(expected_status)]
        print(f"🔍 Waiting for {expected_status} in {sync_key}, current: {len(matching)}/{target_count}")
        print(f"🧠 Current: {current_statuses}")
        if len(matching) >= target_count:
            break
        time.sleep(0.2)
 
 
# === 执行任务调度 ===
def run_scheduler_for_robot(node, robot_name: str, task_data: Dict[str, Any], target_counts: Dict[tuple, int], executor):
    print(f"\n🤖 Robot `{robot_name}` starting task scheduler...\n")
 
    robot_tasks = task_data["robots"].get(robot_name, [])
    robot_tasks.sort(key=lambda x: x["sequence"])
 
    try:
        for task in robot_tasks:
            task["robot"] = robot_name
            seq = task["sequence"]
            sync_group = task.get("sync_group")
            sync_key = (seq, sync_group)
            task_id = task["task_id"]
 
            if sync_group:
                stop_event = threading.Event()
                _ = start_periodic_status_publisher(robot_name, task_id, seq, sync_group, "ready", stop_event)
                wait_for_all_status(sync_key, target_counts[sync_key], "ready")
                stop_event.set()
                print(f"[{robot_name}] ✅ All ready → executing task {task_id}")
 
            execute_action(node, task, executor)
 
            if sync_group:
                stop_event = threading.Event()
                _ = start_periodic_status_publisher(robot_name, task_id, seq, sync_group, "finished", stop_event)
                wait_for_all_status(sync_key, target_counts[sync_key], "finished")
                stop_event.set()
                print(f"[{robot_name}] 🎉 All finished for {sync_key}")
 
            time.sleep(0.3)
 
        print(f"🎯 All tasks completed for `{robot_name}`!")
        return True
    except Exception:
        logger.warning(f"⚠️ Failed to execute tasks:\n{traceback.format_exc()}")
        return False
 
 
# === 入口 ===
def run(task_data: Dict[str, Any]):
    global ros_node, status_pub
 
    robot_id = config.get("robot_id")
 
    ros = ROSManager()
    ros.start()
 
    ros_node = rclpy.create_node(f"status_node_{robot_id}")
    ros.add_node(ros_node)
 
    status_pub = ros_node.create_publisher(String, "/robot_status", 10)
    ros_node.create_subscription(String, "/robot_status", status_callback, 10)
 
    print("==================task_data==============\n")
    print(task_data)
    sync_target_counts = count_robots_per_sync_key(task_data)
 
    try:
        success = run_scheduler_for_robot(ros_node, robot_id, task_data, sync_target_counts, ros.executor)
        return success
    finally:
        time.sleep(1.0)  # ⏳ 等待线程关闭，防止节点销毁中 publish
        ros.shutdown()
 
 
# === 调试入口 ===
if __name__ == "__main__":
    raw_response = dedent("""
    {
      "robots": {
        "robot1": [
          {"task_id": "t0", "action": "move", "parameters": {"direction": "forward", "value": 2, "unit": "meters"}, "sync_group": null, "sequence": 1},
          {"task_id": "t1", "action": "turn", "parameters": {"direction": "left", "value": 90, "unit": "degrees"}, "sync_group": null, "sequence": 2}
        ]
      }
    }
    """)
    task_data = json.loads(raw_response)
    run(task_data)