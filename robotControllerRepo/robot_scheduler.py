#!/usr/bin/env python3
import os, sys
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)

import json
import time
from collections import defaultdict
from typing import Dict, Any
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from textwrap import dedent
import threading
from phasespace.rigid_tracker import RigidTracker
from robotControllerRepo.robot_controller import execute_action
from config import config
from loguru import logger
import traceback
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy

# === 全局变量 ===
ros_node = None
status_pub = None
status_cache = defaultdict(lambda: defaultdict(str))  # {(seq, sync_group): {robot: status}}
target_counts = {}


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
status_events = defaultdict(threading.Event)
cache_lock = threading.Lock()
def status_callback(msg):
    try:
        data = json.loads(msg.data)
        key = (data["sequence"], data["sync_group"])
        robot = data["robot"]
        status = data["status"]

        while cache_lock:
            status_cache[key][robot] = status
 
            # 计算达到“当前阶段”要求的机器人数量
            reached_ready    = sum(status_rank(s)>=status_rank("ready")    for s in status_cache[key].values())
            reached_finished = sum(status_rank(s)>=status_rank("finished") for s in status_cache[key].values())
 
            if reached_ready   >= target_counts.get(key, 0):
                status_events[(key, "ready")].set()
            if reached_finished>= target_counts.get(key, 0):
                status_events[(key, "finished")].set()
            print(f"📥 {robot} → {status} @ {key}")
    except Exception:
        print(f"⚠️ status_callback error: \n{traceback.format_exc()}")

def status_rank(s: str) -> int:
    return {"ready": 1, "running": 2, "finished": 3}.get(s, 0)

# === 等待所有机器人 status 为某个值 ===
def wait_for_all_status(sync_key: tuple, expected: str, timeout=15):
    ev = status_events[(sync_key, expected)]
    ok = ev.wait(timeout)
    if not ok:
        raise TimeoutError(f"{sync_key} 等待 {expected} 超时")
    ev.clear()           # 复位给下一个阶段用
  
# === 执行任务调度 ===
def run_scheduler_for_robot(node, robot_name: str, task_data: Dict[str, Any]):
    isExecute = False

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
                # Step 1: 启动 ready 状态的周期广播
                stop_event = threading.Event()
                _ = start_periodic_status_publisher(robot_name, task_id, seq, sync_group, "ready", stop_event)

                # Step 2: 等待所有机器人 ready
                wait_for_all_status(sync_key, "ready")

                # Step 3: 停止广播
                stop_event.set()

                print(f"[{robot_name}] ✅ All ready → executing task {task_id}")

            

            # Step 3: 执行动作
            execute_action(node, task)

            # Step 4: 发布 finished 状态
            if sync_group:
                # Step 4: 启动 finished 状态的周期广播
                stop_event = threading.Event()
                _ = start_periodic_status_publisher(robot_name, task_id, seq, sync_group, "finished", stop_event)

                # Step 5: 等待所有机器人 finished
                wait_for_all_status(sync_key, "finished")

                # Step 6: 停止广播
                stop_event.set()

                print(f"[{robot_name}] 🎉 All finished for {sync_key}")


        print(f"🎯 All tasks completed for `{robot_name}`!")

        isExecute = True
        return isExecute
    except Exception:
        logger.warning(f"⚠️ Failed to execute tasks:\n{traceback.format_exc()}")
        isExecute = False
        return isExecute



def shutdown_node(node: Node) -> bool:
    """
    尝试安全销毁 ROS2 Node。
    返回 True 表示 destroy 成功；False 表示未执行或出错。
    """
 
    if node is None or not rclpy.ok():
        # rclpy 已 shutdown 或 node 无效
        return False
 
    # 若 node 被某个 executor 管理，先移除
    if getattr(node, "_executor", None) is not None:
        try:
            node._executor.remove_node(node)
        except Exception:
            # 不阻塞主流程，但留痕
            node.get_logger().warning(f"⚠️ Unable to remove node from executor: :\n{traceback.format_exc()}")
 
    # 记录 shutdown 日志
    node.get_logger().info("🛑 Safe shutdown")
 
    # 真正销毁
    try:
        node.destroy_node()
        return True
    except Exception:
        node.get_logger().exception(f"❌ Node destruction failed: \n{traceback.format_exc()}")
        return False


 
def run(task_data: Dict[str, Any]):
    global ros_node, status_pub, target_counts
    robot_id = config.get("robot_id")
 
    # 1) init（多次调用保护）
    if not rclpy.is_initialized():
        rclpy.init()
 
    # 2) node & executor
    ros_node = rclpy.create_node(f"status_node_{robot_id}")
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
 
    # 3) pub / sub（可靠 QoS）
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    status_pub = ros_node.create_publisher(String, "/robot_status", qos)
    ros_node.create_subscription(String, "/robot_status", status_callback, qos)
 
    # 4) spin thread
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
 
    try:
        target_counts = count_robots_per_sync_key(task_data)
        run_scheduler_for_robot(ros_node, robot_id, task_data, target_counts)
        return True
    except Exception:
        logger.exception("⚠️ Failed to schedule tasks :\n{traceback.format_exc()}")
        return False
    finally:
        print(f"🛑 Shutting down ROS node for {robot_id}")
        executor.shutdown()          # ← 先停执行器
        shutdown_node(ros_node)      # ← 再销毁 node
        rclpy.shutdown()             # ← 最后关 context
        spin_thread.join(timeout=1)  # ← 等后台线程退出


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