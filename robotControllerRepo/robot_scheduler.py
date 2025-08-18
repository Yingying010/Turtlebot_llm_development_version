#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, sys
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)
import json, time, threading, traceback
from collections import defaultdict
from typing import Dict, Any, Optional
from ttsRepo.stream_tts import tts_manager
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from loguru import logger
from textwrap import dedent

# === 你项目里的依赖 ===
from robotControllerRepo.robot_controller import execute_action
from config import config

# =========================
# 全局状态
# =========================
ros_node: Optional[Node] = None
status_pub = None

# （仍保留）状态缓存：仅用于打印与可视化
status_cache = defaultdict(lambda: defaultdict(str))  # {(seq, sg): {robot: status}}

# 事件（屏障）
status_events = defaultdict(threading.Event)

# 锁
cache_lock = threading.Lock()

# 心跳
HEARTBEAT_INTERVAL = 2.0
_heartbeat_stop = threading.Event()
_heartbeat_thread: threading.Thread | None = None

# === CHANGED: 用集合做同步组屏障，天然去重 ===
# key 统一用 (None, sync_group) 表示“同步屏障”
sync_need   : Dict[tuple, set] = {}               # (None, sg) -> set(robot) 解析时确定
sync_ready  = defaultdict(set)                    # (None, sg) -> set(robot)
sync_finish = defaultdict(set)                    # (None, sg) -> set(robot)

# 串行阶段仍用“需要 1 个完成”即可：
target_counts: Dict[tuple, int] = {}              # (seq, None) -> 1；(None, sg) 不再使用这里的计数

# =========================
# 工具函数
# =========================
def status_rank(s: str) -> int:
    return {"ready": 1, "running": 2, "finished": 3}.get(s, 0)

def _safe_publish(payload: dict):
    """安全发布到 /robot_status。"""
    if status_pub is None or not rclpy.ok():
        return
    try:
        status_pub.publish(String(data=json.dumps(payload)))
    except Exception:
        logger.exception("publish failed")

def wait_for_all_status(sync_key: tuple, expected: str, timeout=60):
    """等待某个键的某个状态达到门槛。"""
    ev = status_events[(sync_key, expected)]
    ok = ev.wait(timeout)
    if not ok:
        raise TimeoutError(f"{sync_key} 等待 {expected} 超时")
    ev.clear()

# === CHANGED: 解析命令时，建立每个 sync_group 的成员集合 ===
def build_sync_need(task_data: Dict[str, Any]) -> Dict[tuple, set]:
    sg_map = defaultdict(set)
    for robot, tasks in task_data.get("robots", {}).items():
        for t in tasks:
            sg = t.get("sync_group")
            if sg is not None:
                sg_map[(None, sg)].add(robot)
    return dict(sg_map)

# =========================
# 本地计划校验（仅用 task_data["robots"]）
# =========================
def validate_local_plan(task_data: Dict[str, Any]):
    seen_seq_owner: Dict[int, str] = {}
    for robot, tasks in task_data.get("robots", {}).items():
        seen_seq_in_robot = set()
        for t in tasks:
            s = t.get("sequence")
            sg = t.get("sync_group")
            if s is not None and sg is not None:
                raise ValueError(f"Task cannot have both sequence and sync_group: {t}")
            if s is not None:
                if s in seen_seq_in_robot:
                    raise ValueError(f"Duplicate sequence={s} in robot '{robot}'")
                seen_seq_in_robot.add(s)
                if s in seen_seq_owner and seen_seq_owner[s] != robot:
                    raise ValueError(
                        f"Duplicate global sequence={s} found on '{seen_seq_owner[s]}' and '{robot}'"
                    )
                seen_seq_owner[s] = robot

def build_prev_stage_map(task_data: Dict[str, Any]) -> Dict[int, Optional[int]]:
    stages = sorted({
        t["sequence"]
        for tasks in task_data.get("robots", {}).values()
        for t in tasks
        if t.get("sequence") is not None
    })
    prev = {}
    for i, s in enumerate(stages):
        prev[s] = stages[i - 1] if i > 0 else None
    return prev

def count_robots_per_sync_key(task_data: Dict[str, Any]) -> Dict[tuple, int]:
    out: Dict[tuple, int] = {}
    # 串行阶段：固定需要 1
    seqs = {
        t["sequence"]
        for tasks in task_data.get("robots", {}).values()
        for t in tasks
        if t.get("sequence") is not None
    }
    for s in seqs:
        out[(s, None)] = 1
    # 同步组不再用计数（我们用集合去重）；此处不赋值
    return out

# =========================
# 状态入账 & 触发（只在订阅回调里调用）
# =========================
def _apply_status_and_maybe_signal(robot: str, seq, sync_group, status: str):
    """
    - 同步屏障：用集合去重，只有当 ready/finished 集合 == need 集合时，触发事件
    - 串行阶段：维持“完成数达到 1”即可
    """
    if seq is not None and sync_group is None:
        key = (seq, None)
    elif sync_group is not None and seq is None:
        key = (None, sync_group)
    else:
        key = (None, None)

    with cache_lock:
        # 仅用于观测/打印
        status_cache[key][robot] = status

        # === 同步屏障（集合去重） ===
        if key[0] is None and key[1] is not None:
            need = sync_need.get(key, set())

            if status == "ready":
                sync_ready[key].add(robot)
                if need and need.issubset(sync_ready[key]):
                    status_events[(key, "ready")].set()

            elif status == "finished":
                sync_finish[key].add(robot)
                if need and need.issubset(sync_finish[key]):
                    status_events[(key, "finished")].set()
                    # 组完成，立即清理（避免下一轮复用）
                    sync_ready.pop(key, None)
                    sync_finish.pop(key, None)
                    sync_need.pop(key, None)

        # === 串行阶段（只看 finished 达到 1） ===
        if key[0] is not None and key[1] is None:
            need_cnt = target_counts.get(key, 0)
            if status == "finished" and need_cnt > 0:
                # 串行阶段的“完成阈值”就是 1（谁完成谁触发）
                status_events[(key, "finished")].set()

        # 打印
        ready_cnt    = len(sync_ready.get(key, set())) if key in sync_ready else 0
        finished_cnt = len(sync_finish.get(key, set())) if key in sync_finish else 0
        need_sz      = len(sync_need.get(key, set())) if key in sync_need else target_counts.get(key, 0)
        print(f"📥(acc) {robot} → {status} @ {key} (need={need_sz}, ready={ready_cnt}, fin={finished_cnt})")

def ensure_task_ids(task_data: Dict[str, Any]):
    for robot, tasks in task_data.get("robots", {}).items():
        for i, t in enumerate(tasks):
            seq = t.get("sequence")
            sg  = t.get("sync_group")
            tag = f"seq{seq}" if seq is not None else (f"sg{sg}" if sg is not None else "local")
            t["task_id"] = f"{robot}-{tag}-{i}"

# === CHANGED: 发布时不再本地入账（避免“一发一收记两次”）===
def publish_state(robot: str, task_id, sequence, sync_group, status: str):
    payload = {
        "kind": "state",
        "robot": robot,
        "task_id": task_id,
        "sequence": sequence,
        "sync_group": sync_group,
        "status": status,
        "ts": time.time(),
    }
    _safe_publish(payload)
    print(f"[{robot}] 📣 State: {status} @ ({sequence}, {sync_group})")

# =========================
# 订阅回调（唯一的入账入口）
# =========================
def status_callback(msg):
    try:
        data = json.loads(msg.data)
        kind = data.get("kind", "state")
        if kind != "state":
            return  # 忽略心跳等其他类型

        src_robot = data["robot"]
        tid = data.get("task_id")
        seq = data.get("sequence")
        sg = data.get("sync_group")
        status = data["status"]

        if src_robot == config.get("robot_id"):
            print(f"📩 收到【自己】状态: {src_robot} → {status} @ ({seq}, {sg})")
        else:
            print(f"📩 收到【对方】状态: {src_robot} → {status} @ ({seq}, {sg})")

        print(f"📩 {src_robot} → {status} @ ({seq}, {sg}) tid={tid}")

        # === 唯一入账点 ===
        _apply_status_and_maybe_signal(src_robot, seq, sg, status)

    except Exception:
        print(f"⚠️ status_callback error: \n{traceback.format_exc()}")

# =========================
# 心跳
# =========================
def start_heartbeat(robot: str):
    global _heartbeat_thread, _heartbeat_stop
    if _heartbeat_thread and _heartbeat_thread.is_alive():
        return
    _heartbeat_stop.clear()

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

# =========================
# 调度器主逻辑
# =========================
def run_scheduler_for_robot(node, robot_name: str, task_data: Dict[str, Any], executor):
    print(f"\n🤖 Robot `{robot_name}` starting task scheduler...\n")

    is_successful_overall = False
    tasks = task_data["robots"].get(robot_name, [])

    # 互斥校验
    for t in tasks:
        if t.get("sequence") is not None and t.get("sync_group") is not None:
            raise ValueError(f"Task cannot have both sequence and sync_group: {t}")

    # 前驱关系
    prev_stage = build_prev_stage_map(task_data)

    try:
        for task in tasks:
            task["robot"] = robot_name
            tid   = task.get("task_id")
            seq = task.get("sequence")
            sg = task.get("sync_group")

            # A) 跨机器人串行阶段（只有 sequence）
            if seq is not None and sg is None:
                p = prev_stage.get(seq)
                if p is not None:
                    tts_manager.say_sync("i'm waiting for the other robots to finish")
                    wait_for_all_status((p, None), "finished")
                    print(f"[{robot_name}] ⏩ Stage {p} finished → start Stage {seq}")
                    tts_manager.say_sync(f"sequence {p} is finished, i'm going to start the mission")

                publish_state(robot_name, tid, seq, None, "running")
                ok = execute_action(node, executor, task)
                is_successful_overall = ok or is_successful_overall

                publish_state(robot_name, tid, seq, None, "finished")
                continue

            # B) 跨机器人同步（只有 sync_group）
            if sg is not None and seq is None:
                key = (None, sg)

                publish_state(robot_name, tid, None, sg, "ready")
                tts_manager.say_sync("i'm waiting for the other robots to synchronise with me")
                wait_for_all_status(key, "ready")
                tts_manager.say_sync("All robots are ready for action.")
                print(f"[{robot_name}] ✅ All ready @ sync_group={sg}")

                publish_state(robot_name, tid, None, sg, "running")
                ok = execute_action(node, executor, task)
                is_successful_overall = ok or is_successful_overall

                publish_state(robot_name, tid, None, sg, "finished")
                wait_for_all_status(key, "finished")
                print(f"[{robot_name}] 🎉 All finished @ sync_group={sg}")
                continue

            # C) 无依赖（本地默认顺序）
            ok = execute_action(node, executor, task)
            is_successful_overall = ok or is_successful_overall

        if is_successful_overall:
            print(f"🎯 All tasks completed for `{robot_name}`!")
        return is_successful_overall

    except Exception:
        logger.warning(f"⚠️ Failed to execute tasks:\n{traceback.format_exc()}")
        return False

# =========================
# 节点生命周期
# =========================
def shutdown_node(node: Node) -> bool:
    if node is None or not rclpy.ok():
        return False
    try:
        node.get_logger().info("🛑 Safe shutdown")
        node.destroy_node()
        return True
    except Exception:
        node.get_logger().exception(f"❌ Node destruction failed: \n{traceback.format_exc()}")
        return False

# =========================
# 入口
# =========================
def run(task_data: Dict[str, Any]):
    global ros_node, status_pub, target_counts, sync_need

    validate_local_plan(task_data)

    robot_id = config.get("robot_id")
    is_successful = False

    # 1) 统计阶段屏障需求（串行阶段）
    target_counts = count_robots_per_sync_key(task_data)
    # 2) 统计同步组成员集合（同步屏障）
    sync_need = build_sync_need(task_data)

    print("🎯 target_counts =", target_counts)
    print("👥 sync_need =", {k: sorted(v) for k, v in sync_need.items()})

    # init（多次调用保护）
    try:
        rclpy.init()
    except RuntimeError:
        pass

    ros_node = rclpy.create_node(f"status_node_{robot_id}")
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)

    # QoS：RELIABLE + TRANSIENT_LOCAL
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

    global status_pub
    status_pub = ros_node.create_publisher(String, "/robot_status", qos)
    ros_node.create_subscription(String, "/robot_status", status_callback, qos)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    start_heartbeat(robot_id)

    try:
        ensure_task_ids(task_data)
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

# =========================
# 自测示例
# =========================
if __name__ == "__main__":
    raw_response = dedent("""
    {
        "robots": {
            "robot1": [
                {"action": "navigate", "parameters": {"target": "table"}, "sync_group": 0}
            ],
            "robot2": [
                {"action": "navigate", "parameters": {"target": "table"}, "sync_group": 0}
            ]
        }
    }
    """)
    task_data = json.loads(raw_response)
    run(task_data)
