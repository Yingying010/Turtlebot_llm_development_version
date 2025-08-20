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
import config

# =========================
# 全局状态
# =========================
ros_node: Optional[Node] = None
status_pub = None
sync_group_id = None
need_count = None
ready_count = None
remaining_count = None
ts = None
# 全局状态（添加）
sync_go_events = defaultdict(threading.Event)  # sg -> Event
sync_go_times: Dict[int, float] = {}           # sg -> start_at(Unix time)

global_barrier = {
    "kind": "barrier", 
    "sync_group": sync_group_id, 
    "need": need_count, 
    "ready": ready_count, 
    "remaining": remaining_count, 
    "ts": ts
}

# 状态缓存：key 为 (seq, sync_group) 二元组
# - 跨机器人串行阶段：使用 (seq, None)，只统计 finished
# - 跨机器人同步：使用 (None, sg)，统计 ready / finished
status_cache = defaultdict(lambda: defaultdict(str))  # {(seq, sg): {robot: status}}

# 目标计数：屏障需要达到的数量
# - (seq, None) → 1（因为序号全局唯一）
# - (None, sg)  → 该同步组中的机器人数量
target_counts: Dict[tuple, int] = {}

# 事件（屏障）
status_events = defaultdict(threading.Event)

# 锁
cache_lock = threading.Lock()

data = {}

# 心跳
HEARTBEAT_INTERVAL = 2.0
_heartbeat_stop = threading.Event()
_heartbeat_thread: threading.Thread | None = None


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
    """等待某个键的某个状态达到 target_counts 要求。"""
    ev = status_events[(sync_key, expected)]
    ok = ev.wait(timeout)
    if not ok:
        raise TimeoutError(f"{sync_key} 等待 {expected} 超时")
    ev.clear()

def _counts_for_key(key: tuple):
    with cache_lock:
        need = target_counts.get(key, 0)
        # ✅ 精确匹配，避免各种比较歧义
        ready = sum(1 for s in status_cache[key].values() if s == "ready")
        finished = sum(1 for s in status_cache[key].values() if s == "finished")
    remaining = max(need - finished, 0)
    return need, ready, finished, remaining


def publish_barrier_snapshot(sync_group: int):
    """将同步组的聚合状态发布到 /robot_status（kind=barrier）"""
    key = (None, sync_group)
    need, ready, finished, remaining = _counts_for_key(key)
    payload = {
        "kind": "barrier",
        "sync_group": sync_group,
        "need": need,
        "ready": ready,
        "remaining": remaining,
        "ts": time.time(),
    }
    # global_barrier = payload
    _safe_publish(payload)
    print(f"📊 Barrier snapshot sg={sync_group} → need={need}, ready={ready}, remaining={remaining}")

def publish_sync_go(sync_group: int, delay: float = 0.3):
    """领导者发布统一起跑时间（当前时间+delay），所有人等到这个时间再运行。"""
    start_at = time.time() + delay
    payload = {
        "kind": "go",
        "sync_group": sync_group,
        "start_at": start_at,
        "ts": time.time(),
    }
    # 先本地记一份，避免领导者还要等自己发的消息回环
    sync_go_times[sync_group] = start_at
    sync_go_events[sync_group].set()
    _safe_publish(payload)
    print(f"🚦 publish GO | sg={sync_group} | start_at={start_at:.3f} (delay={delay}s)")
 
 
def wait_for_sync_go(sync_group: int, timeout: float = 5.0):
    """等待收到 GO，并在指定 start_at 时刻同步起跑。"""
    ev = sync_go_events[sync_group]
    ok = ev.wait(timeout)
    if not ok:
        raise TimeoutError(f"sync_group={sync_group} 等待 GO 超时")
    start_at = sync_go_times[sync_group]
    now = time.time()
    remain = start_at - now
    if remain > 0:
        time.sleep(remain)
    print(f"🚦 GO LATCHED | sg={sync_group} | now={time.time():.3f}")

def build_seq_owner_map(task_data: Dict[str, Any]) -> Dict[int, str]:
    """
    为每个全局唯一的 sequence 找到其归属机器人。
    依赖你已有的 validate_local_plan()，保证 sequence 在全局唯一。
    """
    owner = {}
    for robot, tasks in task_data.get("robots", {}).items():
        for t in tasks:
            s = t.get("sequence")
            if s is not None:
                owner[int(s)] = robot
    return owner

# =========================
# 本地计划校验（仅用 task_data["robots"]）
# =========================
def validate_local_plan(task_data: Dict[str, Any]):
    """
    1) 校验单任务互斥：不能同时出现 sequence 和 sync_group
    2) 全局 sequence 唯一：同一个 sequence 只能出现在一个机器人的一条任务上
    """
    seen_seq_owner: Dict[int, str] = {}
    for robot, tasks in task_data.get("robots", {}).items():
        # 额外：统计该机器人内的 sequence 是否重复
        seen_seq_in_robot = set()

        for t in tasks:
            s = t.get("sequence")
            sg = t.get("sync_group")

            if s is not None and sg is not None:
                raise ValueError(f"Task cannot have both sequence and sync_group: {t}")

            if s is not None:
                # 同一机器人内不得重复 sequence
                if s in seen_seq_in_robot:
                    raise ValueError(f"Duplicate sequence={s} in robot '{robot}'")
                seen_seq_in_robot.add(s)

                # 同一 sequence 不能出现在不同机器人
                if s in seen_seq_owner and seen_seq_owner[s] != robot:
                    raise ValueError(
                        f"Duplicate global sequence={s} found on '{seen_seq_owner[s]}' and '{robot}'"
                    )
                seen_seq_owner[s] = robot



def build_prev_stage_map(task_data: Dict[str, Any]) -> Dict[int, Optional[int]]:
    """
    构建全局阶段的前驱映射：
    - 收集所有 sequence 值
    - 按升序排序
    - 对每个阶段给出它的前一阶段（第一个为 None）
    """
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
    """
    计算屏障需要的目标数量：
    - (seq, None) → 1  （序列号全局唯一、且每个阶段只有一个执行者）
    - (None, sg)  → 同步组中机器人数量
    """
    out: Dict[tuple, int] = {}

    # 序列阶段：固定需要 1
    seqs = {
        t["sequence"]
        for tasks in task_data.get("robots", {}).values()
        for t in tasks
        if t.get("sequence") is not None
    }
    for s in seqs:
        out[(s, None)] = 1

    # 同步组：统计参与机器人数量
    sg_map = defaultdict(set)
    for robot, tasks in task_data.get("robots", {}).items():
        for t in tasks:
            sg = t.get("sync_group")
            if sg is not None:
                sg = int(sg)
                sg_map[(None, sg)].add(robot)
    out.update({k: len(v) for k, v in sg_map.items()})
    return out


# =========================
# 状态入账 & 广播
# =========================
def _apply_status_and_maybe_signal(robot: str, seq, sync_group, status: str):
    """
    - 同步：key = (None, sg)，使用 ready / finished 阈值
    - 阶段：key = (seq, None)，只使用 finished 阈值
    - 无依赖：key = (None, None)，不触发任何屏障
    """
    if seq is not None and sync_group is None:
        key = (seq, None)
    elif sync_group is not None and seq is None:
        key = (None, sync_group)
    else:
        key = (None, None)

    print(f"DEBUG apply_status: robot={robot}, status={status}, key={key}, type={type(key[1])}")

    with cache_lock:
        status_cache[key][robot] = status
        need = target_counts.get(key, 0)

        reached_ready = sum(status_rank(s) >= status_rank("ready") for s in status_cache[key].values())
        reached_finished = sum(status_rank(s) >= status_rank("finished") for s in status_cache[key].values())

        # 同步屏障：ready / finished
        if key[0] is None and key[1] is not None and need > 0:
            if reached_ready >= need:
                status_events[(key, "ready")].set()
            if reached_finished >= need:
                status_events[(key, "finished")].set()

        # 阶段屏障：只有 finished
        if key[0] is not None and key[1] is None and need > 0:
            if reached_finished >= need:
                status_events[(key, "finished")].set()
        
    # —— 在任何状态变化后，如果是同步组，发布一次聚合快照 —— 
    if key[0] is None and key[1] is not None:
        publish_barrier_snapshot(key[1])

    print(f"(local) {robot} → {status} @ {key} (need={need}, ready={reached_ready}, fin={reached_finished})")




def ensure_task_ids(task_data: Dict[str, Any]):
    for robot, tasks in task_data.get("robots", {}).items():
        for i, t in enumerate(tasks):
            # 生成一个可复现的ID：robot-序号或组-索引-动作
            seq = t.get("sequence")
            sg  = t.get("sync_group")
            tag = f"seq{seq}" if seq is not None else (f"sg{sg}" if sg is not None else "local")
            t["task_id"] = f"{robot}-{tag}-{i}"


def publish_state(robot: str, task_id, sequence, sync_group, status: str):
    """
    统一的状态发布
    """
    _apply_status_and_maybe_signal(robot, sequence, sync_group, status)
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
# 订阅回调
# =========================
def status_callback(msg):
    try:

        data = json.loads(msg.data)
        kind = data.get("kind", "state")
 
        # 1) 接收 barrier：只打印，别入账

        if kind == "barrier":
            sg = data.get("sync_group")
            need = int(data.get("need", 0))
            ready = int(data.get("ready", 0))
            remaining = int(data.get("remaining", max(need - ready, 0)))
            print(f"barrier | sync_group={sg} | need={need}, ready={ready}, remaining={remaining} | ts={data.get('ts')}")
            return
        
        if kind == "go":
            sg = int(data.get("sync_group"))
            start_at = float(data.get("start_at", time.time()))
            sync_go_times[sg] = start_at
            sync_go_events[sg].set()
            print(f"📨 recv GO | sg={sg} | start_at={start_at:.3f} | ts={data.get('ts')}")
            return
 
        # 2) 非 state/barrier 一律忽略（心跳等）
        if kind != "state":
            return
        
        # 3) 入账对方/自己的 state（关键！）
        src_robot = data.get("robot")
        seq = data.get("sequence")
        sg = data.get("sync_group")

        if sg is not None:
            sg = int(sg)

        status = data.get("status")
 
        # （可选）调试输出
        print(f"📩 state | {src_robot} → {status} @ (seq={seq}, sg={sg}) tid={data.get('task_id')} ts={data.get('ts')}")
 
        # 入账 + 可能触发事件 + 触发 barrier 快照
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
# 调度器主逻辑（按数组顺序遍历，遇到依赖插入屏障等待）
# =========================
def run_scheduler_for_robot(node, robot_name: str, task_data: Dict[str, Any],
                            executor, prev_stage: Dict[int, Optional[int]],
                            seq_owner_map: Dict[int, str]):
    print(f"\n🤖 Robot `{robot_name}` starting task scheduler...\n")

    is_successful_overall = False
    tasks = task_data["robots"].get(robot_name, [])

    # 互斥校验（单任务不能同时包含 sequence 与 sync_group）
    for t in tasks:
        if t.get("sequence") is not None and t.get("sync_group") is not None:
            raise ValueError(f"Task cannot have both sequence and sync_group: {t}")

    try:
        for task in tasks:
            task["robot"] = robot_name
            tid   = task.get("task_id")
            seq = task.get("sequence")
            sg = task.get("sync_group")
            if sg is not None:
                sg = int(sg)

            # A) 跨机器人串行阶段（只有 sequence）
            if seq is not None and sg is None:
                # 有前驱阶段则等待 (prev_seq, None, "finished")
                p = prev_stage.get(seq)
                if p is not None:
                    # 只有“上一个阶段的 owner 不是自己”时，才需要等待 + TTS
                    prev_owner = seq_owner_map.get(int(p))
                    if prev_owner is not None and prev_owner != robot_name:
                        tts_manager.say_sync("i'm waiting for the other robots to finish")
                        wait_for_all_status((p, None), "finished")
                        print(f"[{robot_name}] ⏩ Stage {p} finished by {prev_owner} → start Stage {seq}")
                        tts_manager.say_sync(f"sequence {p} is finished, i'm going to start the mission")
                    else:
                        # 上一个阶段是自己（或不存在 owner），无需等待与播报
                        print(f"[{robot_name}] previous stage {p} is mine → proceed without waiting")

                # 本阶段执行
                publish_state(robot_name, tid, seq, None, "running")
                ok = execute_action(node, executor, task)
                is_successful_overall = ok or is_successful_overall

                # 广播本阶段完成
                publish_state(robot_name, tid, seq, None, "finished")
                continue

            # B) 跨机器人同步（只有 sync_group）
            if sg is not None and seq is None:
                key = (None, sg)

                # —— 首次进入该同步组时，先发初始快照（如 need=2, ready=0, remaining=2）——
                publish_barrier_snapshot(sg)

                # 发布 ready 并等待齐活
                publish_state(robot_name, tid, None, sg, "ready")
                print(f"publish_state: {robot_name}")

                publish_barrier_snapshot(sg)


                # 语音：若未齐活则报“还在等待X个”
                need, ready, _, _ = _counts_for_key(key)
                waiting = max(need - ready, 0)

                if waiting > 0:
                    tts_manager.say_sync(f"i'm waiting for {waiting} robots to synchronise with me")

                # 等待 ready 齐活
                wait_for_all_status(key, "ready")
                print(f"sync_group={sg} | All robots are ready")
                
                # —— 选举“领导者”（统一且可重复：按机器人名最小）——
                with cache_lock:
                    participants = list(status_cache[key].keys())  # 已在该组报过 ready 的机器人
                leader = min(participants) if participants else robot_name
                
                # 领导者发布一个“未来时刻”的 GO；其他成员等待 GO
                if robot_name == leader:
                    publish_sync_go(sg, delay=1)  # 你也可以把 0.3 调成 0.1~0.5 之间
                else:
                    print(f"⏳ waiting GO from leader={leader} @ sg={sg}")
                
                # 所有人：等到 GO 的 timepoint 再起跑
                wait_for_sync_go(sg)
                
                # 同步起跑（此刻基本同时）
                publish_state(robot_name, tid, None, sg, "running")
                ok = execute_action(node, executor, task)
                is_successful_overall = ok or is_successful_overall
                
                publish_state(robot_name, tid, None, sg, "finished")
                print(f"sync_group={sg} | All robots are finished")
                continue

            # C) 无依赖（本地默认顺序）：直接执行，不发任何屏障状态
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
    global ros_node, status_pub, target_counts

    # —— 本地计划校验（只用 task_data["robots"]）——
    validate_local_plan(task_data)

    robot_id = config.get("robot_id")
    is_successful = False

    # 1) 先统计屏障规模
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

    # 4) QoS：RELIABLE + TRANSIENT_LOCAL
    qos = QoSProfile(depth=10)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

    # 5) pub/sub
    global status_pub
    status_pub = ros_node.create_publisher(String, "/robot_status", qos)
    ros_node.create_subscription(String, "/robot_status", status_callback, qos)

    # 6) spin + 心跳
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    start_heartbeat(robot_id)

    try:
        ensure_task_ids(task_data)
        prev_stage = build_prev_stage_map(task_data)
        seq_owner_map = build_seq_owner_map(task_data)
        is_successful = run_scheduler_for_robot(ros_node, robot_id, task_data, executor, prev_stage, seq_owner_map)
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
# 自测示例（注意：sequence 全局唯一）
# =========================
if __name__ == "__main__":
    # raw_response = dedent("""
    # {
    #   "robots": {
    #     "robot1": [
    #       {"action": "move",  "parameters": {"direction": "forward", "value": 3, "unit": "seconds"}},
    #       {"action": "turn",  "parameters": {"direction": "left",    "value": 180, "unit": "degrees"}, "sync_group": "1"},
    #       {"action": "move",  "parameters": {"direction": "forward", "value": 3, "unit": "seconds"}, "sequence": 1},
    #       {"action": "turn",  "parameters": {"direction": "right",   "value": 45, "unit": "degrees"}}
    #     ],
    #     "robot2": [
    #       {"action": "navigate",  "parameters": {"target": "table"}},
    #       {"action": "turn","parameters": {"direction": "right",   "value": 180, "unit": "degrees"}, "sync_group": "1"},
    #       {"action": "move",  "parameters": {"direction": "forward", "value": 3, "unit": "seconds"}, "sequence": 2}
    #     ]
    #   }
    # }
    # """)

    # raw_response = dedent("""
    # {
    #   "robots": {
    #     "robot1": [
    #       {"action": "navigate",  "parameters": {"target": "table"}}
    #     ],
    #     "robot2": [
    #       {"action": "navigate",  "parameters": {"target": "table"}}
    #     ]
    #   }
    # }
    # """)

    raw_response = dedent("""
    {
        "robots": {
            "robot1": [
                {
                "action": "navigate",
                "parameters": {
                    "target": "table"
                },
                "sync_group": 0
                }
            ],
            "robot2": [
                {
                "action": "navigate",
                "parameters": {
                    "target": "table"
                },
                "sync_group": 0
                }
            ]
        }
    }
    """)
    task_data = json.loads(raw_response)
    run(task_data)