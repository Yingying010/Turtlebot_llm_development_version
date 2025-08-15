#!/usr/bin/env python3

import os, sys, math, time, threading
from typing import Dict, Optional, Tuple
# 允许从项目根目录导入
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from rclpy.publisher import Publisher
from rclpy._rclpy_pybind11 import InvalidHandle
from phasespace.rigid_tracker import RigidTracker
from config import semantic_locations
from ttsRepo.stream_tts import tts_manager
import hashlib, struct, time
from std_msgs.msg import String
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# 全局：所有收到的 claim（持久化于内存）
rendezvous_claims = {}  # key=(xk,yk) → {robot_name: {"angle":..., "radius":..., "ts":...}}

def _qos_transient_local():
    qos = QoSProfile(depth=100)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability  = DurabilityPolicy.TRANSIENT_LOCAL
    return qos

def init_rendezvous_comms(node: Node):
    # 订阅已有/未来的占位声明
    def _on_claim(msg: String):
        try:
            data = json.loads(msg.data)
            xk, yk = data["key"]
            robot = data["robot"]
            angle = float(data["angle_deg"])
            radius = float(data["radius"])
            ts = float(data.get("ts", time.time()))
            key = (float(xk), float(yk))
            with cache_lock:
                store = rendezvous_claims.setdefault(key, {})
                # 只接受“字典序最小”的首次声明（简化并发冲突）
                if robot not in store:
                    store[robot] = {"angle": angle, "radius": radius, "ts": ts}
        except Exception as e:
            print(f"[RDZ-CLAIM] parse error: {e}")

    node.rdz_pub = node.create_publisher(String, "/rendezvous_claims", _qos_transient_local())
    node.rdz_sub = node.create_subscription(String, "/rendezvous_claims", _on_claim, _qos_transient_local())
 
# === 全局缓存 + 锁 ===
robot_position_cache: Dict[str, Dict[str, float]] = {}
cache_lock = threading.Lock()
publisher_dict: Dict[Tuple[int, str], Publisher] = {}

# === Rendezvous (同点会合) 分配器 ===
# RENDEZVOUS_RADIUS = 200.0      # 圆周半径，可改
# RENDEZVOUS_EPS    = 1e-3      # 目标点判定精度
GOLDEN_ANGLE_DEG  = 137.508   # 黄金角（均匀分布）

rendezvous_book: Dict[Tuple[float, float], Dict] = {}

def _point_key(x: float, y: float) -> Tuple[float, float]:
    # 也可以用更粗的量化以合并“几乎一致”的点
    return (round(x, 3), round(y, 3))

def _hash_to_angle(center_key: Tuple[float, float], robot_name: str) -> float:
    # center_key 是 (round(x,3), round(y,3))
    payload = f"{center_key[0]},{center_key[1]}|{robot_name}".encode("utf-8")
    h = hashlib.md5(payload).digest()
    # 取前4字节为无符号整数，映射到 [0,360)
    u32 = struct.unpack("I", h[:4])[0]
    return (u32 % 36000) / 100.0  # 两位小数


def _assign_rendezvous_slot_distributed(
    node: Node, center: Dict[str, float], robot_name: str,
    base_radius: float = 20.0, delta_radius: float = 20.0 * 0.5,
    max_retry: int = 5
) -> Dict[str, float]:
    x0, y0 = float(center["x"]), float(center["y"])
    key = _point_key(x0, y0)
 
    # 读取本地视图
    with cache_lock:
        store = rendezvous_claims.get(key, {}).copy()
    n = len(store)
 
    # 已有自己的声明：直接复用（后面仍会根据最终人数重算 R）
    if robot_name in store:
        angle_deg = float(store[robot_name]["angle"])
    else:
        # 先挑一个候选角（黄金角）
        tried = set(float(v["angle"]) for v in store.values())
        idx = n
        angle_deg = None
        for _ in range(n + max_retry):
            cand = (idx * GOLDEN_ANGLE_DEG) % 360.0
            if all(abs(cand - a) > 1e-3 for a in tried):
                angle_deg = cand
                break
            idx += 1
        if angle_deg is None:
            angle_deg = _hash_to_angle(key, robot_name)  # 兜底
 
        # 先发布 presence/claim（让别人能“看见我”）
        claim = {
            "key": [key[0], key[1]],
            "robot": robot_name,
            "angle_deg": float(angle_deg),
            # 这里的半径只是占位，最终会按合并后的 n_effective 重算
            "radius": float(base_radius),
            "ts": time.time(),
        }
        msg = String(); msg.data = json.dumps(claim)
        node.rdz_pub.publish(msg)
        print(f"[RDZ-PUB] {robot_name} claim → {claim}")
 
    # 等待收敛一点点，再合并最终视图（含自己）
    time.sleep(0.5)
    with cache_lock:
        store2 = rendezvous_claims.get(key, {})
        merged = dict(store2)
        if robot_name not in merged:
            merged[robot_name] = {"angle": angle_deg, "radius": base_radius, "ts": time.time()}
 
    # 冲突化解：同角时按“字典序更小名字优先”，自己让步到下一个可用角
    taken_by = {}
    for r, v in merged.items():
        a = float(v["angle"])
        if (a not in taken_by) or (r < taken_by[a]):
            taken_by[a] = r
    my_angle = float(merged[robot_name]["angle"])
    if taken_by.get(my_angle) != robot_name:
        print(f"[RDZ-CONFLICT] {robot_name} angle {my_angle:.2f}° taken by {taken_by[my_angle]}; reselecting...")
        tried = set(float(v["angle"]) for v in merged.values())
        # 从当前 idx 继续往后找（或直接从 0 开始都行）
        idx += 1
        found = None
        for _ in range(max_retry):
            cand = (idx * GOLDEN_ANGLE_DEG) % 360.0
            if all(abs(cand - a) > 1e-3 for a in tried):
                found = cand; break
            idx += 1
        angle_deg = found if found is not None else _hash_to_angle(key, robot_name)
        print(f"[RDZ-RESOLVE] {robot_name} new angle → {angle_deg:.2f}°")
 
    # ✅ 最关键：根据“最终视图”决定是否分散 + 动态半径
    n_effective = len(merged)  # 观测到的参与者数量
    if n_effective <= 1:
        R = 0.0  # 只有我一个 → 不分散，直达中心
        print(f"[RDZ-SKIP] {robot_name} is alone @ {key}; go to center with R=0")
    else:
        R = base_radius + delta_radius * (n_effective - 1)
 
    # 最终目标（并回写半径供自适应 tolerance 使用）
    theta = math.radians(angle_deg)
    x_new = x0 + R * math.cos(theta)
    y_new = y0 + R * math.sin(theta)
 
    new_target = {"x": x_new, "y": y_new, "rdz_radius": float(R)}
    if "heading_deg" in center and center["heading_deg"] is not None:
        new_target["heading_deg"] = center["heading_deg"]
 
    print(f"[RDZ-FINAL] {robot_name} @ {key} → n={n_effective}, angle={angle_deg:.1f}°, R={R:.1f}, goal=({x_new:.1f},{y_new:.1f})")
    return new_target
 
 
# === 将 RigidTracker 加入共享 executor，并等待数据就绪 ===

def getRobotPositionCache(robot_name: str, executor: MultiThreadedExecutor) -> Optional[Node]:
    rigid_node = RigidTracker(
        position_cache=robot_position_cache,
        robot_name=robot_name,
        position_lock=cache_lock,  # 传入同一把锁，避免读写冲突
    )
    executor.add_node(rigid_node)
    print(f"⏳ Waiting for position data of {robot_name}...")
    tts_manager.say(f"Initializing tracker for {robot_name}, waiting for position data.")
    for _ in range(50):  # ~10s
        with cache_lock:
            ok = robot_name in robot_position_cache
        if ok:
            print(f"✅ Got position data for {robot_name}.")
            tts_manager.say(f"Position data acquired for {robot_name}.")
            return rigid_node
        time.sleep(0.2)
    print(f"❌ Timeout: No position data for {robot_name}")
    # tts_manager.say(f"Can't get position data for {robot_name}. Please check the tracking system.")
    return None

def get_current_position(robot_name: str) -> tuple:
    with cache_lock:
        rigid = robot_position_cache.get(robot_name)
        if rigid:
            x = rigid["x"]
            y = rigid["z"]
            heading_y = rigid["heading_y"]
            return x, y, heading_y
    print(f"⚠️ No position data for {robot_name}")
    tts_manager.say(f"Can't get position data for {robot_name}. Please check the tracking system.")
    return 0.0, 0.0, 0.0
 
 
def _ensure_pub(node: Node, robot_name: str) -> Publisher:
    key = (id(node), robot_name)
    pub = publisher_dict.get(key)
    if pub is None:
        pub = node.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
        publisher_dict[key] = pub
    return pub
 
 
def safe_publish_twist(node: Node, robot_name: str, twist: Twist):
    key = (id(node), robot_name)
    pub = _ensure_pub(node, robot_name)
    try:
        pub.publish(twist)
    except InvalidHandle:
        pub = node.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
        publisher_dict[key] = pub
        pub.publish(twist)
 
 
def rotate_to_face_target(node: Node, robot_id: str, target: Dict[str, float],
                          angle_tolerance_deg: float = 5.0):
    x_target, y_target = target["x"], target["y"]
    x_now, y_now, _ = get_current_position(robot_id)
    dx = x_target - x_now
    dz = y_target - y_now
    target_angle = math.degrees(math.atan2(dx, dz)) % 360
 
    print(f"\n🔄 ROTATE | target: ({x_target:.1f}, {y_target:.1f}) → {target_angle:.1f}°")
 
    while True:
        _, _, heading_y_now = get_current_position(robot_id)
        angle_error = (target_angle - heading_y_now + 180) % 360 - 180

        if abs(angle_error) < angle_tolerance_deg:
            print("✅ ROTATE done.")
            break
 
        new_direction = 1 if angle_error > 0 else -1
        twist = Twist()

        if abs(angle_error) > 25:
            twist.angular.z = 0.5 * new_direction
        elif abs(angle_error) > 10:
            twist.angular.z = 0.3 * new_direction
        else:
            twist.angular.z = 0.15 * new_direction
 
        safe_publish_twist(node, robot_id, twist)
        print(f"↪️ turning {'left' if new_direction==1 else 'right'} | heading_y: {heading_y_now:.1f} | "
              f"target_angle: {target_angle:.1f} | error: {angle_error:.1f}° | speed: {twist.angular.z:.2f}")

        time.sleep(0.1)
 
    safe_publish_twist(node, robot_id, Twist())
    time.sleep(0.2)
 
 
def rotate_to_final_heading(node: Node, robot_name: str, heading_deg: float,
                            angle_tolerance_deg: float = 5.0):
    if heading_deg is None:
        print("No final heading specified; skipping final rotate.")
        return True  # 不需要旋转，视为成功

    try:
        heading_deg = float(heading_deg)
    except (TypeError, ValueError):
        print(f"⚠️ Invalid heading_deg: {heading_deg}, skipping final rotate.")
        return True

    print(f"ROTATE TO HEADING: {heading_deg:.1f}°")
    while True:
        _, _, heading_y_now = get_current_position(robot_name)
        angle_error = (heading_deg - heading_y_now + 180) % 360 - 180
        if abs(angle_error) < angle_tolerance_deg:
            print("✅ Final heading aligned.")
            break

        direction = 1 if angle_error > 0 else -1
        twist = Twist()

        if abs(angle_error) > 25:
            twist.angular.z = 0.5 * direction
        elif abs(angle_error) > 10:
            twist.angular.z = 0.3 * direction
        else:
            twist.angular.z = 0.15 * direction
 
        safe_publish_twist(node, robot_name, twist)
        print(f"↪️ adjusting to heading {heading_deg:.1f}°, current={heading_y_now:.1f}°, error={angle_error:.1f}°")
        time.sleep(0.1)
 
    safe_publish_twist(node, robot_name, Twist())
    time.sleep(0.2)
 
 
def move_forward_until_reached(node: Node, robot_name: str, target: Dict[str, float],
                               tolerance: float = 50.0, max_acceptable_angle_error: float = 25.0):
    x_target, y_target = target["x"], target["y"]

    print(f"\n🚗 NEED TO MOVE → ({x_target:.1f}, {y_target:.1f})")

    while True:
        x_now, y_now, heading_y_now = get_current_position(robot_name)
        dx = x_target - x_now
        dz = y_target - y_now
        distance = math.hypot(dx, dz)

        if distance < tolerance:
            print("🎉 Reached target.")
            break

        # 目标方向角与误差
        target_angle = math.degrees(math.atan2(dx, dz)) % 360
        angle_error = (target_angle - heading_y_now + 180) % 360 - 180
 
        if abs(angle_error) > max_acceptable_angle_error:
            print(f"🔁 Too much angle error: {angle_error:.1f}°, rotating first...")
            rotate_to_face_target(node, robot_name, target)
            continue
 
        # 前进（小速度：精度更好）
        twist = Twist()
        twist.linear.x = 0.1
        safe_publish_twist(node, robot_name, twist)
        print(f"🚗 Moving | dist={distance:.2f} | heading={heading_y_now:.1f}°, target={target_angle:.1f}°, "
              f"error={angle_error:.1f}°")

        time.sleep(0.2)

        safe_publish_twist(node, robot_name, Twist())

        time.sleep(0.1)
 
 
def navigate_to_position(node: Node, robot_name: str, target: Dict[str, float]):
    x_target, y_target = target["x"], target["y"]
    x_now, y_now, _ = get_current_position(robot_name)
    dx = x_target - x_now
    dy = y_target - y_now

    distance = math.hypot(dx, dy)
 
    print(f"\n🧭 NAVIGATE {robot_name} → ({x_target:.1f}, {y_target:.1f}) | dist={distance:.2f}")
 
    # Phase 1: 先对准
    rotate_to_face_target(node, robot_name, target)
 
    # Phase 2: 直行
    move_forward_until_reached(node, robot_name, target)
 
    # Phase 3: 若给了目标朝向则调整
    if "heading_deg" in target:
        rotate_to_final_heading(node, robot_name, target["heading_deg"])
 
    print(f"✅ {robot_name} navigation complete.")
 
 
def navigate_to_target(node: Node, executor: MultiThreadedExecutor, robot_name: str, target):
    is_successful = False

    # 在函数最前面加一次（可用一个flag避免重复init）
    if not hasattr(node, "rdz_pub"):
        init_rendezvous_comms(node)

    # 1) 确保位置跟踪节点已接入 executor 并数据就绪
    tracker_robot = getRobotPositionCache(robot_name, executor)
    if tracker_robot is None:
        print("❌ Abort navigation due to missing pose.")
        tts_manager.say(f"Can't get position data for {robot_name}. Please check the tracking system.")
        return is_successful
 
    # 2) 解析语义位置或直接使用坐标
    if isinstance(target, str):
        # firstly, try to find real position
        tracker_target = getRobotPositionCache(target, executor)
        if tracker_target:
            x, y, heading = get_current_position(target)
            resolved_target = {"x": x, "y": y, "heading_deg": heading}
            print(f"🔍 Resolved semantic target in tracking system '{target}' → {resolved_target}")
        else:
            if target in semantic_locations:
                resolved_target = semantic_locations[target]
                print(f"🔍 Resolved semantic target '{target}' → {resolved_target}")
            else:
                print(f"❌ Error: target '{target}' not found in semantic_locations")
                tts_manager.say(f"Can't get position data for {target}. Please check the tracking system or config file")
                resolved_target = None

    else:
        resolved_target = target
 
    # 3) 执行导航
    if isinstance(resolved_target, dict) and "x" in resolved_target and "y" in resolved_target:
        print(f"[DEBUG] {robot_name} BEFORE rendezvous assign: {resolved_target}")
        resolved_target = _assign_rendezvous_slot_distributed(node, resolved_target, robot_name,
                                                      base_radius=200.0, delta_radius=10.0)

        print(f"[DEBUG] {robot_name} AFTER rendezvous assign:  {resolved_target}")
 
        if "heading_deg" in resolved_target:
            print(f"[DEBUG] {robot_name} final heading: {resolved_target['heading_deg']}°")
        navigate_to_position(node, robot_name, resolved_target)
        is_successful = True
    else:
        print(f"[DEBUG] ⚠️ {robot_name} invalid resolved target: {resolved_target}")
        is_successful = False
 
    print(f"[DEBUG] {robot_name} navigate_to_target finished, success={is_successful}")
    return is_successful
 