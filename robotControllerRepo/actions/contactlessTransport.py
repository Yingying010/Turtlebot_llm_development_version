#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Collaborative Transport with PhaseSpace (using navigate.py)

流程:
  Phase1: 计算两车编队位姿（以微粒点为中心、指向目标，背靠背，固定间距）
  Phase2: 调用 navigate.py 的 navigate_to_target 导航到编队位姿并校正朝向（带超时）
  Phase3: 分布式同步直线运输：robot1 前进(+v)，robot2 后退(-v)，保持基线不变（READY/GO 等待带超时）

运行:
  robot1:
    python3 utils/test_collaborate_particles_decentralized.py --robot_id robot1
  robot2:
    python3 utils/test_collaborate_particles_decentralized.py --robot_id robot2
"""

import math
import time
import json
import argparse
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from std_msgs.msg import String

# ===== 使用你现有的导航函数 =====
import os, sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from robotControllerRepo.actions.navigate import navigate_to_target

# ===== 全局参数（直接改这里就行） =====
PARTICLE = (0.0, -1000.0)     # 微粒点坐标
TARGET   = (1500.0, -1500.0)    # 目标点坐标
BASELINE = 760           # 两车中心间距 38cm
SPEED    = 0.02          # 运输速度 (m/s)

# 超时配置（秒）
NAV_TIMEOUT_SEC          = 60.0   # Phase 2 导航总超时
WAIT_READY_TIMEOUT_SEC   = 30.0   # 等对方 READY 的超时
WAIT_GO_TIMEOUT_SEC      = 30.0   # follower 等 GO 的超时

# ===== 工具函数 =====
def plan_formation(particle_xy, target_xy, baseline):
    px, py = particle_xy
    tx, ty = target_xy
    phi = -math.atan2(ty - py, tx - px)
    
    # 关键补偿：坐标系中 x 是反的
    ux, uy = math.cos(phi), math.sin(phi)

    half = 0.5 * baseline
    r2 = (px + half * ux, py - half * uy, -(phi + math.pi))           # robot1
    r1 = (px - half * ux, py + half * uy, -phi) # robot2
    path_len = math.hypot(tx - px, ty - py)
    return phi, r1, r2, path_len

# ===== 简单直线匀速控制（运输阶段用） =====
class Motion:
    def __init__(self, node: Node, robot_ns: str):
        self.node = node
        self.pub = node.create_publisher(Twist, f'/{robot_ns}/cmd_vel', 10)

    def stop(self):
        self.pub.publish(Twist())

    def drive_constant(self, forward: bool, dist: float, speed: float):
        dur = dist / max(speed, 1e-6)
        tw = Twist()
        tw.linear.x = speed if forward else -speed
        t0 = time.time()
        while time.time() - t0 < dur:
            self.pub.publish(tw)
            time.sleep(0.02)
        self.stop()

# ===== 协调协议 =====
MSG_SYN, MSG_ACK, MSG_READY, MSG_GO, MSG_ABORT = "syn", "ack", "ready", "go", "abort"

class TransportManager:
    """
    协作运输管理器（不继承 Node；复用外部 node 和 executor）
    将来在 controller.py 里可直接：TransportManager(controller_node, controller_executor, "robot1")
    """
    def __init__(self, node: Node, executor: MultiThreadedExecutor, robot_id: str):
        self.node = node
        self.executor = executor
        self.robot_id = robot_id
        self.peer_id = "robot1" if robot_id == "robot2" else "robot2"

        # Phase1: 队形规划
        self.phi, self.r1, self.r2, self.path_len = plan_formation(PARTICLE, TARGET, BASELINE)
        self.is_r1 = (robot_id == "robot1")
        self.node.get_logger().info(
            f"🧭 Formation phi={math.degrees(self.phi):.1f}° | path={self.path_len:.2f} m | baseline={BASELINE:.2f} m"
        )

        # 通信
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub = node.create_publisher(String, "/transport_coordination", qos)
        self.sub = node.create_subscription(String, "/transport_coordination", self.on_bus, qos)

        # 同步状态与事件（回调里置位，工作线程只等待事件）
        self.have_ack = False
        self.peer_ready = False
        self.start_at = None
        self.aborted = False

        self.ack_event = threading.Event()
        self.peer_ready_event = threading.Event()
        self.go_event = threading.Event()
        self.abort_event = threading.Event()

        # 直线控制
        self.motion = Motion(node, robot_id)

        # 启动工作线程（不调用 spin / spin_once）
        self.worker = threading.Thread(target=self._main, daemon=True)
        self.worker.start()

    # ---------- BUS 回调（由主线程 executor.spin() 驱动） ----------
    def on_bus(self, msg: String):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        typ = d.get("type")
        who = d.get("who")

        if typ == MSG_SYN and who != self.robot_id:
            # 对方发 SYN → 回 ACK
            self._publish(MSG_ACK, {"who": self.robot_id})

        elif typ == MSG_ACK and who == self.peer_id:
            self.have_ack = True
            self.ack_event.set()
            self.node.get_logger().info("🤝 got ACK")

        elif typ == MSG_READY and who == self.peer_id:
            self.peer_ready = True
            self.peer_ready_event.set()
            self.node.get_logger().info("✅ peer READY")

        elif typ == MSG_GO:
            self.start_at = float(d["start_at"])
            if "dist" in d:
                self.path_len = float(d["dist"])
            self.go_event.set()
            self.node.get_logger().info(f"📨 GO received: start_at={self.start_at:.6f}")

        elif typ == MSG_ABORT:
            self.aborted = True
            self.abort_event.set()
            self.motion.stop()
            # 同时唤醒所有等待，防止死锁
            self.ack_event.set()
            self.peer_ready_event.set()
            self.go_event.set()
            self.node.get_logger().warning("⛔ Received ABORT from peer. Stopping.")

    def _publish(self, typ, payload):
        out = String()
        out.data = json.dumps({"type": typ, **payload, "ts": time.time()})
        self.pub.publish(out)

    def _abort(self, reason: str):
        if not self.aborted:
            self.aborted = True
            self._publish(MSG_ABORT, {"who": self.robot_id, "reason": reason})
            self.node.get_logger().error(f"⛔ ABORT: {reason}")
        self.motion.stop()
        # 唤醒等待
        self.abort_event.set()
        self.ack_event.set()
        self.peer_ready_event.set()
        self.go_event.set()

    # ---------- 主流程（Phase1→Phase2→Phase3） ----------
    def _main(self):
        # Phase1: 握手（事件等待，不阻塞 executor）
        self._publish(MSG_SYN, {"who": self.robot_id})
        self.ack_event.wait(timeout=3.0)  # 最多等 3s；没拿到也继续后面的流程

        # Phase2: 导航到编队位姿（调用 navigate.py 的 navigate_to_target，并增加超时保护）
        if self.is_r1:
            x_goal, y_goal, yaw_goal = self.r1
        else:
            x_goal, y_goal, yaw_goal = self.r2

        target = {"x": x_goal, "y": y_goal, "heading_deg": math.degrees(yaw_goal)}
        self.node.get_logger().info(f"🚗 {self.robot_id} navigating to {target} ...")

        nav_done = threading.Event()
        nav_result = {"ok": None}

        def _run_nav():
            try:
                ok = navigate_to_target(self.node, self.executor, self.robot_id, target)
            except Exception as e:
                self.node.get_logger().error(f"navigate_to_target raised: {e}")
                ok = False
            nav_result["ok"] = ok
            nav_done.set()

        nav_thread = threading.Thread(target=_run_nav, daemon=True)
        nav_thread.start()

        if not nav_done.wait(timeout=NAV_TIMEOUT_SEC):
            # 导航超时
            self._abort(f"Navigation timeout (> {NAV_TIMEOUT_SEC}s)")
            return

        if nav_result["ok"] is False:
            self._abort("Navigation failed (navigate_to_target returned False)")
            return

        if self.aborted:
            return

        # 广播 READY 并等待对方 READY（带超时）
        self._publish(MSG_READY, {"who": self.robot_id})
        if not self.peer_ready_event.wait(timeout=WAIT_READY_TIMEOUT_SEC):
            self._abort(f"Peer READY timeout (> {WAIT_READY_TIMEOUT_SEC}s)")
            return

        if self.aborted:
            return

        # Phase3: 分布式同步运输
        if self.is_r1:
            self.start_at = time.time() + 0.6  # 预留 600ms
            self._publish(MSG_GO, {"start_at": self.start_at, "dist": self.path_len, "speed": SPEED})
        else:
            if not self.go_event.wait(timeout=WAIT_GO_TIMEOUT_SEC):
                self._abort(f"GO wait timeout (> {WAIT_GO_TIMEOUT_SEC}s)")
                return

        if self.aborted:
            return

        # 忙等到绝对时刻（最后 2ms 忙等，其余轻睡）
        while True:
            remain = self.start_at - time.time()
            if remain <= 0:
                break
            time.sleep(0.0005 if remain < 0.002 else 0.005)

        forward = self.is_r1  # r1 前进(+v)，r2 后退(-v)
        self.node.get_logger().info(
            f"🏁 START transport | forward={forward} | dist={self.path_len:.2f} m | v={SPEED:.3f} m/s"
        )
        self.motion.drive_constant(forward, self.path_len, SPEED)
        self.node.get_logger().info("✅ Transport done")

# ========== 主入口：创建一次 node 和 executor，后续都复用 ==========
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot_id", required=True, choices=["robot1", "robot2"])
    args = parser.parse_args()

    rclpy.init()
    node = Node("collab_transport")
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # 交给管理器；未来 controller.py 可直接传入自己的 node/executor
    manager = TransportManager(node, executor, args.robot_id)

    try:
        executor.spin()  # 只在主线程 spin，所有回调/事件都靠它驱动
    except KeyboardInterrupt:
        pass
    finally:
        manager.motion.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
