
import math
import time
import json
import argparse
import threading
from loguru import logger
from typing import Dict, Optional

import os, sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from std_msgs.msg import String

import config
from ttsRepo.stream_tts import tts_manager
from phasespace.rigid_tracker import RigidTracker

import os, sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from robotControllerRepo.actions.navigate import navigate_to_target, get_current_position

position_cache: Dict[str, Dict[str, float]] = {}
cache_lock = threading.Lock()
semantic_locations = config.get("semantic_locations")

PARTICLE = (0.0, -500.0)
TARGET   = (500.0, -1000.0)
GAP = 65 
LENGTH = 138
WIDTH = 178
SPEED    = 0.05

NAV_TIMEOUT_SEC          = 120.0 
WAIT_READY_TIMEOUT_SEC   = 120.0 
WAIT_GO_TIMEOUT_SEC      = 120.0

# ===== 工具函数 =====
def getRobotPositionCache(name: str, executor: MultiThreadedExecutor) -> Optional[Node]:
    rigid_node = RigidTracker(
        position_cache=position_cache,
        name=name,
        position_lock=cache_lock,
    )
    executor.add_node(rigid_node)
    print(f"[POSITION] Initializing tracker for {name}")
    tts_manager.say_sync(f"Initializing tracker for {name}, waiting for position data.")
    
    for _ in range(50): 
        with cache_lock:
            if name in position_cache:
                print(f"[POSITION] Position data acquired for {name}")
                tts_manager.say_sync(f"Position data acquired for {name}.")
                return rigid_node
        time.sleep(0.2)
    
    print(f"[ERROR] Position tracking timeout for {name}")
    return None

def get_position_with_polling(robot_name: str, timeout_sec=120, poll_interval=0.1):
    start_time = time.time()
    while time.time() - start_time < timeout_sec:
        x, y, heading = get_current_position(robot_name)
        if None not in (x, y, heading):
            return x, y, heading
        time.sleep(poll_interval)
    logger.error(f"[POSITION] Timeout getting position for {robot_name}")
    return None, None, None

def plan_formation(particle_xy, target_xy):
    # error
    r1_x_error=0
    r2_x_error=-20

    r1_y_error=0
    r2_y_error=0

    r1_heading_error=0
    r2_heading_error=0

    px, py = particle_xy.get("x"), particle_xy.get("y")
    tx, ty = target_xy.get("x"), target_xy.get("y")

    phi = math.atan2(ty - py, tx - px)

    r1_x = GAP + LENGTH/2 - px
    r1_y = py  - (GAP + WIDTH/2)
    r1_heading = phi + math.pi

    r2_x = -(GAP + LENGTH/2 - px)
    r2_y = py + (GAP + WIDTH/2)
    r2_heading = phi

    r1 = (r1_x + r1_x_error, r1_y + r1_y_error, r1_heading + r1_heading_error)
    r2 = (r2_x + r2_x_error, r2_y + r2_y_error, r2_heading + r2_heading_error)

    path_dis = math.hypot(tx - px, ty - py)

    return phi, r1, r2, path_dis

class Motion:
    def __init__(self, node: Node, robot_ns: str):
        self.node = node
        self.pub = node.create_publisher(Twist, f'/{robot_ns}/cmd_vel', 10)

    def stop(self):
        self.pub.publish(Twist())

    def drive_constant(self, forward: bool, dist_mm: float, speed_mps: float):
        dist_m = dist_mm / 1000.0
        dur = dist_m / max(speed_mps, 1e-6)

        tw = Twist()
        tw.linear.x = speed_mps if forward else -speed_mps

        t0 = time.time()
        while time.time() - t0 < dur:
            self.pub.publish(tw)
            time.sleep(0.02)
        self.stop()

MSG_SYN, MSG_ACK, MSG_READY, MSG_GO, MSG_ABORT = "syn", "ack", "ready", "go", "abort"


def check_formation_symmetry(node: Node, robot1_pose, robot2_pose, particle_xy):
    x1, y1, theta1 = robot1_pose
    x2, y2, theta2 = robot2_pose
    px, py = particle_xy

    dx = x1 - x2
    dy = y1 - y2
    dist = math.hypot(dx, dy)

    mid_x = (x1 + x2) / 2
    mid_y = (y1 + y2) / 2
    offset = math.hypot(mid_x - px, mid_y - py)

    dtheta = (theta1 - theta2 + 180) % 360 - 180
    angle_ok = abs(abs(dtheta) - 180) < 10

    print(f"Formation check:")
    print(f"  gap: {dist:.1f} mm (expected: {GAP} mm)")
    print(f"  center offset: {offset:.1f} mm (should be ~0)")
    print(f"  heading diff: {dtheta:.1f}° (should be ±180°)")
    print(f"  symmetry: {'YES' if abs(dist - GAP) < 30 and offset < 50 and angle_ok else 'NO'}")

class TransportManager:
    def __init__(self, node: Node, executor: MultiThreadedExecutor, robot_id: str, item, start, goal):
        self.node = node
        self.executor = executor
        self.robot_id = robot_id
        self.peer_id = "robot1" if robot_id == "robot2" else "robot2"

        self.completed = False
        self.success = False


        # Resolve start
        if isinstance(start, str):
            resolved_start = getRobotPositionCache(start, executor)
            if resolved_start:
                x, y, heading = get_position_with_polling(start)
                resolved_start = {"x": x, "y": y, "heading": heading}
                print(f"[TARGET_RESOLUTION] Dynamic target '{start}' resolved to ({x:.1f}, {y:.1f})")
            else:
                if start in semantic_locations:
                    resolved_start = semantic_locations[start]
                    print(f"[TARGET_RESOLUTION] Semantic target '{start}' resolved")
                else:
                    logger.error(f"[ERROR] Target '{start}' not found")
                    tts_manager.say_sync(f"Can't resolve target {start}")
                    return False
        else:
            resolved_start = start

        if not (isinstance(resolved_start, dict) and "x" in resolved_start and "y" in resolved_start):
            print(f"[ERROR] Invalid target format: {resolved_start}")
            return False
        
        # Resolve goal
        if isinstance(goal, str):
            resolved_goal = getRobotPositionCache(goal, executor)
            if resolved_goal:
                x, y, heading = get_position_with_polling(goal)
                resolved_goal = {"x": x, "y": y, "heading": heading}
                print(f"[TARGET_RESOLUTION] Dynamic target '{goal}' resolved to ({x:.1f}, {y:.1f})")
            else:
                if goal in semantic_locations:
                    resolved_goal = semantic_locations[goal]
                    print(f"[TARGET_RESOLUTION] Semantic target '{goal}' resolved")
                else:
                    logger.error(f"[ERROR] Target '{goal}' not found")
                    tts_manager.say_sync(f"Can't resolve target {goal}")
                    return False
        else:
            resolved_goal = goal

        if not (isinstance(resolved_goal, dict) and "x" in resolved_goal and "y" in resolved_goal):
            print(f"[ERROR] Invalid target format: {resolved_goal}")
            return False

        # Phase1:
        logger.info(f"[COORDINATION] Starting navigation coordination for {resolved_start} --> {resolved_goal}")
        self.phi, self.r1, self.r2, self.path_len = plan_formation(resolved_start, resolved_goal)
        self.is_r1 = (robot_id == "robot1")

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub = node.create_publisher(String, "/transport_coordination", qos)
        self.sub = node.create_subscription(String, "/transport_coordination", self.on_bus, qos)

        self.have_ack = False
        self.peer_ready = False
        self.start_at = None
        self.aborted = False

        self.ack_event = threading.Event()
        self.peer_ready_event = threading.Event()
        self.go_event = threading.Event()
        self.abort_event = threading.Event()

        self.motion = Motion(node, robot_id)

        self.worker = threading.Thread(target=self._main, daemon=True)
        self.worker.start()

    def _publish(self, typ, payload):
        msg = String()
        msg.data = json.dumps({"type": typ, **payload, "ts": time.time()})
        self.pub.publish(msg)

    def _abort(self, reason: str):
        if not self.aborted:
            self.aborted = True
            self._publish(MSG_ABORT, {"who": self.robot_id, "reason": reason})
            self.node.get_logger().error(f"ABORT: {reason}")
        self.motion.stop()
        self.abort_event.set()
        self.ack_event.set()
        self.peer_ready_event.set()
        self.go_event.set()

    def on_bus(self, msg: String):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        typ = d.get("type")
        who = d.get("who")

        if typ == MSG_SYN and who != self.robot_id:
            self._publish(MSG_ACK, {"who": self.robot_id})

        elif typ == MSG_ACK and who == self.peer_id:
            self.have_ack = True
            self.ack_event.set()
            logger.info("ready1")

        elif typ == MSG_READY and who == self.peer_id:
            self.peer_ready = True
            self.peer_ready_event.set()
            logger.info("ready2")

        elif typ == MSG_GO:
            self.start_at = float(d["start_at"])
            if "dist" in d:
                self.path_len = float(d["dist"])
            self.go_event.set()
            logger.info("go")

        elif typ == MSG_ABORT:
            self.aborted = True
            self.abort_event.set()
            self.motion.stop()
            self.ack_event.set()
            self.peer_ready_event.set()
            self.go_event.set()
            logger.warning("Received ABORT from other robots. Stopping...")

    def _main(self):
        try:
            self._publish(MSG_SYN, {"who": self.robot_id})
            self.ack_event.wait(timeout=3.0)

            if self.is_r1:
                x_goal, y_goal, yaw_goal = self.r1
            else:
                x_goal, y_goal, yaw_goal = self.r2

            target = {"x": x_goal, "y": y_goal, "heading_deg": math.degrees(yaw_goal)}
            self.node.get_logger().info(f"[{self.robot_id}]: navigating to {target} ...")

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
                self._abort(f"Navigation timeout (> {NAV_TIMEOUT_SEC}s)")
                return

            if nav_result["ok"] is False:
                self._abort("Navigation failed (navigate_to_target returned False)")
                return
            
            if self.is_r1:
                x1, y1, h1 = get_position_with_polling("robot1")
                x2, y2, h2 = get_position_with_polling("robot2")

                robot1_pose = (x1, y1, h1)
                robot2_pose = (x2, y2, h2)
                particle_xy = PARTICLE
                check_formation_symmetry(self.node, robot1_pose, robot2_pose, particle_xy)

            if self.aborted:
                return

            self._publish(MSG_READY, {"who": self.robot_id})
            if not self.peer_ready_event.wait(timeout=WAIT_READY_TIMEOUT_SEC):
                self._abort(f"Peer READY timeout (> {WAIT_READY_TIMEOUT_SEC}s)")
                return

            if self.aborted:
                return

            if self.is_r1:
                self.start_at = time.time() + 0.6
                self._publish(MSG_GO, {"start_at": self.start_at, "dist": self.path_len, "speed": SPEED})
            else:
                if not self.go_event.wait(timeout=WAIT_GO_TIMEOUT_SEC):
                    self._abort(f"GO wait timeout (> {WAIT_GO_TIMEOUT_SEC}s)")
                    return

            if self.aborted:
                return
            
            while True:
                remain = self.start_at - time.time()
                if remain <= 0:
                    break
                time.sleep(0.0005 if remain < 0.002 else 0.005)

            forward = self.is_r1 

            time.sleep(180)

            self.node.get_logger().info(
                f"START transport | forward={forward} | dist={self.path_len:.2f} m | v={SPEED:.3f} m/s"
            )
            self.motion.drive_constant(forward, self.path_len, SPEED)
            logger.info("Contactless Transport Done!")
            
            self.success = True
            
        except Exception as e:
            logger.error(f"Transport error in {self.robot_id}: {e}")
            self._abort(f"Transport error: {e}")
        finally:
            self.completed = True
