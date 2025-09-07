#!/usr/bin/env python3

# follow.py —— 不改 rigid_tracker.py，单进程跟随
 
import os, sys, math, time, threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from typing import Dict, Tuple, Optional, List
from rclpy.publisher import Publisher
from rclpy._rclpy_pybind11 import InvalidHandle
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy
from WhisperRepo.whisper_background import run_continuous_listen


sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from ttsRepo.stream_tts import tts_manager
from phasespace.rigid_tracker import RigidTracker

robot_position_cache: Dict[str, Dict[str, float]] = {}
cache_lock = threading.Lock()
publisher_dict: Dict[Tuple[int, str], Publisher] = {}

SPEECH_TOPIC = "/speech_text"
CTRL_TOPIC   = "/follow_service"

_WHISPER_REF = 0
_WHISPER_LOCK = threading.Lock()
_WHISPER_THREAD: threading.Thread | None = None
_WHISPER_STOP: threading.Event | None = None

def _whisper_ref_inc():
    global _WHISPER_REF, _WHISPER_THREAD, _WHISPER_STOP
    with _WHISPER_LOCK:
        _WHISPER_REF += 1
        if _WHISPER_REF == 1:
            _WHISPER_STOP = threading.Event()
            _WHISPER_THREAD = threading.Thread(
                target=run_continuous_listen,
                kwargs={"stop_event": _WHISPER_STOP, "sleep_after_publish": 0.05},
                daemon=True,
                name="whisper-bg",
            )
            _WHISPER_THREAD.start()

def _whisper_ref_dec():
    global _WHISPER_REF, _WHISPER_THREAD, _WHISPER_STOP
    with _WHISPER_LOCK:
        _WHISPER_REF = max(0, _WHISPER_REF - 1)
        if _WHISPER_REF == 0 and _WHISPER_THREAD:
            try:
                if _WHISPER_STOP:
                    _WHISPER_STOP.set()
                _WHISPER_THREAD.join(timeout=2.0)
            finally:
                _WHISPER_THREAD = None
                _WHISPER_STOP = None
 
def getRobotPositionCache(robot_name: str, executor: MultiThreadedExecutor) -> Optional[Node]:
    rigid_node = RigidTracker(
        position_cache=robot_position_cache,
        robot_name=robot_name,
        position_lock=cache_lock, 
    )
    executor.add_node(rigid_node)
    print(f"Waiting for position data of {robot_name}...")
    tts_manager.say_sync(f"Initializing tracker for {robot_name}, waiting for position data.")
    for _ in range(50):  # ~10s
        with cache_lock:
            ok = robot_name in robot_position_cache
        if ok:
            print(f"Got position data for {robot_name}.")
            tts_manager.say_sync(f"Position data acquired for {robot_name}.")
            return rigid_node
        time.sleep(0.2)
    print(f"Timeout: No position data for {robot_name}")
    tts_manager.say_sync(f"Can't get position data for {robot_name}. Please check the tracking system.")
    return None

def get_current_position(robot_name: str) -> tuple:
    with cache_lock:
        rigid = robot_position_cache.get(robot_name)
        if rigid:
            x = rigid["x"]
            y = rigid["z"]
            heading_y = rigid["heading_y"]
            return x, y, heading_y
    print(f"No position data for {robot_name}")
    tts_manager.say_sync(f"Can't get position data for {robot_name}. Please check the tracking system.")
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
                          stop_event: threading.Event | None = None,
                          angle_tolerance_deg: float = 5.0):
    if stop_event and stop_event.is_set():
        safe_publish_twist(node, robot_id, Twist())
        return

    x_target, y_target = target["x"], target["y"]
    x_now, y_now, _ = get_current_position(robot_id)
    dx = x_target - x_now
    dz = y_target - y_now
    target_angle = math.degrees(math.atan2(dx, dz)) % 360
    print(f"\nROTATE | target: ({x_target:.1f}, {y_target:.1f}) → {target_angle:.1f}°")

    while rclpy.ok():
        if stop_event and stop_event.is_set():
            break

        _, _, heading_y_now = get_current_position(robot_id)
        angle_error = (target_angle - heading_y_now + 180) % 360 - 180

        if abs(angle_error) < angle_tolerance_deg:
            print("ROTATE done.")
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
        print(f"↪turning {'left' if new_direction==1 else 'right'} | heading_y: {heading_y_now:.1f} | "
              f"target_angle: {target_angle:.1f} | error: {angle_error:.1f}° | speed: {twist.angular.z:.2f}")
        time.sleep(0.1)

    safe_publish_twist(node, robot_id, Twist())
    time.sleep(0.1)

def move_forward_until_reached(node: Node, robot_name: str, target: Dict[str, float],
                               stop_event: threading.Event | None = None,
                               tolerance: float = 20.0,
                               max_acceptable_angle_error: float = 25.0):
    if stop_event and stop_event.is_set():
        safe_publish_twist(node, robot_name, Twist())
        return

    x_target, y_target = target["x"], target["y"]
    print(f"\nNEED TO MOVE → ({x_target:.1f}, {y_target:.1f})")

    while rclpy.ok():
        if stop_event and stop_event.is_set():
            break

        x_now, y_now, heading_y_now = get_current_position(robot_name)
        dx = x_target - x_now
        dz = y_target - y_now
        distance = math.hypot(dx, dz)

        if distance < tolerance:
            print("Reached target.")
            break

        target_angle = math.degrees(math.atan2(dx, dz)) % 360
        angle_error = (target_angle - heading_y_now + 180) % 360 - 180

        if abs(angle_error) > max_acceptable_angle_error:
            print(f"Too much angle error: {angle_error:.1f}°, rotating first...")
            rotate_to_face_target(node, robot_name, target, stop_event=stop_event)
            continue

        twist = Twist()
        twist.linear.x = 0.1
        safe_publish_twist(node, robot_name, twist)
        print(f"Moving | dist={distance:.2f} | heading={heading_y_now:.1f}°, "
              f"target={target_angle:.1f}°, error={angle_error:.1f}°")
        time.sleep(0.2)

        safe_publish_twist(node, robot_name, Twist())
        time.sleep(0.1)

    safe_publish_twist(node, robot_name, Twist())
    time.sleep(0.05)
  
FOLLOW_DIST_MM = 200.0
HYSTERESIS_MM = 50.0

def follow_loop(ctrl_node: Node, follower: str, target: str, stop_event: threading.Event):
    tts_manager.say_sync(f"{follower} is now following {target}")
    try:
        while rclpy.ok() and not stop_event.is_set():
            fx, fz, _ = get_current_position(follower)
            tx, tz, _ = get_current_position(target)
            dx, dz = tx - fx, tz - fz
            dist = math.hypot(dx, dz)
            tgt = {"x": tx, "y": tz}
 
            if dist > FOLLOW_DIST_MM + HYSTERESIS_MM:
                rotate_to_face_target(ctrl_node, follower, tgt, stop_event=stop_event)
                if stop_event.is_set(): break
                move_forward_until_reached(ctrl_node, follower, tgt,
                                           tolerance=FOLLOW_DIST_MM + HYSTERESIS_MM,
                                           stop_event=stop_event)
            else:
                safe_publish_twist(ctrl_node, follower, Twist())
                rotate_to_face_target(ctrl_node, follower, tgt, stop_event=stop_event)
            time.sleep(0.15)
    finally:
        safe_publish_twist(ctrl_node, follower, Twist())
        tts_manager.say_sync(f"{follower} stopped following {target}")
 
class SpeechStopListener(Node):
    def __init__(self, stop_event: threading.Event, name_suffix: str):
        super().__init__(f"follow_speech_listener_{name_suffix}")
        self.stop_event = stop_event
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.sub = self.create_subscription(String, SPEECH_TOPIC, self.on_text, qos)
        self.keywords: List[str] = [
            "stop follow", "stop following", "stop following me"
        ]

    def on_text(self, msg: String):
        text = (msg.data or "").strip().lower()
        for kw in self.keywords:
            if kw in text:
                self.get_logger().info(f"🛑 stop keyword detected: '{kw}' in '{text}'")
                self.stop_event.set()
                break


# === 主入口 ===
def follow_run(node: Node, follower: str, target: str, executor: MultiThreadedExecutor):
    stop_event = threading.Event()
    is_successful = False

    tracker_follower = getRobotPositionCache(follower, executor)
    if tracker_follower is None:
        print(f"Abort follow: no pose for {follower}.")
        _whisper_ref_dec()
        return is_successful
 
    tracker_target = getRobotPositionCache(target, executor)
    if tracker_target is None:
        print(f"Abort follow: no pose for {target}.")
        try:
            executor.remove_node(tracker_follower)
        except Exception:
            pass
        try:
            tracker_follower.destroy_node()
        except Exception:
            pass
        _whisper_ref_dec()
        return is_successful
    

    _whisper_ref_inc()
 
    listener = SpeechStopListener(stop_event, name_suffix=follower)
    executor.add_node(listener)

    th = threading.Thread(
        target=follow_loop,
        args=(node, follower, target, stop_event),
        daemon=True,
        name=f"follow-{follower}",
    )
    th.start()

    try:
        while rclpy.ok() and th.is_alive() and not stop_event.is_set():
            time.sleep(0.2)
    finally:
        stop_event.set()
        th.join(timeout=2.0)

        safe_publish_twist(node, follower, Twist())
        time.sleep(0.05)

        for n in (listener, tracker_follower, tracker_target):
            try:
                executor.remove_node(n)
            except Exception:
                pass
            try:
                n.destroy_node()
            except Exception:
                pass

        _whisper_ref_dec()

        is_successful = True
        
        return is_successful