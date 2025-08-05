import sys, math, time
from typing import Dict
from geometry_msgs.msg import Twist

# 保证根路径可 import
sys.path.append("/home/ubuntu2204/Desktop/yingying/ai-assistant_macos")

from phasespace.rigid_tracker import RigidTracker
from config import semantic_locations

# === 全局缓存 ===
robot_position_cache: Dict[str, Dict] = {}
publisher_dict: Dict[str, Twist] = {}

# ── 位置订阅 ───────────────────────────────────────────────
def get_robot_position(robot_id: str, executor):
    node = RigidTracker(robot_position_cache, robot_id)
    executor.add_node(node)

    for _ in range(30):
        if robot_id in robot_position_cache:
            break
        time.sleep(0.2)
    else:
        print(f"❌ No position data for {robot_id}")
    return node

def current_pose(robot: str):
    d = robot_position_cache.get(robot)
    return (d["x"], d["z"], d["heading_y"]) if d else (0.0, 0.0, 0.0)

# ── 基元动作 ───────────────────────────────────────────────
def rotate_to_face(robot, pub, target, tol=5):
    x_t, y_t = target["x"], target["y"]
    x, y, _ = current_pose(robot)
    target_ang = math.degrees(math.atan2(x_t - x, y_t - y)) % 360
    while True:
        _, _, heading = current_pose(robot)
        err = (target_ang - heading + 180) % 360 - 180
        if abs(err) < tol:
            break
        twist = Twist();  twist.angular.z = 0.5 if err > 0 else -0.5
        pub.publish(twist);  time.sleep(0.1)
    pub.publish(Twist());  time.sleep(0.2)

def rotate_to_heading(robot, pub, heading, tol=5):
    if heading is None:
        return
    while True:
        _, _, h = current_pose(robot)
        err = (heading - h + 180) % 360 - 180
        if abs(err) < tol:
            break
        twist = Twist();  twist.angular.z = 0.5 if err > 0 else -0.5
        pub.publish(twist);  time.sleep(0.1)
    pub.publish(Twist());  time.sleep(0.2)

def forward_until(robot, pub, target, tol=20):
    x_t, y_t = target["x"], target["y"]
    while True:
        x, y, _ = current_pose(robot)
        if math.hypot(x_t - x, y_t - y) < tol:
            break
        twist = Twist();  twist.linear.x = 0.1
        pub.publish(twist);  time.sleep(0.2);  pub.publish(Twist());  time.sleep(0.1)

# ── 高层 API ───────────────────────────────────────────────
def navigate_to_position(node, robot, tgt):
    pub = publisher_dict.setdefault(
        robot, node.create_publisher(Twist, f"/{robot}/cmd_vel", 10)
    )
    rotate_to_face(robot, pub, tgt)
    forward_until(robot, pub, tgt)
    rotate_to_heading(robot, pub, tgt.get("heading_deg"))
    print(f"✅ {robot} navigation complete.")

def navigate_to_target(node, robot, target, executor):
    get_robot_position(robot, executor)

    # 解析语义点
    if isinstance(target, str):
        if target not in semantic_locations:
            print(f"❌ Unknown target '{target}'");  return
        target = semantic_locations[target]

    if isinstance(target, dict) and "x" in target and "y" in target:
        navigate_to_position(node, robot, target)
    else:
        print(f"⚠️ Invalid target {target}")
