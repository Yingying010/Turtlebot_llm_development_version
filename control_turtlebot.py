import os, sys
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
import rclpy
import time
from loguru import logger
from geometry_msgs.msg import Twist
from actions.move import move_linear
from actions.rotate import rotate
from actions.imitate import imitate_robot, stop_imitation, imitation_threads
from actions.navigate import navigate_to_target
import threading
import time
from typing import Dict, List, Union
from subscriber.rigid_tracker import RigidTracker

# def move_linear(robot_id, direction, value, unit):
#     print(f"🚗 {robot_id} moving {direction} for {value} {unit}")

# def rotate(robot_id, direction, value, unit, target="self"):
#     print(f"🔁 {robot_id} turning {direction} {value}{unit} around {target}")

# def navigate_to_target(robot_id, target):
#     print(f"🧭 {robot_id} navigating to {target}")

def follow_target(robot_id, target):
    print(f"👣 {robot_id} following {target}")

def face_to_target(robot_id, target):
    print(f"🧍 {robot_id} facing {target}")

# def imitate_robot(robot_id, target_robot_id):
#     print(f"🎭 {robot_id} imitating {target_robot_id}")





def execute_action(command: Dict, robot_position_cache):
    robot_id = command["robot_id"]
    action   = command.get("action")

    if action == "move":
        move_linear(robot_id, command["direction"], command["value"], command["unit"])

    elif action == "turn":
        rotate(robot_id, command["direction"], command["value"], command["unit"], command.get("target", "self"))

    elif action == "navigate":
        if "position" in command:
            navigate_to_target(robot_id, command["position"], robot_position_cache)
        elif "target" in command:
            navigate_to_target(robot_id, command["target"], robot_position_cache)
        else:
            print(f"❌ Missing position or target in navigate command: {command}")

    # elif action == "follow":
    #     follow_target(robot_id, command["target"], robot_position_cache)

    # elif action == "imitate":
    #     imitate_robot(robot_id, command["target"], robot_position_cache)

    # elif action == "face_to":
    #     face_to_target(robot_id, command["target"], robot_position_cache)

    else:
        print(f"⚠️ Unknown action: {action}")


def flatten_response(response: dict):
    commands = []
    for robot_id, cmd_list in response.items():
        for cmd in cmd_list:
            cmd["robot_id"] = robot_id
            commands.append(cmd)
    return commands


def control_from_json_response(response: Dict[str, List[Dict]],robot_position_cache):
    for robot_id, commands in response.items():
        print(f"\n[🤖 Executing for {robot_id}]")
        for command in commands:
            execute_action(command, robot_position_cache)


# 共享的位置缓存
robot_position_cache = {}

def controller(command: Dict[str, List[Dict]]):
    rclpy.init()  # ✅ 初始化 ROS
    tracker_nodes = {}

    # ✅ 启动每个机器人的 PhaseSpace tracker 节点 + spin 线程
    for robot_id in command.keys():
        node = RigidTracker(robot_position_cache, robot_id)
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()

        # 等待 tracker 数据就绪
        print(f"⏳ Waiting for position data of {robot_id}...")
        for _ in range(30):
            if robot_id in robot_position_cache:
                print(f"✅ Got position data for {robot_id}.")
                print("📦 robot_position_cache =", robot_position_cache)
                break
            time.sleep(0.2)
        else:
            print(f"❌ Timeout: No position data for {robot_id}")
            return  # 或 raise 异常中止

        tracker_nodes[robot_id] = (node, spin_thread)

    # ✅ 多线程执行每个机器人的动作序列
    threads = []
    for robot_id, actions in command.items():
        for action in actions:
            action["robot_id"] = robot_id
        t = threading.Thread(target=execute_robot_commands, args=(robot_id, actions,robot_position_cache))
        t.start()
        threads.append(t)

    for t in threads:
        t.join()

    # ✅ 关闭 ROS
    if len(imitation_threads) == 0:
        for node, _ in tracker_nodes.values():
            node.destroy_node()
        rclpy.shutdown()
    else:
        print("⏳ ROS still running: imitation is active.")


def execute_robot_commands(robot_id: str, commands: List[Dict], robot_position_cache):
    print(f"[🤖 Start executing for {robot_id}]")
    for cmd in commands:
        execute_action(cmd, robot_position_cache)
    print(f"[🤖 Finished executing for {robot_id}]")


# 示例用法（测试）
if __name__ == "__main__":
    # 你的 JSON 中的某个 response
    # example1 = {"turtlebot1": [{"action": "move", "direction": "forward", "value": 2, "unit": "meters"}, {"action": "turn", "direction": "left", "target": "self", "value": 45, "unit": "degrees"}], "turtlebot2": [{"action": "move", "direction": "backward", "value": 1, "unit": "meters"}, {"action": "turn", "direction": "right", "target": "self", "value": 90, "unit": "degrees"}]}
    example2 =  {'robot1': [{'action': 'navigate', 'position': {'x':100,'y':200}}]}
    # example3 =  {'turtlebot1':[{"action": "imitate","target": "turtlebot2"}]}
    controller(example2)
