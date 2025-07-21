import rclpy
import time
from loguru import logger
from geometry_msgs.msg import Twist
from actions.move import move_linear
from actions.rotate import rotate
from actions.imitate import imitate_robot, stop_imitation, imitation_threads
import threading
import time
from typing import Dict, List, Union

# def move_linear(robot_id, direction, value, unit):
#     print(f"🚗 {robot_id} moving {direction} for {value} {unit}")

# def rotate(robot_id, direction, value, unit, target="self"):
#     print(f"🔁 {robot_id} turning {direction} {value}{unit} around {target}")

def navigate_to_target(robot_id, target):
    print(f"🧭 {robot_id} navigating to {target}")

def follow_target(robot_id, target):
    print(f"👣 {robot_id} following {target}")

def face_to_target(robot_id, target):
    print(f"🧍 {robot_id} facing {target}")

# def imitate_robot(robot_id, target_robot_id):
#     print(f"🎭 {robot_id} imitating {target_robot_id}")





def execute_action(command: Dict):
    robot_id = command["robot_id"]
    action   = command.get("action")

    if action == "move":
        move_linear(robot_id, command["direction"], command["value"], command["unit"])

    elif action == "turn":
        rotate(robot_id, command["direction"], command["value"], command["unit"], command.get("target", "self"))

    elif action == "navigate_to":
        navigate_to_target(robot_id, command["target"])

    elif action == "follow":
        follow_target(robot_id, command["target"])

    elif action == "imitate":
        imitate_robot(robot_id, command["target"])

    elif action == "face_to":
        face_to_target(robot_id, command["target"])

    else:
        print(f"⚠️ Unknown action: {action}")


def flatten_response(response: dict):
    commands = []
    for robot_id, cmd_list in response.items():
        for cmd in cmd_list:
            cmd["robot_id"] = robot_id
            commands.append(cmd)
    return commands


def control_from_json_response(response: Dict[str, List[Dict]]):
    for robot_id, commands in response.items():
        print(f"\n[🤖 Executing for {robot_id}]")
        for command in commands:
            execute_action(robot_id, command)

def run(command: Dict[str, List[Dict]]):
    rclpy.init()  # ✅ 主线程先初始化 ROS
    # 每个机器人一个线程，内部动作顺序执行
    threads = []

    for robot_id, actions in command.items():
        # 给每个动作添加 robot_id 字段
        for action in actions:
            action["robot_id"] = robot_id

        # 单线程顺序执行当前机器人的所有动作
        t = threading.Thread(target=execute_robot_commands, args=(robot_id, actions))
        t.start()
        threads.append(t)

    for t in threads:
        t.join()

    # ✅ 如果当前没有正在 imitation 的机器人，再 shutdown
    if len(imitation_threads) == 0:
        rclpy.shutdown()
    else:
        print("⏳ ROS still running: imitation is active.")


def execute_robot_commands(robot_id: str, commands: List[Dict]):
    print(f"[🤖 Start executing for {robot_id}]")
    for cmd in commands:
        execute_action(cmd)
    print(f"[🤖 Finished executing for {robot_id}]")


# 示例用法（测试）
if __name__ == "__main__":
    # 你的 JSON 中的某个 response
    # example1 = {"turtlebot1": [{"action": "move", "direction": "forward", "value": 2, "unit": "meters"}, {"action": "turn", "direction": "left", "target": "self", "value": 45, "unit": "degrees"}], "turtlebot2": [{"action": "move", "direction": "backward", "value": 1, "unit": "meters"}, {"action": "turn", "direction": "right", "target": "self", "value": 90, "unit": "degrees"}]}
    example2 =  {'robot1': [{'action': 'move', 'direction': 'forward', 'value': 3, 'unit': 'seconds'}]}
    # example3 =  {'turtlebot1':[{"action": "imitate","target": "turtlebot2"}]}
    run(example2)
