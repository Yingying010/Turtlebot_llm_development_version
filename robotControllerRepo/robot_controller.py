import os, sys
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)

import rclpy
from rclpy.node import Node
import time
from loguru import logger
from robotControllerRepo.actions.move import move
from robotControllerRepo.actions.rotate import rotate
from robotControllerRepo.actions.imitate import imitate_robot
from robotControllerRepo.actions.navigate import navigate_to_target
from robotControllerRepo.actions.follow import follow_run
from robotControllerRepo.actions.face import face_target
import time
from typing import Dict, List
from ttsRepo.stream_tts import tts_manager

# def move_linear(robot_id, direction, value, unit):
#     print(f"🚗 {robot_id} moving {direction} for {value} {unit}")

# def rotate(robot_id, direction, value, unit, target="self"):
#     print(f"🔁 {robot_id} turning {direction} {value}{unit} around {target}")

# def navigate_to_target(robot_id, target):
#     print(f"🧭 {robot_id} navigating to {target}")

# def follow_target(robot_id, target, robot_position_cache):
#     print(f"👣 {robot_id} following {target}")

# def face_to_target(robot_id, target, robot_position_cache):
#     print(f"🧍 {robot_id} facing {target}")

# def imitate_robot(robot_id, target_robot_id):
#     print(f"🎭 {robot_id} imitating {target_robot_id}")





def execute_action(node, task: Dict):
    print(node)
    robot = task["robot"]
    action = task["action"]
    params = task["parameters"]

    print(f"\n🚗 {robot} is executing {action} → {params}")
    tts_manager.say(f"{robot} is executing {action}")

    if action == "move":
        move(node, robot, params["direction"], params["value"], params["unit"])

    elif action == "turn":
        rotate(node, robot, params["direction"], params["value"], params["unit"], params.get("target", "self"))

    elif action == "navigate":
        if "position" in params:
            navigate_to_target(node, robot, params["position"])
            # print(f"  → Navigating to {params.get('position')}")
        elif "target" in params:
            navigate_to_target(node, robot, params["target"])
            # print(f"  → Navigating to {params.get('target')}")
        else:
            print(f"Missing 'position' or 'target' in navigate params: {params}")

    elif action == "follow":
        if "target" in params:
            target = params["target"]
            follow_run(robot, target)
        else:
            logger.warning(f"Missing 'target' in follow params: {params}")

    elif action == "imitate":
        imitate_robot(node, robot, params["target"])

    elif action == "face_to":
        if "target" in params:
            target = params["target"]
            face_target(node, robot, target)
        else:
            logger.warning(f"Missing 'target' in follow params: {params}")
        

    elif action == "wait":
        print(f"  → Waiting for {params['duration_sec']} seconds")
        time.sleep(params["duration_sec"])
    elif action == "collect":
        print(f"  → Collecting {params['item']}")
        time.sleep(1)
    elif action == "deliver":
        print(f"  → Delivering {params['item']}")
        time.sleep(1)
    else:
        print(f"⚠️ Unknown action: {action}")

    print(f"✅ {robot} completed task {task['task_id']}")



def execute_robot_commands(node:Node, robot_id: str, commands: List[Dict], robot_position_cache):
    print(f"[🤖 Start executing for {robot_id}]")
    for cmd in commands:
        execute_action(node, cmd, robot_position_cache)
    print(f"[🤖 Finished executing for {robot_id}]")


# 示例用法（测试）
if __name__ == "__main__":
    # 你的 JSON 中的某个 response
    # example1 = {"turtlebot1": [{"action": "move", "direction": "forward", "value": 2, "unit": "meters"}, {"action": "turn", "direction": "left", "target": "self", "value": 45, "unit": "degrees"}], "turtlebot2": [{"action": "move", "direction": "backward", "value": 1, "unit": "meters"}, {"action": "turn", "direction": "right", "target": "self", "value": 90, "unit": "degrees"}]}
    # example2 =  {"robots":{"robot1": [
    #     {
    #     "task_id": "t0",
    #     "action": "navigate",
    #     "parameters": {
    #       "position": {
    #         "x": 100,
    #         "y": 200,
    #         "heading_deg": 0
    #       }
    #     },
    #     "sync_group": None,
    #     "sequence": 0
    #   },
    # ]}}
    # example3 =  {'turtlebot1':[{"action": "imitate","target": "turtlebot2"}]}
    robot_id = "robot1"
    commands = {'robots': {'robot1': [{'task_id': 't0', 'action': 'move', 'parameters': {'direction': 'forward', 'value': 2, 'unit': 'meter'}, 'sync_group': None, 'sequence': 1}, {'task_id': 't1', 'action': 'turn', 'parameters': {'direction': 'left', 'value': 90, 'unit': 'degree'}, 'sync_group': None, 'sequence': 2}]}}
