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
from robotControllerRepo.actions.face import face_run
from robotControllerRepo.actions.collect import collect_item
from robotControllerRepo.actions.deliver import deliver_item
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





def execute_action(node, executor, task: Dict):
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
            is_successful = navigate_to_target(node, executor, robot, params["position"])
            # print(f"  → Navigating to {params.get('position')}")
        elif "target" in params:
            is_successful = navigate_to_target(node, executor, robot, params["target"])
            # print(f"  → Navigating to {params.get('target')}")
        else:
            print(f"Missing 'position' or 'target' in navigate params: {params}")

    elif action == "follow":
        if "target" in params:
            target = params["target"]
            is_successful = follow_run(node, robot, target, executor)
        else:
            logger.warning(f"Missing 'target' in follow params: {params}")

    elif action == "imitate":
        imitate_robot(node, robot, params["target"])

    elif action == "face":
        if "target" in params:
            target = params["target"]
            is_successful = face_run(node, robot, target, executor)
        else:
            logger.warning(f"Missing 'target' in follow params: {params}")

    elif action == "collect":
        item = params["item"]
        if "position" in params:
            is_successful = collect_item(node, robot, item, params["position"], executor)
        elif "target" in params:
            is_successful = collect_item(node, robot, item, params["target"], executor)
        else:
            print(f"Missing 'position' or 'target' in navigate params: {params}")
    elif action == "deliver":
        item = params["item"]
        if "position" in params:
            is_successful = deliver_item(node, robot, item, params["position"], executor)
        elif "target" in params:
            is_successful = deliver_item(node, robot, item, params["target"], executor)
        else:
            print(f"Missing 'position' or 'target' in navigate params: {params}")
    
    elif action == "wait":
        print(f"  → Waiting for {params['duration_sec']} seconds")
        time.sleep(params["duration_sec"])


    else:
        print(f"⚠️ Unknown action: {action}")

    if is_successful == True:
        print(f"✅ {robot} completed task {task['task_id']}")

    return is_successful




def execute_robot_commands(node:Node, robot_id: str, commands: List[Dict], robot_position_cache):
    print(f"[🤖 Start executing for {robot_id}]")
    for cmd in commands:
        execute_action(node, cmd, robot_position_cache)
    print(f"[🤖 Finished executing for {robot_id}]")
