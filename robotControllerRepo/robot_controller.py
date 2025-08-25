# robot_controller.py
import os, sys
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)

import rclpy
from rclpy.node import Node
import time
from typing import Dict, List
from loguru import logger
from ttsRepo.stream_tts import tts_manager

# === Action Modules ===
from robotControllerRepo.actions.move import move
from robotControllerRepo.actions.rotate import rotate, rotate_deg
from robotControllerRepo.actions.imitate import imitate_robot
from robotControllerRepo.actions.navigate import navigate_to_target
from robotControllerRepo.actions.follow import follow_run
from robotControllerRepo.actions.face import face_run
from robotControllerRepo.actions.pickup import pickup_item
from robotControllerRepo.actions.dropoff import dropoff_item
from robotControllerRepo.actions.find import run_find
from robotControllerRepo.actions.contactlessTransport import TransportManager

# === Global ===
_transport_managers = {}

# === Unified executor ===
def execute_action(node: Node, executor, task: Dict, handle_follow_up: bool = True) -> bool:
    robot = task["robot"]
    action = task["action"]
    params = task["parameters"]

    logger.info(f"🚗 {robot} is executing {action} → {params}")
    tts_manager.say(f"{robot} is executing {action}")

    try:
        if action == "move":
            result = move(node, robot, params["direction"], params["value"], params["unit"])
            return result
        
        elif action == "turn":
            result = rotate(node, robot, params["direction"], params["value"], params["unit"], params.get("target", "self"))
            return result
        
        elif action == "navigate":
            if "position" in params:
                result = navigate_to_target(node, executor, robot, params["position"])
                return result
            elif "target" in params:
                result = navigate_to_target(node, executor, robot, params["target"])
                return result
            else:
                logger.warning("Missing 'position' or 'target' in navigate params")
                return False

        elif action == "follow":
            result = follow_run(node, robot, params["target"], executor)
            return result

        elif action == "imitate":
            result = imitate_robot(node, robot, params["target"])
            return result

        elif action == "face":
            result = face_run(node, robot, params["target"], executor)
            return result

        elif action == "pickup":
            result = pickup_item(params["item"])
            return result

        elif action == "dropoff":
            result = dropoff_item(params["item"])
            return result
        
        elif action == "contactless_transport":
            result = execute_contactless_transport(node, executor, robot, params)
            return result

        elif action == "find":
            result = run_find(node, robot, params["item"], executor)
            return result

        elif action == "wait":
            duration = params.get("duration_sec", 1.0)
            logger.info(f"Waiting for {duration} seconds")
            time.sleep(duration)
            return True

        else:
            logger.error(f"❌ Unknown action: {action}")
            return False

    except Exception as e:
        logger.error(f"❌ Error executing {action}: {e}")
        import traceback
        traceback.print_exc()
        return False

# === 协作运输 ===
def execute_contactless_transport(node, executor, robot_name: str, params: Dict) -> bool:
    global _transport_managers
    try:
        item = params.get("item", "unknown")
        start = params.get("start_position", "table")
        goal = params.get("goal_position", "lucy")

        logger.info(f"🚛 {robot_name} transporting {item} from {start} to {goal}")
        tts_manager.say(f"{robot_name} transporting {item}")

        key = f"{robot_name}_transport"
        if key not in _transport_managers:
            mgr = TransportManager(node, executor, robot_name)
            _transport_managers[key] = mgr
            mgr.worker.join(timeout=300)
            del _transport_managers[key]
            if mgr.aborted:
                logger.error("Transport aborted")
                return False
        else:
            logger.warning("Transport already in progress")

        logger.info("✅ Transport complete")
        return True

    except Exception as e:
        logger.error(f"Transport error: {e}")
        return False

# === 批量任务执行 ===
def execute_robot_commands(node: Node, robot_id: str, commands: List[Dict], executor):
    logger.info(f"[🤖 Start executing for {robot_id}]")
    for cmd in commands:
        execute_action(node, executor, cmd)
    logger.info(f"[🤖 Finished executing for {robot_id}]")
