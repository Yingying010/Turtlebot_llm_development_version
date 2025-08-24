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
from robotControllerRepo.actions.navigate import navigate_to_target, navigate_to
from robotControllerRepo.actions.follow import follow_run
from robotControllerRepo.actions.face import face_run
from robotControllerRepo.actions.pickup import pickup_item
from robotControllerRepo.actions.dropoff import dropoff_item
from robotControllerRepo.actions.find import execute_find_with_llm_replanning, create_llm_replanning_function
from robotControllerRepo.actions.contactlessTransport import TransportManager
from llmParserRepo.yolo_perception import detect_once

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
            return move(node, robot, params["direction"], params["value"], params["unit"])

        elif action == "turn":
            return rotate(node, robot, params["direction"], params["value"], params["unit"], params.get("target", "self"))

        elif action == "navigate":
            if "position" in params:
                return navigate_to_target(node, executor, robot, params["position"])
            elif "target" in params:
                return navigate_to_target(node, executor, robot, params["target"])
            else:
                logger.warning("Missing 'position' or 'target' in navigate params")
                return False

        elif action == "follow":
            return follow_run(node, robot, params["target"], executor)

        elif action == "imitate":
            return imitate_robot(node, robot, params["target"])

        elif action == "face":
            return face_run(node, robot, params["target"], executor)

        elif action == "pickup":
            return pickup_item(node, robot, params["item"], executor)

        elif action == "dropoff":
            return dropoff_item(params["item"])

        elif action == "wait":
            duration = params.get("duration_sec", 1.0)
            logger.info(f"Waiting for {duration} seconds")
            time.sleep(duration)
            return True

        elif action == "contactless_transport":
            return execute_contactless_transport(node, executor, robot, params)

        elif action == "find":
            # === 特殊处理：支持 follow_up_tasks ===
            llm_replanning_fn = create_llm_replanning_function()
            from llmParserRepo.gpt_localParser import HistoryStore, MEMORY_PATH
            history_store = HistoryStore(MEMORY_PATH)

            result = execute_find_with_llm_replanning(
                node=node,
                robot_name=robot,
                params=params,
                detect_fn=detect_once,
                rotate_fn=lambda r, deg: rotate_deg(node, r, deg),
                navigate_to_fn=lambda r, t: navigate_to(node, executor, r, t),
                event_pub=None,
                llm_replanning_fn=llm_replanning_fn,
                history_store=history_store
            )

            if not result.get("ok", False):
                logger.warning("Find failed: " + result.get("reason", "unknown"))
                return False

            follow_ups = result.get("follow_up_tasks", [])
            if handle_follow_up and follow_ups:
                logger.info(f"🔄 Executing {len(follow_ups)} follow-up tasks")
                tts_manager.say("Now executing follow-up tasks")
                for i, sub_task in enumerate(follow_ups, 1):
                    full_task = {
                        "robot": robot,
                        "action": sub_task["action"],
                        "parameters": sub_task["parameters"]
                    }
                    logger.info(f"📋 Follow-up task {i}/{len(follow_ups)}: {full_task}")
                    success = execute_action(node, executor, full_task, handle_follow_up=False)
                    if not success:
                        logger.warning(f"⚠️ Follow-up task {i} failed")
                tts_manager.say("All follow-up tasks completed")

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
