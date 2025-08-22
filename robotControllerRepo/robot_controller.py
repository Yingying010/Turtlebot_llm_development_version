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
from robotControllerRepo.actions.find import run_action as run_find 
from robotControllerRepo.actions.find import create_llm_replanning_function
from robotControllerRepo.actions.rotate import rotate_deg
from robotControllerRepo.actions.navigate import navigate_to_target, navigate_to
from llmParserRepo.gpt_yolo_localParser import detect_once, HistoryStore, MEMORY_PATH
import time
from typing import Dict, List
from ttsRepo.stream_tts import tts_manager

def normalize_task_parameters(task: Dict, robot_name: str) -> Dict:
    """
    标准化LLM生成的任务参数格式，确保与robot_controller兼容
    
    Args:
        task: LLM生成的任务 {"action": "...", "parameters": {...}}
        robot_name: 机器人名称
    
    Returns:
        标准化后的任务
    """
    action = task.get("action", "")
    params = task.get("parameters", {}).copy()
    
    # 🔧 collect动作参数标准化
    if action == "collect":
        # LLM可能生成的格式 → 标准格式
        if "object_id" in params and "item" not in params:
            params["item"] = params.pop("object_id")
        if "object_class" in params and "item" not in params:
            params["item"] = params.pop("object_class")
        if "object" in params and "item" not in params:
            params["item"] = params.pop("object")
            
        # 如果没有target，可能需要从blackboard获取
        if "target" not in params:
            item_name = params.get("item", "unknown")
            params["target"] = f"{item_name}_target"  # 假设blackboard key
            logger.warning(f"[NORMALIZE] collect action missing target, using: {params['target']}")
    
    # 🔧 deliver动作参数标准化  
    elif action == "deliver":
        # LLM可能生成的格式 → 标准格式
        if "object_id" in params and "item" not in params:
            params["item"] = params.pop("object_id")
        if "object_class" in params and "item" not in params:
            params["item"] = params.pop("object_class")
        if "object" in params and "item" not in params:
            params["item"] = params.pop("object")
            
        if "recipient" in params and "target" not in params:
            params["target"] = params.pop("recipient")
        if "to" in params and "target" not in params:
            params["target"] = params.pop("to")
        if "destination" in params and "target" not in params:
            params["target"] = params.pop("destination")
            
        # 处理用户引用
        if params.get("target") in ["user", "master", "human"]:
            params["target"] = robot_name.replace("robot", "master")  # robot1 → master1
    
    # 🔧 navigate动作参数标准化
    elif action == "navigate":
        if "destination" in params and "target" not in params:
            params["target"] = params.pop("destination")
        if "location" in params and "target" not in params:
            params["target"] = params.pop("location")
    
    # 🔧 face动作参数标准化
    elif action == "face":
        if "object" in params and "target" not in params:
            params["target"] = params.pop("object")
        if "direction" in params and "target" not in params:
            params["target"] = params.pop("direction")
    
    # 🔧 wait动作参数标准化
    elif action == "wait":
        if "duration" in params and "duration_sec" not in params:
            params["duration_sec"] = params.pop("duration")
        if "time" in params and "duration_sec" not in params:
            params["duration_sec"] = params.pop("time")
    
    logger.debug(f"[NORMALIZE] {action}: {task.get('parameters', {})} → {params}")
    
    return {
        "action": action,
        "parameters": params
    }


def execute_action(node, executor, task: Dict):
    """
    增强版execute_action: 支持动态后续任务执行
    """
    is_successful = False
    robot = task["robot"]
    action = task["action"]
    params = task["parameters"]

    print(f"\n🚗 {robot} is executing {action} → {params}")
    tts_manager.say(f"{robot} is executing {action}")

    if action == "move":
        is_successful = move(node, robot, params["direction"], params["value"], params["unit"])

    elif action == "turn":
        is_successful = rotate(node, robot, params["direction"], params["value"], params["unit"], params.get("target", "self"))

    elif action == "navigate":
        if "position" in params:
            is_successful = navigate_to_target(node, executor, robot, params["position"])
        elif "target" in params:
            is_successful = navigate_to_target(node, executor, robot, params["target"])
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
            print(f"Missing 'position' or 'target' in collect params: {params}")
            
    elif action == "deliver":
        item = params["item"]
        if "position" in params:
            is_successful = deliver_item(node, robot, item, params["position"], executor)
        elif "target" in params:
            is_successful = deliver_item(node, robot, item, params["target"], executor)
        else:
            print(f"Missing 'position' or 'target' in deliver params: {params}")
    
    elif action == "wait":
        print(f"  → Waiting for {params['duration_sec']} seconds")
        time.sleep(params["duration_sec"])
        is_successful = True

    elif action == "find":
        # 🔥 增强版find处理：支持LLM智能重规划和后续任务执行
        is_successful, follow_up_tasks = execute_find_with_followup(node, executor, task)
        
        # 🎯 执行LLM生成的后续任务
        if is_successful and follow_up_tasks:
            logger.info(f"🔄 Executing {len(follow_up_tasks)} follow-up tasks generated by LLM...")
            tts_manager.say(f"Now executing follow-up tasks.")
            
            for i, follow_task in enumerate(follow_up_tasks, 1):
                logger.info(f"📋 Follow-up task {i}/{len(follow_up_tasks)}: {follow_task['action']}")
                
                # 🔧 标准化任务参数格式
                normalized_task = normalize_task_parameters(follow_task, robot)
                
                # 构建完整的任务对象
                full_follow_task = {
                    "robot": robot,
                    "action": normalized_task["action"],
                    "parameters": normalized_task["parameters"]
                }
                
                # 递归执行后续任务（但不处理其follow_up_tasks避免无限递归）
                follow_success = execute_action_single(node, executor, full_follow_task)
                
                if follow_success:
                    logger.info(f"✅ Follow-up task {i} completed successfully")
                else:
                    logger.warning(f"⚠️ Follow-up task {i} failed")
                    # 可以选择继续执行剩余任务或者停止
                    continue
                    
            logger.info(f"🎉 All follow-up tasks completed!")
            tts_manager.say("All tasks completed successfully.")

    else:
        print(f"⚠️ Unknown action: {action}")

    if is_successful == True:
        print(f"✅ {robot} completed task {task['action']}")

    return is_successful


def execute_action_single(node, executor, task: Dict):
    """
    单一任务执行版本（不处理follow_up_tasks，避免递归）
    用于执行find动作生成的后续任务
    """
    robot = task["robot"]
    action = task["action"]
    params = task["parameters"]

    logger.debug(f"[SINGLE] Executing {action} for {robot}: {params}")

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
            logger.error(f"Missing 'position' or 'target' in navigate params: {params}")
            return False

    elif action == "follow":
        if "target" in params:
            return follow_run(node, robot, params["target"], executor)
        else:
            logger.error(f"Missing 'target' in follow params: {params}")
            return False

    elif action == "face":
        if "target" in params:
            return face_run(node, robot, params["target"], executor)
        else:
            logger.error(f"Missing 'target' in face params: {params}")
            return False

    elif action == "collect":
        item = params["item"]
        if "position" in params:
            return collect_item(node, robot, item, params["position"], executor)
        elif "target" in params:
            return collect_item(node, robot, item, params["target"], executor)
        else:
            logger.error(f"Missing 'position' or 'target' in collect params: {params}")
            return False
            
    elif action == "deliver":
        item = params["item"]
        if "position" in params:
            return deliver_item(node, robot, item, params["position"], executor)
        elif "target" in params:
            return deliver_item(node, robot, item, params["target"], executor)
        else:
            logger.error(f"Missing 'position' or 'target' in deliver params: {params}")
            return False
    
    elif action == "wait":
        duration = params.get("duration_sec", 1.0)
        logger.info(f"[SINGLE] Waiting for {duration} seconds")
        time.sleep(duration)
        return True

    else:
        logger.error(f"[SINGLE] Unknown action: {action}")
        return False


def execute_find_with_followup(node, executor, task: Dict) -> tuple:
    """
    执行find动作并返回后续任务
    
    Returns:
        tuple: (is_successful: bool, follow_up_tasks: List[Dict])
    """
    robot = task["robot"]
    params = task["parameters"]
    
    try:
        # 创建LLM重规划函数
        llm_replanning_fn = create_llm_replanning_function()
        
        # 获取历史存储实例
        history_store = HistoryStore(MEMORY_PATH)
        
        # 调用find动作
        result = run_find(
            node,
            task,
            detect_fn=detect_once,
            rotate_fn=lambda robot, deg: rotate_deg(node, robot, deg),
            navigate_to_fn=lambda robot, target: navigate_to(node, executor, robot, target),
            event_pub=None,
            llm_replanning_fn=llm_replanning_fn,
            history_store=history_store
        )
        
        # 检查find是否成功
        if not result.get("ok", False):
            logger.error(f"Find action failed: {result.get('reason', 'unknown')}")
            return False, []
        
        # 提取后续任务
        follow_up_tasks = result.get("follow_up_tasks", [])
        
        # 记录重规划信息
        if result.get("replanning_success", False):
            source = result.get("replanning_source", "unknown")
            reasoning = result.get("replanning_reasoning", "")
            logger.info(f"🧠 LLM Replanning: {source}")
            if reasoning:
                logger.info(f"💭 Reasoning: {reasoning}")
        
        return True, follow_up_tasks
        
    except Exception as e:
        logger.error(f"Find action error: {e}")
        return False, []


def execute_robot_commands(node:Node, robot_id: str, commands: List[Dict], robot_position_cache):
    """原有的批量命令执行函数（保持向后兼容）"""
    print(f"[🤖 Start executing for {robot_id}]")
    for cmd in commands:
        execute_action(node, cmd, robot_position_cache)
    print(f"[🤖 Finished executing for {robot_id}]")