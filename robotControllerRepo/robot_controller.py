# 在文件顶部添加导入
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
from robotControllerRepo.actions.find import execute_find_with_llm_replanning, create_llm_replanning_function
from robotControllerRepo.actions.rotate import rotate_deg
from robotControllerRepo.actions.navigate import navigate_to_target, navigate_to
# 🔥 新增：导入协作运输模块
from robotControllerRepo.actions.contactlessTransport import TransportManager
# 🔥 修正：导入独立的YOLO感知模块
from llmParserRepo.yolo_perception import detect_once
from typing import Dict, List
from ttsRepo.stream_tts import tts_manager

# 全局存储运输管理器实例，避免重复创建
_transport_managers = {}

def normalize_task_parameters(task: Dict) -> Dict:
    """
    标准化LLM生成的任务参数格式，确保与robot_controller兼容
    """
    action = task.get("action", "")
    params = task.get("parameters", {}).copy()
    
    # 🔥 新增：contactless_transport动作参数标准化
    if action == "contactless_transport":
        # 确保必要的参数存在
        if "item" not in params:
            params["item"] = "unknown_item"
        if "start_position" not in params:
            params["start_position"] = "table"  # 默认值
        if "goal_position" not in params:
            params["goal_position"] = "lucy"   # 默认值
    
    # 📧 collect动作参数标准化 - 只保留item参数
    elif action == "collect":
        # LLM可能生成的格式 → 标准格式
        if "object_id" in params and "item" not in params:
            params["item"] = params.pop("object_id")
        if "object_class" in params and "item" not in params:
            params["item"] = params.pop("object_class")
        if "object" in params and "item" not in params:
            params["item"] = params.pop("object")
            
        # 移除target参数（新版collect不需要）
        if "target" in params:
            logger.debug(f"[NORMALIZE] Removing target from collect params: {params.pop('target')}")
    
    # 📧 deliver动作参数标准化 - 只保留item参数  
    elif action == "deliver":
        # LLM可能生成的格式 → 标准格式
        if "object_id" in params and "item" not in params:
            params["item"] = params.pop("object_id")
        if "object_class" in params and "item" not in params:
            params["item"] = params.pop("object_class")
        if "object" in params and "item" not in params:
            params["item"] = params.pop("object")
            
        # 移除target参数（新版deliver不需要）
        if "target" in params:
            logger.debug(f"[NORMALIZE] Removing target from deliver params: {params.pop('target')}")
        if "recipient" in params:
            logger.debug(f"[NORMALIZE] Removing recipient from deliver params: {params.pop('recipient')}")
        if "to" in params:
            logger.debug(f"[NORMALIZE] Removing to from deliver params: {params.pop('to')}")
        if "destination" in params:
            logger.debug(f"[NORMALIZE] Removing destination from deliver params: {params.pop('destination')}")
    
    # 📧 navigate动作参数标准化
    elif action == "navigate":
        if "destination" in params and "target" not in params:
            params["target"] = params.pop("destination")
        if "location" in params and "target" not in params:
            params["target"] = params.pop("location")
    
    # 📧 face动作参数标准化
    elif action == "face":
        if "object" in params and "target" not in params:
            params["target"] = params.pop("object")
        if "direction" in params and "target" not in params:
            params["target"] = params.pop("direction")
    
    # 📧 wait动作参数标准化
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
        # 🔥 新版collect：只需要item参数
        item = params["item"]
        is_successful = collect_item(node, robot, item, executor)
            
    elif action == "deliver":
        # 🔥 新版deliver：只需要item参数
        item = params["item"]
        is_successful = deliver_item(node, robot, item, executor)
    
    elif action == "wait":
        print(f"  → Waiting for {params['duration_sec']} seconds")
        time.sleep(params["duration_sec"])
        is_successful = True

    # 🔥 新增：协作运输动作处理
    elif action == "contactless_transport":
        is_successful = execute_contactless_transport(node, executor, robot, params)

    elif action == "find":
        # 🔥 增强版find处理：支持LLM智能重规划和后续任务执行
        is_successful, follow_up_tasks = execute_find_with_followup(node, executor, task)
        
        # 🎯 执行LLM生成的后续任务
        if is_successful and follow_up_tasks:
            logger.info(f"🔄 Executing {len(follow_up_tasks)} follow-up tasks generated by LLM...")
            tts_manager.say(f"Now executing follow-up tasks.")
            
            for i, follow_task in enumerate(follow_up_tasks, 1):
                logger.info(f"📋 Follow-up task {i}/{len(follow_up_tasks)}: {follow_task['action']}")
                
                # 📧 标准化任务参数格式
                normalized_task = normalize_task_parameters(follow_task)
                
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
        # 🔥 新版collect：只需要item参数
        item = params["item"]
        return collect_item(node, robot, item, executor)
            
    elif action == "deliver":
        # 🔥 新版deliver：只需要item参数  
        item = params["item"]
        return deliver_item(node, robot, item, executor)
    
    elif action == "wait":
        duration = params.get("duration_sec", 1.0)
        logger.info(f"[SINGLE] Waiting for {duration} seconds")
        time.sleep(duration)
        return True

    # 🔥 新增：协作运输动作处理（单一版本）
    elif action == "contactless_transport":
        return execute_contactless_transport(node, executor, robot, params)

    else:
        logger.error(f"[SINGLE] Unknown action: {action}")
        return False


# 🔥 新增：协作运输执行函数
def execute_contactless_transport(node, executor, robot_name: str, params: Dict) -> bool:
    """
    执行协作运输任务
    
    Args:
        node: ROS节点
        executor: ROS执行器
        robot_name: 机器人名称 (robot1 或 robot2)
        params: 参数字典，包含 item, start_position, goal_position
    
    Returns:
        bool: 执行是否成功
    """
    global _transport_managers
    
    try:
        item = params.get("item", "unknown_item")
        start_pos = params.get("start_position", "table")
        goal_pos = params.get("goal_position", "lucy")
        
        logger.info(f"🚛 {robot_name}: Starting contactless transport")
        logger.info(f"   📦 Item: {item}")
        logger.info(f"   🏁 From: {start_pos} → To: {goal_pos}")
        
        tts_manager.say(f"Starting collaborative transport of {item}")
        
        # 检查是否已有运输管理器实例
        transport_key = f"{robot_name}_transport"
        
        if transport_key not in _transport_managers:
            # 创建新的运输管理器实例
            logger.info(f"🔧 Creating new transport manager for {robot_name}")
            transport_manager = TransportManager(node, executor, robot_name)
            _transport_managers[transport_key] = transport_manager
            
            # 等待运输完成（检查工作线程状态）
            # TransportManager的工作线程会自动处理整个流程
            logger.info(f"⏳ {robot_name}: Waiting for transport completion...")
            
            # 等待工作线程完成（最多等待5分钟）
            transport_manager.worker.join(timeout=300)  
            
            if transport_manager.worker.is_alive():
                logger.error(f"❌ {robot_name}: Transport timeout after 5 minutes")
                tts_manager.say("Transport operation timed out")
                return False
            
            # 清理管理器实例
            del _transport_managers[transport_key]
            
            if transport_manager.aborted:
                logger.error(f"❌ {robot_name}: Transport was aborted")
                tts_manager.say("Transport operation was aborted")
                return False
            
        else:
            logger.warning(f"⚠️ Transport manager already exists for {robot_name}")
        
        logger.info(f"✅ {robot_name}: Contactless transport completed successfully")
        tts_manager.say(f"Collaborative transport of {item} completed")
        return True
        
    except Exception as e:
        logger.error(f"❌ {robot_name}: Contactless transport failed: {e}")
        import traceback
        traceback.print_exc()
        tts_manager.say("Transport operation failed due to an error")
        
        # 清理失败的管理器实例
        transport_key = f"{robot_name}_transport"
        if transport_key in _transport_managers:
            del _transport_managers[transport_key]
        
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
        
        # 🔥 修正：使用正确的导入路径获取历史存储
        from llmParserRepo.gpt_localParser import HistoryStore, MEMORY_PATH
        history_store = HistoryStore(MEMORY_PATH)
        
        # 🔥 修正：直接调用 execute_find_with_llm_replanning 并传递正确参数
        result = execute_find_with_llm_replanning(
            node=node,                    # 传递 node
            robot_name=robot,             # 传递 robot_name  
            params=params,                # 传递 params
            # 传递上下文函数作为关键字参数
            detect_fn=detect_once,        # 🔥 修正：使用来自 yolo_perception 的 detect_once
            rotate_fn=lambda robot_name, deg: rotate_deg(node, robot_name, deg),
            navigate_to_fn=lambda robot_name, target: navigate_to(node, executor, robot_name, target),
            event_pub=None,  # 可以传递事件发布器，如果有的话
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
        import traceback
        traceback.print_exc()
        return False, []


def execute_robot_commands(node:Node, robot_id: str, commands: List[Dict], robot_position_cache):
    """原有的批量命令执行函数（保持向后兼容）"""
    print(f"[🤖 Start executing for {robot_id}]")
    for cmd in commands:
        execute_action(node, cmd, robot_position_cache)
    print(f"[🤖 Finished executing for {robot_id}]")