#!/usr/bin/env python3

# collect.py —— 独立导航 + 语音反馈 + 模拟收集
 
import os, sys, time, math
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy._rclpy_pybind11 import InvalidHandle
from typing import Dict, List, Tuple, Optional
from geometry_msgs.msg import Twist
import threading
# 添加项目根路径
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import config
from ttsRepo.stream_tts import tts_manager
from phasespace.rigid_tracker import RigidTracker
from rclpy.executors import MultiThreadedExecutor
from robotControllerRepo.actions.navigate import navigate_to_target

semantic_locations = config.get("semantic_locations")

# === 执行 collect 行为 ===
def collect_item(node: Node, robot_name: str, item: str, target, executor):
    # === 阶段 1：导航到目标位置 ===
    is_successful = navigate_to_target(node, executor, robot_name, target)
    print(f"🎯 Reached target location, collecting {item}...")
 
    # === 阶段 2：收集 ===
    print(f"🗣️ Speaking: I am collecting {item}")
    tts_manager.say(f"I am collecting {item}")
    time.sleep(10)  # 模拟收集时间
    tts_manager.say(f"{item} collected successfully")

    return is_successful



 