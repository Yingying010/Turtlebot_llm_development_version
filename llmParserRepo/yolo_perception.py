import os
import time
import json
import subprocess
from pathlib import Path
from typing import Optional, Callable, Any, Dict, List
from loguru import logger

CAMERA_FRAME_PATH = "/tmp/yolo_parser_frame.jpg"
YOLO_OUTPUT_PATH = "/tmp/yolo_parser_result.json"


# ===========================
# 摄像头 + YOLO 处理模块
# ===========================

def load_camera():
    """使用 rpicam-still 抓一帧图片"""
    try:
        subprocess.run(
            [
                "rpicam-still",
                "-t", "1000",
                "--width", "1640",
                "--height", "1232",
                "-o", CAMERA_FRAME_PATH,
            ],
            check=True,
        )
        logger.info(f"📸 Saved camera frame to {CAMERA_FRAME_PATH}")
        return CAMERA_FRAME_PATH
    except subprocess.CalledProcessError as e:
        logger.warning(f"⚠️ Failed to capture frame from camera: {e}")
        return None


def run_yolo(image_path: str):
    """
    使用命令行方式运行YOLO模型
    假设 ultralytics 或 yolov8 已经配置好
    """
    try:
        cmd = [
            "yolo", "task=detect",
            f"mode=predict",
            f"model=yolov8n.pt",
            f"conf=0.5",
            f"source={image_path}",
            f"save=false",
            f"save_txt=false",
            f"show=false",
            f"project=/tmp",
            f"name=yolo_parser_result",
            f"exist_ok=true",
        ]
        subprocess.run(cmd, check=True)
        result_json = os.path.join("/tmp", "yolo_parser_result", "predict", "predict.json")
        if os.path.exists(result_json):
            logger.info(f"📦 YOLO result saved to {result_json}")
            return result_json
        else:
            logger.warning("⚠️ YOLO result file not found")
            return None
    except subprocess.CalledProcessError as e:
        logger.warning(f"⚠️ YOLO command failed: {e}")
        return None


class YOLOPerceiver:
    """
    负责执行相机+YOLO感知流程，并返回结构化感知结果
    """

    def __init__(self, camera_fn: Callable[[], Optional[str]] = load_camera,
                 yolo_fn: Callable[[str], Optional[str]] = run_yolo):
        self.camera_fn = camera_fn
        self.yolo_fn = yolo_fn

    def perceive(self) -> Optional[Dict[str, Any]]:
        frame_path = self.camera_fn()
        if frame_path is None:
            return None

        result_path = self.yolo_fn(frame_path)
        if result_path is None:
            return None

        try:
            with open(result_path, "r") as f:
                data = json.load(f)
            logger.info("✅ Loaded YOLO detection result")
            return {
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                "frame_path": frame_path,
                "result": data,
            }
        except Exception as e:
            logger.warning(f"⚠️ Failed to load YOLO result JSON: {e}")
            return None


# ===========================
# 对话中调用感知模块
# ===========================

def detect_once(camera=None, yolo=None) -> Optional[Dict[str, Any]]:
    """
    简单调用一次相机+YOLO感知，返回数据结构
    """
    perceiver = YOLOPerceiver(camera_fn=camera or load_camera,
                              yolo_fn=yolo or run_yolo)
    return perceiver.perceive()


def detect_and_store(history_store, detect_fn=detect_once):
    """
    运行感知并保存感知结果到历史
    """
    result = detect_fn()
    if result:
        history_store.add_perception(result)
        logger.info("🧠 Perception result added to history.")
    else:
        logger.warning("⚠️ Perception failed, nothing added.")
    return result


def perceive_and_parse(user_input: str,
                       history_store,
                       detect_fn=detect_once,
                       prompt_builder: Callable[[str, Any], str] = None,
                       parse_fn: Callable[[str], Any] = None) -> Optional[Any]:
    """
    带感知的智能解析函数：
    - 执行一次感知
    - 构造prompt（当前感知 + 历史）
    - 调用LLM解析
    """
    perception = detect_and_store(history_store, detect_fn=detect_fn)
    if not perception:
        logger.warning("⚠️ Camera or YOLO unavailable, skipping perception.")
        return None

    if not prompt_builder or not parse_fn:
        raise ValueError("prompt_builder and parse_fn must be provided")

    prompt = prompt_builder(user_input, history_store)
    logger.debug(f"📨 Prompt built:\n{prompt}")
    response = parse_fn(prompt)
    return response