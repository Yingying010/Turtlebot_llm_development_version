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


def run_yolo_python(image_path: str):
    """
    使用Python API运行YOLO模型，直接返回检测结果
    """
    try:
        from ultralytics import YOLO
        
        # 加载模型
        model = YOLO("yolov8n.pt")
        
        # 执行检测
        results = model.predict(
            source=image_path,
            conf=0.5,
            verbose=False,
            save=False
        )
        
        if not results:
            logger.warning("⚠️ No YOLO results returned")
            return []
            
        result = results[0]
        detections = []
        
        # 处理检测结果
        if hasattr(result, 'boxes') and result.boxes is not None:
            names = result.names  # 类别名称映射
            
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                
                # 计算中心点
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                
                # 获取类别名称
                cls_name = names.get(cls_id, str(cls_id)).lower()
                
                detection = {
                    "class": cls_name,
                    "conf": round(conf, 4),
                    "center_xy": [cx, cy],
                    "bbox_xyxy": [x1, y1, x2, y2]
                }
                detections.append(detection)
        
        logger.info(f"📦 YOLO Python detection: {len(detections)} objects detected")
        return detections
        
    except ImportError:
        logger.warning("⚠️ ultralytics not available, falling back to command line")
        return run_yolo_command(image_path)
    except Exception as e:
        logger.warning(f"⚠️ YOLO Python detection failed: {e}")
        return run_yolo_command(image_path)


def run_yolo_command(image_path: str):
    """
    使用命令行方式运行YOLO模型 - 备用方案
    """
    try:
        # 创建临时目录
        import tempfile
        temp_dir = tempfile.mkdtemp()
        
        cmd = [
            "yolo", "task=detect",
            "mode=predict",
            "model=yolov8n.pt",
            "conf=0.5",
            f"source={image_path}",
            "save=false",
            "save_txt=false", 
            "show=false",
            f"project={temp_dir}",
            "name=yolo_result",
            "exist_ok=true",
        ]
        
        # 运行YOLO命令
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        
        # 由于save=false，我们需要从命令输出中解析结果
        output_lines = result.stdout.split('\n')
        detections = []
        
        # 查找包含检测结果的行
        for line in output_lines:
            if 'image' in line and ':' in line:
                # 解析类似 "image 1/1 /path: 512x640 1 bottle, 3 chairs, 1381.9ms" 的行
                if ' ' in line and any(word in line for word in ['bottle', 'chair', 'person', 'cup']):
                    parts = line.split()
                    # 简单解析 - 这是一个基本实现
                    for i, part in enumerate(parts):
                        if part.isdigit() and i+1 < len(parts):
                            count = int(part)
                            class_name = parts[i+1].rstrip(',')
                            # 为每个检测到的物体创建一个基本条目
                            for j in range(count):
                                detection = {
                                    "class": class_name.lower(),
                                    "conf": 0.5,  # 默认置信度
                                    "center_xy": [320, 240],  # 默认中心点
                                    "bbox_xyxy": [100, 100, 540, 380]  # 默认边界框
                                }
                                detections.append(detection)
        
        logger.info(f"📦 YOLO command parsed: {len(detections)} objects")
        return detections
        
    except Exception as e:
        logger.warning(f"⚠️ YOLO command failed: {e}")
        return []


def run_yolo(image_path: str):
    """
    运行YOLO检测 - 优先使用Python API
    """
    # 优先使用Python API
    detections = run_yolo_python(image_path)
    if detections:
        return detections
        
    # 备用：命令行解析
    return run_yolo_command(image_path)


class YOLOPerceiver:
    """
    负责执行相机+YOLO感知流程，并返回结构化感知结果
    """

    def __init__(self, camera_fn: Callable[[], Optional[str]] = load_camera,
                 yolo_fn: Callable[[str], Any] = run_yolo):
        self.camera_fn = camera_fn
        self.yolo_fn = yolo_fn

    def perceive(self) -> Optional[Dict[str, Any]]:
        frame_path = self.camera_fn()
        if frame_path is None:
            return None

        # 🔥 修复：直接获取检测结果列表
        detections = self.yolo_fn(frame_path)
        if not detections:
            logger.warning("⚠️ No detections from YOLO")
            detections = []

        # 🔥 修复：构造标准格式的返回结果
        return {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "frame_path": frame_path,
            "result": {
                "detections": detections,
                "image": {"width": 1640, "height": 1232}
            }
        }


# ===========================
# 对话中调用感知模块
# ===========================

def detect_once(camera=None, yolo=None) -> Optional[Dict[str, Any]]:
    """
    简单调用一次相机+YOLO感知，返回数据结构
    🔥 修复：确保返回正确的数据格式
    """
    perceiver = YOLOPerceiver(camera_fn=camera or load_camera,
                              yolo_fn=yolo or run_yolo)
    result = perceiver.perceive()
    
    if result:
        logger.info(f"✅ detect_once successful: {len(result['result']['detections'])} detections")
    else:
        logger.warning("⚠️ detect_once failed")
        
    return result


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