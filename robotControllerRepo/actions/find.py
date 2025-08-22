# find.py
# -*- coding: utf-8 -*-
"""
独立版通用"查找"动作模块 - 避免循环导入
- 旋转扫描 + 多点搜索 + 智能摄像头处理
- 多帧检测提高稳定性和准确性
- 智能LLM重规划：根据对话历史决定后续动作
- 命中后把目标信息写入黑板（/tmp/robot_blackboard_<robot>.json）
- 完全独立，不依赖其他模块的复杂导入

集成方式：
在robot_controller.py中调用find动作时，传入以下上下文：
```python
from robotControllerRepo.actions.find import create_llm_replanning_function

# 创建LLM重规划函数
llm_replanning_fn = create_llm_replanning_function()

# 在execute_action中调用
result = run_find(
    node, task,
    detect_fn=detect_once,
    rotate_fn=lambda robot, deg: rotate_deg(node, robot, deg),
    navigate_to_fn=lambda robot, target: navigate_to(node, executor, robot, target),
    event_pub=event_publisher,
    llm_replanning_fn=llm_replanning_fn,  # 传入LLM重规划函数
    history_store=history_store_instance   # 传入历史存储实例
)
```
"""

import os
import json
import time
import cv2
import math
import threading
import subprocess
import numpy as np
from typing import Any, Dict, List, Optional
from collections import Counter
import config
from ttsRepo.stream_tts import tts_manager

# 简单的日志工具
class SimpleLogger:
    def info(self, msg): print(f"[INFO] {msg}")
    def warning(self, msg): print(f"[WARN] {msg}")
    def error(self, msg): print(f"[ERROR] {msg}")
    def debug(self, msg): print(f"[DEBUG] {msg}")

logger = SimpleLogger()

# -------- robot id & master id -------

def get_robot_id():
    return config.get("robot_id")

def get_master_name():
    return config.get("master_id")

# -------- 黑板工具（/tmp） --------
def _bb_path(robot_name: str) -> str:
    return f"/tmp/robot_blackboard_{robot_name}.json"

def _bb_read(robot_name: str) -> Dict[str, Any]:
    p = _bb_path(robot_name)
    if os.path.exists(p):
        try:
            with open(p, "r") as f:
                return json.load(f)
        except Exception as e:
            logger.warning(f"Blackboard read failed: {e}")
            return {}
    return {}

def _bb_write(robot_name: str, data: Dict[str, Any]) -> None:
    p = _bb_path(robot_name)
    try:
        with open(p, "w") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
    except Exception as e:
        logger.warning(f"Blackboard write failed: {e}")

def bb_set(robot_name: str, key: str, value: Dict[str, Any]) -> None:
    db = _bb_read(robot_name)
    db.setdefault("objects", {})[key] = value
    _bb_write(robot_name, db)

# -------- 独立的YOLO检测器 --------
class StandaloneYOLODetector:
    """独立的YOLO检测器，不依赖外部模块"""
    def __init__(self, weights_path: str = "yolov8n.pt"):
        try:
            from ultralytics import YOLO
            self.model = YOLO(weights_path)
            self.available = True
            logger.info(f"YOLO model loaded: {weights_path}")
        except Exception as e:
            logger.error(f"Failed to load YOLO: {e}")
            self.model = None
            self.available = False
    
    def detect_objects(self, image_path_or_array, conf_threshold: float = 0.5) -> List[Dict]:
        """检测图像中的物体"""
        if not self.available:
            return []
        
        try:
            results = self.model.predict(
                source=image_path_or_array,
                conf=conf_threshold,
                verbose=False
            )[0]
            
            detections = []
            names = results.names
            
            for box in getattr(results, "boxes", []):
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                cls_name = names.get(cls_id, str(cls_id)).lower()
                
                detections.append({
                    "class": cls_name,
                    "conf": conf,
                    "center_xy": [cx, cy],
                    "bbox_xyxy": [x1, y1, x2, y2]
                })
            
            return detections
            
        except Exception as e:
            logger.error(f"YOLO detection failed: {e}")
            return []

# -------- 智能参数优化 --------
def get_optimized_params(target_class: str, has_waypoints: bool) -> tuple:
    """根据物体类型和搜索策略优化检测参数"""
    conf_thres = 0.5
    max_rot_deg = 360
    
    # 针对不同物体类型的优化
    if target_class in ["cup", "bottle", "phone", "remote", "keys"]:  # 小物体
        conf_thres = 0.45  # 稍微宽松，避免漏检小物体
    elif target_class in ["person", "chair", "table", "sofa", "bed"]:  # 大物体
        conf_thres = 0.6   # 可以更严格，大物体通常检测置信度高
    elif target_class in ["book", "laptop", "mouse", "keyboard"]:  # 中等物体
        conf_thres = 0.5   # 保持默认
    
    # 如果有多个搜索点，每个点只需半圈扫描（提高效率）
    if has_waypoints:
        max_rot_deg = 180  # 多点搜索时每点半圈即可
    
    logger.debug(f"Optimized params for '{target_class}': conf_thres={conf_thres}, max_rot_deg={max_rot_deg}")
    return conf_thres, max_rot_deg

# -------- 摄像头处理 --------
def _create_video_capture(video_source: int = 0) -> Optional[cv2.VideoCapture]:
    """创建并配置摄像头capture对象"""
    logger.info(f"[find] 📷 Attempting to open camera: {video_source}")
    
    # 尝试不同的后端
    backends_to_try = [
        cv2.CAP_V4L2,      # Linux V4L2
        cv2.CAP_GSTREAMER, # GStreamer
        cv2.CAP_FFMPEG,    # FFmpeg
        cv2.CAP_ANY        # 自动选择
    ]
    
    for backend in backends_to_try:
        logger.debug(f"[find] 🔧 Trying backend: {backend}")
        cap = cv2.VideoCapture(video_source, backend)
        
        if cap.isOpened():
            # 设置摄像头参数
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            # 测试读取帧
            ret, frame = cap.read()
            if ret and frame is not None:
                logger.info(f"[find] ✅ Camera opened successfully with backend {backend}")
                logger.info(f"[find] 📏 Frame shape: {frame.shape}")
                return cap
            else:
                logger.warning(f"[find] ⚠️ Camera opened but cannot read frames with backend {backend}")
                cap.release()
        else:
            logger.warning(f"[find] ❌ Failed to open camera with backend {backend}")
    
    logger.error(f"[find] ❌ Failed to open camera with any backend")
    return None

def _test_camera_simple() -> bool:
    """简单测试摄像头是否可用"""
    logger.info("[find] 🧪 Testing camera access...")
    
    cap = _create_video_capture(0)
    if cap is None:
        return False
    
    try:
        # 尝试读取10帧
        success_count = 0
        for i in range(10):
            ret, frame = cap.read()
            if ret and frame is not None:
                success_count += 1
            time.sleep(0.1)
        
        logger.info(f"[find] 📊 Camera test: {success_count}/10 frames read successfully")
        return success_count >= 5  # 至少成功读取一半
        
    finally:
        cap.release()

# -------- 多帧图像检测 --------
def _detect_with_camera_multiframe(detector: StandaloneYOLODetector, 
                                  target_class: str, 
                                  conf_thres: float,
                                  num_frames: int = 5,
                                  frame_interval: float = 0.3) -> Optional[Dict]:
    """使用摄像头进行多帧检测，提高检测稳定性"""
    if not detector.available:
        return None
    
    logger.debug(f"[find] 📷 Starting {num_frames}-frame detection for {target_class}")
    
    all_detections = []  # 存储所有帧的检测结果
    successful_frames = 0
    
    # 尝试多帧检测
    for frame_idx in range(num_frames):
        logger.debug(f"[find] 📸 Capturing frame {frame_idx + 1}/{num_frames}")
        
        frame_result = _detect_single_frame(detector, target_class, conf_thres, frame_idx)
        if frame_result:
            all_detections.extend(frame_result)
            successful_frames += 1
            
        # 帧间间隔，让场景稍微变化
        if frame_idx < num_frames - 1:
            time.sleep(frame_interval)
    
    logger.info(f"[find] 📊 Multi-frame detection: {successful_frames}/{num_frames} frames successful, {len(all_detections)} total detections")
    
    if not all_detections:
        logger.info(f"[find] ❌ No {target_class} detected in any frame")
        return None
    
    # 综合分析多帧结果
    best_detection = _analyze_multiframe_results(all_detections, target_class, conf_thres)
    
    if best_detection:
        logger.info(f"[find] ✅ Best detection: conf={best_detection['conf']:.3f}, frames_detected={best_detection.get('detection_count', 1)}")
    
    return best_detection


def _detect_single_frame(detector: StandaloneYOLODetector, 
                        target_class: str, 
                        conf_thres: float, 
                        frame_idx: int) -> List[Dict]:
    """检测单帧图像"""
    # 使用rpicam-still拍照（针对树莓派）
    try:
        ts = time.strftime("%Y%m%d-%H%M%S")
        img_path = f"/tmp/find_frame_{ts}_{frame_idx}.jpg"
        
        result = subprocess.run([
            "rpicam-still", "-t", "500",  # 减少拍照时间
            "--width", "640", "--height", "480",
            "-o", img_path
        ], capture_output=True, check=True, timeout=3)
        
        if os.path.exists(img_path):
            detections = detector.detect_objects(img_path, conf_thres)
            os.remove(img_path)  # 清理临时文件
            
            # 找到目标物体并添加帧信息
            targets = []
            for d in detections:
                if d["class"] == target_class and d["conf"] >= conf_thres:
                    d["frame_idx"] = frame_idx
                    targets.append(d)
            
            return targets
        
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        logger.debug(f"[find] rpicam-still failed for frame {frame_idx}, trying OpenCV...")
        
        # 回退到OpenCV
        cap = _create_video_capture(0)
        if cap:
            try:
                # 读取几帧来稳定摄像头
                for _ in range(5):
                    cap.read()
                    
                ret, frame = cap.read()
                if ret and frame is not None:
                    detections = detector.detect_objects(frame, conf_thres)
                    targets = []
                    for d in detections:
                        if d["class"] == target_class and d["conf"] >= conf_thres:
                            d["frame_idx"] = frame_idx
                            targets.append(d)
                    return targets
            finally:
                cap.release()
    
    return []


def _analyze_multiframe_results(all_detections: List[Dict], 
                               target_class: str, 
                               conf_thres: float) -> Optional[Dict]:
    """分析多帧检测结果，返回最可靠的检测"""
    if not all_detections:
        return None
    
    logger.debug(f"[find] 🔍 Analyzing {len(all_detections)} detections across multiple frames")
    
    # 策略1: 直接返回置信度最高的检测
    best_by_conf = max(all_detections, key=lambda x: x["conf"])
    
    # 策略2: 检查是否有多帧都检测到的稳定区域
    # 将相近位置的检测归类
    position_groups = _group_detections_by_position(all_detections)
    
    if len(position_groups) > 1:
        # 有多个可能的位置，选择检测次数最多且置信度较高的
        best_group = max(position_groups, key=lambda g: (len(g), max(d["conf"] for d in g)))
        stable_detection = max(best_group, key=lambda x: x["conf"])
        stable_detection["detection_count"] = len(best_group)
        stable_detection["position_stability"] = len(best_group) / len(set(d["frame_idx"] for d in all_detections))
        
        logger.debug(f"[find] 📍 Found stable position group with {len(best_group)} detections")
        return stable_detection
    else:
        # 只有一个位置区域，返回置信度最高的
        best_by_conf["detection_count"] = len(all_detections)
        best_by_conf["position_stability"] = len(set(d["frame_idx"] for d in all_detections)) / len(set(d["frame_idx"] for d in all_detections))
        return best_by_conf


def _group_detections_by_position(detections: List[Dict], 
                                position_tolerance: float = 50.0) -> List[List[Dict]]:
    """根据位置将检测结果分组"""
    groups = []
    
    for detection in detections:
        center_x, center_y = detection["center_xy"]
        
        # 找到位置相近的组
        assigned = False
        for group in groups:
            for existing in group:
                ex_x, ex_y = existing["center_xy"]
                distance = math.sqrt((center_x - ex_x)**2 + (center_y - ex_y)**2)
                
                if distance <= position_tolerance:
                    group.append(detection)
                    assigned = True
                    break
            if assigned:
                break
        
        # 如果没有找到相近的组，创建新组
        if not assigned:
            groups.append([detection])
    
    return groups


# -------- 兼容性包装器 --------
def _detect_with_camera(detector: StandaloneYOLODetector, target_class: str, conf_thres: float) -> Optional[Dict]:
    """兼容旧接口的包装器，默认使用多帧检测"""
    return _detect_with_camera_multiframe(detector, target_class, conf_thres, num_frames=5)

# -------- 分步旋转扫描 --------
def _rotate_scan_stepwise(robot_name: str,
                         rotate_fn,
                         detect_fn,
                         target_class: str,
                         *,
                         max_rot_deg: int = 360,
                         step_deg: int = 30,
                         pause: float = 0.5,
                         conf_thres: float = 0.5) -> Optional[Dict[str, Any]]:
    """分步旋转扫描"""
    logger.info(f"[find] 🔄 Starting stepwise rotate+detect scan")
    logger.info(f"[find] 🔧 Step parameters: max_rot={max_rot_deg}°, step={step_deg}°, pause={pause}s")
    
    # 先看当前帧
    logger.info(f"[find] 📷 Checking current view first...")
    hit = detect_fn(target_class, conf_thres) if detect_fn else None
    if hit:
        logger.info(f"[find] 🎯 Found {target_class} in current view!")
        return hit

    turned = 0
    step_count = 0
    while turned < max_rot_deg:
        step_count += 1
        logger.info(f"[find] 🔄 Step {step_count}: Rotating {step_deg}° (total: {turned}°/{max_rot_deg}°)")
        
        try:
            rotate_fn(robot_name, step_deg)
            logger.info(f"[find] ✅ Rotation step completed")
        except Exception as e:
            logger.error(f"[find] ❌ Rotation step failed: {e}")
            break
            
        # 暂停让机器人稳定
        time.sleep(pause)
        
        # 检测当前视角
        logger.info(f"[find] 📷 Detecting after rotation step {step_count}...")
        hit = detect_fn(target_class, conf_thres) if detect_fn else None
        if hit:
            logger.info(f"[find] 🎯 Found {target_class} after {turned + step_deg}° rotation!")
            return hit
            
        turned += step_deg
        
    logger.info(f"[find] 🛑 Stepwise scan complete: {step_count} steps, {turned}° total rotation")
    return None

# -------- 主要的find功能 --------
def execute_find(node: Any, robot_name: str, params: Dict[str, Any], **ctx) -> Dict[str, Any]:
    """独立的查找动作：旋转扫描"""
    # === 解析参数 ===
    target: str = params.get("target_class", "cup")
    save_as: str = params.get("save_as", target)
    timeout_sec: float = float(params.get("timeout_sec", 25))
    use_spin: bool = bool(params.get("spin_scan", True))
    waypoints: List[Any] = params.get("search_waypoints", [])
    
    # === 多帧检测参数 ===
    num_frames: int = int(params.get("detection_frames", 3))  # 默认3帧
    frame_interval: float = float(params.get("frame_interval", 0.3))  # 帧间间隔0.3秒

    # === 系统优化参数 ===
    conf_thres, max_rot_deg = get_optimized_params(target, bool(waypoints))
    
    # === 获取上下文函数 ===
    rotate_fn = ctx.get("rotate_fn")
    navigate_to_fn = ctx.get("navigate_to_fn")
    event_pub = ctx.get("event_pub")

    if use_spin and not callable(rotate_fn):
        logger.warning("spin_scan=True but rotate_fn missing, will skip spin scan.")
        use_spin = False
    if waypoints and not callable(navigate_to_fn):
        logger.warning("search_waypoints provided but navigate_to_fn missing, will skip waypoint search.")
        waypoints = []

    # === 初始化检测器 ===
    detector = StandaloneYOLODetector()
    if not detector.available:
        logger.error("YOLO detector not available")
        return {"ok": False, "found": False, "reason": "YOLO unavailable"}

    # === 摄像头测试 ===
    camera_ok = _test_camera_simple()
    logger.info(f"[find] 📷 Camera test result: {'✅ PASS' if camera_ok else '❌ FAIL'}")

    # === 开始搜索 ===
    t0 = time.time()
    # tts_manager.say(f"Okay, I'll look around for the {target}.")
    logger.info(f"[find] robot={robot_name} target={target} timeout={timeout_sec}s conf>={conf_thres}")
    logger.info(f"[find] Multi-frame detection: {num_frames} frames, {frame_interval}s interval")

    hit: Optional[Dict[str, Any]] = None

    # 创建检测函数
    def detect_current_view(target_class: str, conf_threshold: float):
        return _detect_with_camera_multiframe(detector, target_class, conf_threshold, 
                                            num_frames=num_frames, frame_interval=frame_interval)

    # === 1. 当前帧检测 ===
    hit = detect_current_view(target, conf_thres)

    # === 2. 旋转扫描 ===
    if not hit and use_spin:
        logger.info(f"[find] 🔄 Starting rotation scan")
        hit = _rotate_scan_stepwise(
            robot_name=robot_name,
            rotate_fn=rotate_fn,
            detect_fn=detect_current_view,
            target_class=target,
            conf_thres=conf_thres,
            max_rot_deg=max_rot_deg,
            step_deg=30,
            pause=0.8  # 增加暂停时间确保稳定
        )

    # === 3. 多点搜索 ===
    if not hit and waypoints:
        logger.info(f"[find] 🗺️ Starting waypoint search: {waypoints}")
        for i, wp in enumerate(waypoints):
            if time.time() - t0 > timeout_sec:
                logger.info("[find] timeout during waypoints loop")
                break
                
            logger.info(f"[find] 🚶 Moving to waypoint {i+1}/{len(waypoints)}: {wp}")
            try:
                navigate_to_fn(robot_name, wp)
            except Exception as e:
                logger.warning(f"navigate_to_fn error on {wp}: {e}")
                continue

            time.sleep(1.0)  # 到达后等待稳定
            
            # 到达后先检测一次
            hit = detect_current_view(target, conf_thres)
            if hit:
                logger.info(f"[find] 🎯 Found {target} at waypoint {wp}!")
                break

            # 如果没找到，在这个位置做半圈扫描
            if use_spin:
                logger.info(f"[find] 🔄 Scanning at waypoint {wp}")
                hit = _rotate_scan_stepwise(
                    robot_name=robot_name,
                    rotate_fn=rotate_fn,
                    detect_fn=detect_current_view,
                    target_class=target,
                    conf_thres=conf_thres,
                    max_rot_deg=180,  # waypoint处只扫描半圈
                    step_deg=45,      # 更大步长，提高效率
                    pause=0.8
                )

            if hit:
                logger.info(f"[find] 🎯 Found {target} during scan at waypoint {wp}!")
                break

    # === 处理结果 ===
    if not hit:
        tts_manager.say("I couldn't find it.")
        payload = {"robot": robot_name, "target": target, "found": False, "ts": time.time()}
        if callable(event_pub):
            try:
                event_pub("find_result", payload)
            except Exception as e:
                logger.warning(f"event_pub error: {e}")
        logger.info(f"[find] not found: {target}")
        return {"ok": True, "found": False, "blackboard_key": save_as}

    # === 成功找到 ===
    record = {
        "class": target,
        "center_xy": hit.get("center_xy"),
        "bbox_xyxy": hit.get("bbox_xyxy"),
        "map_xy": hit.get("map_xy") if "map_xy" in hit else None,
        "conf": float(hit.get("conf", 0.0)),
        "timestamp": time.time(),
        # 多帧检测相关信息
        "detection_count": hit.get("detection_count", 1),
        "position_stability": hit.get("position_stability", 1.0),
        "detection_method": "multiframe" if hit.get("detection_count", 1) > 1 else "single"
    }
    bb_set(robot_name, save_as, record)

    payload = {"robot": robot_name, "target": target, "found": True,
               "key": save_as, "conf": record["conf"], "ts": record["timestamp"]}
    if callable(event_pub):
        try:
            event_pub("find_result", payload)
        except Exception as e:
            logger.warning(f"event_pub error: {e}")

    tts_manager.say(f"I found the {target}.")
    logger.info(f"[find] found {target} (key={save_as}, conf={record['conf']:.3f})")
    logger.info(f"[find] Detection quality: {record['detection_count']} frames, stability={record['position_stability']:.2f}")
    return {"ok": True, "found": True, "blackboard_key": save_as, "record": record}

# -------- 智能动态重规划 --------
def execute_find_with_llm_replanning(node: Any, robot_name: str, params: Dict[str, Any], **ctx) -> Dict[str, Any]:
    """智能版：找到物体后调用LLM决定后续动作"""
    basic_result = execute_find(node, robot_name, params, **ctx)
    
    if not basic_result.get("found", False):
        logger.info(f"[find] {robot_name} didn't find {params.get('target_class')}, no follow-up actions needed")
        return basic_result
    
    # === 获取LLM重规划功能 ===
    llm_replanning_fn = ctx.get("llm_replanning_fn")
    history_store = ctx.get("history_store")
    
    if not callable(llm_replanning_fn):
        logger.warning("[find] No LLM replanning function provided, falling back to simple mode")
        return execute_find_with_simple_replanning(node, robot_name, params, **ctx)
    
    # === 准备上下文信息 ===
    target_class = params.get("target_class", "cup")
    blackboard_key = basic_result.get("blackboard_key")
    record = basic_result.get("record", {})
    
    # 构建发现物体的上下文
    discovery_context = {
        "found_object": {
            "class": target_class,
            "confidence": record.get("conf", 0.0),
            "position": record.get("center_xy", [0, 0]),
            "blackboard_key": blackboard_key,
            "detection_quality": {
                "frames_detected": record.get("detection_count", 1),
                "position_stability": record.get("position_stability", 1.0),
                "method": record.get("detection_method", "single")
            }
        },
        "robot_name": robot_name,
        "search_params": params
    }
    
    # 获取对话历史
    chat_history = []
    if history_store:
        try:
            chat_history = history_store.recent_chat_messages(max_turns=5)  # 最近5轮对话
        except Exception as e:
            logger.warning(f"[find] Failed to get chat history: {e}")
    
    logger.info(f"[find] Calling LLM for intelligent replanning...")
    logger.info(f"[find] Context: found {target_class} with conf={record.get('conf', 0):.3f}")
    
    try:
        # === 调用LLM进行智能规划 ===
        replanning_result = llm_replanning_fn(
            discovery_context=discovery_context,
            chat_history=chat_history,
            robot_name=robot_name
        )
        
        if replanning_result.get("success", False):
            follow_up_tasks = replanning_result.get("tasks", [])
            reasoning = replanning_result.get("reasoning", "")
            
            basic_result["follow_up_tasks"] = follow_up_tasks
            basic_result["replanning_success"] = True
            basic_result["replanning_source"] = "llm"
            basic_result["replanning_reasoning"] = reasoning
            
            logger.info(f"[find] LLM generated {len(follow_up_tasks)} follow-up tasks")
            logger.info(f"[find] LLM reasoning: {reasoning}")
            
            return basic_result
        else:
            logger.warning(f"[find] LLM replanning failed: {replanning_result.get('error', 'unknown')}")
            
    except Exception as e:
        logger.error(f"[find] LLM replanning error: {e}")
    
    # === 回退到简单模式 ===
    logger.info("[find] Falling back to simple replanning")
    return execute_find_with_simple_replanning(node, robot_name, params, **ctx)


def execute_find_with_simple_replanning(node: Any, robot_name: str, params: Dict[str, Any], **ctx) -> Dict[str, Any]:
    """简化版：找到物体后生成基本的后续任务（回退模式）"""
    basic_result = execute_find(node, robot_name, params, **ctx)
    
    if not basic_result.get("found", False):
        return basic_result
    
    # === 简单的后续任务生成 ===
    target_class = params.get("target_class", "cup")
    blackboard_key = basic_result.get("blackboard_key")
    
    # 生成标准的 collect + deliver 任务
    follow_up_tasks = [
        {
            "action": "collect",
            "parameters": {
                "item": target_class,
                "target": blackboard_key
            }
        },
        {
            "action": "deliver", 
            "parameters": {
                "item": target_class,
                "target": "master"  # 假设送给主人
            }
        }
    ]
    
    basic_result["follow_up_tasks"] = follow_up_tasks
    basic_result["replanning_success"] = True
    basic_result["replanning_source"] = "simple_fallback"
    
    logger.info(f"[find] Generated {len(follow_up_tasks)} simple fallback tasks")
    return basic_result


# -------- LLM重规划实现 --------
def create_llm_replanning_function():
    """创建LLM重规划函数，可以在robot_controller中注入"""
    
    def llm_replanning_function(discovery_context: Dict, chat_history: List[Dict], robot_name: str) -> Dict:
        """调用LLM进行智能重规划"""
        try:
            # 导入LLM相关模块（延迟导入避免循环依赖）
            import os
            import openai
            import json
            from textwrap import dedent
            
            api_key = os.getenv("OPENAI_API_KEY")
            if not api_key:
                return {"success": False, "error": "No OpenAI API key"}
            
            client = openai.OpenAI(api_key=api_key)
            
            # === 构建LLM提示 ===
            found_object = discovery_context["found_object"]
            
            system_prompt = dedent(f"""
            You are an intelligent robot task planner. A robot has just found an object and you need to decide what to do next.

            Robot: {get_robot_id()}
            Master: {get_master_name()}
            Found Object: {found_object["class"]} (confidence: {found_object["confidence"]:.3f})
            Detection Quality: {found_object["detection_quality"]["frames_detected"]} frames, stability: {found_object["detection_quality"]["position_stability"]:.2f}
            Blackboard Key: {found_object["blackboard_key"]}

            Based on the conversation history, decide if the robot should:
            1. Just report finding the object (no further action)
            2. Navigate to the object location first, then collect it
            3. Navigate to object, collect it, then navigate to recipient and deliver it
            4. Navigate closer to examine the object
            5. Other specific actions

            IMPORTANT: Analyze the user's original intent from the conversation history. Don't assume they always want collect+deliver.

            CRITICAL: Use the exact parameter format expected by the robot controller:

            For "collect" action (robot should already be at the object location):
            {{
                "action": "collect",
                "parameters": {{
                    "item": "<object_class>"
                }}
            }}

            For "deliver" action (robot should already be at the destination):
            {{
                "action": "deliver", 
                "parameters": {{
                    "item": "<object_class>"
                }}
            }}

            For "navigate" action:
            {{
                "action": "navigate",
                "parameters": {{
                    "target": "<blackboard_key_or_location>"
                }}
            }}

            For "face" action:
            {{
                "action": "face",
                "parameters": {{
                    "target": "<target_name_or_coordinates>"
                }}
            }}

            Typical task sequences:
            - Find + Navigate to object + Collect + Navigate to recipient + Deliver
            - Find + Navigate to object + Collect (if user just wants the robot to get it)
            - Find + Navigate closer + Face object (if user wants to examine it)

            Respond in JSON format:
            {{
                "action_needed": true/false,
                "tasks": [
                    {{
                        "action": "navigate|collect|deliver|face|wait",
                        "parameters": {{ ... }}
                    }}
                ],
                "reasoning": "Brief explanation of why these actions were chosen"
            }}

            If no action is needed, set "action_needed": false and "tasks": [].

            Remember: 
            - collect/deliver only need "item" parameter
            - Use navigate action to move to locations before collect/deliver
            - Use blackboard_key for navigating to found objects
            - When user says "bring to me", "deliver to me", "here", use "{get_master_name()}" as the target
            - When user says "bring to someone else", use that person's name as the target
            """).strip()




            # 构建对话历史字符串
            history_text = ""
            if chat_history:
                history_text = "Recent conversation:\n"
                for msg in chat_history[-6:]:  # 最近3轮对话
                    role = msg.get("role", "unknown")
                    content = msg.get("content", "")
                    history_text += f"{role}: {content}\n"
            else:
                history_text = "No recent conversation history available."
            
            user_prompt = f"""
            {history_text}

            The robot has successfully found a {found_object["class"]}. What should it do next?
            """
            
            # === 调用LLM ===
            logger.debug(f"[find] LLM prompt: {user_prompt[:200]}...")
            
            response = client.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.3,
                max_tokens=500
            )
            
            raw_response = response.choices[0].message.content.strip()
            logger.debug(f"[find] LLM raw response: {raw_response}")
            
            # === 解析LLM响应 ===
            try:
                # 提取JSON部分
                if "```json" in raw_response:
                    json_start = raw_response.find("```json") + 7
                    json_end = raw_response.find("```", json_start)
                    json_text = raw_response[json_start:json_end].strip()
                elif "{" in raw_response and "}" in raw_response:
                    json_start = raw_response.find("{")
                    json_end = raw_response.rfind("}") + 1
                    json_text = raw_response[json_start:json_end]
                else:
                    json_text = raw_response
                
                parsed = json.loads(json_text)
                
                action_needed = parsed.get("action_needed", False)
                tasks = parsed.get("tasks", [])
                reasoning = parsed.get("reasoning", "LLM decision")
                
                # 验证任务格式
                validated_tasks = []
                if action_needed and tasks:
                    for task in tasks:
                        if isinstance(task, dict) and "action" in task and "parameters" in task:
                            validated_tasks.append(task)
                        else:
                            logger.warning(f"[find] Invalid task format: {task}")
                
                return {
                    "success": True,
                    "tasks": validated_tasks,
                    "reasoning": reasoning,
                    "action_needed": action_needed
                }
                
            except json.JSONDecodeError as e:
                logger.error(f"[find] Failed to parse LLM JSON response: {e}")
                return {"success": False, "error": f"JSON parse error: {e}"}
            
        except Exception as e:
            logger.error(f"[find] LLM replanning function error: {e}")
            return {"success": False, "error": str(e)}
    
    return llm_replanning_function

# -------- 对外接口 --------
def run_action(node: Any, task: Dict[str, Any], **ctx) -> Dict[str, Any]:
    """与 robot_controller 的单步接口保持一致"""
    robot = task.get("robot") or task.get("parameters", {}).get("robot")
    if not robot:
        return {"ok": False, "reason": "robot name missing"}
    
    # 使用智能LLM重规划版本
    return execute_find_with_llm_replanning(node, robot, task.get("parameters", {}), **ctx)

# -------- 真实的机器人控制函数 --------
def real_rotate(node, robot_name: str, degrees: float):
    """真实的机器人旋转函数"""
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        import math
        
        # 创建发布器
        publisher = node.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
        
        # 设置旋转参数
        twist = Twist()
        angular_speed = math.radians(30)  # 30°/s
        
        direction = "left" if degrees >= 0 else "right"
        twist.angular.z = angular_speed if degrees >= 0 else -angular_speed
        
        # 计算旋转时间
        angle_rad = math.radians(abs(degrees))
        duration = angle_rad / abs(angular_speed)
        
        logger.info(f"🔄 Real rotation: {robot_name} turning {direction} {abs(degrees)}° for {duration:.2f}s")
        
        # 等待发布器初始化
        time.sleep(0.2)
        
        # 持续发布旋转命令
        start_time = time.time()
        while time.time() - start_time < duration:
            publisher.publish(twist)
            time.sleep(0.01)
        
        # 停止旋转
        publisher.publish(Twist())
        logger.info(f"✅ Real rotation completed: {robot_name}")
        return True
        
    except Exception as e:
        logger.error(f"❌ Real rotation failed: {e}")
        return False

def real_navigate(node, robot_name: str, target):
    """简单的导航实现（占位）"""
    logger.info(f"🚶 Navigation: {robot_name} -> {target} (placeholder)")
    time.sleep(1.0)  # 模拟导航时间
    return True

# -------- 独立运行的主函数 --------
def run_standalone_test():
    """独立运行find测试，使用真实的ROS控制"""
    import sys
    
    try:
        import rclpy
        from rclpy.node import Node
        
        # 初始化ROS
        rclpy.init()
        logger.info("🚀 ROS initialized for standalone find test")
        
        # 创建ROS节点
        node = rclpy.create_node('standalone_find_test')
        logger.info("🤖 ROS node created: standalone_find_test")
        
        # 获取机器人名称
        robot_name = sys.argv[1] if len(sys.argv) > 1 else "robot1"
        target_class = sys.argv[2] if len(sys.argv) > 2 else "cup"
        
        logger.info(f"🎯 Test parameters: robot={robot_name}, target={target_class}")
        
        # 创建真实的旋转函数
        def rotate_fn(robot, deg):
            return real_rotate(node, robot, deg)
        
        def nav_fn(robot, target):
            return real_navigate(node, robot, target)
        
        def event_fn(kind, payload):
            logger.info(f"📡 Event: {kind} -> {payload}")
        
        # 创建LLM重规划函数
        llm_replanning_fn = create_llm_replanning_function()
        
        # 模拟历史存储（在实际使用中从gpt_yolo_localParser获取）
        class MockHistoryStore:
            def recent_chat_messages(self, max_turns=5):
                # 模拟一些对话历史
                return [
                    {"role": "user", "content": f"Can you help me find my {target_class}?"},
                    {"role": "assistant", "content": f"I'll look for your {target_class}."}
                ]
        
        mock_history = MockHistoryStore()
        
        print(f"🧪 Testing REAL find functionality for {robot_name}...")
        print(f"🎯 Looking for: {target_class}")
        print("⚠️  Make sure your robot is ready and ROS topics are active!")
        
        # 等待用户确认
        input("Press Enter when robot is ready, or Ctrl+C to cancel...")
        
        # 执行真实的find测试
        res = execute_find_with_llm_replanning(
            node=node,
            robot_name=robot_name,
            params={
                "target_class": target_class,
                "save_as": f"{target_class}_target",
                "timeout_sec": 30,  # 增加超时时间
                "search_waypoints": [],  # 暂时不使用waypoints
                "detection_frames": 5,   # 使用5帧检测提高稳定性
                "frame_interval": 0.2    # 减少帧间间隔加快检测速度
            },
            rotate_fn=rotate_fn,
            navigate_to_fn=nav_fn,
            event_pub=event_fn,
            llm_replanning_fn=llm_replanning_fn,  # 传入LLM重规划函数
            history_store=mock_history              # 传入历史存储
        )
        
        print("\n" + "="*50)
        print("🎉 FIND TEST RESULT:")
        print(f"✅ Success: {res.get('ok', False)}")
        print(f"🎯 Found: {res.get('found', False)}")
        if res.get('found'):
            record = res.get('record', {})
            print(f"📦 Object: {res.get('blackboard_key')}")
            print(f"🎯 Confidence: {record.get('conf', 0):.3f}")
            print(f"📊 Detection frames: {record.get('detection_count', 1)}")
            print(f"🎚️ Position stability: {record.get('position_stability', 1.0):.2f}")
            print(f"🔧 Method: {record.get('detection_method', 'unknown')}")
            
            # 显示重规划信息
            if res.get('replanning_success'):
                print(f"🧠 Replanning: {res.get('replanning_source', 'unknown')}")
                if res.get('replanning_reasoning'):
                    print(f"💭 Reasoning: {res.get('replanning_reasoning')}")
                
                follow_up_tasks = res.get('follow_up_tasks', [])
                if follow_up_tasks:
                    print(f"📋 Follow-up tasks: {len(follow_up_tasks)}")
                    for i, task in enumerate(follow_up_tasks, 1):
                        print(f"   {i}. {task.get('action', 'unknown')} -> {task.get('parameters', {})}")
                else:
                    print("📋 No follow-up actions needed")
        print("="*50)
        
        return res
        
    except KeyboardInterrupt:
        print("\n🛑 Test cancelled by user")
        return {"ok": False, "cancelled": True}
        
    except Exception as e:
        logger.error(f"❌ Standalone test failed: {e}")
        return {"ok": False, "error": str(e)}
        
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
            logger.info("🔚 ROS shutdown completed")
        except:
            pass

# -------- 命令行测试 --------
if __name__ == "__main__":
    print("🤖 Standalone Find.py Test")
    print("Usage: python3 find.py [robot_name] [target_class]")
    print("Example: python3 find.py robot1 cup")
    print("")
    
    result = run_standalone_test()
    
    if result.get("ok"):
        print("🎉 Test completed successfully!")
        exit(0)
    else:
        print("❌ Test failed!")
        exit(1)