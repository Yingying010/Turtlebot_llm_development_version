# find.py
# -*- coding: utf-8 -*-
import os, sys, time
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from ttsRepo.stream_tts import tts_manager
import json
import time
import cv2
import math
import subprocess
from typing import Any, Dict, List, Optional
import config
from ttsRepo.stream_tts import tts_manager
from robotControllerRepo.actions.navigate import navigate_after_follow
from robotControllerRepo.actions.pickup import pickup_item
from robotControllerRepo.actions.dropoff import dropoff_item

# 简单的日志工具
class SimpleLogger:
    def info(self, msg): print(f"[INFO] {msg}")
    def warning(self, msg): print(f"[WARN] {msg}")
    def error(self, msg): print(f"[ERROR] {msg}")
    def debug(self, msg): print(f"[DEBUG] {msg}")

logger = SimpleLogger()

# -------- 🆕 简单滤波器 --------
class SimpleDetectionFilter:
    """简单的单帧检测滤波器，不保存历史记录"""
    
    def __init__(self, confidence_boost: float = 0.1, position_penalty_threshold: float = 100.0):
        """
        Args:
            confidence_boost: 对高置信度检测的额外加分
            position_penalty_threshold: 位置偏差惩罚阈值(像素)
        """
        self.confidence_boost = confidence_boost
        self.position_penalty_threshold = position_penalty_threshold
        logger.info(f"🔍 Simple filter initialized: boost={confidence_boost}, threshold={position_penalty_threshold}")
    
    def filter_detections(self, detections: List[Dict], item: str, conf_thres: float) -> Optional[Dict]:
        """
        对单帧检测结果进行滤波
        
        Args:
            detections: 当前帧的所有检测结果
            item: 目标类别
            conf_thres: 置信度阈值
        
        Returns:
            滤波后的最佳检测结果
        """
        # 过滤出目标类别且满足置信度的检测
        valid_detections = [
            d for d in detections 
            if d.get("class") == item and d.get("conf", 0) >= conf_thres
        ]
        
        if not valid_detections:
            return None
        
        logger.debug(f"🔍 Filter: Processing {len(valid_detections)} valid detections")
        
        # 如果只有一个检测，直接返回
        if len(valid_detections) == 1:
            result = valid_detections[0].copy()
            result["filter_score"] = result.get("conf", 0)
            result["filter_method"] = "single_detection"
            return result
        
        # 多个检测时进行滤波评分
        best_detection = self._score_detections(valid_detections)
        
        if best_detection:
            logger.info(f"🎯 Filter selected: conf={best_detection.get('conf', 0):.3f}, "
                       f"filter_score={best_detection.get('filter_score', 0):.3f}")
        
        return best_detection
    
    def _score_detections(self, detections: List[Dict]) -> Optional[Dict]:
        """对检测结果进行评分"""
        image_center = [320, 240]  # 假设图像中心
        
        for det in detections:
            conf = det.get("conf", 0)
            center_xy = det.get("center_xy", [0, 0])
            bbox_xyxy = det.get("bbox_xyxy", [0, 0, 0, 0])
            
            # 基础分数：置信度
            score = conf
            
            # 🔥 滤波规则
            
            # 1. 高置信度加分
            if conf > 0.8:
                score += self.confidence_boost * 2
            elif conf > 0.6:
                score += self.confidence_boost
            
            # 2. 图像中心位置加分（中心的检测通常更可靠）
            center_distance = math.sqrt(
                (center_xy[0] - image_center[0])**2 + 
                (center_xy[1] - image_center[1])**2
            )
            center_bonus = max(0, (1 - center_distance / 200.0) * 0.1)
            score += center_bonus
            
            # 3. 边界框大小合理性（过大或过小都扣分）
            bbox_width = bbox_xyxy[2] - bbox_xyxy[0]
            bbox_height = bbox_xyxy[3] - bbox_xyxy[1]
            bbox_area = bbox_width * bbox_height
            
            # 合理的目标大小范围
            if 500 < bbox_area < 50000:  # 合理大小
                score += 0.05
            elif bbox_area < 100 or bbox_area > 100000:  # 太小或太大
                score -= 0.1
            
            # 4. 长宽比合理性（过于狭长的检测框通常不可靠）
            if bbox_height > 0:
                aspect_ratio = bbox_width / bbox_height
                if 0.3 < aspect_ratio < 3.0:  # 合理长宽比
                    score += 0.05
                else:
                    score -= 0.05
            
            det["filter_score"] = score
            
            logger.debug(f"🔍 Detection score: conf={conf:.3f}, center_bonus={center_bonus:.3f}, "
                        f"area={bbox_area:.0f}, final_score={score:.3f}")
        
        # 选择评分最高的检测
        best_det = max(detections, key=lambda x: x.get("filter_score", 0))
        
        # 只有评分足够高才认为是有效检测
        min_score = 0.4  # 最小可接受评分
        if best_det.get("filter_score", 0) < min_score:
            logger.debug(f"🔍 Best detection score {best_det.get('filter_score', 0):.3f} below threshold {min_score}")
            return None
        
        best_det["filter_method"] = "scored_selection"
        return best_det

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
def get_optimized_params(item: str, has_waypoints: bool) -> tuple:
    """根据物体类型和搜索策略优化检测参数"""
    conf_thres = 0.5
    max_rot_deg = 360
    
    # 针对不同物体类型的优化
    if item in ["cup", "bottle", "phone", "remote", "keys"]:  # 小物体
        conf_thres = 0.4  # 更宽松，避免漏检小物体
    elif item in ["person", "chair", "table", "sofa", "bed"]:  # 大物体
        conf_thres = 0.6   # 可以更严格，大物体通常检测置信度高
    elif item in ["book", "laptop", "mouse", "keyboard"]:  # 中等物体
        conf_thres = 0.5   # 保持默认
    
    # 如果有多个搜索点，每个点只需半圈扫描（提高效率）
    if has_waypoints:
        max_rot_deg = 180  # 多点搜索时每点半圈即可
    
    logger.debug(f"Optimized params for '{item}': conf_thres={conf_thres}, max_rot_deg={max_rot_deg}")
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
        # 尝试读取5帧
        success_count = 0
        for i in range(5):
            ret, frame = cap.read()
            if ret and frame is not None:
                success_count += 1
            time.sleep(0.1)
        
        logger.info(f"[find] 📊 Camera test: {success_count}/5 frames read successfully")
        return success_count >= 3  # 至少成功读取大部分
        
    finally:
        cap.release()

# -------- 🆕 基于简单滤波的单帧检测 --------
def _detect_with_simple_filter(detector: StandaloneYOLODetector,
                              filter_obj: SimpleDetectionFilter,
                              item: str, 
                              conf_thres: float) -> Optional[Dict]:
    """使用简单滤波器的单帧检测"""
    if not detector.available:
        return None
    
    logger.debug(f"[find] 📷 Single-frame detection with simple filter for {item}")
    
    # 获取单帧检测结果
    all_detections = _capture_and_detect(detector, conf_thres * 0.8)  # 稍微降低阈值让滤波器处理
    
    if not all_detections:
        logger.debug(f"[find] 📷 No raw detections in current frame")
        return None
    
    # 通过简单滤波器处理
    filtered_result = filter_obj.filter_detections(all_detections, item, conf_thres)
    
    if filtered_result:
        logger.info(f"[find] ✅ Filtered detection: conf={filtered_result.get('conf', 0):.3f}, "
                   f"filter_score={filtered_result.get('filter_score', 0):.3f}")
        
        # 添加滤波相关的元数据
        filtered_result["detection_method"] = "single_frame_filtered"
        filtered_result["filter_used"] = True
        
        return filtered_result
    else:
        logger.debug(f"[find] 🔍 Filter rejected all detections")
        return None

def _capture_and_detect(detector: StandaloneYOLODetector, conf_thres: float) -> List[Dict]:
    """拍照并检测，返回所有检测结果"""
    # 优先使用rpicam-still拍照（针对树莓派）
    try:
        ts = time.strftime("%Y%m%d-%H%M%S")
        img_path = f"/tmp/find_frame_{ts}.jpg"
        
        result = subprocess.run([
            "rpicam-still", "-t", "300",  # 快速拍照
            "--width", "640", "--height", "480",
            "-o", img_path
        ], capture_output=True, check=True, timeout=2)
        
        if os.path.exists(img_path):
            all_detections = detector.detect_objects(img_path, conf_thres)
            os.remove(img_path)  # 清理临时文件
            return all_detections
        
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        logger.debug(f"[find] rpicam-still failed, trying OpenCV...")
        
        # 回退到OpenCV
        cap = _create_video_capture(0)
        if cap:
            try:
                # 少读几帧加快速度
                for _ in range(2):
                    cap.read()
                    
                ret, frame = cap.read()
                if ret and frame is not None:
                    all_detections = detector.detect_objects(frame, conf_thres)
                    return all_detections
            finally:
                cap.release()
    
    return []

# -------- 旧的多帧检测（作为备选） --------
def _detect_with_camera_multiframe(detector: StandaloneYOLODetector, 
                                  item: str, 
                                  conf_thres: float,
                                  num_frames: int = 3,
                                  frame_interval: float = 0.3) -> Optional[Dict]:
    """传统多帧检测（备选方法）"""
    if not detector.available:
        return None
    
    logger.debug(f"[find] 📷 Multi-frame detection: {num_frames} frames for {item}")
    
    all_detections = []
    successful_frames = 0
    
    for frame_idx in range(num_frames):
        logger.debug(f"[find] 📸 Capturing frame {frame_idx + 1}/{num_frames}")
        
        frame_detections = _capture_and_detect(detector, conf_thres)
        targets = [d for d in frame_detections if d.get("class") == item and d.get("conf", 0) >= conf_thres]
        
        if targets:
            for t in targets:
                t["frame_idx"] = frame_idx
            all_detections.extend(targets)
            successful_frames += 1
            
        # 帧间间隔
        if frame_idx < num_frames - 1:
            time.sleep(frame_interval)
    
    logger.info(f"[find] 📊 Multi-frame: {successful_frames}/{num_frames} frames, {len(all_detections)} detections")
    
    if not all_detections:
        return None
    
    # 选择置信度最高的检测
    best_detection = max(all_detections, key=lambda x: x.get("conf", 0))
    best_detection["detection_method"] = "multiframe"
    best_detection["detection_count"] = len(all_detections)
    best_detection["successful_frames"] = successful_frames
    
    return best_detection

# -------- 检测方法选择器 --------
def _detect_with_camera(detector: StandaloneYOLODetector,
                       simple_filter: SimpleDetectionFilter,
                       item: str, 
                       conf_thres: float,
                       use_filter: bool = True,
                       num_frames: int = 1) -> Optional[Dict]:
    """检测方法选择器"""
    if use_filter and num_frames == 1:
        # 🆕 使用简单滤波器的快速单帧检测
        logger.debug(f"[find] Using simple filtered single-frame detection")
        return _detect_with_simple_filter(detector, simple_filter, item, conf_thres)
    else:
        # 传统多帧检测
        logger.debug(f"[find] Using traditional multi-frame detection ({num_frames} frames)")
        return _detect_with_camera_multiframe(detector, item, conf_thres, num_frames)

# -------- 分步旋转扫描 --------
def _rotate_scan_stepwise(robot_name: str,
                         rotate_fn,
                         detect_fn,
                         item: str,
                         *,
                         max_rot_deg: int = 360,
                         step_deg: int = 30,
                         pause: float = 0.5,
                         conf_thres: float = 0.5) -> Optional[Dict[str, Any]]:
    """分步旋转扫描"""
    logger.info(f"[find] 🔄 Starting stepwise rotate+detect scan")
    logger.info(f"[find] 🔧 Parameters: max_rot={max_rot_deg}°, step={step_deg}°, pause={pause}s")
    
    # 先检查当前视角
    logger.info(f"[find] 📷 Checking current view first...")
    hit = detect_fn(item, conf_thres) if detect_fn else None
    if hit:
        logger.info(f"[find] 🎯 Found {item} in current view!")
        return hit

    turned = 0
    step_count = 0
    while turned < max_rot_deg:
        step_count += 1
        logger.info(f"[find] 🔄 Step {step_count}: Rotating {step_deg}° (total: {turned}°/{max_rot_deg}°)")
        
        try:
            rotate_fn(robot_name, step_deg)
            logger.info(f"[find] ✅ Rotation completed")
        except Exception as e:
            logger.error(f"[find] ❌ Rotation failed: {e}")
            break
            
        # 暂停让机器人稳定
        time.sleep(pause)
        
        # 检测当前视角
        logger.info(f"[find] 📷 Detecting after rotation...")
        hit = detect_fn(item, conf_thres) if detect_fn else None
        if hit:
            logger.info(f"[find] 🎯 Found {item} after {turned + step_deg}° rotation!")
            return hit
            
        turned += step_deg
        
    logger.info(f"[find] 🛑 Scan complete: {step_count} steps, {turned}° total")
    return None

# -------- find --------
def execute_find(node: Any, robot_name: str, params: Dict[str, Any]) -> Dict[str, Any]:
    """独立的查找动作：旋转扫描 + 🆕 简单滤波"""
    # === 解析参数 ===
    item: str = params.get("item")
    save_as: str = params.get("save_as", item)  # 新增：用于黑板保存键名
    timeout_sec = 25
    use_spin = False
    waypoints = []
    rotate_fn = None
    navigate_to_fn = None
    event_pub = None
    
    # === 🆕 滤波相关参数 ===
    use_filter: bool = bool(params.get("use_filter", True))  # 默认开启滤波
    num_frames: int = int(params.get("detection_frames", 1 if use_filter else 3))  # 滤波时默认1帧

    # === 系统优化参数 ===
    conf_thres, max_rot_deg = get_optimized_params(item, bool(waypoints))

    # === 初始化检测器和滤波器 ===
    detector = StandaloneYOLODetector()
    if not detector.available:
        logger.error("YOLO detector not available")
        return {"ok": False, "found": False, "reason": "YOLO unavailable"}

    # 🆕 初始化简单滤波器
    simple_filter = SimpleDetectionFilter(confidence_boost=0.1, position_penalty_threshold=100.0)

    # === 摄像头测试 ===
    camera_ok = _test_camera_simple()
    logger.info(f"[find] 📷 Camera test result: {'✅ PASS' if camera_ok else '❌ FAIL'}")

    # === 开始搜索 ===
    t0 = time.time()
    logger.info(f"[find] robot={robot_name} target={item} timeout={timeout_sec}s conf>={conf_thres}")
    
    if use_filter:
        logger.info(f"[find] 🔍 Using SIMPLE FILTER: {num_frames} frame, fast detection")
    else:
        logger.info(f"[find] 📷 Using MULTI-FRAME: {num_frames} frames")

    hit: Optional[Dict[str, Any]] = None

    # 创建检测函数
    def detect_current_view(item: str, conf_threshold: float):
        return _detect_with_camera(
            detector=detector,
            simple_filter=simple_filter,
            item=item,
            conf_thres=conf_threshold,
            use_filter=use_filter,
            num_frames=num_frames
        )

    # === 1. 当前帧检测 ===
    hit = detect_current_view(item, conf_thres)

    # === 2. 旋转扫描 ===
    if not hit and use_spin:
        logger.info(f"[find] 🔄 Starting rotation scan")
        hit = _rotate_scan_stepwise(
            robot_name=robot_name,
            rotate_fn=rotate_fn,
            detect_fn=detect_current_view,
            item=item,
            conf_thres=conf_thres,
            max_rot_deg=max_rot_deg,
            step_deg=30,
            pause=0.5 if use_filter else 0.8  # 滤波时可以更快
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
            hit = detect_current_view(item, conf_thres)
            if hit:
                logger.info(f"[find] 🎯 Found {item} at waypoint {wp}!")
                break

            # 如果没找到，在这个位置做扫描
            if use_spin:
                logger.info(f"[find] 🔄 Scanning at waypoint {wp}")
                hit = _rotate_scan_stepwise(
                    robot_name=robot_name,
                    rotate_fn=rotate_fn,
                    detect_fn=detect_current_view,
                    item=item,
                    conf_thres=conf_thres,
                    max_rot_deg=180,  # waypoint处只扫描半圈
                    step_deg=45,      # 更大步长
                    pause=0.5 if use_filter else 0.8
                )

            if hit:
                logger.info(f"[find] 🎯 Found {item} during scan at waypoint {wp}!")
                break

    # === 处理结果 ===
    if not hit:
        tts_manager.say_sync("I couldn't find it.")
        payload = {"robot": robot_name, "target": item, "found": False, "ts": time.time()}
        if callable(event_pub):
            try:
                event_pub("find_result", payload)
            except Exception as e:
                logger.warning(f"event_pub error: {e}")
        logger.info(f"[find] not found: {item}")
        return {"ok": True, "found": False, "blackboard_key": save_as}

    # === 成功找到 ===
    record = {
        "class": item,
        "center_xy": hit.get("center_xy"),
        "bbox_xyxy": hit.get("bbox_xyxy"),
        "map_xy": hit.get("map_xy") if "map_xy" in hit else None,
        "conf": float(hit.get("conf", 0.0)),
        "timestamp": time.time(),
        # 🆕 滤波相关信息
        "filter_used": hit.get("filter_used", False),
        "filter_score": hit.get("filter_score"),
        "filter_method": hit.get("filter_method"),
        "detection_method": hit.get("detection_method", "unknown"),
        # 保留兼容性
        "detection_count": hit.get("detection_count", 1),
        "successful_frames": hit.get("successful_frames", 1)
    }
    bb_set(robot_name, save_as, record)

    payload = {"robot": robot_name, "target": item, "found": True,
               "key": save_as, "conf": record["conf"], "ts": record["timestamp"]}
    if callable(event_pub):
        try:
            event_pub("find_result", payload)
        except Exception as e:
            logger.warning(f"event_pub error: {e}")

    tts_manager.say_sync(f"I found the {item}.")
    logger.info(f"[find] found {item} (key={save_as}, conf={record['conf']:.3f})")
    
    # 🆕 显示滤波信息
    if record.get("filter_used"):
        logger.info(f"[find] Filter: score={record.get('filter_score', 0):.3f}, "
                   f"method={record.get('filter_method')}")
    else:
        logger.info(f"[find] Multi-frame: frames={record.get('detection_count', 1)}")
    
    return {"ok": True, "found": True, "blackboard_key": save_as, "record": record}

# -------- replan --------
def execute_find_with_llm_replanning(node: Any, robot_name: str, params: Dict[str, Any], **ctx) -> Dict[str, Any]:
    # execute find and get result
    basic_result = execute_find(node, robot_name, params, **ctx)
    
    if not basic_result.get("found", False):
        logger.info(f"[find] {robot_name} didn't find {params.get('item')}, no follow-up actions needed")
        return basic_result
    
    # === find item and replanning ===
    llm_replanning_fn = ctx.get("llm_replanning_fn")
    history_store = ctx.get("history_store")
    
    if not callable(llm_replanning_fn):
        logger.warning("[find] No LLM replanning function provided, falling back to simple mode")
        return execute_find_with_simple_replanning(node, robot_name, params, **ctx)
    
    # === 准备上下文信息 ===
    item = params.get("item", "cup")
    blackboard_key = basic_result.get("blackboard_key")
    record = basic_result.get("record", {})
    
    # 构建发现物体的上下文
    discovery_context = {
        "found_object": {
            "class": item,
            "confidence": record.get("conf", 0.0),
            "position": record.get("center_xy", [0, 0]),
            "blackboard_key": blackboard_key,
            "detection_quality": {
                "filter_used": record.get("filter_used", False),
                "filter_score": record.get("filter_score"),
                "detection_method": record.get("detection_method", "unknown")
            }
        },
        "robot_name": robot_name,
        "search_params": params
    }
    
    # 获取对话历史
    chat_history = []
    if history_store:
        try:
            chat_history = history_store.recent_chat_messages(max_turns=5)
        except Exception as e:
            logger.warning(f"[find] Failed to get chat history: {e}")
    
    logger.info(f"[find] Calling LLM for intelligent replanning...")
    logger.info(f"[find] Context: found {item} with conf={record.get('conf', 0):.3f}")
    
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

# --------- no llm (backup) -------
def execute_find_with_simple_replanning(node, robot_name, params, **ctx):
    # If there is no LLM, only execute find, without adding any additional actions
    logger.info(f"[find] Using simple replanning fallback — no actions will be added")
    result = execute_find(node, robot_name, params, **ctx)
    result["follow_up_tasks"] = []
    result["replanning_success"] = False
    result["replanning_source"] = "fallback"
    result["replanning_reasoning"] = "No LLM available, so no follow-up tasks planned"
    return result

# -------- LLM重规划实现 --------
def create_llm_replanning_function():
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
            
            blackboard_key = found_object["blackboard_key"]

            system_prompt = dedent(f"""
            You are an intelligent robot task planner. A robot has just found an object and you need to decide what to do next.

            Robot: {get_robot_id()}
            Master: {get_master_name()}
            Found Object: {found_object["class"]} (confidence: {found_object["confidence"]:.3f})
            Detection Quality: {found_object["detection_quality"]}
            Blackboard Key: {blackboard_key}

            Based on the conversation history, decide if the robot should:
            1. Just report finding the object (no further action)
            2. pickup the object
            3. pickup and deliver the object to someone
            4. Other specific actions

            ========================
            Action Definitions
            ========================
            1) navigate
            - To a named target: {{"target": "<target_name>"}}

            2) pickup
            - Pick up an item: {{"item": "<item>"}}

            3) dropoff
            - Drop off an item: {{"item": "<item>"}}

            ========================
            Mandatory Planning Rules
            ========================
            - If the user asks to "bring"/"deliver" the found item to someone (e.g., the master), the REQUIRED sequence is:
            A) navigate to the OBJECT location
                - Use the provided Blackboard Key as the named target:
            B) pickup the item
            C) navigate to the RECIPIENT (person/location)
                - For "bring to me", use the master name {get_master_name()}:
                {{"action": "navigate", "parameters": {{"target": "{get_master_name()}"}}}}
            D) dropoff the item

            - If the user only says "collect <item>" (no delivery target), the sequence is:
            navigate (to object via blackboard key) -> pickup

            - If the user only asked to find the object (no collect/deliver), then just report:
            action_needed = false, tasks = []

            - If the object was not found, do not plan pickup/navigate/dropoff:
            action_needed = false

            - Do NOT invent extra actions. Keep parameters EXACTLY as defined above.
            - Output strict JSON:

            {{
            "action_needed": true/false,
            "tasks": [
                {{
                "action": "pickup|navigate|dropoff|wait",
                "parameters": {{ ... }}
                }}
            ],
            "reasoning": "Brief explanation"
            }}

            When the user says "bring to me", use "{get_master_name()}" as the recipient target.
            """).strip()


            # 构建对话历史字符串
            history_text = ""
            if chat_history:
                history_text = "Recent conversation:\n"
                for msg in chat_history[-6:]:
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
def run_find(node: Any, task: Dict[str, Any], **ctx) -> Dict[str, Any]:
    """与 robot_controller 的单步接口保持一致"""
    robot = task.get("robot") or task.get("parameters", {}).get("robot")
    if not robot:
        return {"ok": False, "reason": "robot name missing"}
    
    # 得到LLM重规划版本
    replanning_result = execute_find_with_llm_replanning(node, robot, task.get("parameters", {}), **ctx)

    if not replanning_result.get("found", False):
        return replanning_result
    
    follow_ups = replanning_result.get("follow_up_tasks", [])
    if follow_ups:
        logger.info(f"[find] Executing {len(follow_ups)} follow-up tasks for {robot}")
        tts_manager.say_sync("Now executing follow-up tasks")

        for i, sub_task in enumerate(follow_ups, 1):
            action = sub_task["action"]
            parameters = sub_task["parameters"]
            logger.info(f"[find] 📋 Follow-up {i}/{len(follow_ups)} → action={action} parameters={parameters}")

            try:
                if action == "navigate":
                    target = parameters.get("target")
                    if target is not None:
                        logger.info(f"[find] 🚗 Navigating to {target}")
                        navigate_after_follow(node, robot, target)

                elif action == "pickup":
                    item = parameters.get("item")
                    if item:
                        logger.info(f"[find] ✋ Pickup: {item}")
                        success = pickup_item(item)
                        if not success:
                            logger.warning(f"[find] ❌ Pickup failed")
                    else:
                        logger.warning(f"[find] ⚠️ Pickup missing item name")

                elif action == "dropoff":
                    item = parameters.get("item")
                    if item:
                        logger.info(f"[find] 📦 Dropoff: {item}")
                        success = dropoff_item(item)
                        if not success:
                            logger.warning(f"[find] ❌ Dropoff failed")
                    else:
                        logger.warning(f"[find] ⚠️ Dropoff missing item name")

                else:
                    logger.warning(f"[find] ⚠️ Unsupported follow-up action: {action}")

            except Exception as e:
                logger.exception(f"[find] ❌ Error during follow-up task: {e}")

        tts_manager.say_sync("All follow-up tasks completed")

    return replanning_result

