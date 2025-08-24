# find.py
# -*- coding: utf-8 -*-
import os, sys, time, math, json
from typing import Any, Dict, List, Optional, Callable

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

import cv2
import subprocess
from loguru import logger
from ttsRepo.stream_tts import tts_manager
from robotControllerRepo.actions.navigate import navigate_after_follow
from robotControllerRepo.actions.pickup import pickup_item
from robotControllerRepo.actions.dropoff import dropoff_item
# from llmParserRepo.yolo_perception import detect_once as yolo_detect_once  # ← 已改为本文件内YOLO链路
import config
from geometry_msgs.msg import Twist

# ========= 工具 =========
def get_robot_id() -> str:
    return config.get("robot_id")

def get_master_name() -> str:
    return config.get("master_id")

def _bb_path(robot_name: str) -> str:
    return f"/tmp/robot_blackboard_{robot_name}.json"

def _bb_read(robot_name: str) -> Dict[str, Any]:
    p = _bb_path(robot_name)
    if os.path.exists(p):
        try:
            with open(p, "r") as f:
                return json.load(f)
        except Exception as e:
            logger.warning(f"[find] Blackboard read failed: {e}")
    return {}

def _bb_write(robot_name: str, data: Dict[str, Any]) -> None:
    p = _bb_path(robot_name)
    try:
        with open(p, "w") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
    except Exception as e:
        logger.warning(f"[find] Blackboard write failed: {e}")

def bb_set(robot_name: str, key: str, value: Dict[str, Any]) -> None:
    db = _bb_read(robot_name)
    db.setdefault("objects", {})[key] = value
    _bb_write(robot_name, db)

# === 历史记录（从 memory.chattinglog.json 读取，NDJSON 每行一个 JSON） ===
_HISTORY_PATH = "memory/chattinglog.jsonl"

def get_last_user_message_from_jsonlog(json_path="memory.chattinglog.json") -> str:
    try:
        with open(json_path, "r", encoding="utf-8") as f:
            lines = f.readlines()
        for line in reversed(lines):
            data = json.loads(line.strip())
            if data.get("type") == "user":
                return data.get("content", "")
    except Exception as e:
        print(f"[find.py] Failed to load user message from log: {e}")
    return ""

def _recent_chat_messages_from_file(path: str, max_turns: int = 5):
    """
    读取最后 max_turns 条 user/assistant 类型的对话，返回
    [{"role":"user|assistant","content":"..."}]
    """
    if not os.path.exists(path):
        return []

    msgs = []
    try:
        from collections import deque
        with open(path, "r", encoding="utf-8") as f:
            # 只保留最后 100 行，避免大文件内存占用；然后再挑 user/assistant
            tail = deque(f, maxlen=100)
        for line in tail:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except Exception:
                continue

            typ = obj.get("type")
            content = obj.get("content")
            if typ in ("user", "assistant"):
                # assistant 可能是 JSON 字符串，把它当纯文本传给 LLM
                if isinstance(content, dict):
                    content = json.dumps(content, ensure_ascii=False)
                msgs.append({"role": "user" if typ == "user" else "assistant",
                             "content": str(content) if content is not None else ""})
        # 只取末尾 max_turns 条
        return msgs[-max_turns:]
    except Exception as e:
        logger.warning(f"[find] read history failed: {e}")
        return []

class _FileHistoryStore:
    """只实现 recent_chat_messages，用于 execute_find_with_llm_replanning"""
    def __init__(self, path: str):
        self.path = path
    def recent_chat_messages(self, max_turns: int = 5):
        return _recent_chat_messages_from_file(self.path, max_turns=max_turns)


# -------- 简单滤波器 --------
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
    
    def filter_detections(self, detections: List[Dict], target_class: str, conf_thres: float) -> Optional[Dict]:
        """
        对单帧检测结果进行滤波
        """
        valid_detections = [
            d for d in detections 
            if d.get("class") == target_class and d.get("conf", 0) >= conf_thres
        ]
        if not valid_detections:
            return None
        
        logger.debug(f"🔍 Filter: Processing {len(valid_detections)} valid detections")
        
        if len(valid_detections) == 1:
            result = valid_detections[0].copy()
            result["filter_score"] = result.get("conf", 0)
            result["filter_method"] = "single_detection"
            return result
        
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
            
            # 1) 高置信度加分
            if conf > 0.8:
                score += self.confidence_boost * 2
            elif conf > 0.6:
                score += self.confidence_boost
            
            # 2) 中心加分
            center_distance = math.sqrt(
                (center_xy[0] - image_center[0])**2 + 
                (center_xy[1] - image_center[1])**2
            )
            center_bonus = max(0, (1 - center_distance / 200.0) * 0.1)
            score += center_bonus
            
            # 3) 面积合理性
            bbox_width = bbox_xyxy[2] - bbox_xyxy[0]
            bbox_height = bbox_xyxy[3] - bbox_xyxy[1]
            bbox_area = bbox_width * bbox_height
            if 500 < bbox_area < 50000:
                score += 0.05
            elif bbox_area < 100 or bbox_area > 100000:
                score -= 0.1
            
            # 4) 长宽比合理性
            if bbox_height > 0:
                aspect_ratio = bbox_width / bbox_height
                if 0.3 < aspect_ratio < 3.0:
                    score += 0.05
                else:
                    score -= 0.05
            
            det["filter_score"] = score
            logger.debug(f"🔍 Detection score: conf={conf:.3f}, center_bonus={center_bonus:.3f}, "
                        f"area={bbox_area:.0f}, final_score={score:.3f}")
        
        best_det = max(detections, key=lambda x: x.get("filter_score", 0))
        if best_det.get("filter_score", 0) < 0.4:
            logger.debug(f"🔍 Best detection score {best_det.get('filter_score', 0):.3f} below threshold 0.4")
            return None
        
        best_det["filter_method"] = "scored_selection"
        return best_det

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

def _create_video_capture(video_source: int = 0):
    """尝试打开摄像头（多后端），返回可用的 cap 或 None"""
    backends = [cv2.CAP_V4L2, cv2.CAP_GSTREAMER, cv2.CAP_FFMPEG, cv2.CAP_ANY]
    for be in backends:
        cap = cv2.VideoCapture(video_source, be)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            ok, frame = cap.read()
            if ok and frame is not None:
                return cap
            cap.release()
    return None

def _capture_and_detect(detector: StandaloneYOLODetector, conf_thres: float) -> List[Dict]:
    """拍照并检测，返回所有检测结果"""
    # 优先使用rpicam-still拍照（针对树莓派）
    try:
        ts = time.strftime("%Y%m%d-%H%M%S")
        img_path = f"/tmp/find_frame_{ts}.jpg"
        
        subprocess.run([
            "rpicam-still", "-t", "300",  # 快速拍照
            "--width", "640", "--height", "480",
            "-o", img_path
        ], capture_output=True, check=True, timeout=2)
        
        if os.path.exists(img_path):
            all_detections = detector.detect_objects(img_path, conf_thres)
            os.remove(img_path)  # 清理临时文件
            return all_detections
        
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired, FileNotFoundError):
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

def _detect_with_simple_filter(detector: StandaloneYOLODetector,
                              filter_obj: SimpleDetectionFilter,
                              target_class: str, 
                              conf_thres: float) -> Optional[Dict]:
    """使用简单滤波器的单帧检测"""
    if not detector.available:
        return None
    
    logger.debug(f"[find] 📷 Single-frame detection with simple filter for {target_class}")
    
    # 获取单帧检测结果（稍微放宽阈值，让滤波器打分挑选）
    all_detections = _capture_and_detect(detector, conf_thres * 0.8)
    if not all_detections:
        logger.debug(f"[find] 📷 No raw detections in current frame")
        return None
    
    # 通过简单滤波器处理
    filtered_result = filter_obj.filter_detections(all_detections, target_class, conf_thres)
    
    if filtered_result:
        logger.info(f"[find] ✅ Filtered detection: conf={filtered_result.get('conf', 0):.3f}, "
                   f"filter_score={filtered_result.get('filter_score', 0):.3f}")
        filtered_result["detection_method"] = "single_frame_filtered"
        filtered_result["filter_used"] = True
        return filtered_result
    
    logger.debug(f"[find] 🔍 Filter rejected all detections")
    return None

def _detect_with_camera_multiframe(detector: StandaloneYOLODetector, 
                                  target_class: str, 
                                  conf_thres: float,
                                  num_frames: int = 3,
                                  frame_interval: float = 0.3) -> Optional[Dict]:
    """传统多帧检测（备选方法）"""
    if not detector.available:
        return None
    
    logger.debug(f"[find] 📷 Multi-frame detection: {num_frames} frames for {target_class}")
    
    all_detections = []
    successful_frames = 0
    
    for frame_idx in range(num_frames):
        logger.debug(f"[find] 📸 Capturing frame {frame_idx + 1}/{num_frames}")
        
        frame_detections = _capture_and_detect(detector, conf_thres)
        targets = [d for d in frame_detections if d.get("class") == target_class and d.get("conf", 0) >= conf_thres]
        
        if targets:
            for t in targets:
                t["frame_idx"] = frame_idx
            all_detections.extend(targets)
            successful_frames += 1
            
        if frame_idx < num_frames - 1:
            time.sleep(frame_interval)
    
    logger.info(f"[find] 📊 Multi-frame: {successful_frames}/{num_frames} frames, {len(all_detections)} detections")
    
    if not all_detections:
        return None
    
    best_detection = max(all_detections, key=lambda x: x.get("conf", 0))
    best_detection["detection_method"] = "multiframe"
    best_detection["detection_count"] = len(all_detections)
    best_detection["successful_frames"] = successful_frames
    return best_detection

def _detect_with_camera(detector: StandaloneYOLODetector,
                       simple_filter: SimpleDetectionFilter,
                       target_class: str, 
                       conf_thres: float,
                       use_filter: bool = True,
                       num_frames: int = 1) -> Optional[Dict]:
    """检测方法选择器"""
    if use_filter and num_frames == 1:
        logger.debug(f"[find] Using simple filtered single-frame detection")
        return _detect_with_simple_filter(detector, simple_filter, target_class, conf_thres)
    else:
        logger.debug(f"[find] Using traditional multi-frame detection ({num_frames} frames)")
        return _detect_with_camera_multiframe(detector, target_class, conf_thres, num_frames)



# ========= 参数优化 =========
def get_optimized_params(item: str) -> float:
    if item in ["cup", "bottle", "phone", "remote", "keys"]:
        return 0.4
    if item in ["person", "chair", "table", "sofa", "bed"]:
        return 0.6
    if item in ["book", "laptop", "mouse", "keyboard"]:
        return 0.5
    return 0.5

# ========= 轻量旋转（直接 cmd_vel） =========
_pub_cache: Dict[str, Any] = {}

def _get_cmd_vel_pub(node: Any, robot_name: str):
    """
    懒加载并缓存 /<robot>/cmd_vel publisher
    """
    if Twist is None:
        return None

    key = f"{robot_name}:/cmd_vel"
    pub = _pub_cache.get(key)
    if pub:
        return pub
    try:
        topic = f"/{robot_name}/cmd_vel"
        pub = node.create_publisher(Twist, topic, 10)
        _pub_cache[key] = pub
        logger.info(f"[find] ✅ Created publisher: {topic}")
        return pub
    except Exception as e:
        logger.error(f"[find] ❌ create_publisher failed: {e}")
        return None

def rotate_by_deg(node: Any,
                  robot_name: str,
                  delta_deg: float,
                  angular_speed_deg_s: float = None) -> bool:
    """
    以固定角速度转动指定角度（时间法）
    - 正角度：左转（+z）
    - 负角度：右转（-z）
    """
    if Twist is None:
        logger.error("[find] Twist unavailable, cannot rotate")
        return False

    if abs(delta_deg) < 1e-3:
        return True

    # 允许通过 config 覆盖默认角速度
    if angular_speed_deg_s is None:
        angular_speed_deg_s = float(config.get("find_rotate_speed_deg_s", 25.0))

    pub = _get_cmd_vel_pub(node, robot_name)
    if pub is None:
        return False

    duration = abs(delta_deg) / max(angular_speed_deg_s, 1e-6)
    wz = (angular_speed_deg_s * math.pi / 180.0) * (1.0 if delta_deg >= 0 else -1.0)

    tw = Twist()
    tw.angular.z = wz

    start = time.time()
    rate = 0.02  # 50Hz
    logger.info(f"[find] 🔄 rotate_by_deg: {delta_deg:.1f}°, ω={angular_speed_deg_s:.1f}°/s, t={duration:.2f}s")
    try:
        while time.time() - start < duration:
            pub.publish(tw)
            time.sleep(rate)
    finally:
        # 停止
        tw_stop = Twist()
        pub.publish(tw_stop)
    return True

def _rotate_scan_stepwise(node: Any,
                          robot_name: str,
                          detect_fn: Callable[[str, float], Optional[Dict]],
                          item: str,
                          *,
                          max_rot_deg: int = 360,
                          step_deg: int = 30,
                          pause: float = 0.5,
                          conf_thres: float = 0.5) -> Optional[Dict]:
    """
    分步旋转 + 每步检测（轻量）
    """
    logger.info(f"[find] 🔄 stepwise scan: max={max_rot_deg}°, step={step_deg}°, pause={pause}s")

    # 先看当前视角
    hit = detect_fn(item, conf_thres)
    if hit:
        logger.info(f"[find] 🎯 Found {item} in current view")
        return hit

    turned = 0
    while turned < max_rot_deg:
        if not rotate_by_deg(node, robot_name, step_deg):
            logger.error("[find] rotate_by_deg failed, abort scan")
            break
        time.sleep(pause)
        hit = detect_fn(item, conf_thres)
        if hit:
            logger.info(f"[find] 🎯 Found {item} after ~{turned+step_deg}° rotation")
            return hit
        turned += step_deg

    logger.info("[find] 🛑 scan finished, no hit")
    return None

# ========= 单帧检测 =========
_DETECTOR_SINGLETON: Optional[StandaloneYOLODetector] = None
_FILTER_SINGLETON: Optional[SimpleDetectionFilter] = None

def _get_detector() -> StandaloneYOLODetector:
    global _DETECTOR_SINGLETON
    if _DETECTOR_SINGLETON is None:
        _DETECTOR_SINGLETON = StandaloneYOLODetector()
    return _DETECTOR_SINGLETON

def _get_filter() -> SimpleDetectionFilter:
    global _FILTER_SINGLETON
    if _FILTER_SINGLETON is None:
        _FILTER_SINGLETON = SimpleDetectionFilter(confidence_boost=0.1)
    return _FILTER_SINGLETON

def _single_frame_detect(item: str, conf_thres: float) -> Optional[Dict]:
    """
    使用 StandaloneYOLODetector 拍一帧并通过 SimpleDetectionFilter 选出最佳目标。
    （取代 yolo_perception.detect_once 方案）
    """
    detector = _get_detector()
    if not detector.available:
        logger.error("❌ YOLO detector unavailable")
        return None

    # 直接走“单帧 + 简单滤波”的检测路径
    hit = _detect_with_simple_filter(
        detector=detector,
        filter_obj=_get_filter(),
        target_class=item,
        conf_thres=conf_thres
    )

    # 调试日志：若没命中，打印这一帧 top5（原始类名，便于排查）
    if hit is None:
        dets = _capture_and_detect(detector, conf_thres * 0.8) or []
        if dets:
            top = sorted(dets, key=lambda d: d.get("conf", 0.0), reverse=True)[:5]
            logger.info("[find] det top5: " + ", ".join(
                f"{d.get('class','?')}@{d.get('conf',0.0):.2f}" for d in top
            ))
    else:
        hit["detection_method"] = "single_frame_filtered"

    return hit


# ========= find（仅三参，无 ctx） =========
def execute_find(node: Any, robot_name: str, item: str) -> Dict[str, Any]:
    if not item:
        return {"ok": False, "found": False, "reason": "no item"}

    conf_thres = get_optimized_params(item)
    logger.info(f"[find] robot={robot_name} target={item} conf>={conf_thres}")

    # ---- 配置开关（可在 config 里覆盖）----
    use_rotate_scan: bool = bool(config.get("find_use_rotate_scan", True))
    rotate_step_deg: int  = int(config.get("find_rotate_step_deg", 30))
    rotate_max_deg: int   = int(config.get("find_rotate_max_deg", 360))
    rotate_pause_s: float = float(config.get("find_rotate_pause_s", 0.5))
    # 多点搜索目标（命名地点或语义地点）
    waypoints: List[str] = list(config.get("find_waypoints", []))  # 例如 ["table", "desk", "sofa"]

    # ========== 1) 当前视角 ==========
    hit = _single_frame_detect(item, conf_thres)
    if hit:
        return _on_found(robot_name, item, hit)

    # ========== 2) 旋转扫描 ==========
    if use_rotate_scan and Twist is not None:
        logger.info("[find] 🔄 No hit → start rotate scan")
        hit = _rotate_scan_stepwise(
            node=node,
            robot_name=robot_name,
            detect_fn=lambda i, th: _single_frame_detect(i, th),
            item=item,
            max_rot_deg=rotate_max_deg,
            step_deg=rotate_step_deg,
            pause=rotate_pause_s,
            conf_thres=conf_thres
        )
        if hit:
            return _on_found(robot_name, item, hit)
    else:
        if Twist is None:
            logger.warning("[find] ⚠️ Twist unavailable → skip rotate scan")
        else:
            logger.info("[find] ℹ️ Rotate scan disabled by config")

    # ========== 3) 多点搜索 ==========
    if waypoints:
        logger.info(f"[find] 🗺️ Start waypoint search: {waypoints}")
        for idx, target in enumerate(waypoints, 1):
            try:
                tts_manager.say(f"Moving to {target}")
                navigate_after_follow(node, robot_name, target)
            except Exception as e:
                logger.warning(f"[find] navigate_after_follow({target}) error: {e}")
                continue

            time.sleep(1.0)  # 稳定一下
            # 3.1 该点单帧检测
            hit = _single_frame_detect(item, conf_thres)
            if hit:
                return _on_found(robot_name, item, hit)

            # 3.2 该点半圈扫描（提高召回）
            if use_rotate_scan and Twist is not None:
                logger.info(f"[find] 🔄 Half-scan at waypoint {target}")
                hit = _rotate_scan_stepwise(
                    node=node,
                    robot_name=robot_name,
                    detect_fn=lambda i, th: _single_frame_detect(i, th),
                    item=item,
                    max_rot_deg=180,
                    step_deg=max(rotate_step_deg, 30),
                    pause=rotate_pause_s,
                    conf_thres=conf_thres
                )
                if hit:
                    return _on_found(robot_name, item, hit)

    # ========== 未找到 ==========
    tts_manager.say_sync("I couldn't find it.")
    logger.info(f"[find] not found: {item}")
    return {"ok": True, "found": False, "blackboard_key": item}

def _on_found(robot_name: str, item: str, hit: Dict[str, Any]) -> Dict[str, Any]:
    record = {
        "class": item,
        "center_xy": hit.get("center_xy"),
        "bbox_xyxy": hit.get("bbox_xyxy"),
        "map_xy": hit.get("map_xy") if "map_xy" in hit else None,
        "conf": float(hit.get("conf", 0.0)),
        "timestamp": time.time(),
        "filter_used": True,
        "filter_score": hit.get("filter_score"),
        "filter_method": hit.get("filter_method", "single_frame_simple"),
        "detection_method": hit.get("detection_method", "single_frame_filtered"),
        "detection_count": 1,
        "successful_frames": 1,
    }
    bb_set(robot_name, item, record)
    tts_manager.say_sync(f"I found the {item}.")
    logger.info(f"[find] ✅ found {item} (key={item}, conf={record['conf']:.3f})")
    return {"ok": True, "found": True, "blackboard_key": item, "record": record}

# ========= LLM 重规划（与之前相同） =========
# find.py
# ...前面保持不变，略...

# ========= LLM 重规划（已改支持 original_user_command ）=========
def create_llm_replanning_function() -> Callable[[Dict, List[Dict], str, str], Dict]:
    def llm_replanning_function(
        discovery_context: Dict,
        chat_history: List[Dict],
        robot_name: str,
        original_user_command: str
    ) -> Dict:
        try:
            import os, json
            import openai
            from textwrap import dedent

            api_key = os.getenv("OPENAI_API_KEY")
            if not api_key:
                return {"success": False, "error": "No OpenAI API key"}

            client = openai.OpenAI(api_key=api_key)
            found_object = discovery_context["found_object"]
            blackboard_key = found_object["blackboard_key"]

            system_prompt = dedent(f"""
            You are an intelligent robot task planner. A robot has just found an object and you need to decide what to do next.

            Robot: {get_robot_id()}
            Master: {get_master_name()}
            Found Object: {found_object["class"]} (confidence: {found_object["confidence"]:.3f})
            Detection Quality: {found_object["detection_quality"]}
            Blackboard Key: {found_object["blackboard_key"]}

            Based on the conversation history, decide if the robot should:
            1. Just report finding the object (no further action)
            2. pickup the object 
            3. pickup and deliver the object to someone (requires navigate in between)
            4. Other specific actions

            ========================
            Action Definitions
            ========================
            1. navigate  
            - To a named target: {{"target": "<target_name>"}}  
            - To coordinates: {{"position": {{"x": <num>, "y": <num>, "heading_deg": <num_or_null>}}}}

            2. pickup  
            - Pick up an item: {{"item": "<item>"}}

            3. dropoff  
            - Drop off an item: {{"item": "<item>"}}

            ========================
            Important Rules
            ========================
            - When the user says "bring to me" or "deliver to <person>", the action sequence MUST be:
            1) navigate to the item
            2) pickup the item  
            3) navigate to the target (e.g., the master {get_master_name()} or specified person/location)  
            4) dropoff the item  

            - Do NOT skip the navigate step before dropoff.
            - The output must always follow strict JSON format:

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

            """).strip()


            history_text = "No recent conversation history available."
            if chat_history:
                history_text = "Recent conversation:\n" + "\n".join(
                    f"{m.get('role','unknown')}: {m.get('content','')}" for m in chat_history[-6:]
                )

            user_prompt = dedent(f"""
            The user originally said: \"{original_user_command}\"

            {history_text}

            The robot has successfully found a {found_object["class"]}. Based on the user request, what should it do next?
            """).strip()

            resp = client.chat.completions.create(
                model="gpt-4o",
                messages=[{"role": "system", "content": system_prompt},
                          {"role": "user", "content": user_prompt}],
                temperature=0.3,
                max_tokens=500
            )
            raw = resp.choices[0].message.content.strip()
            if "```json" in raw:
                s = raw.find("```json") + 7
                e = raw.find("```", s)
                raw = raw[s:e].strip()
            elif "{" in raw and "}" in raw:
                s = raw.find("{")
                e = raw.rfind("}") + 1
                raw = raw[s:e]

            parsed = json.loads(raw)
            tasks = []
            if parsed.get("action_needed") and isinstance(parsed.get("tasks"), list):
                for t in parsed["tasks"]:
                    if isinstance(t, dict) and "action" in t and "parameters" in t:
                        tasks.append(t)

            return {"success": True,
                    "tasks": tasks,
                    "reasoning": parsed.get("reasoning", ""),
                    "action_needed": bool(parsed.get("action_needed", False))}
        except Exception as e:
            logger.error(f"[find] LLM replanning function error: {e}")
            return {"success": False, "error": str(e)}
    return llm_replanning_function


def execute_find_with_llm_replanning(
    node: Any,
    robot_name: str,
    item: str,
    llm_replanning_fn: Optional[Callable[[Dict[str, Any], List[Dict[str, str]], str, str], Dict[str, Any]]] = None,
    history_store: Any = None
) -> Dict[str, Any]:
    basic = execute_find(node, robot_name, item)
    if not basic.get("found"):
        logger.info(f"[find] {robot_name} didn't find {item}, no follow-up actions.")
        return basic

    if llm_replanning_fn is None:
        llm_replanning_fn = create_llm_replanning_function()

    record = basic.get("record", {}) or {}
    discovery_context = {
        "found_object": {
            "class": item,
            "confidence": record.get("conf", 0.0),
            "position": record.get("center_xy", [0, 0]),
            "blackboard_key": basic.get("blackboard_key", item),
            "detection_quality": {
                "filter_used": record.get("filter_used", False),
                "filter_score": record.get("filter_score"),
                "detection_method": record.get("detection_method", "unknown"),
            }
        },
        "robot_name": robot_name
    }

    chat_history = []
    if history_store:
        try:
            chat_history = history_store.recent_chat_messages(max_turns=5)
        except Exception as e:
            logger.warning(f"[find] get chat history failed: {e}")

    original_command = get_last_user_message_from_jsonlog()
    if original_command:
        logger.info(f"[find] Loaded original user command: {original_command}")
    else:
        logger.warning("[find] No recent user command found in chat log.")

    logger.info(f"[find] Calling LLM replanning...")
    try:
        plan = llm_replanning_fn(
            discovery_context=discovery_context,
            chat_history=chat_history,
            robot_name=robot_name,
            original_user_command=original_command
        )
        if plan.get("success") and plan.get("action_needed") and plan.get("tasks"):
            basic["follow_up_tasks"] = plan["tasks"]
            basic["replanning_success"] = True
            basic["replanning_source"] = "llm"
            basic["replanning_reasoning"] = plan.get("reasoning", "")
            logger.info(f"[find] LLM generated {len(plan['tasks'])} follow-up tasks")
            return basic
        else:
            logger.warning(f"[find] LLM replanning not used or failed: {plan.get('error','no tasks')}")
    except Exception as e:
        logger.error(f"[find] LLM replanning error: {e}")

    basic["follow_up_tasks"] = []
    basic["replanning_success"] = False
    basic["replanning_source"] = "fallback"
    basic["replanning_reasoning"] = "No LLM plan available"
    return basic

# ========= 对外入口 =========
def run_find(node: Any, robot_name: str, item: str, executor) -> Dict[str, Any]:
    # 自动注入历史：只读 memory.chattinglog.json；不存在就不用历史（不兜底）
    history_store = _FileHistoryStore(_HISTORY_PATH) if os.path.exists(_HISTORY_PATH) else None

    result = execute_find_with_llm_replanning(
        node, robot_name, item,
        history_store=history_store
    )

    if not result.get("found"):
        return result

    follow_ups = result.get("follow_up_tasks", []) or []
    if not follow_ups:
        return result

    logger.info(f"[find] Executing {len(follow_ups)} follow-up tasks for {robot_name}")
    tts_manager.say_sync("Now executing follow-up tasks")

    for i, task in enumerate(follow_ups, 1):
        action = task.get("action")
        params = task.get("parameters", {}) or {}
        logger.info(f"[find] 📋 Follow-up {i}/{len(follow_ups)} → action={action} params={params}")

        try:
            if action == "navigate":
                target = params.get("target")
                if target:
                    navigate_after_follow(node, robot_name, target, executor)
                else:
                    logger.warning("[find] navigate missing target")
            elif action == "pickup":
                obj = params.get("item")
                if obj:
                    ok = pickup_item(obj)
                    if not ok:
                        logger.warning("[find] pickup failed")
                else:
                    logger.warning("[find] pickup missing item")
            elif action == "dropoff":
                obj = params.get("item")
                if obj:
                    ok = dropoff_item(obj)
                    if not ok:
                        logger.warning("[find] dropoff failed")
                else:
                    logger.warning("[find] dropoff missing item")
            else:
                logger.warning(f"[find] Unsupported follow-up action: {action}")
        except Exception as e:
            logger.exception(f"[find] Error during follow-up task: {e}")

    tts_manager.say_sync("All follow-up tasks completed")
    return result
