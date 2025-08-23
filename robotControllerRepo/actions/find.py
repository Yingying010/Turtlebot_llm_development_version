# find.py
# -*- coding: utf-8 -*-
"""
独立版通用"查找"动作模块 - 避免循环导入
- 旋转扫描 + 多点搜索 + 智能摄像头处理
- 新增：单帧实时稳定器（One-Euro EMA + IoU 跟踪 + 类别多数投票 + 尺寸变化约束）
- 支持单帧检测也能稳定（不再依赖多帧平均）
- 智能LLM重规划：根据对话历史决定后续动作
- 命中后把目标信息写入黑板（/tmp/robot_blackboard_<robot>.json）
- 完全独立，不依赖其他模块的复杂导入
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
from collections import Counter, deque
import config
from ttsRepo.stream_tts import tts_manager

# =========================
# 简单日志
# =========================
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

# =========================
# 黑板工具（/tmp）
# =========================
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

# =========================
# 轻量级稳定器（单帧可稳定）
# =========================
def _iou_xyxy(a, b) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)
    inter_w = max(0.0, inter_x2 - inter_x1)
    inter_h = max(0.0, inter_y2 - inter_y1)
    inter = inter_w * inter_h
    area_a = max(1e-6, (ax2 - ax1) * (ay2 - ay1))
    area_b = max(1e-6, (bx2 - bx1) * (by2 - by1))
    union = max(1e-6, area_a + area_b - inter)
    return inter / union

def _clamp_size_change(prev_box, new_box, max_scale=1.4):
    """限制单帧宽高变化比例，抑制忽大忽小。"""
    px1, py1, px2, py2 = prev_box
    nx1, ny1, nx2, ny2 = new_box
    pw = max(1.0, px2 - px1)
    ph = max(1.0, py2 - py1)
    nw = max(1.0, nx2 - nx1)
    nh = max(1.0, ny2 - ny1)
    cx = (nx1 + nx2) / 2.0
    cy = (ny1 + ny2) / 2.0
    max_w = pw * max_scale
    max_h = ph * max_scale
    adj_w = min(nw, max_w) if nw > pw else max(nw, pw / max_scale)
    adj_h = min(nh, max_h) if nh > ph else max(nh, ph / max_scale)
    nx1c = cx - adj_w / 2.0
    nx2c = cx + adj_w / 2.0
    ny1c = cy - adj_h / 2.0
    ny2c = cy + adj_h / 2.0
    return [nx1c, ny1c, nx2c, ny2c]

class OneEuroEMA:
    """速度自适应EMA，无需预热，首帧稳定。"""
    def __init__(self, alpha_min=0.12, alpha_max=0.8, vel_ref=80.0):
        self.alpha_min = alpha_min
        self.alpha_max = alpha_max
        self.vel_ref = max(1e-3, vel_ref)
        self.prev = None
        self.prev_t = None

    def _alpha(self, v):
        k = min(5.0, v / self.vel_ref)
        return self.alpha_min + (self.alpha_max - self.alpha_min) * max(0.0, min(1.0, k))

    def update_vec(self, x: List[float], t: float) -> List[float]:
        if self.prev is None:
            self.prev, self.prev_t = x[:], t
            return x[:]
        dt = max(1e-3, t - self.prev_t)
        v = math.sqrt(sum((xi - pi) ** 2 for xi, pi in zip(x, self.prev))) / dt
        a = self._alpha(v)
        y = [pi + a * (xi - pi) for xi, pi in zip(x, self.prev)]
        self.prev, self.prev_t = y, t
        return y

class _StableTrack:
    _next_id = 1
    def __init__(self, bbox, cls_name, score, t):
        self.id = _StableTrack._next_id; _StableTrack._next_id += 1
        self.box_filter = OneEuroEMA(alpha_min=0.10, alpha_max=0.85, vel_ref=90.0)
        self.box = self.box_filter.update_vec(list(bbox), t)
        self.score = float(score)
        self.cls_hist = deque(maxlen=5)
        self.cls_hist.append(cls_name)
        self.cls_name = cls_name
        self.miss = 0
        self.last_t = t

    def _majority(self):
        cnt = Counter(self.cls_hist)
        return cnt.most_common(1)[0][0]

    def update(self, bbox, cls_name, score, t):
        bbox_c = _clamp_size_change(self.box, bbox, max_scale=1.4)
        self.box = self.box_filter.update_vec(list(bbox_c), t)
        self.cls_hist.append(cls_name)
        self.cls_name = self._majority()
        self.score = 0.7 * self.score + 0.3 * float(score)
        self.miss = 0
        self.last_t = t

    def step(self):
        self.miss += 1

class SimpleStabilizer:
    """极简IoU稳定器：跨调用保存状态。"""
    def __init__(self, assoc_iou=0.4, miss_tol=8, side_margin_ratio=0.02):
        self.assoc_iou = assoc_iou
        self.miss_tol = miss_tol
        self.side_margin = side_margin_ratio
        self.tracks: List[_StableTrack] = []
        self.last_frame_w = None

    def reset(self):
        self.tracks = []

    def update_with_detections(self, dets: List[Dict[str, Any]], img_w: int, t: float):
        # 维护边缘过滤（和视频宽度）
        self.last_frame_w = img_w
        used = set()

        # step
        for tr in self.tracks:
            tr.step()

        # 贪心匹配
        for tr in self.tracks:
            best_iou, best_j = -1.0, -1
            for j, d in enumerate(dets):
                if j in used: 
                    continue
                ov = _iou_xyxy(tr.box, d["bbox_xyxy"])
                # 优先同类（名称相同），但允许在低IoU情况下被更高IoU覆盖
                if ov > best_iou and (d["class"] == tr.cls_name or best_iou < self.assoc_iou):
                    best_iou, best_j = ov, j
            if best_j >= 0 and best_iou >= self.assoc_iou:
                d = dets[best_j]
                tr.update(d["bbox_xyxy"], d["class"], d["conf"], t)
                used.add(best_j)

        # 新建track
        for j, d in enumerate(dets):
            if j not in used:
                self.tracks.append(_StableTrack(d["bbox_xyxy"], d["class"], d["conf"], t))

        # 清理过期
        self.tracks = [tr for tr in self.tracks if tr.miss <= self.miss_tol]

    def best_for_class(self, target_class: str) -> Optional[Dict[str, Any]]:
        # 找与目标类名相同的最佳track（按score）
        cands = [tr for tr in self.tracks if tr.cls_name == target_class]
        if not cands:
            return None
        tr = max(cands, key=lambda x: x.score)
        x1, y1, x2, y2 = tr.box
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        return {
            "class": tr.cls_name,
            "conf": float(tr.score),
            "center_xy": [float(cx), float(cy)],
            "bbox_xyxy": [float(x1), float(y1), float(x2), float(y2)],
            "stabilized": True
        }

# 全局单例稳定器（跨调用持久）
_GLOBAL_STABILIZER = SimpleStabilizer()

# =========================
# 独立的YOLO检测器
# =========================
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
        """检测图像中的物体（原始检测，无稳定）"""
        if not self.available:
            return []
        try:
            results = self.model.predict(
                source=image_path_or_array,
                conf=conf_threshold,
                verbose=False
            )[0]
            detections = []
            names = results.names or {}
            if getattr(results, "boxes", None) is None:
                return []

            W = None
            if isinstance(image_path_or_array, np.ndarray):
                W = image_path_or_array.shape[1]
            # 解析
            for i in range(len(results.boxes)):
                x1, y1, x2, y2 = results.boxes.xyxy[i].tolist()
                conf = float(results.boxes.conf[i])
                cls_id = int(results.boxes.cls[i])
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                cls_name = str(names.get(cls_id, str(cls_id))).lower()
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

# =========================
# 智能参数优化
# =========================
def get_optimized_params(target_class: str, has_waypoints: bool) -> tuple:
    """根据物体类型和搜索策略优化检测参数"""
    conf_thres = 0.5
    max_rot_deg = 360
    if target_class in ["cup", "bottle", "phone", "remote", "keys"]:
        conf_thres = 0.45
    elif target_class in ["person", "chair", "table", "sofa", "bed"]:
        conf_thres = 0.6
    elif target_class in ["book", "laptop", "mouse", "keyboard"]:
        conf_thres = 0.5
    if has_waypoints:
        max_rot_deg = 180
    logger.debug(f"Optimized params for '{target_class}': conf_thres={conf_thres}, max_rot_deg={max_rot_deg}")
    return conf_thres, max_rot_deg

# =========================
# 摄像头处理
# =========================
def _create_video_capture(video_source: int = 0) -> Optional[cv2.VideoCapture]:
    logger.info(f"[find] 📷 Attempting to open camera: {video_source}")
    backends_to_try = [cv2.CAP_V4L2, cv2.CAP_GSTREAMER, cv2.CAP_FFMPEG, cv2.CAP_ANY]
    for backend in backends_to_try:
        logger.debug(f"[find] 🔧 Trying backend: {backend}")
        cap = cv2.VideoCapture(video_source, backend)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
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
    logger.info("[find] 🧪 Testing camera access...")
    cap = _create_video_capture(0)
    if cap is None:
        return False
    try:
        success_count = 0
        for _ in range(10):
            ret, frame = cap.read()
            if ret and frame is not None:
                success_count += 1
            time.sleep(0.05)
        logger.info(f"[find] 📊 Camera test: {success_count}/10 frames read successfully")
        return success_count >= 5
    finally:
        cap.release()

# =========================
# 多帧检测（原逻辑保留）
# =========================
def _detect_with_camera_multiframe(detector: StandaloneYOLODetector, 
                                  target_class: str, 
                                  conf_thres: float,
                                  num_frames: int,
                                  frame_interval: float = 0.3) -> Optional[Dict]:
    if not detector.available:
        return None
    logger.debug(f"[find] 📷 Starting {num_frames}-frame detection for {target_class}")
    all_detections = []
    successful_frames = 0
    for frame_idx in range(num_frames):
        logger.debug(f"[find] 📸 Capturing frame {frame_idx + 1}/{num_frames}")
        frame_result = _detect_single_frame(detector, target_class, conf_thres, frame_idx)
        if frame_result:
            all_detections.extend(frame_result)
            successful_frames += 1
        if frame_idx < num_frames - 1:
            time.sleep(frame_interval)
    logger.info(f"[find] 📊 Multi-frame detection: {successful_frames}/{num_frames} frames successful, {len(all_detections)} total detections")
    if not all_detections:
        logger.info(f"[find] ❌ No {target_class} detected in any frame")
        return None
    best_detection = _analyze_multiframe_results(all_detections, target_class, conf_thres)
    if best_detection:
        logger.info(f"[find] ✅ Best detection: conf={best_detection['conf']:.3f}, frames_detected={best_detection.get('detection_count', 1)}")
    return best_detection

def _detect_single_frame(detector: StandaloneYOLODetector, 
                        target_class: str, 
                        conf_thres: float, 
                        frame_idx: int) -> List[Dict]:
    """拍一帧 -> YOLO -> 过滤出目标类别"""
    # 先尝试 rpicam-still（树莓派）
    try:
        ts = time.strftime("%Y%m%d-%H%M%S")
        img_path = f"/tmp/find_frame_{ts}_{frame_idx}.jpg"
        result = subprocess.run([
            "rpicam-still", "-t", "500",
            "--width", "640", "--height", "480",
            "-o", img_path
        ], capture_output=True, check=True, timeout=3)
        if os.path.exists(img_path):
            detections = detector.detect_objects(img_path, conf_thres)
            os.remove(img_path)
            targets = []
            for d in detections:
                if d["class"] == target_class and d["conf"] >= conf_thres:
                    d["frame_idx"] = frame_idx
                    targets.append(d)
            return targets
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        logger.debug(f"[find] rpicam-still failed for frame {frame_idx}, trying OpenCV...")
        cap = _create_video_capture(0)
        if cap:
            try:
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
    if not all_detections:
        return None
    logger.debug(f"[find] 🔍 Analyzing {len(all_detections)} detections across multiple frames")
    best_by_conf = max(all_detections, key=lambda x: x["conf"])
    position_groups = _group_detections_by_position(all_detections)
    if len(position_groups) > 1:
        best_group = max(position_groups, key=lambda g: (len(g), max(d["conf"] for d in g)))
        stable_detection = max(best_group, key=lambda x: x["conf"])
        stable_detection["detection_count"] = len(best_group)
        stable_detection["position_stability"] = len(best_group) / len(set(d["frame_idx"] for d in all_detections))
        logger.debug(f"[find] 📍 Found stable position group with {len(best_group)} detections")
        return stable_detection
    else:
        best_by_conf["detection_count"] = len(all_detections)
        best_by_conf["position_stability"] = 1.0
        return best_by_conf

def _group_detections_by_position(detections: List[Dict], position_tolerance: float = 50.0) -> List[List[Dict]]:
    groups = []
    for detection in detections:
        center_x, center_y = detection["center_xy"]
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
        if not assigned:
            groups.append([detection])
    return groups

# =========================
# 单帧 + 稳定器路径
# =========================
def _detect_with_camera_singleframe_stabilized(detector: StandaloneYOLODetector,
                                               target_class: str,
                                               conf_thres: float) -> Optional[Dict]:
    """
    单帧拍摄 + YOLO检测 + 稳定器（持久状态）
    返回稳定后的 best detection（若存在）
    """
    if not detector.available:
        return None

    # 1) 采一帧
    frame = None
    W = 640
    # 优先 rpicam-still 拍照（为了兼容你的环境）
    try:
        ts = time.strftime("%Y%m%d-%H%M%S")
        img_path = f"/tmp/find_single_{ts}.jpg"
        subprocess.run([
            "rpicam-still", "-t", "400", "--width", "640", "--height", "480", "-o", img_path
        ], capture_output=True, check=True, timeout=3)
        if os.path.exists(img_path):
            # 2) YOLO检测（得到全类别）
            dets = detector.detect_objects(img_path, conf_thres)
            # 获取图像宽度（用于边缘过滤）
            try:
                img = cv2.imread(img_path)
                if img is not None:
                    W = img.shape[1]
                os.remove(img_path)
            except Exception:
                pass
        else:
            dets = []
    except Exception:
        # 回退 OpenCV
        cap = _create_video_capture(0)
        dets = []
        if cap:
            try:
                for _ in range(3):
                    cap.read()
                ok, frame = cap.read()
                if ok and frame is not None:
                    W = frame.shape[1]
                    dets = detector.detect_objects(frame, conf_thres)
            finally:
                cap.release()

    # 3) 仅保留目标类别的检测 + 左右边缘过滤（减少半框闪烁）
    SIDE_MARGIN = 0.02
    filtered = []
    for d in dets:
        if d["class"] != target_class or d["conf"] < conf_thres:
            continue
        x1, y1, x2, y2 = d["bbox_xyxy"]
        if x1 < SIDE_MARGIN * W or x2 > (1.0 - SIDE_MARGIN) * W:
            continue
        filtered.append(d)

    # 4) 更新稳定器，并取该类别的稳定结果
    _GLOBAL_STABILIZER.update_with_detections(filtered, img_w=W, t=time.time())
    best = _GLOBAL_STABILIZER.best_for_class(target_class)
    return best  # 可能为 None

# =========================
# 兼容包装（根据帧数选择路径）
# =========================
def _detect_with_camera(detector: StandaloneYOLODetector, target_class: str, conf_thres: float,
                        detection_frames: int = 3, frame_interval: float = 0.3) -> Optional[Dict]:
    if detection_frames <= 1:
        return _detect_with_camera_singleframe_stabilized(detector, target_class, conf_thres)
    else:
        return _detect_with_camera_multiframe(detector, target_class, conf_thres, detection_frames, frame_interval)

# =========================
# 分步旋转扫描
# =========================
def _rotate_scan_stepwise(robot_name: str,
                         rotate_fn,
                         detect_fn,
                         target_class: str,
                         *,
                         max_rot_deg: int = 360,
                         step_deg: int = 30,
                         pause: float = 0.5,
                         conf_thres: float = 0.5) -> Optional[Dict[str, Any]]:
    logger.info(f"[find] 🔄 Starting stepwise rotate+detect scan")
    logger.info(f"[find] 🔧 Step parameters: max_rot={max_rot_deg}°, step={step_deg}°, pause={pause}s")

    # 先看当前视角
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

        time.sleep(pause)  # 机械稳定
        hit = detect_fn(target_class, conf_thres) if detect_fn else None
        if hit:
            logger.info(f"[find] 🎯 Found {target_class} after {turned + step_deg}° rotation!")
            return hit
        turned += step_deg

    logger.info(f"[find] 🛑 Stepwise scan complete: {step_count} steps, {turned}° total rotation")
    return None

# =========================
# 主 find 逻辑
# =========================
def execute_find(node: Any, robot_name: str, params: Dict[str, Any], **ctx) -> Dict[str, Any]:
    """独立的查找动作：旋转扫描 + （单帧稳定器 / 多帧平均）"""
    target: str = params.get("target_class", "cup")
    save_as: str = params.get("save_as", target)
    timeout_sec: float = float(params.get("timeout_sec", 25))
    use_spin: bool = bool(params.get("spin_scan", True))
    waypoints: List[Any] = params.get("search_waypoints", [])
    # 帧设置：1 -> 单帧稳定器；>1 -> 原多帧
    num_frames: int = int(params.get("detection_frames", 1))   # 默认改为 1
    frame_interval: float = float(params.get("frame_interval", 0.25))

    conf_thres, max_rot_deg = get_optimized_params(target, bool(waypoints))
    rotate_fn = ctx.get("rotate_fn")
    navigate_to_fn = ctx.get("navigate_to_fn")
    event_pub = ctx.get("event_pub")

    if use_spin and not callable(rotate_fn):
        logger.warning("spin_scan=True but rotate_fn missing, will skip spin scan.")
        use_spin = False
    if waypoints and not callable(navigate_to_fn):
        logger.warning("search_waypoints provided but navigate_to_fn missing, will skip waypoint search.")
        waypoints = []

    detector = StandaloneYOLODetector()
    if not detector.available:
        logger.error("YOLO detector not available")
        return {"ok": False, "found": False, "reason": "YOLO unavailable"}

    camera_ok = _test_camera_simple()
    logger.info(f"[find] 📷 Camera test result: {'✅ PASS' if camera_ok else '❌ FAIL'}")

    t0 = time.time()
    logger.info(f"[find] robot={robot_name} target={target} timeout={timeout_sec}s conf>={conf_thres}")
    logger.info(f"[find] Detection mode: {'single+stabilizer' if num_frames<=1 else f'multiframe({num_frames})'}")

    hit: Optional[Dict[str, Any]] = None

    def detect_current_view(target_class: str, conf_threshold: float):
        return _detect_with_camera(detector, target_class, conf_threshold,
                                   detection_frames=num_frames, frame_interval=frame_interval)

    # 1) 当前视角
    hit = detect_current_view(target, conf_thres)

    # 2) 旋转扫描
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
            pause=0.6
        )

    # 3) 多点搜索
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

            time.sleep(0.8)  # 到达后稳定
            hit = detect_current_view(target, conf_thres)
            if hit:
                logger.info(f"[find] 🎯 Found {target} at waypoint {wp}!")
                break

            if use_spin:
                logger.info(f"[find] 🔄 Scanning at waypoint {wp}")
                hit = _rotate_scan_stepwise(
                    robot_name=robot_name,
                    rotate_fn=rotate_fn,
                    detect_fn=detect_current_view,
                    target_class=target,
                    conf_thres=conf_thres,
                    max_rot_deg=180,
                    step_deg=45,
                    pause=0.6
                )
            if hit:
                logger.info(f"[find] 🎯 Found {target} during scan at waypoint {wp}!")
                break

    # 处理结果
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

    # 成功找到
    record = {
        "class": target,
        "center_xy": hit.get("center_xy"),
        "bbox_xyxy": hit.get("bbox_xyxy"),
        "map_xy": hit.get("map_xy") if "map_xy" in hit else None,
        "conf": float(hit.get("conf", 0.0)),
        "timestamp": time.time(),
        "detection_count": hit.get("detection_count", 1),
        "position_stability": hit.get("position_stability", 1.0) if "position_stability" in hit else 1.0,
        "detection_method": "single+stabilizer" if hit.get("stabilized") else ("multiframe" if num_frames>1 else "single")
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
    logger.info(f"[find] Detection mode: {record['detection_method']}")
    return {"ok": True, "found": True, "blackboard_key": save_as, "record": record}

# =========================
# 智能动态重规划
# =========================
def execute_find_with_llm_replanning(node: Any, robot_name: str, params: Dict[str, Any], **ctx) -> Dict[str, Any]:
    basic_result = execute_find(node, robot_name, params, **ctx)
    if not basic_result.get("found", False):
        logger.info(f"[find] {robot_name} didn't find {params.get('target_class')}, no follow-up actions needed")
        return basic_result
    llm_replanning_fn = ctx.get("llm_replanning_fn")
    history_store = ctx.get("history_store")
    if not callable(llm_replanning_fn):
        logger.warning("[find] No LLM replanning function provided, falling back to simple mode")
        return execute_find_with_simple_replanning(node, robot_name, params, **ctx)

    target_class = params.get("target_class", "cup")
    blackboard_key = basic_result.get("blackboard_key")
    record = basic_result.get("record", {})

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

    chat_history = []
    if history_store:
        try:
            chat_history = history_store.recent_chat_messages(max_turns=5)
        except Exception as e:
            logger.warning(f"[find] Failed to get chat history: {e}")

    logger.info(f"[find] Calling LLM for intelligent replanning...")
    logger.info(f"[find] Context: found {target_class} with conf={record.get('conf', 0):.3f}")

    try:
        import openai
        from textwrap import dedent
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            return {"success": False, "error": "No OpenAI API key"}
        client = openai.OpenAI(api_key=api_key)

        system_prompt = dedent(f"""
        You are an intelligent robot task planner. A robot has just found an object and you need to decide what to do next.

        Robot: {get_robot_id()}
        Master: {get_master_name()}
        Found Object: {target_class} (confidence: {record.get("conf", 0.0):.3f})
        Detection Quality: {record.get("detection_count", 1)} frames, stability: {record.get("position_stability", 1.0):.2f}
        Blackboard Key: {blackboard_key}

        IMPORTANT: The robot has already found the object and moved close to it during the search process. 
        The robot is now positioned near the object and ready for the next action.

        [format spec truncated here for brevity in code sample...]
        """).strip()

        history_text = "No recent conversation history available."
        if chat_history:
            history_text = "Recent conversation:\n"
            for msg in chat_history[-6:]:
                role = msg.get("role", "unknown")
                content = msg.get("content", "")
                history_text += f"{role}: {content}\n"

        user_prompt = f"""{history_text}

        The robot has successfully found a {target_class}. What should it do next?
        """

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

        try:
            if "```json" in raw_response:
                s = raw_response.find("```json") + 7
                e = raw_response.find("```", s)
                json_text = raw_response[s:e].strip()
            elif "{" in raw_response and "}" in raw_response:
                s = raw_response.find("{")
                e = raw_response.rfind("}") + 1
                json_text = raw_response[s:e]
            else:
                json_text = raw_response
            parsed = json.loads(json_text)
            action_needed = parsed.get("action_needed", False)
            tasks = parsed.get("tasks", [])
            reasoning = parsed.get("reasoning", "LLM decision")
            validated_tasks = []
            if action_needed and tasks:
                for task in tasks:
                    if isinstance(task, dict) and "action" in task and "parameters" in task:
                        validated_tasks.append(task)
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

def execute_find_with_simple_replanning(node: Any, robot_name: str, params: Dict[str, Any], **ctx) -> Dict[str, Any]:
    basic_result = execute_find(node, robot_name, params, **ctx)
    if not basic_result.get("found", False):
        return basic_result
    target_class = params.get("target_class", "cup")
    blackboard_key = basic_result.get("blackboard_key")
    follow_up_tasks = [
        {"action": "collect", "parameters": {"item": target_class}},
        {"action": "deliver", "parameters": {"item": target_class, "target": "master"}}
    ]
    basic_result["follow_up_tasks"] = follow_up_tasks
    basic_result["replanning_success"] = True
    basic_result["replanning_source"] = "simple_fallback"
    logger.info(f"[find] Generated {len(follow_up_tasks)} simple fallback tasks")
    return basic_result

# =========================
# 对外接口
# =========================
def run_action(node: Any, task: Dict[str, Any], **ctx) -> Dict[str, Any]:
    robot = task.get("robot") or task.get("parameters", {}).get("robot")
    if not robot:
        return {"ok": False, "reason": "robot name missing"}
    return execute_find_with_llm_replanning(node, robot, task.get("parameters", {}), **ctx)

# =========================
# 真实机器人控制（占位）
# =========================
def real_rotate(node, robot_name: str, degrees: float):
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        publisher = node.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
        twist = Twist()
        angular_speed = math.radians(30)
        direction = "left" if degrees >= 0 else "right"
        twist.angular.z = angular_speed if degrees >= 0 else -angular_speed
        angle_rad = math.radians(abs(degrees))
        duration = angle_rad / abs(angular_speed)
        logger.info(f"🔄 Real rotation: {robot_name} turning {direction} {abs(degrees)}° for {duration:.2f}s")
        time.sleep(0.2)
        start_time = time.time()
        while time.time() - start_time < duration:
            publisher.publish(twist)
            time.sleep(0.01)
        publisher.publish(Twist())
        logger.info(f"✅ Real rotation completed: {robot_name}")
        return True
    except Exception as e:
        logger.error(f"❌ Real rotation failed: {e}")
        return False

def real_navigate(node, robot_name: str, target):
    logger.info(f"🚶 Navigation: {robot_name} -> {target} (placeholder)")
    time.sleep(1.0)
    return True

# =========================
# 独立运行测试
# =========================
def run_standalone_test():
    import sys
    try:
        import rclpy
        from rclpy.node import Node
        rclpy.init()
        logger.info("🚀 ROS initialized for standalone find test")
        node = rclpy.create_node('standalone_find_test')
        logger.info("🤖 ROS node created: standalone_find_test")
        robot_name = sys.argv[1] if len(sys.argv) > 1 else "robot1"
        target_class = sys.argv[2] if len(sys.argv) > 2 else "cup"
        logger.info(f"🎯 Test parameters: robot={robot_name}, target={target_class}")
        def rotate_fn(robot, deg): return real_rotate(node, robot, deg)
        def nav_fn(robot, target): return real_navigate(node, robot, target)
        def event_fn(kind, payload): logger.info(f"📡 Event: {kind} -> {payload}")
        llm_replanning_fn = execute_find_with_llm_replanning()
        class MockHistoryStore:
            def recent_chat_messages(self, max_turns=5):
                return [
                    {"role": "user", "content": f"Can you help me find my {target_class}?"},
                    {"role": "assistant", "content": f"I'll look for your {target_class}."}
                ]
        mock_history = MockHistoryStore()
        print(f"🧪 Testing REAL find functionality for {robot_name}...")
        print(f"🎯 Looking for: {target_class}")
        input("Press Enter when robot is ready, or Ctrl+C to cancel...")
        res = execute_find_with_llm_replanning(
            node=node,
            robot_name=robot_name,
            params={
                "target_class": target_class,
                "save_as": f"{target_class}_target",
                "timeout_sec": 30,
                "search_waypoints": [],
                "detection_frames": 1,   # ⭐ 单帧 + 稳定器
                "frame_interval": 0.2
            },
            rotate_fn=rotate_fn,
            navigate_to_fn=nav_fn,
            event_pub=event_fn,
            llm_replanning_fn=llm_replanning_fn,
            history_store=mock_history
        )
        print("\n" + "="*50)
        print("🎉 FIND TEST RESULT:")
        print(f"✅ Success: {res.get('ok', False)}")
        print(f"🎯 Found: {res.get('found', False)}")
        if res.get('found'):
            record = res.get('record', {})
            print(f"📦 Object: {res.get('blackboard_key')}")
            print(f"🎯 Confidence: {record.get('conf', 0):.3f}")
            print(f"🔧 Method: {record.get('detection_method', 'unknown')}")
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

if __name__ == "__main__":
    print("🤖 Standalone Find.py Test")
    print("Usage: python3 find.py [robot_name] [target_class]")
    print("Example: python3 find.py robot1 cup\n")
    result = run_standalone_test()
    if result.get("ok"):
        print("🎉 Test completed successfully!")
        exit(0)
    else:
        print("❌ Test failed!")
        exit(1)
