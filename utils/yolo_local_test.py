#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
yolo_test.py
抓一帧 → YOLO 推理（无任何预览窗口）→ 保存原图/标注图到 /tmp → 打印 perception_report JSON
"""

import os, sys, time, json, threading
from typing import Any, Dict, List, Tuple, Optional
from collections import Counter

import cv2
import numpy as np
from ultralytics import YOLO

# ----------------------- 日志辅助 -----------------------
def ts() -> str:
    return time.strftime("%H:%M:%S")

def log(msg: str):
    print(f"[{ts()}] {msg}", flush=True)

# ----------------------- 配置 -----------------------
YOLO_WEIGHTS = os.getenv("YOLO_WEIGHTS", "yolov8n.pt")
YOLO_DEVICE  = os.getenv("YOLO_DEVICE", "cpu")
CONF         = float(os.getenv("YOLO_CONF", "0.25"))
IOU          = float(os.getenv("YOLO_IOU", "0.45"))
TOPK         = int(os.getenv("TOPK", "20"))

DEFAULT_GST_PIPELINE = (
    "libcamerasrc ! "
    "video/x-raw,width=640,height=480,framerate=30/1 ! "
    "videoconvert ! video/x-raw,format=BGR ! appsink"
)
_video_env = os.getenv("VIDEO_SOURCE", DEFAULT_GST_PIPELINE)

if _video_env.isdigit():
    DEFAULT_SOURCE: Any = int(_video_env)
else:
    DEFAULT_SOURCE: Any = _video_env

# ----------------------- 工具函数 -----------------------
def is_gstreamer_pipeline(s: str) -> bool:
    if not isinstance(s, str):
        return False
    s_low = s.strip().lower()
    return any(k in s_low for k in ["appsink", "libcamerasrc", "v4l2src", "nvarguscamerasrc", "!"])

def backend_for_source(source: Any) -> int:
    if isinstance(source, int):
        return cv2.CAP_V4L2
    if isinstance(source, str):
        if is_gstreamer_pipeline(source):
            return cv2.CAP_GSTREAMER
        if source.startswith("v4l2://"):
            return cv2.CAP_FFMPEG
        return cv2.CAP_FFMPEG
    return 0  # auto

def backend_name(backend: int) -> str:
    names = {
        cv2.CAP_V4L2: "CAP_V4L2",
        cv2.CAP_GSTREAMER: "CAP_GSTREAMER",
        cv2.CAP_FFMPEG: "CAP_FFMPEG",
        0: "AUTO",
    }
    return names.get(backend, str(backend))

def read_image_if_file(path: str) -> Optional[np.ndarray]:
    if isinstance(path, str) and os.path.isfile(path):
        return cv2.imread(path)
    return None

# ----------------------- 视频源封装 -----------------------
class GlobalVideoSource:
    def __init__(self, source: Any):
        self.source = source
        self.backend = backend_for_source(source)
        self.cap = None
        self.lock = threading.Lock()
        self.ready = False
        self._file_image = None

    def open(self):
        with self.lock:
            if self.cap is not None:
                return

            # 文件路径：直接读一次
            if isinstance(self.source, str):
                img = read_image_if_file(self.source)
                if img is not None:
                    self.cap = "FILE_IMAGE"
                    self._file_image = img
                    self.ready = True
                    log(f"以文件模式读取：{self.source}  shape={img.shape}")
                    return

            # 打印 OpenCV / GStreamer 能力
            try:
                build_info = cv2.getBuildInformation()
                gst_ok = "GStreamer" in build_info
            except Exception:
                gst_ok = False
            log(f"OpenCV GStreamer 支持: {gst_ok}")

            log(f"准备打开视频源：{self.source}  后端={backend_name(self.backend)}")
            self.cap = cv2.VideoCapture(self.source, self.backend)

            # 某些后端没这个属性也没关系
            try:
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass

            if not self.cap or not self.cap.isOpened():
                raise RuntimeError(f"无法打开视频源：{self.source}")

            log("VideoCapture 打开成功，预热中……")
            deadline = time.time() + 12.0
            ok_cnt = 0
            tries = 0
            while time.time() < deadline and ok_cnt < 10:
                tries += 1
                ok, _ = self.cap.read()
                if ok:
                    ok_cnt += 1
                else:
                    time.sleep(0.03)
            self.ready = ok_cnt > 0
            log(f"预热完成：尝试 {tries} 次，成功 {ok_cnt} 次，ready={self.ready}")

    def read(self, drop_n=5, timeout=4.0) -> np.ndarray:
        if self.cap is None or not self.ready:
            self.open()

        if self.cap == "FILE_IMAGE":
            log("从文件图像返回一帧")
            return self._file_image.copy()

        log(f"开始读取帧：drop_n={drop_n}  timeout={timeout}s")
        for _ in range(max(0, drop_n)):
            self.cap.read()

        t0 = time.time()
        reads = 0
        while time.time() - t0 < timeout:
            reads += 1
            ok, frame = self.cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue
            h, w = frame.shape[:2]
            m = float(frame.mean()); v = float(frame.var())
            if h < 32 or w < 32 or m < 1.0 or v < 5.0:
                continue
            log(f"读取成功：{w}x{h}，read调用次数={reads}")
            return frame
        raise RuntimeError("读取视频帧超时/有效帧未到达")

    @staticmethod
    def grab_one_frame_once(source: Any, warmup_frames: int = 15, open_timeout_sec: float = 12.0) -> np.ndarray:
        # 文件路径：直接读
        if isinstance(source, str):
            img = read_image_if_file(source)
            if img is not None:
                log(f"[一次性] 文件读取成功：{source}  shape={img.shape}")
                return img

        backend = backend_for_source(source)
        log(f"[一次性] 打开视频源：{source}  后端={backend_name(backend)}")
        cap = cv2.VideoCapture(source, backend)

        t0 = time.time()
        attempts = 0
        while time.time() - t0 < open_timeout_sec:
            attempts += 1
            ok, _ = cap.read()
            if ok:
                break
            time.sleep(0.05)
        log(f"[一次性] 打开尝试次数：{attempts}")

        got = 0
        for i in range(warmup_frames):
            ok, _ = cap.read()
            if ok:
                got += 1
            else:
                time.sleep(0.02)
        log(f"[一次性] 预热帧成功数：{got}/{warmup_frames}")

        ok, frame = cap.read()
        cap.release()
        if not ok or frame is None:
            raise RuntimeError("grab_one_frame_once: 无法读取有效帧")
        log(f"[一次性] 读取成功：{frame.shape[1]}x{frame.shape[0]}")
        return frame

    def release(self):
        with self.lock:
            if self.cap not in (None, "FILE_IMAGE"):
                self.cap.release()
            self.cap = None
            self.ready = False
            log("视频源已释放")

# ----------------------- YOLO / 解析 -----------------------
def assign_ids(dets: List[Dict], topk: int = 20):
    dets = dets[:topk]
    totals = Counter(d["class"] for d in dets)
    seen   = Counter()
    objs = []
    for d in dets:
        cls = d["class"]
        seen[cls] += 1
        oid = cls if totals[cls] == 1 else f"{cls}{seen[cls]}"
        objs.append((oid, cls, d))
    return objs

def main():
    # 打印关键信息
    log(f"Python: {sys.executable}")
    log(f"OpenCV: {cv2.__version__}")
    log(f"Numpy:  {np.__version__}")
    try:
        import torch
        log(f"Torch:  {torch.__version__}")
    except Exception:
        log("Torch:  未检测到（如果 ultralytics 报错请安装 torch）")

    log(f"VIDEO_SOURCE={DEFAULT_SOURCE}")
    log(f"YOLO_WEIGHTS={YOLO_WEIGHTS}  YOLO_DEVICE={YOLO_DEVICE}  CONF={CONF}  IOU={IOU}  TOPK={TOPK}")

    source = DEFAULT_SOURCE

    # 抓一帧（先一次性直读，失败再持久句柄）
    try:
        frame = GlobalVideoSource.grab_one_frame_once(source, warmup_frames=15, open_timeout_sec=12.0)
    except Exception as e1:
        log(f"[一次性] 读取失败：{e1}  → 尝试持久句柄")
        try:
            vs = GlobalVideoSource(source)
            frame = vs.read(drop_n=5, timeout=6.0)
            vs.release()
        except Exception as e2:
            log(f"[持久句柄] 读取失败：{e2}")
            sys.stderr.write(f"[ERROR] 无法从源读取帧：{source}\n  1st: {e1}\n  2nd: {e2}\n")
            sys.exit(1)

    # YOLO 单帧推理
    log("加载 YOLO 模型中……")
    model = YOLO(YOLO_WEIGHTS)
    log("开始 YOLO 推理……")
    t0 = time.time()
    res = model.predict(source=frame, conf=CONF, iou=IOU, device=YOLO_DEVICE, verbose=False)[0]
    dt = (time.time() - t0) * 1000
    log(f"YOLO 推理完成，用时 {dt:.1f} ms")

    annotated = res.plot()
    h, w = res.orig_shape

    # 解析检测结果
    detections: List[Dict] = []
    names = res.names
    boxes = getattr(res, "boxes", None)
    if boxes is not None and len(boxes) > 0:
        for b in boxes:
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            conf = float(b.conf[0]); cls_id = int(b.cls[0])
            cx = (x1 + x2) / 2.0; cy = (y1 + y2) / 2.0
            cls_name = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else str(cls_id)
            detections.append({
                "class": cls_name.lower(),
                "conf": round(conf, 4),
                "bbox_xyxy": [float(x1), float(y1), float(x2), float(y2)],
                "center_xy": [float(cx), float(cy)]
            })

    log(f"检测结果：{len(detections)} 个  图像尺寸：{w}x{h}")

    # 构造 perception_report
    assigned = assign_ids(detections, topk=TOPK)
    objects = [{
        "id": oid,
        "class": cls,
        "center_xy": [round(float(d["center_xy"][0]), 1), round(float(d["center_xy"][1]), 1)],
        "bbox_xyxy": [round(float(v), 1) for v in d["bbox_xyxy"]],
        "conf": d["conf"],
    } for (oid, cls, d) in assigned]
    summary = dict(Counter([cls for _, cls, _ in assigned]))

    perception_report = {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "summary": summary,
        "objects": objects
    }

    # 保存两张图
    try:
        ts_now = time.strftime("%Y%m%d-%H%M%S")
        raw_p = f"/tmp/yolo_min_raw_{ts_now}.jpg"
        ann_p = f"/tmp/yolo_min_annotated_{ts_now}.jpg"
        cv2.imwrite(raw_p, frame)
        cv2.imwrite(ann_p, annotated)
        log(f"已保存：{raw_p}")
        log(f"已保存：{ann_p}")
    except Exception as e:
        log(f"保存图片失败（忽略）：{e}")

    # 打印最终 JSON
    out = {"perception_report": perception_report}
    print(json.dumps(out, ensure_ascii=False, indent=2), flush=True)
    log("完成。")

if __name__ == "__main__":
    main()
