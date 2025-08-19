#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
yolo_test.py
抓一帧 → YOLO 推理（无任何预览窗口）→ 保存原图/标注图到 /tmp → 打印 perception_report JSON

使用（推荐方案A：CSI相机 + GStreamer）：
  sudo apt update && sudo apt install -y python3-opencv \
      gstreamer1.0-tools gstreamer1.0-libav \
      gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
  export VIDEO_SOURCE='libcamerasrc ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw,format=BGR ! appsink'
  export YOLO_DEVICE=cpu
  export YOLO_WEIGHTS=yolov8n.pt
  python3 yolo_test.py

其他可选 VIDEO_SOURCE：
  export VIDEO_SOURCE="v4l2:///dev/video0"   # FFmpeg后端打开 /dev/video0
  export VIDEO_SOURCE=0                      # CAP_V4L2 打开索引 0（很多 CSI 会失败）
  export VIDEO_SOURCE=/path/to/image.jpg     # 直接单张图片测试
"""

import os, sys, time, json, threading
from typing import Any, Dict, List, Tuple, Optional
from collections import Counter

import cv2
import numpy as np
from ultralytics import YOLO

# ================== 配置 ==================
YOLO_WEIGHTS = os.getenv("YOLO_WEIGHTS", "yolov8n.pt")
YOLO_DEVICE  = os.getenv("YOLO_DEVICE", "cpu")
CONF         = float(os.getenv("YOLO_CONF", "0.25"))
IOU          = float(os.getenv("YOLO_IOU", "0.45"))
TOPK         = int(os.getenv("TOPK", "20"))

# 默认使用 GStreamer 的 libcamerasrc 管道（方案A）
DEFAULT_GST_PIPELINE = (
    "libcamerasrc ! "
    "video/x-raw,width=640,height=480,framerate=30/1 ! "
    "videoconvert ! video/x-raw,format=BGR ! appsink"
)
_video_env = os.getenv("VIDEO_SOURCE", DEFAULT_GST_PIPELINE)

# 支持数字索引、字符串管道/URL或文件路径
if _video_env.isdigit():
    DEFAULT_SOURCE: Any = int(_video_env)
else:
    DEFAULT_SOURCE: Any = _video_env


# ================== 辅助函数 ==================
def is_gstreamer_pipeline(s: str) -> bool:
    s_low = s.strip().lower()
    return any(k in s_low for k in ["appsink", "libcamerasrc", "v4l2src", "nvarguscamerasrc", "!"])

def backend_for_source(source: Any) -> int:
    if isinstance(source, int):
        return cv2.CAP_V4L2
    if isinstance(source, str):
        if is_gstreamer_pipeline(source):
            return cv2.CAP_GSTREAMER
        # 显式 v4l2 协议串走 FFmpeg
        if source.startswith("v4l2://"):
            return cv2.CAP_FFMPEG
        # 其他字符串默认 FFmpeg（文件/网络）
        return cv2.CAP_FFMPEG
    return 0  # 自动

def read_image_if_file(path: str) -> Optional[np.ndarray]:
    # 若 VIDEO_SOURCE 是现有文件路径，则直接读图
    if isinstance(path, str) and os.path.isfile(path):
        img = cv2.imread(path)
        return img
    return None


# ================== OpenCV 统一视频源 ==================
class GlobalVideoSource:
    def __init__(self, source: Any):
        self.source = source
        self.backend = backend_for_source(source)
        self.cap = None
        self.lock = threading.Lock()
        self.ready = False

    def open(self):
        with self.lock:
            if self.cap is None:
                # 文件路径直接不走 VideoCapture
                if isinstance(self.source, str):
                    img = read_image_if_file(self.source)
                    if img is not None:
                        self.cap = "FILE_IMAGE"
                        self.ready = True
                        self._file_image = img
                        return

                # GStreamer 可用性检测（仅提示）
                if isinstance(self.source, str) and is_gstreamer_pipeline(self.source):
                    try:
                        info = cv2.getBuildInformation()
                        if "GStreamer" not in info:
                            raise RuntimeError(
                                "OpenCV 未启用 GStreamer。请改用 `sudo apt install python3-opencv` 安装 apt 版OpenCV，"
                                "或切换 VIDEO_SOURCE='v4l2:///dev/video0'"
                            )
                    except Exception as e:
                        # 无 getBuildInformation 时忽略，直接尝试打开
                        pass

                self.cap = cv2.VideoCapture(self.source, self.backend)
                try:
                    self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                except Exception:
                    pass
                if not self.cap or not self.cap.isOpened():
                    raise RuntimeError(f"无法打开视频源：{self.source}")

                # 预热
                deadline = time.time() + 12.0  # libcamerasrc 冷启动略慢
                ok_cnt = 0
                while time.time() < deadline and ok_cnt < 10:
                    ok, _ = self.cap.read()
                    if ok:
                        ok_cnt += 1
                    else:
                        time.sleep(0.03)
                self.ready = ok_cnt > 0

    def read(self, drop_n=5, timeout=4.0) -> np.ndarray:
        if self.cap is None or not self.ready:
            self.open()

        # 文件模式
        if self.cap == "FILE_IMAGE":
            return self._file_image.copy()

        for _ in range(max(0, drop_n)):
            self.cap.read()

        t0 = time.time()
        while time.time() - t0 < timeout:
            ok, frame = self.cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue
            # 过滤明显坏帧
            h, w = frame.shape[:2]
            m = float(frame.mean()); v = float(frame.var())
            if h < 32 or w < 32:
                continue
            if m < 1.0 or v < 5.0:
                continue
            return frame
        raise RuntimeError("读取视频帧超时/有效帧未到达")

    @staticmethod
    def grab_one_frame_once(source: Any, warmup_frames: int = 15, open_timeout_sec: float = 12.0) -> np.ndarray:
        # 文件路径直接读
        if isinstance(source, str):
            img = read_image_if_file(source)
            if img is not None:
                return img

        backend = backend_for_source(source)
        cap = cv2.VideoCapture(source, backend)

        t0 = time.time()
        while time.time() - t0 < open_timeout_sec:
            ok, _ = cap.read()
            if ok:
                break
            time.sleep(0.05)

        got = 0
        for _ in range(warmup_frames):
            ok, _ = cap.read()
            if ok:
                got += 1
            else:
                time.sleep(0.02)

        ok, frame = cap.read()
        cap.release()
        if not ok or frame is None:
            raise RuntimeError("grab_one_frame_once: 无法读取有效帧")
        return frame

    def release(self):
        with self.lock:
            if self.cap not in (None, "FILE_IMAGE"):
                self.cap.release()
            self.cap = None
            self.ready = False


# ================== YOLO 推理 + 输出 ==================
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
    source = DEFAULT_SOURCE

    # 抓一帧（优先用一次性直读）
    try:
        frame = GlobalVideoSource.grab_one_frame_once(source, warmup_frames=15, open_timeout_sec=12.0)
    except Exception as e:
        # 退回到持久句柄再试一次
        try:
            vs = GlobalVideoSource(source)
            frame = vs.read(drop_n=5, timeout=6.0)
            vs.release()
        except Exception as e2:
            sys.stderr.write(f"[ERROR] 无法从源读取帧：{source}\n 1st: {e}\n 2nd: {e2}\n")
            sys.exit(1)

    # YOLO 单帧推理
    model = YOLO(YOLO_WEIGHTS)
    res = model.predict(source=frame, conf=CONF, iou=IOU, device=YOLO_DEVICE, verbose=False)[0]
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

    print(f"[DEBUG] det_count={len(detections)}, image=({w}x{h})")

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

    # 保存两张图（不弹窗）
    try:
        ts = time.strftime("%Y%m%d-%H%M%S")
        cv2.imwrite(f"/tmp/yolo_min_raw_{ts}.jpg", frame)
        cv2.imwrite(f"/tmp/yolo_min_annotated_{ts}.jpg", annotated)
    except Exception:
        pass

    # 打印最终 JSON
    out = {"perception_report": perception_report}
    print(json.dumps(out, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
