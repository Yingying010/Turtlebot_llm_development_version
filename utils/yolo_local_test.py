#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys, time, json, subprocess
from typing import Any, Dict, List, Optional
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
DEFAULT_GST_PIPELINE = (
    "libcamerasrc ! video/x-raw,width=640,height=480,framerate=15/1 ! "
    "videoconvert ! video/x-raw,format=BGR ! appsink"
)

def debug_print(msg: str):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}")

def try_capture_frame_from_gstreamer() -> Optional[np.ndarray]:
    debug_print("尝试用 GStreamer 打开视频源...")
    cap = cv2.VideoCapture(DEFAULT_GST_PIPELINE, cv2.CAP_GSTREAMER)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    deadline = time.time() + 10
    while time.time() < deadline:
        ok, frame = cap.read()
        if ok and frame is not None and frame.mean() > 1:
            debug_print("✅ 成功从 GStreamer 视频流读取一帧")
            cap.release()
            return frame
        time.sleep(0.1)

    debug_print("❌ GStreamer 视频流读取失败")
    cap.release()
    return None

def capture_frame_with_rpicam_still(path="/tmp/frame.jpg") -> Optional[np.ndarray]:
    debug_print("📸 尝试用 rpicam-still 拍照...")
    try:
        subprocess.run(["rpicam-still", "-t", "1000", "-o", path], check=True)
        if os.path.exists(path):
            img = cv2.imread(path)
            if img is not None:
                debug_print("✅ 拍照成功并读取图像")
                return img
    except Exception as e:
        debug_print(f"❌ rpicam-still 拍照失败：{e}")
    return None

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
    debug_print(f"Python: {sys.executable}")
    debug_print(f"OpenCV: {cv2.__version__}")
    debug_print(f"YOLO: {YOLO_WEIGHTS}, DEVICE: {YOLO_DEVICE}")

    frame = try_capture_frame_from_gstreamer()
    if frame is None:
        frame = capture_frame_with_rpicam_still()

    if frame is None:
        debug_print("🚫 无法获取图像，退出")
        sys.exit(1)

    model = YOLO(YOLO_WEIGHTS)
    res = model.predict(source=frame, conf=CONF, iou=IOU, device=YOLO_DEVICE, verbose=False)[0]
    annotated = res.plot()
    h, w = res.orig_shape

    detections = []
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

    ts = time.strftime("%Y%m%d-%H%M%S")
    cv2.imwrite(f"/tmp/yolo_fallback_raw_{ts}.jpg", frame)
    cv2.imwrite(f"/tmp/yolo_fallback_annotated_{ts}.jpg", annotated)

    out = {"perception_report": perception_report}
    print(json.dumps(out, ensure_ascii=False, indent=2))

if __name__ == "__main__":
    main()
