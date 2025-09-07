#!/usr/bin/env python3
import os, sys, json, time, subprocess
import cv2
import numpy as np
from ultralytics import YOLO
from collections import Counter

YOLO_WEIGHTS = os.getenv("YOLO_WEIGHTS", "yolov8n.pt")
YOLO_DEVICE  = os.getenv("YOLO_DEVICE", "cpu")
CONF         = float(os.getenv("YOLO_CONF", "0.25"))
IOU          = float(os.getenv("YOLO_IOU", "0.45"))
TOPK         = int(os.getenv("TOPK", "20"))
IMG_PATH     = "/tmp/frame.jpg"

def capture_frame():
    print("📸 Running rpicam-still to capture image...")
    try:
        subprocess.run(["rpicam-still", "-t", "1000", "--width", "1640", "--height", "1232", "-o", IMG_PATH], check=True)

        img = cv2.imread(IMG_PATH)
        if img is None:
            raise RuntimeError("无法读取图像")
        return img
    except Exception as e:
        print("❌ 拍照失败:", e)
        sys.exit(1)

def assign_ids(dets, topk=20):
    dets = dets[:topk]
    totals = Counter(d["class"] for d in dets)
    seen = Counter()
    objs = []
    for d in dets:
        cls = d["class"]
        seen[cls] += 1
        oid = cls if totals[cls] == 1 else f"{cls}{seen[cls]}"
        objs.append((oid, cls, d))
    return objs

def main():
    print("🚀 Using YOLO:", YOLO_WEIGHTS)
    model = YOLO(YOLO_WEIGHTS)
    frame = capture_frame()

    res = model.predict(source=frame, conf=CONF, iou=IOU, device=YOLO_DEVICE, verbose=False)[0]
    h, w = res.orig_shape
    detections = []
    names = res.names

    if res.boxes is not None:
        for b in res.boxes:
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
    cv2.imwrite(f"/tmp/yolo_still_raw_{ts}.jpg", frame)
    cv2.imwrite(f"/tmp/yolo_still_annotated_{ts}.jpg", res.plot())

    print(json.dumps({"perception_report": perception_report}, indent=2, ensure_ascii=False))

if __name__ == "__main__":
    main()
