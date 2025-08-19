#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2, json, time
from collections import Counter
from ultralytics import YOLO

SRC = "udp://@:8888?fifo_size=1000000&overrun_nonfatal=1&buffer_size=1000000&probesize=32&analyzeduration=0"
WEIGHTS = "yolov8n.pt"
CONF = 0.10   # 低一些确保能检出
IOU  = 0.45
TOPK = 20

def grab_one_frame_once(source: str,
                        warmup_frames: int = 8,
                        open_timeout_sec: float = 8.0):
    backend = cv2.CAP_FFMPEG if isinstance(source, str) else 0
    t0 = time.time()
    cap = cv2.VideoCapture(source, backend)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass

    # 等待流真正出帧
    ok = False
    while time.time() - t0 < open_timeout_sec:
        ok, _ = cap.read()
        if ok:
            break
        time.sleep(0.05)

    # 预热：丢掉起始不稳定帧
    got = 0
    for _ in range(warmup_frames):
        ok, _ = cap.read()
        if ok: got += 1
        else:  time.sleep(0.02)

    ok, frame = cap.read()
    cap.release()
    if not ok or frame is None:
        raise RuntimeError("无法读取有效帧")
    return frame

def assign_ids(dets):
    dets = dets[:TOPK]
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
    # 1) 抓一帧（与你“最小可跑”一致的直读方式）
    frame = grab_one_frame_once(SRC)

    # 2) YOLO 推理（单帧）
    model = YOLO(WEIGHTS)
    res = model.predict(source=frame, conf=CONF, iou=IOU, verbose=False)[0]
    annotated = res.plot()
    h, w = res.orig_shape

    # 3) 解析检测结果
    detections = []
    names = res.names  # id->name
    boxes = getattr(res, "boxes", None)
    if boxes is not None and len(boxes) > 0:
        for b in boxes:
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            conf = float(b.conf[0])
            cls_id = int(b.cls[0])
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            cls_name = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else str(cls_id)
            detections.append({
                "class": cls_name.lower(),
                "conf": round(conf, 4),
                "bbox_xyxy": [float(x1), float(y1), float(x2), float(y2)],
                "center_xy": [float(cx), float(cy)]
            })

    print(f"[DEBUG] det_count={len(detections)}, image=({w}x{h})")

    # 4) 构造 perception_report
    assigned = assign_ids(detections)
    objects = [{
        "id": oid,
        "class": cls,
        "center_xy": [round(float(d["center_xy"][0]), 1), round(float(d["center_xy"][1]), 1)],
        "bbox_xyxy": [round(float(v), 1) for v in d["bbox_xyxy"]],
        "conf": d["conf"],
    } for (oid, cls, d) in assigned]

    summary = Counter([cls for _, cls, _ in assigned])
    summary = dict(summary)

    perception_report = {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "summary": summary,
        "objects": objects
    }

    # 5) 保存两张图方便核验
    try:
        ts = time.strftime("%Y%m%d-%H%M%S")
        cv2.imwrite(f"/tmp/yolo_min_raw_{ts}.jpg", frame)
        cv2.imwrite(f"/tmp/yolo_min_annotated_{ts}.jpg", annotated)
    except Exception:
        pass

    # 6) 打印最终 JSON
    out = {"perception_report": perception_report}
    print(json.dumps(out, ensure_ascii=False, indent=2))

if __name__ == "__main__":
    main()
