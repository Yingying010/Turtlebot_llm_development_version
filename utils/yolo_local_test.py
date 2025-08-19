
import cv2
import numpy as np
from ultralytics import YOLO
import openai
from loguru import logger
import os, sys, re, json, time, traceback, threading
from typing import Dict, List, Optional, Tuple, Any
from textwrap import dedent
from pathlib import Path
from collections import Counter

DEFAULT_MODEL = os.getenv("LLM_MODEL", "gpt-4o-mini")
YOLO_WEIGHTS = os.getenv("YOLO_WEIGHTS", "yolov8n.pt")
YOLO_CONF   = float(os.getenv("YOLO_CONF", "0.25"))
YOLO_IOU    = float(os.getenv("YOLO_IOU", "0.45"))
YOLO_DEVICE = os.getenv("YOLO_DEVICE", "cpu")
WEIGHTS = "yolov8n.pt"
CONF = 0.10   # 低一些确保能检出
IOU  = 0.45
TOPK = 20
_video_env = os.getenv("VIDEO_SOURCE", "0")   # 树莓派默认用本地相机
if _video_env.isdigit():
    DEFAULT_SOURCE: Any = int(_video_env)
else:
    DEFAULT_SOURCE: Any = _video_env

# ================== OpenCV 统一视频源（关键修正） ==================
class GlobalVideoSource:
    def __init__(self, source: Any):
        self.source = source
        if isinstance(source, int):
            self.backend = cv2.CAP_V4L2              # 本地 /dev/video*（USB/兼容层）
        elif isinstance(source, str) and "appsink" in source:
            self.backend = cv2.CAP_GSTREAMER         # GStreamer 管道
        else:
            self.backend = cv2.CAP_FFMPEG            # 文件/网络/UDP
        self.cap = None
        self.lock = threading.Lock()
        self.ready = False

    def open(self):
        with self.lock:
            if self.cap is None:
                self.cap = cv2.VideoCapture(self.source, self.backend)
                try:
                    self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                except Exception:
                    pass
                if not self.cap.isOpened():
                    raise RuntimeError(f"无法打开视频源：{self.source}")
                # 预热，丢掉起始不完整帧
                deadline = time.time() + 5.0
                ok_cnt = 0
                while time.time() < deadline and ok_cnt < 10:
                    ok, _ = self.cap.read()
                    if ok:
                        ok_cnt += 1
                    else:
                        time.sleep(0.03)
                self.ready = ok_cnt > 0

    def read(self, drop_n=5, timeout=2.0):
        if self.cap is None or not self.ready:
            self.open()
        for _ in range(max(0, drop_n)):
            self.cap.read()

        t0 = time.time()
        while time.time() - t0 < timeout:
            ok, frame = self.cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue
            # 过滤明显坏帧
            m = float(frame.mean()); v = float(frame.var())
            h, w = frame.shape[:2]
            if h < 32 or w < 32:      # 尺寸异常
                continue
            if m < 1.0 or v < 5.0:    # 黑帧/坏帧
                continue
            return frame
        raise RuntimeError("读取视频帧超时/有效帧未到达")
    
    @staticmethod
    def grab_one_frame_once(source: Any, warmup_frames: int = 8, open_timeout_sec: float = 8.0) -> np.ndarray:
        if isinstance(source, int):
            backend = cv2.CAP_V4L2
        elif isinstance(source, str) and "appsink" in source:
            backend = cv2.CAP_GSTREAMER
        else:
            backend = cv2.CAP_FFMPEG
        cap = cv2.VideoCapture(source, backend)

        t0 = time.time()

        # 等待流真正打开（有的环境 isOpened 也可能 True 但实际没帧）
        while time.time() - t0 < open_timeout_sec:
            ok, _ = cap.read()
            if ok:
                break
            time.sleep(0.05)

        # 预热：丢掉起始不稳定帧（PPS/SPS）
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
            if self.cap is not None:
                self.cap.release()
                self.cap = None
                self.ready = False

VIDEO = GlobalVideoSource(DEFAULT_SOURCE)


class YOLOPerceiver:
    def __init__(self,
                 weights: str = YOLO_WEIGHTS,
                 conf: float = YOLO_CONF,
                 iou: float = YOLO_IOU,
                 device: Optional[str] = YOLO_DEVICE):
        self.model = YOLO(weights)
        self.conf = conf
        self.iou = iou
        self.device = device

    def detect_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, Dict]:
        # FIX: 只传入 frame，不使用流/URL/生成器
        res = self.model.predict(
            source=frame, conf=self.conf, iou=self.iou, device=self.device, verbose=False
        )[0]
        annotated = res.plot()
        h, w = res.orig_shape
        detections: List[Dict] = []
        names = res.names  # id -> class name
        result_boxes = getattr(res, "boxes", None)
        if result_boxes is not None and len(result_boxes) > 0:
            for b in result_boxes:
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
        return annotated, {"image": {"width": int(w), "height": int(h)}, "detections": detections}

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
    # 1) 抓一帧（与你“最小可跑”一致的直读方式）
    frame = GlobalVideoSource.grab_one_frame_once(DEFAULT_SOURCE)

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

