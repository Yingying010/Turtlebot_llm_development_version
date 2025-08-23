#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
实时YOLO稳态检测（含类别名与打印）：
- One-Euro风格自适应EMA滤波（首帧即稳定）
- 极简IoU多目标跟踪（保ID，短丢帧不抖）
- 类别稳态：多数投票，输出稳定的**类名**
- 尺寸变化约束：抑制忽大忽小
- 控制台打印：Detect: bottle, Confidence: 0.93

可选环境变量：
  YOLO_SRC      输入源（默认：UDP示例，可改为0/RTSP/文件）
  YOLO_MODEL    权重（默认 yolov8n.pt）
  YOLO_IMGSZ    输入尺寸（默认 640）
  YOLO_CONF     置信度阈值（默认 0.5）
  YOLO_NMS_IOU  NMS IoU（默认 0.45）
"""

import os
import cv2
import time
import math
import collections
from typing import List, Tuple, Dict, Any
from ultralytics import YOLO

# ======================
# 配置
# ======================
SRC = os.getenv(
    "YOLO_SRC",
    # 改成你的源：0（USB摄像头）, "rtsp://...", "video.mp4" 等
    "udp://@:8888?fifo_size=1000000&overrun_nonfatal=1&buffer_size=1000000&probesize=32&analyzeduration=0"
)
MODEL = os.getenv("YOLO_MODEL", "yolov8n.pt")
IMG_SIZE = int(os.getenv("YOLO_IMGSZ", "640"))
CONF = float(os.getenv("YOLO_CONF", "0.5"))
IOU_NMS = float(os.getenv("YOLO_NMS_IOU", "0.45"))

ASSOC_IOU = 0.4          # 跟踪关联阈值（IoU）
MISS_TOL = 10            # 允许丢失帧数量
SIDE_MARGIN = 0.02       # 丢弃紧贴左右边缘的框(比例)
MAX_SCALE_CHANGE = 1.4   # 单帧允许的最大尺寸倍率变化

# ======================
# 工具函数
# ======================
def clamp01(x: float) -> float:
    return max(0.0, min(1.0, x))

def iou_xyxy(a, b) -> float:
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

def clamp_size_change(prev_box, new_box, max_scale=1.4):
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
    """速度自适应的EMA滤波，无需预热。"""
    def __init__(self, alpha_min=0.12, alpha_max=0.8, vel_ref=80.0):
        self.alpha_min = alpha_min
        self.alpha_max = alpha_max
        self.vel_ref = max(1e-3, vel_ref)
        self.prev = None
        self.prev_t = None

    def _alpha(self, v):
        k = min(5.0, v / self.vel_ref)
        return self.alpha_min + (self.alpha_max - self.alpha_min) * clamp01(k)

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

# ======================
# 跟踪器（保留类别名）
# ======================
class Track:
    _next_id = 1
    def __init__(self, bbox, cls_id, cls_name, score, t):
        self.id = Track._next_id; Track._next_id += 1

        self.cls_hist = collections.deque(maxlen=5)   # 名称投票
        self.cls_id_hist = collections.deque(maxlen=5)
        self.cls_hist.append(cls_name)
        self.cls_id_hist.append(int(cls_id))
        self.cls_name = cls_name
        self.cls_id = int(cls_id)

        self.score = float(score)
        self.filter = OneEuroEMA(alpha_min=0.10, alpha_max=0.85, vel_ref=90.0)
        self.box = self.filter.update_vec(list(bbox), t)
        self.last_t = t
        self.miss = 0

    def _majority(self, dq: collections.deque):
        if not dq: 
            return None
        cnt = collections.Counter(dq)
        return cnt.most_common(1)[0][0]

    def update(self, bbox, cls_id, cls_name, score, t):
        # 尺寸约束 + 平滑
        bbox_c = clamp_size_change(self.box, bbox, max_scale=MAX_SCALE_CHANGE)
        self.box = self.filter.update_vec(list(bbox_c), t)

        # 类别稳态
        self.cls_hist.append(cls_name)
        self.cls_id_hist.append(int(cls_id))
        self.cls_name = self._majority(self.cls_hist) or cls_name
        self.cls_id   = self._majority(self.cls_id_hist) or int(cls_id)

        # 置信度平滑
        self.score = 0.7 * self.score + 0.3 * float(score)
        self.last_t = t
        self.miss = 0

    def step(self):
        self.miss += 1

class SimpleTracker:
    def __init__(self, iou_thr=ASSOC_IOU, miss_tolerance=MISS_TOL):
        self.iou_thr = iou_thr
        self.miss_tol = miss_tolerance
        self.tracks: List[Track] = []

    def update(self, dets: List[Dict[str, Any]], t: float):
        """
        dets: [{'bbox':[x1,y1,x2,y2], 'cls_id':int, 'cls_name':str, 'score':float}, ...]
        """
        used = set()
        for tr in self.tracks:
            tr.step()

        # 贪心匹配
        for tr in self.tracks:
            best_iou, best_j = -1.0, -1
            for j, d in enumerate(dets):
                if j in used: 
                    continue
                ov = iou_xyxy(tr.box, d['bbox'])
                # 优先同类，但允许低IoU时放宽
                if ov > best_iou and (d['cls_id'] == tr.cls_id or best_iou < self.iou_thr):
                    best_iou, best_j = ov, j
            if best_j >= 0 and best_iou >= self.iou_thr:
                d = dets[best_j]
                tr.update(d['bbox'], d['cls_id'], d['cls_name'], d['score'], t)
                used.add(best_j)

        # 新轨迹
        for j, d in enumerate(dets):
            if j not in used:
                self.tracks.append(Track(d['bbox'], d['cls_id'], d['cls_name'], d['score'], t))

        # 清理
        self.tracks = [tr for tr in self.tracks if tr.miss <= self.miss_tol]

    def get(self) -> List[Track]:
        return self.tracks

# ======================
# 视频与主循环
# ======================
def open_capture(src: str):
    cap = cv2.VideoCapture(src, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError(f"无法打开视频源：{src}")
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass
    for _ in range(8):  # 启动丢几帧
        cap.read()
    return cap

def main():
    print(f"[INFO] Opening stream: {SRC}")
    cap = open_capture(SRC)

    print(f"[INFO] Loading model: {MODEL}")
    model = YOLO(MODEL)

    tracker = SimpleTracker(iou_thr=ASSOC_IOU, miss_tolerance=MISS_TOL)
    frames = 0
    t0 = time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[WARN] Empty frame")
            time.sleep(0.01)
            continue
        t = time.time()

        r = model.predict(
            source=frame,
            imgsz=IMG_SIZE,
            conf=CONF,
            iou=IOU_NMS,
            agnostic_nms=True,
            verbose=False,
            device=0 if os.getenv("CUDA_VISIBLE_DEVICES") else None,
        )[0]

        # 类名映射：优先结果里的names，其次模型里的names
        names_map = getattr(r, "names", None) or getattr(model, "names", {}) or {}
        dets: List[Dict[str, Any]] = []

        if r.boxes is not None and len(r.boxes) > 0:
            xyxy = r.boxes.xyxy.cpu().numpy()
            cls = r.boxes.cls.cpu().numpy()
            conf = r.boxes.conf.cpu().numpy()
            W = frame.shape[1]

            for i in range(len(xyxy)):
                x1, y1, x2, y2 = xyxy[i].tolist()
                if x1 < SIDE_MARGIN * W or x2 > (1.0 - SIDE_MARGIN) * W:
                    continue
                cls_id = int(cls[i])
                cls_name = str(names_map.get(cls_id, cls_id))
                dets.append({
                    "bbox": [x1, y1, x2, y2],
                    "cls_id": cls_id,
                    "cls_name": cls_name,
                    "score": float(conf[i]),
                })

        tracker.update(dets, t)

        # —— 控制台打印：Detect: name, Confidence: 0.93 ——
        # 按track输出（稳定后的结果，推荐）
        for tr in tracker.get():
            print(f"Detect: {tr.cls_name}, Confidence: {tr.score:.2f}")

        # 可视化
        vis = frame.copy()
        for tr in tracker.get():
            x1, y1, x2, y2 = [int(v) for v in tr.box]
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"id{tr.id} {tr.cls_name} {tr.score:.2f}"
            cv2.putText(vis, label, (x1, max(0, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("YOLO stabilized (with names + print)", vis)

        frames += 1
        if frames % 30 == 0:
            now = time.time()
            fps = frames / (now - t0 + 1e-6)
            print(f"FPS: {fps:.2f}  tracks:{len(tracker.get())}")
            t0, frames = now, 0

        if cv2.waitKey(1) & 0xFF == 27:  # ESC
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
