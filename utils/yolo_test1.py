#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
实时YOLO稳态检测（单帧可稳定）：
- One-Euro风格自适应EMA滤波（首帧即可稳定，无窗口预热）
- IoU极简关联：保持目标ID，短暂丢帧不抖
- 类别稳态：短时类别跳变做多数投票/平滑
- 尺寸变化约束：抑制“忽大忽小”的框闪烁
- 低延迟：丢弃启动时的缓存帧，尽量保持即摄即显

环境变量（可选）：
  YOLO_SRC        输入源（默认：UDP示例，改成你的摄像头/RTSP/文件）
  YOLO_MODEL      权重（默认 yolov8n.pt）
  YOLO_IMGSZ      尺寸（默认 640）
  YOLO_CONF       置信度阈值（默认 0.5）
  YOLO_NMS_IOU    NMS IoU（默认 0.45）
"""

import os
import cv2
import time
import math
import collections
from typing import List, Tuple

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

# 关联阈值与稳态策略
ASSOC_IOU = 0.4          # 跟踪关联阈值（IoU）
MISS_TOL = 10            # 允许丢失帧数量（超过则删除track）
SIDE_MARGIN = 0.02       # 丢弃靠近左右边缘的框(比例)
MAX_SCALE_CHANGE = 1.4   # 单帧允许的最大尺寸倍率变化（>1）


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

    # 中心对齐，限制尺寸增长比例
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
    """
    One-Euro风格自适应EMA滤波：
      - 根据速度自适应alpha：移动快 → 减小平滑（避免拖尾）；静止 → 增强平滑（抑制抖动）
      - 无需窗口预热，首帧即稳定
    """
    def __init__(self, alpha_min=0.12, alpha_max=0.8, vel_ref=80.0):
        self.alpha_min = alpha_min
        self.alpha_max = alpha_max
        self.vel_ref = max(1e-3, vel_ref)
        self.prev = None
        self.prev_t = None

    def _alpha(self, v):
        k = min(5.0, v / self.vel_ref)  # 速度映射系数
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


class Track:
    _next_id = 1

    def __init__(self, bbox, cls, score, t):
        self.id = Track._next_id
        Track._next_id += 1

        self.cls_hist = collections.deque(maxlen=5)  # 类别投票窗口
        self.cls_hist.append(int(cls))
        self.cls = int(cls)

        self.score = float(score)
        self.filter = OneEuroEMA(alpha_min=0.10, alpha_max=0.85, vel_ref=90.0)
        self.box = self.filter.update_vec(list(bbox), t)
        self.last_t = t
        self.miss = 0

    def majority_class(self):
        if not self.cls_hist:
            return self.cls
        counts = collections.Counter(self.cls_hist)
        return counts.most_common(1)[0][0]

    def update(self, bbox, cls, score, t):
        # 尺寸变化约束（基于当前平滑后的框）
        bbox_c = clamp_size_change(self.box, bbox, max_scale=MAX_SCALE_CHANGE)
        # 平滑
        self.box = self.filter.update_vec(list(bbox_c), t)

        # 类别稳态：轻度多数投票/平滑
        self.cls_hist.append(int(cls))
        self.cls = self.majority_class()

        # 置信度缓慢更新，避免突跳
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

    def update(self, dets: List[Tuple[List[float], int, float]], t: float):
        """
        dets: [([x1,y1,x2,y2], cls, score), ...]
        """
        used_det = set()

        # 先给每个track加1丢失计数
        for tr in self.tracks:
            tr.step()

        # 贪心IoU匹配
        for tr in self.tracks:
            best_iou, best_j = -1.0, -1
            for j, (bb, cls, sc) in enumerate(dets):
                if j in used_det:
                    continue
                # 优先匹配同类别（放宽：允许短时类别不一致，但同类优先）
                iou_val = iou_xyxy(tr.box, bb)
                if iou_val > best_iou and (cls == tr.cls or best_iou < self.iou_thr):
                    best_iou, best_j = iou_val, j
            if best_j >= 0 and best_iou >= self.iou_thr:
                bb, cls, sc = dets[best_j]
                tr.update(bb, cls, sc, t)
                used_det.add(best_j)

        # 未匹配的检测 -> 新建track
        for j, (bb, cls, sc) in enumerate(dets):
            if j not in used_det:
                self.tracks.append(Track(bb, cls, sc, t))

        # 清理过期
        self.tracks = [tr for tr in self.tracks if tr.miss <= self.miss_tol]

    def get(self) -> List[Track]:
        return self.tracks


def open_capture(src: str):
    cap = cv2.VideoCapture(src, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        # 退化到默认摄像头
        cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError(f"无法打开视频源：{src}")
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass
    # 启动期丢掉几帧，降低缓存和PPS/SPS问题
    for _ in range(8):
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

        # 更稳的推理参数
        res = model.predict(
            source=frame,         # numpy array
            imgsz=IMG_SIZE,
            conf=CONF,
            iou=IOU_NMS,
            agnostic_nms=True,
            verbose=False,
            device=0 if os.getenv("CUDA_VISIBLE_DEVICES") else None,
        )[0]

        dets = []
        if res.boxes is not None and len(res.boxes) > 0:
            xyxy = res.boxes.xyxy.cpu().numpy()
            cls = res.boxes.cls.cpu().numpy()
            conf = res.boxes.conf.cpu().numpy()
            W = frame.shape[1]

            for i in range(len(xyxy)):
                x1, y1, x2, y2 = xyxy[i].tolist()
                # 丢弃紧贴左右边缘的框，避免半边框致抖
                if x1 < SIDE_MARGIN * W or x2 > (1.0 - SIDE_MARGIN) * W:
                    continue
                dets.append(([x1, y1, x2, y2], int(cls[i]), float(conf[i])))

        tracker.update(dets, t)

        # 可视化
        vis = frame.copy()
        for tr in tracker.get():
            x1, y1, x2, y2 = [int(v) for v in tr.box]
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"id{tr.id} c{tr.cls} {tr.score:.2f}"
            cv2.putText(vis, label, (x1, max(0, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("YOLO stabilized", vis)

        frames += 1
        if frames % 30 == 0:
            now = time.time()
            fps = frames / (now - t0 + 1e-6)
            print(f"FPS: {fps:.2f}  tracks:{len(tracker.get())}")
            t0, frames = now, 0

        if cv2.waitKey(1) & 0xFF == 27:  # ESC退出
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
