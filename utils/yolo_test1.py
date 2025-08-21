import cv2, time
from ultralytics import YOLO
 
SRC = "udp://@:8888?fifo_size=1000000&overrun_nonfatal=1&buffer_size=1000000&probesize=32&analyzeduration=0"
 
# 1) 打开UDP流
cap = cv2.VideoCapture(SRC, cv2.CAP_FFMPEG)
assert cap.isOpened(), f"OpenCV failed to open stream: {SRC}"
 
# 可选：尽量减缓延迟（有些OpenCV构建不生效）
try:
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
except Exception:
    pass
 
# 2) 丢掉起始不完整的几帧（PPS/SPS未同步时常见）
warmup = 0
while warmup < 5:
    ret, _ = cap.read()
    if ret:
        warmup += 1
 
# 3) 加载YOLO
model = YOLO("yolov8n.pt")
 
# 4) 循环推理
fps_t0, frames = time.time(), 0
while True:
    ret, frame = cap.read()
    if not ret:
        continue  # UDP短暂丢包时继续读
 
    # 单帧推理
    results = model.predict(frame, verbose=False)  # 也可加 conf=0.25
    r = results[0]
 
    # 打印检测
    for box in r.boxes:
        cls = model.names[int(box.cls)]
        conf = float(box.conf)
        # 你想筛掉置信度低的可以 if conf < 0.25: continue
        print(f"Detect: {cls}，Confidence: {conf:.2f}")
 
    # 可视化（Ultralytics自带绘制）
    annotated = r.plot()  # 返回带框的BGR图
    cv2.imshow("YOLOv8-UDP", annotated)
 
    frames += 1
    if frames % 30 == 0:
        dt = time.time() - fps_t0
        print(f"FPS: {frames/dt:.2f}")
        fps_t0, frames = time.time(), 0
 
    if cv2.waitKey(1) & 0xFF == 27:  # ESC 退出
        break
 
cap.release()
cv2.destroyAllWindows()