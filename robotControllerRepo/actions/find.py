# find.py
# -*- coding: utf-8 -*-
"""
独立版通用"查找"动作模块 - 避免循环导入
- 旋转扫描 + 多点搜索 + 智能摄像头处理
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
from collections import Counter
from ttsRepo.stream_tts import tts_manager

# 简单的日志工具
class SimpleLogger:
    def info(self, msg): print(f"[INFO] {msg}")
    def warning(self, msg): print(f"[WARN] {msg}")
    def error(self, msg): print(f"[ERROR] {msg}")
    def debug(self, msg): print(f"[DEBUG] {msg}")

logger = SimpleLogger()

# -------- 黑板工具（/tmp） --------
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

# -------- 独立的YOLO检测器 --------
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
        """检测图像中的物体"""
        if not self.available:
            return []
        
        try:
            results = self.model.predict(
                source=image_path_or_array,
                conf=conf_threshold,
                verbose=False
            )[0]
            
            detections = []
            names = results.names
            
            for box in getattr(results, "boxes", []):
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                cls_name = names.get(cls_id, str(cls_id)).lower()
                
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

# -------- 智能参数优化 --------
def get_optimized_params(target_class: str, has_waypoints: bool) -> tuple:
    """根据物体类型和搜索策略优化检测参数"""
    conf_thres = 0.5
    max_rot_deg = 360
    
    # 针对不同物体类型的优化
    if target_class in ["cup", "bottle", "phone", "remote", "keys"]:  # 小物体
        conf_thres = 0.45  # 稍微宽松，避免漏检小物体
    elif target_class in ["person", "chair", "table", "sofa", "bed"]:  # 大物体
        conf_thres = 0.6   # 可以更严格，大物体通常检测置信度高
    elif target_class in ["book", "laptop", "mouse", "keyboard"]:  # 中等物体
        conf_thres = 0.5   # 保持默认
    
    # 如果有多个搜索点，每个点只需半圈扫描（提高效率）
    if has_waypoints:
        max_rot_deg = 180  # 多点搜索时每点半圈即可
    
    logger.debug(f"Optimized params for '{target_class}': conf_thres={conf_thres}, max_rot_deg={max_rot_deg}")
    return conf_thres, max_rot_deg

# -------- 摄像头处理 --------
def _create_video_capture(video_source: int = 0) -> Optional[cv2.VideoCapture]:
    """创建并配置摄像头capture对象"""
    logger.info(f"[find] 📷 Attempting to open camera: {video_source}")
    
    # 尝试不同的后端
    backends_to_try = [
        cv2.CAP_V4L2,      # Linux V4L2
        cv2.CAP_GSTREAMER, # GStreamer
        cv2.CAP_FFMPEG,    # FFmpeg
        cv2.CAP_ANY        # 自动选择
    ]
    
    for backend in backends_to_try:
        logger.debug(f"[find] 🔧 Trying backend: {backend}")
        cap = cv2.VideoCapture(video_source, backend)
        
        if cap.isOpened():
            # 设置摄像头参数
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            # 测试读取帧
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
    """简单测试摄像头是否可用"""
    logger.info("[find] 🧪 Testing camera access...")
    
    cap = _create_video_capture(0)
    if cap is None:
        return False
    
    try:
        # 尝试读取10帧
        success_count = 0
        for i in range(10):
            ret, frame = cap.read()
            if ret and frame is not None:
                success_count += 1
            time.sleep(0.1)
        
        logger.info(f"[find] 📊 Camera test: {success_count}/10 frames read successfully")
        return success_count >= 5  # 至少成功读取一半
        
    finally:
        cap.release()

# -------- 简单的图像检测 --------
def _detect_with_camera(detector: StandaloneYOLODetector, target_class: str, conf_thres: float) -> Optional[Dict]:
    """使用摄像头进行单次检测"""
    if not detector.available:
        return None
    
    # 使用rpicam-still拍照（针对树莓派）
    try:
        ts = time.strftime("%Y%m%d-%H%M%S")
        img_path = f"/tmp/find_frame_{ts}.jpg"
        
        result = subprocess.run([
            "rpicam-still", "-t", "1000",
            "--width", "640", "--height", "480",
            "-o", img_path
        ], capture_output=True, check=True)
        
        if os.path.exists(img_path):
            detections = detector.detect_objects(img_path, conf_thres)
            os.remove(img_path)  # 清理临时文件
            
            # 找到目标物体
            targets = [d for d in detections if d["class"] == target_class and d["conf"] >= conf_thres]
            if targets:
                return max(targets, key=lambda x: x["conf"])
        
    except subprocess.CalledProcessError:
        logger.warning("rpicam-still failed, trying OpenCV...")
        
        # 回退到OpenCV
        cap = _create_video_capture(0)
        if cap:
            try:
                ret, frame = cap.read()
                if ret and frame is not None:
                    detections = detector.detect_objects(frame, conf_thres)
                    targets = [d for d in detections if d["class"] == target_class and d["conf"] >= conf_thres]
                    if targets:
                        return max(targets, key=lambda x: x["conf"])
            finally:
                cap.release()
    
    return None

# -------- 分步旋转扫描 --------
def _rotate_scan_stepwise(robot_name: str,
                         rotate_fn,
                         detect_fn,
                         target_class: str,
                         *,
                         max_rot_deg: int = 360,
                         step_deg: int = 30,
                         pause: float = 0.5,
                         conf_thres: float = 0.5) -> Optional[Dict[str, Any]]:
    """分步旋转扫描"""
    logger.info(f"[find] 🔄 Starting stepwise rotate+detect scan")
    logger.info(f"[find] 🔧 Step parameters: max_rot={max_rot_deg}°, step={step_deg}°, pause={pause}s")
    
    # 先看当前帧
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
            
        # 暂停让机器人稳定
        time.sleep(pause)
        
        # 检测当前视角
        logger.info(f"[find] 📷 Detecting after rotation step {step_count}...")
        hit = detect_fn(target_class, conf_thres) if detect_fn else None
        if hit:
            logger.info(f"[find] 🎯 Found {target_class} after {turned + step_deg}° rotation!")
            return hit
            
        turned += step_deg
        
    logger.info(f"[find] 🛑 Stepwise scan complete: {step_count} steps, {turned}° total rotation")
    return None

# -------- 主要的find功能 --------
def execute_find(node: Any, robot_name: str, params: Dict[str, Any], **ctx) -> Dict[str, Any]:
    """独立的查找动作：旋转扫描"""
    # === 解析参数 ===
    target: str = params.get("target_class", "cup")
    save_as: str = params.get("save_as", target)
    timeout_sec: float = float(params.get("timeout_sec", 25))
    use_spin: bool = bool(params.get("spin_scan", True))
    waypoints: List[Any] = params.get("search_waypoints", [])

    # === 系统优化参数 ===
    conf_thres, max_rot_deg = get_optimized_params(target, bool(waypoints))
    
    # === 获取上下文函数 ===
    rotate_fn = ctx.get("rotate_fn")
    navigate_to_fn = ctx.get("navigate_to_fn")
    event_pub = ctx.get("event_pub")

    if use_spin and not callable(rotate_fn):
        logger.warning("spin_scan=True but rotate_fn missing, will skip spin scan.")
        use_spin = False
    if waypoints and not callable(navigate_to_fn):
        logger.warning("search_waypoints provided but navigate_to_fn missing, will skip waypoint search.")
        waypoints = []

    # === 初始化检测器 ===
    detector = StandaloneYOLODetector()
    if not detector.available:
        logger.error("YOLO detector not available")
        return {"ok": False, "found": False, "reason": "YOLO unavailable"}

    # === 摄像头测试 ===
    camera_ok = _test_camera_simple()
    logger.info(f"[find] 📷 Camera test result: {'✅ PASS' if camera_ok else '❌ FAIL'}")

    # === 开始搜索 ===
    t0 = time.time()
    # tts_manager.say(f"Okay, I'll look around for the {target}.")
    logger.info(f"[find] robot={robot_name} target={target} timeout={timeout_sec}s conf>={conf_thres}")

    hit: Optional[Dict[str, Any]] = None

    # 创建检测函数
    def detect_current_view(target_class: str, conf_threshold: float):
        return _detect_with_camera(detector, target_class, conf_threshold)

    # === 1. 当前帧检测 ===
    hit = detect_current_view(target, conf_thres)

    # === 2. 旋转扫描 ===
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
            pause=0.8  # 增加暂停时间确保稳定
        )

    # === 3. 多点搜索 ===
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

            time.sleep(1.0)  # 到达后等待稳定
            
            # 到达后先检测一次
            hit = detect_current_view(target, conf_thres)
            if hit:
                logger.info(f"[find] 🎯 Found {target} at waypoint {wp}!")
                break

            # 如果没找到，在这个位置做半圈扫描
            if use_spin:
                logger.info(f"[find] 🔄 Scanning at waypoint {wp}")
                hit = _rotate_scan_stepwise(
                    robot_name=robot_name,
                    rotate_fn=rotate_fn,
                    detect_fn=detect_current_view,
                    target_class=target,
                    conf_thres=conf_thres,
                    max_rot_deg=180,  # waypoint处只扫描半圈
                    step_deg=45,      # 更大步长，提高效率
                    pause=0.8
                )

            if hit:
                logger.info(f"[find] 🎯 Found {target} during scan at waypoint {wp}!")
                break

    # === 处理结果 ===
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

    # === 成功找到 ===
    record = {
        "class": target,
        "center_xy": hit.get("center_xy"),
        "bbox_xyxy": hit.get("bbox_xyxy"),
        "map_xy": hit.get("map_xy") if "map_xy" in hit else None,
        "conf": float(hit.get("conf", 0.0)),
        "timestamp": time.time()
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
    return {"ok": True, "found": True, "blackboard_key": save_as, "record": record}

# -------- 简化的动态重规划 --------
def execute_find_with_simple_replanning(node: Any, robot_name: str, params: Dict[str, Any], **ctx) -> Dict[str, Any]:
    """简化版：找到物体后生成基本的后续任务"""
    basic_result = execute_find(node, robot_name, params, **ctx)
    
    if not basic_result.get("found", False):
        logger.info(f"[find] {robot_name} didn't find {params.get('target_class')}, no follow-up actions needed")
        return basic_result
    
    # === 简单的后续任务生成 ===
    target_class = params.get("target_class", "cup")
    blackboard_key = basic_result.get("blackboard_key")
    
    # 生成标准的 collect + deliver 任务
    follow_up_tasks = [
        {
            "action": "collect",
            "parameters": {
                "item": target_class,
                "target": blackboard_key
            }
        },
        {
            "action": "deliver", 
            "parameters": {
                "item": target_class,
                "target": "master"  # 假设送给主人
            }
        }
    ]
    
    basic_result["follow_up_tasks"] = follow_up_tasks
    basic_result["replanning_success"] = True
    basic_result["replanning_source"] = "simple"
    
    logger.info(f"[find] Generated {len(follow_up_tasks)} simple follow-up tasks")
    return basic_result

# -------- 对外接口 --------
def run_action(node: Any, task: Dict[str, Any], **ctx) -> Dict[str, Any]:
    """与 robot_controller 的单步接口保持一致"""
    robot = task.get("robot") or task.get("parameters", {}).get("robot")
    if not robot:
        return {"ok": False, "reason": "robot name missing"}
    
    # 使用简化版的重规划
    return execute_find_with_simple_replanning(node, robot, task.get("parameters", {}), **ctx)

# -------- 真实的机器人控制函数 --------
def real_rotate(node, robot_name: str, degrees: float):
    """真实的机器人旋转函数"""
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        import math
        
        # 创建发布器
        publisher = node.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
        
        # 设置旋转参数
        twist = Twist()
        angular_speed = math.radians(30)  # 30°/s
        
        direction = "left" if degrees >= 0 else "right"
        twist.angular.z = angular_speed if degrees >= 0 else -angular_speed
        
        # 计算旋转时间
        angle_rad = math.radians(abs(degrees))
        duration = angle_rad / abs(angular_speed)
        
        logger.info(f"🔄 Real rotation: {robot_name} turning {direction} {abs(degrees)}° for {duration:.2f}s")
        
        # 等待发布器初始化
        time.sleep(0.2)
        
        # 持续发布旋转命令
        start_time = time.time()
        while time.time() - start_time < duration:
            publisher.publish(twist)
            time.sleep(0.01)
        
        # 停止旋转
        publisher.publish(Twist())
        logger.info(f"✅ Real rotation completed: {robot_name}")
        return True
        
    except Exception as e:
        logger.error(f"❌ Real rotation failed: {e}")
        return False

def real_navigate(node, robot_name: str, target):
    """简单的导航实现（占位）"""
    logger.info(f"🚶 Navigation: {robot_name} -> {target} (placeholder)")
    time.sleep(1.0)  # 模拟导航时间
    return True

# -------- 独立运行的主函数 --------
def run_standalone_test():
    """独立运行find测试，使用真实的ROS控制"""
    import sys
    
    try:
        import rclpy
        from rclpy.node import Node
        
        # 初始化ROS
        rclpy.init()
        logger.info("🚀 ROS initialized for standalone find test")
        
        # 创建ROS节点
        node = rclpy.create_node('standalone_find_test')
        logger.info("🤖 ROS node created: standalone_find_test")
        
        # 获取机器人名称
        robot_name = sys.argv[1] if len(sys.argv) > 1 else "robot1"
        target_class = sys.argv[2] if len(sys.argv) > 2 else "cup"
        
        logger.info(f"🎯 Test parameters: robot={robot_name}, target={target_class}")
        
        # 创建真实的旋转函数
        def rotate_fn(robot, deg):
            return real_rotate(node, robot, deg)
        
        def nav_fn(robot, target):
            return real_navigate(node, robot, target)
        
        def event_fn(kind, payload):
            logger.info(f"📡 Event: {kind} -> {payload}")
        
        print(f"🧪 Testing REAL find functionality for {robot_name}...")
        print(f"🎯 Looking for: {target_class}")
        print("⚠️  Make sure your robot is ready and ROS topics are active!")
        
        # 等待用户确认
        input("Press Enter when robot is ready, or Ctrl+C to cancel...")
        
        # 执行真实的find测试
        res = execute_find(
            node=node,
            robot_name=robot_name,
            params={
                "target_class": target_class,
                "save_as": f"{target_class}_target",
                "timeout_sec": 30,  # 增加超时时间
                "search_waypoints": []  # 暂时不使用waypoints
            },
            rotate_fn=rotate_fn,
            navigate_to_fn=nav_fn,
            event_pub=event_fn
        )
        
        print("\n" + "="*50)
        print("🎉 FIND TEST RESULT:")
        print(f"✅ Success: {res.get('ok', False)}")
        print(f"🎯 Found: {res.get('found', False)}")
        if res.get('found'):
            print(f"📦 Object: {res.get('blackboard_key')}")
            print(f"🎯 Confidence: {res.get('record', {}).get('conf', 0):.3f}")
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

# -------- 命令行测试 --------
if __name__ == "__main__":
    print("🤖 Standalone Find.py Test")
    print("Usage: python3 find.py [robot_name] [target_class]")
    print("Example: python3 find.py robot1 cup")
    print("")
    
    result = run_standalone_test()
    
    if result.get("ok"):
        print("🎉 Test completed successfully!")
        exit(0)
    else:
        print("❌ Test failed!")
        exit(1)