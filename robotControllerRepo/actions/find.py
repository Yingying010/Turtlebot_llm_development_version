# find.py
# -*- coding: utf-8 -*-
"""
通用"查找"动作模块（完整版）
- 旋转扫描 + 多点搜索 + 智能摄像头处理
- 命中后把目标信息写入黑板（/tmp/robot_blackboard_<robot>.json）
- 支持两阶段任务规划：基于History的动态重规划
"""

from __future__ import annotations
import os, sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

import os
import json
import time
import cv2
import math
import threading
import numpy as np
from typing import Any, Dict, List, Optional
from collections import Counter
from ttsRepo.stream_tts import tts_manager

# 导入必要的模块
try:
    from loguru import logger
except Exception:
    class _DummyLogger:
        def info(self, *a, **k): print("[INFO]", *a)
        def warning(self, *a, **k): print("[WARN]", *a)
        def error(self, *a, **k): print("[ERROR]", *a)
        def debug(self, *a, **k): print("[DEBUG]", *a)
    logger = _DummyLogger()

# 🔥 延迟导入，避免循环导入
def _get_yolo_perceiver():
    """延迟导入YOLOPerceiver"""
    from llmParserRepo.gpt_yolo_localParser import YOLOPerceiver
    return YOLOPerceiver()

def _get_llm_parser():
    """延迟导入LLM相关模块"""
    from llmParserRepo.gpt_yolo_localParser import (
        PerceptionAwareLLM, build_perception_context, 
        HistoryStore, MEMORY_PATH, safe_json_parse
    )
    return PerceptionAwareLLM, build_perception_context, HistoryStore, MEMORY_PATH, safe_json_parse

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

def bb_get(robot_name: str, key: str) -> Optional[Dict[str, Any]]:
    db = _bb_read(robot_name)
    return db.get("objects", {}).get(key)

def bb_clear(robot_name: str, key: Optional[str] = None) -> None:
    if key is None:
        _bb_write(robot_name, {"objects": {}})
        return
    db = _bb_read(robot_name)
    if "objects" in db and key in db["objects"]:
        db["objects"].pop(key, None)
        _bb_write(robot_name, db)

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

# -------- YOLO/感知 --------
def _assign_ids(dets: List[Dict], topk: int = 3):
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

def _scan_once_with_yolo(detect_fn, target_class: str, conf_thres: float = 0.5) -> Optional[Dict[str, Any]]:
    """单次YOLO检测"""
    try:
        objs: List[Dict[str, Any]] = detect_fn() or []
    except Exception as e:
        logger.error(f"detect_fn failed: {e}")
        return None
    cand = [o for o in objs if o.get("class") == target_class and float(o.get("conf", 0)) >= conf_thres]
    return max(cand, key=lambda o: float(o.get("conf", 0))) if cand else None

# -------- 摄像头处理 --------
def _create_video_capture(video_source: int = 0) -> cv2.VideoCapture:
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

# -------- 实时旋转扫描 --------
def _rotate_scan_realtime_fixed(robot_name: str,
                               rotate_fn,
                               detect_model,
                               *,
                               max_rot_deg: int = 360,
                               deg_per_frame: int = 10,
                               fps: int = 5,
                               conf_thres: float = 0.5,
                               video_source: int = 0,
                               timeout_sec: float = 10.0) -> Optional[Dict[str, Any]]:
    """修复版实时旋转检测"""
    logger.info("[find] 🚀 Starting FIXED real-time rotate+detect scan...")
    
    # 创建摄像头对象
    cap = _create_video_capture(video_source)
    if cap is None:
        logger.error("❌ Failed to create camera capture")
        return None

    try:
        # 预热摄像头
        logger.info("[find] 🔥 Warming up camera...")
        for i in range(5):
            ret, frame = cap.read()
            if ret:
                logger.debug(f"[find] ✅ Warmup frame {i+1} success: {frame.shape}")
            else:
                logger.warning(f"[find] ⚠️ Warmup frame {i+1} failed")
            time.sleep(0.1)
        
        t0 = time.time()
        total_rotated = 0
        frame_interval = 1.0 / fps
        frame_count = 0

        while total_rotated < max_rot_deg and (time.time() - t0) < timeout_sec:
            # 读取帧
            ret, frame = cap.read()
            if not ret or frame is None:
                logger.debug(f"[find] ⚠️ Frame read failed, continuing...")
                time.sleep(0.01)
                continue

            frame_count += 1
            logger.debug(f"[find] 📷 Processing frame {frame_count}, rotated={total_rotated}°")

            # YOLO 检测
            try:
                res = detect_model.model.predict(
                    source=frame,
                    conf=detect_model.conf,
                    iou=detect_model.iou,
                    device=detect_model.device,
                    verbose=False
                )[0]

                names = res.names
                for b in getattr(res, "boxes", []):
                    x1, y1, x2, y2 = b.xyxy[0].tolist()
                    conf = float(b.conf[0])
                    cls_id = int(b.cls[0])
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0
                    cls_name = names.get(cls_id, str(cls_id)).lower()

                    logger.debug(f"[find] 🎯 Detected: {cls_name} (conf={conf:.3f})")
                    
                    if conf >= conf_thres:
                        logger.info(f"🎯 Found {cls_name} with conf={conf:.3f}")
                        return {
                            "class": cls_name,
                            "conf": conf,
                            "bbox_xyxy": [x1, y1, x2, y2],
                            "center_xy": [cx, cy]
                        }
            except Exception as e:
                logger.warning(f"[find] ⚠️ YOLO detection error: {e}")

            # 继续旋转
            if total_rotated < max_rot_deg:
                try:
                    logger.debug(f"[find] 🔄 Rotating {deg_per_frame}° (total: {total_rotated}°/{max_rot_deg}°)")
                    rotate_fn(robot_name, deg_per_frame)
                    total_rotated += deg_per_frame
                except Exception as e:
                    logger.error(f"[find] ❌ Rotation failed: {e}")
                    break

            time.sleep(frame_interval)

        logger.info(f"🛑 Scan complete: {frame_count} frames processed, {total_rotated}° rotated")
        return None

    finally:
        cap.release()

# -------- 分步旋转扫描（回退方案）--------
def _rotate_scan_stepwise(robot_name: str,
                         rotate_fn,
                         detect_fn,
                         target_class: str,
                         *,
                         max_rot_deg: int = 360,
                         step_deg: int = 30,
                         pause: float = 0.5,
                         conf_thres: float = 0.5) -> Optional[Dict[str, Any]]:
    """回退方案：分步旋转扫描"""
    logger.info(f"[find] 🔄 Starting stepwise rotate+detect scan (fallback mode)")
    logger.info(f"[find] 🔧 Step parameters: max_rot={max_rot_deg}°, step={step_deg}°, pause={pause}s")
    
    # 先看当前帧
    logger.info(f"[find] 📷 Checking current view first...")
    hit = _scan_once_with_yolo(detect_fn, target_class, conf_thres)
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
        hit = _scan_once_with_yolo(detect_fn, target_class, conf_thres)
        if hit:
            logger.info(f"[find] 🎯 Found {target_class} after {turned + step_deg}° rotation!")
            return hit
            
        turned += step_deg
        
    logger.info(f"[find] 🛑 Stepwise scan complete: {step_count} steps, {turned}° total rotation")
    return None

# -------- 基础find功能 --------
def execute_find(node: Any, robot_name: str, params: Dict[str, Any], **ctx) -> Dict[str, Any]:
    """查找动作：旋转扫描 + 多点搜索"""
    # === 解析参数 ===
    target: str = params.get("target_class", "cup")
    save_as: str = params.get("save_as", target)
    timeout_sec: float = float(params.get("timeout_sec", 25))
    use_spin: bool = bool(params.get("spin_scan", True))
    waypoints: List[Any] = params.get("search_waypoints", [])

    # === 系统优化参数 ===
    conf_thres, max_rot_deg = get_optimized_params(target, bool(waypoints))
    
    # === 验证上下文函数 ===
    detect_fn = ctx.get("detect_fn")
    rotate_fn = ctx.get("rotate_fn")
    navigate_to_fn = ctx.get("navigate_to_fn")
    event_pub = ctx.get("event_pub")

    if not callable(detect_fn):
        return {"ok": False, "found": False, "reason": "detect_fn missing"}
    if use_spin and not callable(rotate_fn):
        logger.warning("spin_scan=True but rotate_fn missing, will skip spin scan.")
        use_spin = False
    if waypoints and not callable(navigate_to_fn):
        logger.warning("search_waypoints provided but navigate_to_fn missing, will skip waypoint search.")
        waypoints = []

    # === 摄像头测试 ===
    camera_ok = _test_camera_simple()
    logger.info(f"[find] 📷 Camera test result: {'✅ PASS' if camera_ok else '❌ FAIL'}")

    # === 开始搜索 ===
    t0 = time.time()
    tts_manager.say(f"Okay, I'll look around for the {target}.")
    logger.info(f"[find] robot={robot_name} target={target} timeout={timeout_sec}s conf>={conf_thres}")

    hit: Optional[Dict[str, Any]] = None
    detect_model = _get_yolo_perceiver()  # 🔥 使用延迟导入

    # === 1. 当前帧检测 ===
    hit = _scan_once_with_yolo(detect_fn, target, conf_thres)

    # === 2. 旋转扫描 ===
    if not hit and use_spin:
        if camera_ok:
            logger.info(f"[find] 🔄 Starting FIXED real-time rotation scan")
            hit = _rotate_scan_realtime_fixed(
                robot_name=robot_name,
                rotate_fn=rotate_fn,
                detect_model=detect_model,
                conf_thres=conf_thres,
                video_source=0,
                fps=3,  # 降低fps，增加稳定性
                deg_per_frame=15,  # 增加旋转步长
                max_rot_deg=max_rot_deg,
                timeout_sec=timeout_sec - (time.time() - t0)
            )
        else:
            logger.warning("[find] 📷 Camera test failed, using stepwise fallback")
            hit = _rotate_scan_stepwise(
                robot_name=robot_name,
                rotate_fn=rotate_fn,
                detect_fn=detect_fn,
                target_class=target,
                conf_thres=conf_thres,
                max_rot_deg=max_rot_deg,
                step_deg=30,
                pause=0.5
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

            time.sleep(0.5)  # 到达后稍微等待
            
            # 到达后先检测一次
            hit = _scan_once_with_yolo(detect_fn, target, conf_thres)
            if hit:
                logger.info(f"[find] 🎯 Found {target} at waypoint {wp}!")
                break

            # 如果没找到，在这个位置做半圈扫描
            if use_spin:
                logger.info(f"[find] 🔄 Scanning at waypoint {wp}")
                if camera_ok:
                    hit = _rotate_scan_realtime_fixed(
                        robot_name=robot_name,
                        rotate_fn=rotate_fn,
                        detect_model=detect_model,
                        conf_thres=conf_thres,
                        max_rot_deg=180,  # waypoint处只扫描半圈
                        timeout_sec=timeout_sec - (time.time() - t0)
                    )
                else:
                    hit = _rotate_scan_stepwise(
                        robot_name=robot_name,
                        rotate_fn=rotate_fn,
                        detect_fn=detect_fn,
                        target_class=target,
                        conf_thres=conf_thres,
                        max_rot_deg=180,  # waypoint处只扫描半圈
                        step_deg=45,      # 更大步长，提高效率
                        pause=0.5
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

# -------- 基于History的动态重规划 --------
def execute_find_with_history_replanning(node: Any,
                                        robot_name: str,
                                        params: Dict[str, Any],
                                        **ctx) -> Dict[str, Any]:
    """基于History的find增强版：找到物体后，利用历史记录重新解析用户意图"""
    # === 1. 执行基础find逻辑 ===
    basic_result = execute_find(node, robot_name, params, **ctx)
    
    if not basic_result.get("found", False):
        logger.info(f"[find] {robot_name} didn't find {params.get('target_class')}, no follow-up actions needed")
        return basic_result
    
    # === 2. 找到了物体，利用History进行重新规划 ===
    logger.info(f"[find] {robot_name} found object, initiating history-based replanning...")
    
    try:
        # 🔥 使用延迟导入
        PerceptionAwareLLM, build_perception_context, HistoryStore, MEMORY_PATH, safe_json_parse = _get_llm_parser()
        
        # 读取本地历史记录
        store = HistoryStore(MEMORY_PATH)
        recent_messages = store.recent_chat_messages(max_turns=3)  # 获取最近3轮对话
        
        # 找到最近的用户输入
        original_user_input = None
        for msg in reversed(recent_messages):
            if msg.get("role") == "user":
                original_user_input = msg.get("content", "")
                break
        
        if not original_user_input:
            logger.warning("[find] No recent user input found in history")
            return basic_result
        
        logger.info(f"[find] Found original user input: '{original_user_input}'")
        
        # 获取当前视觉上下文
        detect_fn = ctx.get("detect_fn")
        if callable(detect_fn):
            detections = detect_fn()
            det_json = {"detections": detections or []}
            perception_ctx = build_perception_context(det_json)
        else:
            perception_ctx = "CURRENT_PERCEPTION:\n" + json.dumps(
                {"timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"), "objects": []}, 
                ensure_ascii=False
            )
        
        # 构建重新解析的prompt
        replanning_prompt = f"""
        CONTEXT: I just successfully found the {params.get('target_class')} that the user requested. 
        Now I need to determine what to do next based on the user's original command.
        
        Original user command: "{original_user_input}"
        Object found: {params.get('target_class')} (saved as blackboard key: {basic_result.get('blackboard_key')})
        
        Based on the user's original intent, please generate the appropriate follow-up actions.
        For collect actions, use the blackboard key "{basic_result.get('blackboard_key')}" as the target.
        If the user said something like "if you can't find it, never mind", then since I DID find it, 
        continue with the requested actions (collect, deliver, etc.).
        """
        
        # 调用LLM重新解析
        llm = PerceptionAwareLLM()  # 现在可以安全使用
        raw_response = llm.parse(
            user_text=replanning_prompt,
            perception_context=perception_ctx,
            chat_history_messages=recent_messages,
            perception_history_text=None
        )
        
        # 解析LLM响应
        try:
            replanned_data = safe_json_parse(raw_response)
            
            # 提取新的任务列表
            if "robots" in replanned_data and robot_name in replanned_data["robots"]:
                new_tasks = replanned_data["robots"][robot_name]
                logger.info(f"[find] Generated {len(new_tasks)} follow-up tasks based on history")
                
                # 将新任务添加到结果中
                basic_result["follow_up_tasks"] = new_tasks
                basic_result["replanning_success"] = True
                basic_result["replanning_source"] = "history"
                
                # 将重新规划的结果也写入历史记录
                store.append("assistant", f"Found {params.get('target_class')}. Planning follow-up actions based on your request.")
                
            else:
                logger.warning("[find] No follow-up tasks generated by LLM")
                basic_result["follow_up_tasks"] = []
                basic_result["replanning_success"] = False
                
        except Exception as parse_error:
            logger.error(f"[find] Failed to parse LLM replanning response: {parse_error}")
            basic_result["follow_up_tasks"] = []
            basic_result["replanning_success"] = False
            
    except Exception as llm_error:
        logger.error(f"[find] History-based replanning failed: {llm_error}")
        basic_result["follow_up_tasks"] = []
        basic_result["replanning_success"] = False
    
    return basic_result

# -------- 对外接口 --------
def run_action(node: Any, task: Dict[str, Any], **ctx) -> Dict[str, Any]:
    """与 robot_controller 的单步接口保持一致，使用基于History的增强版本"""
    robot = task.get("robot") or task.get("parameters", {}).get("robot")
    if not robot:
        return {"ok": False, "reason": "robot name missing"}
    
    # 使用基于History的增强版本
    return execute_find_with_history_replanning(node, robot, task.get("parameters", {}), **ctx)

# -------- 命令行测试 --------
if __name__ == "__main__":
    import random

    def fake_detect():
        if random.random() < 0.2:
            return [{
                "class": "cup",
                "conf": round(0.7 + random.random() * 0.3, 3),
                "center_xy": [random.randint(200, 1000), random.randint(200, 800)],
                "bbox_xyxy": [100, 100, 200, 200]
            }]
        return []

    def fake_rotate(robot, deg):
        print(f"[stub] rotate {robot} by {deg} deg")

    def fake_nav(robot, target):
        print(f"[stub] navigate {robot} to {target}")

    def fake_event(kind, payload):
        print(f"[event] {kind}: {payload}")

    res = execute_find(
        node=None,
        robot_name="robot1",
        params={
            "target_class": "cup",
            "save_as": "cup_target",
            "timeout_sec": 8,
            "search_waypoints": ["kitchen", [1.2, -0.5]]
        },
        detect_fn=fake_detect,
        rotate_fn=fake_rotate,
        navigate_to_fn=fake_nav,
        event_pub=fake_event
    )
    print("RESULT:", res)