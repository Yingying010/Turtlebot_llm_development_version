# find.py
# -*- coding: utf-8 -*-
from __future__ import annotations
import os, sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

import os
import json
import time
import cv2
import json
from typing import Any, Dict, List, Optional
from ttsRepo.stream_tts import tts_manager
from llmParserRepo.gpt_yolo_localParser import YOLOPerceiver
from llmParserRepo.gpt_yolo_localParser import PerceptionAwareLLM, build_perception_context, HistoryStore, MEMORY_PATH




# -------- 日志与TTS --------
try:
    from loguru import logger
except Exception:
    class _DummyLogger:
        def info(self, *a, **k): print("[INFO]", *a)
        def warning(self, *a, **k): print("[WARN]", *a)
        def error(self, *a, **k): print("[ERROR]", *a)
        def debug(self, *a, **k): print("[DEBUG]", *a)
    logger = _DummyLogger()  # type: ignore

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

# -------- YOLO/感知 --------
def _scan_once_with_yolo(detect_fn, target_class: str, conf_thres: float = 0.5) -> Optional[Dict[str, Any]]:
    """
    detect_fn() -> List[ {class, conf, center_xy, bbox_xyxy, map_xy?} ]
    返回符合 class 且置信度最高的目标；否则 None
    """
    try:
        objs: List[Dict[str, Any]] = detect_fn() or []
    except Exception as e:
        logger.error(f"detect_fn failed: {e}")
        return None
    cand = [o for o in objs if o.get("class") == target_class and float(o.get("conf", 0)) >= conf_thres]
    return max(cand, key=lambda o: float(o.get("conf", 0))) if cand else None

# def _rotate_scan(robot_name: str,
#                  rotate_fn,
#                  detect_fn,
#                  target_class: str,
#                  *,
#                  max_rot_deg: int = 360,
#                  step_deg: int = 30,
#                  pause: float = 0.4,
#                  conf_thres: float = 0.5) -> Optional[Dict[str, Any]]:
#     """
#     原地旋转扫描：每 step_deg 旋转一次，旋转后暂停 pause 秒，做一次检测
#     先看一眼当前帧，避免无谓旋转
#     """
#     # 先看当前帧
#     hit = _scan_once_with_yolo(detect_fn, target_class, conf_thres)
#     if hit:
#         logger.debug("hit before spin-scan")
#         return hit

#     turned = 0
#     while turned < max_rot_deg:
#         try:
#             rotate_fn(robot_name, step_deg)
#         except Exception as e:
#             logger.error(f"rotate_fn error: {e}")
#             break
#         time.sleep(pause)
#         hit = _scan_once_with_yolo(detect_fn, target_class, conf_thres)
#         if hit:
#             return hit
#         turned += step_deg
#     return None


def _rotate_scan_realtime(robot_name: str,
                           rotate_fn,
                           detect_model,
                           *,
                           max_rot_deg: int = 360,
                           deg_per_frame: int = 10,
                           fps: int = 5,
                           conf_thres: float = 0.5,
                           video_source: int = 0,
                           timeout_sec: float = 10.0) -> Optional[Dict[str, Any]]:
    """
    实时版本：机器人边旋转，边持续检测图像，命中目标立即返回。
    """
    logger.info("[find] 🚀 Starting real-time rotate+detect scan...")
    
    # 🔥 添加调试信息
    logger.info(f"[find] 🔧 Scan parameters: max_rot={max_rot_deg}°, deg_per_frame={deg_per_frame}°, fps={fps}, timeout={timeout_sec}s")
    logger.info(f"[find] 🔧 Rotate function: {rotate_fn}")
    logger.info(f"[find] 🔧 Robot name: {robot_name}")
    
    t0 = time.time()
    total_rotated = 0
    frame_interval = 1.0 / fps

    cap = cv2.VideoCapture(video_source)
    if not cap.isOpened():
        logger.error("❌ Failed to open camera for real-time scan")
        return None

    try:
        frame_count = 0
        while total_rotated < max_rot_deg and (time.time() - t0) < timeout_sec:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue

            frame_count += 1
            logger.debug(f"[find] 📷 Processing frame {frame_count}, rotated={total_rotated}°")

            # YOLO 检测
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
                    cap.release()
                    return {
                        "class": cls_name,
                        "conf": conf,
                        "bbox_xyxy": [x1, y1, x2, y2],
                        "center_xy": [cx, cy]
                    }

            # 🔥 关键调试：没检测到目标 → 继续旋转
            if callable(rotate_fn):
                logger.info(f"[find] 🔄 Rotating {deg_per_frame}° (total: {total_rotated}°/{max_rot_deg}°)")
                try:
                    rotate_fn(robot_name, deg_per_frame)
                    logger.info(f"[find] ✅ Rotation command sent successfully")
                except Exception as e:
                    logger.error(f"[find] ❌ Rotation failed: {e}")
                    break
            else:
                logger.error(f"[find] ❌ rotate_fn is not callable: {rotate_fn}")
                break
                
            total_rotated += deg_per_frame
            time.sleep(frame_interval)

        logger.info("🛑 Scan complete, no object found")
        logger.info(f"[find] 📊 Final stats: frames={frame_count}, total_rotated={total_rotated}°, elapsed={time.time()-t0:.1f}s")
        return None

    finally:
        cap.release()


def get_optimized_params(target_class: str, has_waypoints: bool) -> tuple:
    """根据物体类型和搜索策略优化检测参数
    
    Args:
        target_class: 要搜索的物体类型
        has_waypoints: 是否有多个搜索点
        
    Returns:
        (conf_thres, max_rot_deg): 优化后的参数
    """
    # 基础配置
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

def execute_find_with_history_replanning(node: Any,
                                        robot_name: str,
                                        params: Dict[str, Any],
                                        **ctx) -> Dict[str, Any]:
    """
    基于History的find增强版：找到物体后，利用历史记录重新解析用户意图
    """
    # === 1. 执行基础find逻辑 ===
    basic_result = execute_find(node, robot_name, params, **ctx)
    
    if not basic_result.get("found", False):
        logger.info(f"[find] {robot_name} didn't find {params.get('target_class')}, no follow-up actions needed")
        return basic_result
    
    # === 2. 找到了物体，利用History进行重新规划 ===
    logger.info(f"[find] {robot_name} found object, initiating history-based replanning...")
    
    try:
        # 🔥 读取本地历史记录
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
        llm = PerceptionAwareLLM()
        raw_response = llm.parse(
            user_text=replanning_prompt,
            perception_context=perception_ctx,
            chat_history_messages=recent_messages,  # 🔥 传入历史对话上下文
            perception_history_text=None
        )
        
        # 解析LLM响应
        try:
            from llmParserRepo.gpt_yolo_localParser import safe_json_parse
            replanned_data = safe_json_parse(raw_response)
            
            # 提取新的任务列表
            if "robots" in replanned_data and robot_name in replanned_data["robots"]:
                new_tasks = replanned_data["robots"][robot_name]
                logger.info(f"[find] Generated {len(new_tasks)} follow-up tasks based on history")
                
                # 将新任务添加到结果中
                basic_result["follow_up_tasks"] = new_tasks
                basic_result["replanning_success"] = True
                basic_result["replanning_source"] = "history"
                
                # 🔥 将重新规划的结果也写入历史记录
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

# -------- 核心入口 --------
def execute_find(node: Any,
                 robot_name: str,
                 params: Dict[str, Any],
                 **ctx) -> Dict[str, Any]:
    """
    查找动作：旋转扫描 + 多点搜索
    返回: {"ok": True, "found": bool, "blackboard_key": save_as, "record": {...}?}
    """
    # === 1. 解析用户参数 ===
    target: str = params.get("target_class", "cup")
    save_as: str = params.get("save_as", target)
    timeout_sec: float = float(params.get("timeout_sec", 25))
    use_spin: bool = bool(params.get("spin_scan", True))
    waypoints: List[Any] = params.get("search_waypoints", [])

    # === 2. 🔥 调用智能参数优化 ===
    conf_thres, max_rot_deg = get_optimized_params(target, bool(waypoints))
    
    # === 3. 验证必要的上下文函数 ===
    detect_fn = ctx.get("detect_fn")
    rotate_fn = ctx.get("rotate_fn")
    navigate_to_fn = ctx.get("navigate_to_fn")
    event_pub = ctx.get("event_pub")

    # 🔥 添加调试信息
    rotate_fn = ctx.get("rotate_fn")
    logger.info(f"[find] 🔧 Context functions check:")
    logger.info(f"  - detect_fn: {callable(detect_fn)}")
    logger.info(f"  - rotate_fn: {callable(rotate_fn)}")
    logger.info(f"  - navigate_to_fn: {callable(navigate_to_fn)}")

    if use_spin and callable(rotate_fn):
        # 🔥 测试旋转函数
        logger.info(f"[find] 🧪 Testing rotation function...")
        try:
            rotate_fn(robot_name, 10)  # 测试旋转10度
            logger.info(f"[find] ✅ Rotation test successful")
            time.sleep(1)  # 给机器人时间执行
        except Exception as e:
            logger.error(f"[find] ❌ Rotation test failed: {e}")

    if not callable(detect_fn):
        return {"ok": False, "found": False, "reason": "detect_fn missing"}
    if use_spin and not callable(rotate_fn):
        logger.warning("spin_scan=True but rotate_fn missing, will skip spin scan.")
        use_spin = False
    if waypoints and not callable(navigate_to_fn):
        logger.warning("search_waypoints provided but navigate_to_fn missing, will skip waypoint search.")
        waypoints = []

    # === 4. 开始搜索流程 ===
    t0 = time.time()
    tts_manager.say(f"Okay, I'll look around for the {target}.")
    logger.info(f"[find] robot={robot_name} target={target} timeout={timeout_sec}s conf>={conf_thres}")

    hit: Optional[Dict[str, Any]] = None

    # === 5. 实例化 YOLO 模型 ===
    detect_model = YOLOPerceiver()

    # === 6. 当前帧检测 ===
    hit = _scan_once_with_yolo(detect_fn, target, conf_thres)

    # === 7. 原地旋转实时检测 ===
    if not hit and use_spin:
        hit = _rotate_scan_realtime(
            robot_name=robot_name,
            rotate_fn=rotate_fn,
            detect_model=detect_model,
            conf_thres=conf_thres,          # 🔥 使用优化后的参数
            video_source=0,
            fps=5,
            deg_per_frame=10,
            max_rot_deg=max_rot_deg,        # 🔥 使用优化后的参数
            timeout_sec=timeout_sec - (time.time() - t0)
        )

    # === 8. 多点搜索 ===
    if not hit and waypoints:
        for wp in waypoints:
            if time.time() - t0 > timeout_sec:
                logger.info("[find] timeout during waypoints loop")
                break
            try:
                navigate_to_fn(robot_name, wp)
            except Exception as e:
                logger.warning(f"navigate_to_fn error on {wp}: {e}")
                continue

            time.sleep(0.35)
            hit = _scan_once_with_yolo(detect_fn, target, conf_thres)  # 🔥 使用优化参数

            if not hit and use_spin:
                remaining_time = timeout_sec - (time.time() - t0)
                hit = _rotate_scan_realtime(
                    robot_name=robot_name,
                    rotate_fn=rotate_fn,
                    detect_model=detect_model,
                    conf_thres=conf_thres,      # 🔥 使用优化参数
                    video_source=0,
                    fps=5,
                    deg_per_frame=10,
                    max_rot_deg=min(180, max_rot_deg),  # 多点搜索时限制旋转角度
                    timeout_sec=remaining_time
                )

            if hit:
                break

    # === 9. 处理搜索结果 ===
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

    # === 10. 记录成功结果 ===
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


# -------- 兼容：给 robot_controller 直接用的小包装 --------
def run_action(node: Any, task: Dict[str, Any], **ctx) -> Dict[str, Any]:
    """
    与 robot_controller.execute_action 的单步接口保持一致：
    task = {"robot": "robot1", "action": "find", "parameters": {...}}
    """
    robot = task.get("robot") or task.get("parameters", {}).get("robot")
    if not robot:
        return {"ok": False, "reason": "robot name missing"}
    return execute_find_with_history_replanning(node, robot, task.get("parameters", {}), **ctx)


# -------- 可选：命令行自测（stub） --------
if __name__ == "__main__":
    # 你可以临时跑：python3 find.py 观察日志是否正常（以下是简易桩）
    import random

    def fake_detect():
        # 20% 几率“看到杯子”
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
            "spin_scan": True,
            "max_rot_deg": 360,
            "step_deg": 45,
            "search_waypoints": ["kitchen", [1.2, -0.5]]
        },
        detect_fn=fake_detect,
        rotate_fn=fake_rotate,
        navigate_to_fn=fake_nav,
        event_pub=fake_event
    )
    print("RESULT:", res)
