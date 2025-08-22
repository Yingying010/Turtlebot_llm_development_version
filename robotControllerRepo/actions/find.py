# find.py
# -*- coding: utf-8 -*-
"""
通用“查找”动作模块（独立版）
- 旋转扫描 + 多点搜索
- 命中后把目标信息写入黑板（/tmp/robot_blackboard_<robot>.json）
- 供 robot_controller.py 直接调用：execute_find(node, robot_name, params, **ctx)

params 示例:
{
    "target_class": "cup",
    "save_as": "cup_target",         # 写入黑板的 key（默认与 target_class 同名）
    "timeout_sec": 25,
    "conf_thres": 0.5,
    "spin_scan": true,
    "max_rot_deg": 360,
    "step_deg": 30,
    "search_waypoints": ["kitchen", "desk", [1.2, -0.4]]
}

ctx 需要注入的函数:
- detect_fn(): -> List[ {class, conf, center_xy, bbox_xyxy, map_xy?} ]
- rotate_fn(robot_name:str, deg:float)
- navigate_to_fn(robot_name:str, target:any)
- (可选) event_pub(kind:str, payload:dict)
"""

from __future__ import annotations
import os, sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

import os
import json
import time
from typing import Any, Dict, List, Optional
from ttsRepo.stream_tts import tts_manager


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

def _rotate_scan(robot_name: str,
                 rotate_fn,
                 detect_fn,
                 target_class: str,
                 *,
                 max_rot_deg: int = 360,
                 step_deg: int = 30,
                 pause: float = 0.4,
                 conf_thres: float = 0.5) -> Optional[Dict[str, Any]]:
    """
    原地旋转扫描：每 step_deg 旋转一次，旋转后暂停 pause 秒，做一次检测
    先看一眼当前帧，避免无谓旋转
    """
    # 先看当前帧
    hit = _scan_once_with_yolo(detect_fn, target_class, conf_thres)
    if hit:
        logger.debug("hit before spin-scan")
        return hit

    turned = 0
    while turned < max_rot_deg:
        try:
            rotate_fn(robot_name, step_deg)
        except Exception as e:
            logger.error(f"rotate_fn error: {e}")
            break
        time.sleep(pause)
        hit = _scan_once_with_yolo(detect_fn, target_class, conf_thres)
        if hit:
            return hit
        turned += step_deg
    return None

# -------- 核心入口 --------
def execute_find(node: Any,
                 robot_name: str,
                 params: Dict[str, Any],
                 **ctx) -> Dict[str, Any]:
    """
    查找动作：旋转扫描 + 多点搜索
    返回: {"ok": True, "found": bool, "blackboard_key": save_as, "record": {...}?}

    必要 ctx:
      - detect_fn()
      - rotate_fn(robot, deg)
      - navigate_to_fn(robot, target)
    可选 ctx:
      - event_pub(kind, payload)
    """
    target: str = params.get("target_class", "cup")
    save_as: str = params.get("save_as", target)
    timeout_sec: float = float(params.get("timeout_sec", 25))
    conf_thres: float = float(params.get("conf_thres", 0.5))

    use_spin: bool = bool(params.get("spin_scan", True))
    max_rot_deg: int = int(params.get("max_rot_deg", 360))
    step_deg: int = int(params.get("step_deg", 30))
    waypoints: List[Any] = params.get("search_waypoints", [])

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

    t0 = time.time()
    tts_manager.say(f"Okay, I'll look around for the {target}.")
    logger.info(f"[find] robot={robot_name} target={target} timeout={timeout_sec}s conf>={conf_thres}")

    hit: Optional[Dict[str, Any]] = None

    # 1) 当前帧
    hit = _scan_once_with_yolo(detect_fn, target, conf_thres)

    # 2) 旋转扫描
    if not hit and use_spin:
        hit = _rotate_scan(robot_name, rotate_fn, detect_fn, target,
                           max_rot_deg=max_rot_deg, step_deg=step_deg,
                           pause=0.4, conf_thres=conf_thres)

    # 3) 多点搜索
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
            # 到达后快速检查 + 半圈旋转补扫
            time.sleep(0.35)
            hit = _scan_once_with_yolo(detect_fn, target, conf_thres) \
                  or (use_spin and _rotate_scan(robot_name, rotate_fn, detect_fn, target,
                                                max_rot_deg=min(180, max_rot_deg),
                                                step_deg=step_deg, pause=0.35,
                                                conf_thres=conf_thres))
            if hit:
                break

    # 4) 收尾：写黑板 / 事件 / 语音
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

    record = {
        "class": target,
        "center_xy": hit.get("center_xy"),
        "bbox_xyxy": hit.get("bbox_xyxy"),
        "map_xy": hit.get("map_xy"),
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
    return execute_find(node, robot, task.get("parameters", {}) or {}, **ctx)


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
