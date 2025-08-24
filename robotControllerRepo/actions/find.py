# find.py
# -*- coding: utf-8 -*-
import os, sys, time, math, json
from typing import Any, Dict, List, Optional, Callable

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from loguru import logger
from ttsRepo.stream_tts import tts_manager
from robotControllerRepo.actions.navigate import navigate_after_follow
from robotControllerRepo.actions.pickup import pickup_item
from robotControllerRepo.actions.dropoff import dropoff_item
from llmParserRepo.yolo_perception import detect_once as yolo_detect_once
import config
from geometry_msgs.msg import Twist

# ========= 工具 =========
def get_robot_id() -> str:
    return config.get("robot_id")

def get_master_name() -> str:
    return config.get("master_id")

def _bb_path(robot_name: str) -> str:
    return f"/tmp/robot_blackboard_{robot_name}.json"

def _bb_read(robot_name: str) -> Dict[str, Any]:
    p = _bb_path(robot_name)
    if os.path.exists(p):
        try:
            with open(p, "r") as f:
                return json.load(f)
        except Exception as e:
            logger.warning(f"[find] Blackboard read failed: {e}")
    return {}

def _bb_write(robot_name: str, data: Dict[str, Any]) -> None:
    p = _bb_path(robot_name)
    try:
        with open(p, "w") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
    except Exception as e:
        logger.warning(f"[find] Blackboard write failed: {e}")

def bb_set(robot_name: str, key: str, value: Dict[str, Any]) -> None:
    db = _bb_read(robot_name)
    db.setdefault("objects", {})[key] = value
    _bb_write(robot_name, db)


# === 历史记录（从 memory.chattinglog.json 读取，NDJSON 每行一个 JSON） ===
_HISTORY_PATH = "memory.chattinglog.json"

def _recent_chat_messages_from_file(path: str, max_turns: int = 5):
    """
    读取最后 max_turns 条 user/assistant 类型的对话，返回
    [{"role":"user|assistant","content":"..."}]
    """
    if not os.path.exists(path):
        return []

    msgs = []
    try:
        from collections import deque
        with open(path, "r", encoding="utf-8") as f:
            # 只保留最后 100 行，避免大文件内存占用；然后再挑 user/assistant
            tail = deque(f, maxlen=100)
        for line in tail:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except Exception:
                continue

            typ = obj.get("type")
            content = obj.get("content")
            if typ in ("user", "assistant"):
                # assistant 可能是 JSON 字符串，把它当纯文本传给 LLM
                if isinstance(content, dict):
                    content = json.dumps(content, ensure_ascii=False)
                msgs.append({"role": "user" if typ == "user" else "assistant",
                             "content": str(content) if content is not None else ""})
        # 只取末尾 max_turns 条
        return msgs[-max_turns:]
    except Exception as e:
        logger.warning(f"[find] read history failed: {e}")
        return []

class _FileHistoryStore:
    """只实现 recent_chat_messages，用于 execute_find_with_llm_replanning"""
    def __init__(self, path: str):
        self.path = path
    def recent_chat_messages(self, max_turns: int = 5):
        return _recent_chat_messages_from_file(self.path, max_turns=max_turns)


# ========= 简单单帧滤波器（不需要历史） =========
class SimpleDetectionFilter:
    def __init__(self, confidence_boost: float = 0.1):
        self.confidence_boost = confidence_boost

    def pick_best(self, detections: List[Dict], item: str, conf_thres: float) -> Optional[Dict]:
        cands = [d for d in detections if d.get("class") == item and d.get("conf", 0.0) >= conf_thres]
        if not cands:
            return None

        image_center = [320, 240]
        best, best_score = None, -1e9
        for d in cands:
            conf = float(d.get("conf", 0.0))
            cx, cy = d.get("center_xy", [0, 0])
            x1, y1, x2, y2 = d.get("bbox_xyxy", [0, 0, 0, 0])
            w, h = max(0.0, x2 - x1), max(0.0, y2 - y1)
            area = w * h

            score = conf
            if conf > 0.8: score += 2 * self.confidence_boost
            elif conf > 0.6: score += self.confidence_boost

            center_dist = math.hypot(cx - image_center[0], cy - image_center[1])
            score += max(0.0, (1.0 - center_dist / 200.0) * 0.1)

            if 500 < area < 50000: score += 0.05
            if h > 0:
                ar = w / h
                score += 0.05 if 0.3 < ar < 3.0 else -0.05

            if score > best_score:
                best_score = score
                best = dict(d)
                best["filter_score"] = score
                best["filter_method"] = "single_frame_simple"

        if best is None or best_score < 0.4:
            return None
        return best

# ========= 参数优化 =========
def get_optimized_params(item: str) -> float:
    if item in ["cup", "bottle", "phone", "remote", "keys"]:
        return 0.4
    if item in ["person", "chair", "table", "sofa", "bed"]:
        return 0.6
    if item in ["book", "laptop", "mouse", "keyboard"]:
        return 0.5
    return 0.5

# ========= 轻量旋转（直接 cmd_vel） =========
_pub_cache: Dict[str, Any] = {}

def _get_cmd_vel_pub(node: Any, robot_name: str):
    """
    懒加载并缓存 /<robot>/cmd_vel publisher
    """
    if Twist is None:
        return None

    key = f"{robot_name}:/cmd_vel"
    pub = _pub_cache.get(key)
    if pub:
        return pub
    try:
        topic = f"/{robot_name}/cmd_vel"
        pub = node.create_publisher(Twist, topic, 10)
        _pub_cache[key] = pub
        logger.info(f"[find] ✅ Created publisher: {topic}")
        return pub
    except Exception as e:
        logger.error(f"[find] ❌ create_publisher failed: {e}")
        return None

def rotate_by_deg(node: Any,
                  robot_name: str,
                  delta_deg: float,
                  angular_speed_deg_s: float = None) -> bool:
    """
    以固定角速度转动指定角度（时间法）
    - 正角度：左转（+z）
    - 负角度：右转（-z）
    """
    if Twist is None:
        logger.error("[find] Twist unavailable, cannot rotate")
        return False

    if abs(delta_deg) < 1e-3:
        return True

    # 允许通过 config 覆盖默认角速度
    if angular_speed_deg_s is None:
        angular_speed_deg_s = float(config.get("find_rotate_speed_deg_s", 25.0))

    pub = _get_cmd_vel_pub(node, robot_name)
    if pub is None:
        return False

    duration = abs(delta_deg) / max(angular_speed_deg_s, 1e-6)
    wz = (angular_speed_deg_s * math.pi / 180.0) * (1.0 if delta_deg >= 0 else -1.0)

    tw = Twist()
    tw.angular.z = wz

    start = time.time()
    rate = 0.02  # 50Hz
    logger.info(f"[find] 🔄 rotate_by_deg: {delta_deg:.1f}°, ω={angular_speed_deg_s:.1f}°/s, t={duration:.2f}s")
    try:
        while time.time() - start < duration:
            pub.publish(tw)
            time.sleep(rate)
    finally:
        # 停止
        tw_stop = Twist()
        pub.publish(tw_stop)
    return True

def _rotate_scan_stepwise(node: Any,
                          robot_name: str,
                          detect_fn: Callable[[str, float], Optional[Dict]],
                          item: str,
                          *,
                          max_rot_deg: int = 360,
                          step_deg: int = 30,
                          pause: float = 0.5,
                          conf_thres: float = 0.5) -> Optional[Dict]:
    """
    分步旋转 + 每步检测（轻量）
    """
    logger.info(f"[find] 🔄 stepwise scan: max={max_rot_deg}°, step={step_deg}°, pause={pause}s")

    # 先看当前视角
    hit = detect_fn(item, conf_thres)
    if hit:
        logger.info(f"[find] 🎯 Found {item} in current view")
        return hit

    turned = 0
    while turned < max_rot_deg:
        if not rotate_by_deg(node, robot_name, step_deg):
            logger.error("[find] rotate_by_deg failed, abort scan")
            break
        time.sleep(pause)
        hit = detect_fn(item, conf_thres)
        if hit:
            logger.info(f"[find] 🎯 Found {item} after ~{turned+step_deg}° rotation")
            return hit
        turned += step_deg

    logger.info("[find] 🛑 scan finished, no hit")
    return None

# ========= 单帧检测 =========
def _single_frame_detect(item: str, conf_thres: float) -> Optional[Dict]:
    if yolo_detect_once is None:
        logger.error("❌ detect_once 不可用（未能导入 yolo_perception）")
        return None

    result = yolo_detect_once()
    if not result:
        return None

    detections = result.get("result", {}).get("detections", []) or []
    best = SimpleDetectionFilter(0.1).pick_best(detections, item=item, conf_thres=conf_thres)
    if best:
        best["detection_method"] = "single_frame_filtered"
    return best

# ========= find（仅三参，无 ctx） =========
def execute_find(node: Any, robot_name: str, item: str) -> Dict[str, Any]:
    """
    工作流程：
      1) 单帧检测
      2) 若未命中且允许旋转 -> 分步旋转扫描
      3) 若仍未命中且配置了 waypoints -> 逐点导航 + 单帧检测（必要时半圈扫描）
    """
    if not item:
        return {"ok": False, "found": False, "reason": "no item"}

    conf_thres = get_optimized_params(item)
    logger.info(f"[find] robot={robot_name} target={item} conf>={conf_thres}")

    # ---- 配置开关（可在 config 里覆盖）----
    use_rotate_scan: bool = bool(config.get("find_use_rotate_scan", True))
    rotate_step_deg: int  = int(config.get("find_rotate_step_deg", 30))
    rotate_max_deg: int   = int(config.get("find_rotate_max_deg", 360))
    rotate_pause_s: float = float(config.get("find_rotate_pause_s", 0.5))
    # 多点搜索目标（命名地点或语义地点）
    waypoints: List[str] = list(config.get("find_waypoints", []))  # 例如 ["table", "desk", "sofa"]

    # ========== 1) 当前视角 ==========
    hit = _single_frame_detect(item, conf_thres)
    if hit:
        return _on_found(robot_name, item, hit)

    # ========== 2) 旋转扫描 ==========
    if use_rotate_scan and Twist is not None:
        logger.info("[find] 🔄 No hit → start rotate scan")
        hit = _rotate_scan_stepwise(
            node=node,
            robot_name=robot_name,
            detect_fn=lambda i, th: _single_frame_detect(i, th),
            item=item,
            max_rot_deg=rotate_max_deg,
            step_deg=rotate_step_deg,
            pause=rotate_pause_s,
            conf_thres=conf_thres
        )
        if hit:
            return _on_found(robot_name, item, hit)
    else:
        if Twist is None:
            logger.warning("[find] ⚠️ Twist unavailable → skip rotate scan")
        else:
            logger.info("[find] ℹ️ Rotate scan disabled by config")

    # ========== 3) 多点搜索 ==========
    if waypoints:
        logger.info(f"[find] 🗺️ Start waypoint search: {waypoints}")
        for idx, target in enumerate(waypoints, 1):
            try:
                tts_manager.say(f"Moving to {target}")
                navigate_after_follow(node, robot_name, target)
            except Exception as e:
                logger.warning(f"[find] navigate_after_follow({target}) error: {e}")
                continue

            time.sleep(1.0)  # 稳定一下
            # 3.1 该点单帧检测
            hit = _single_frame_detect(item, conf_thres)
            if hit:
                return _on_found(robot_name, item, hit)

            # 3.2 该点半圈扫描（提高召回）
            if use_rotate_scan and Twist is not None:
                logger.info(f"[find] 🔄 Half-scan at waypoint {target}")
                hit = _rotate_scan_stepwise(
                    node=node,
                    robot_name=robot_name,
                    detect_fn=lambda i, th: _single_frame_detect(i, th),
                    item=item,
                    max_rot_deg=180,
                    step_deg=max(rotate_step_deg, 30),
                    pause=rotate_pause_s,
                    conf_thres=conf_thres
                )
                if hit:
                    return _on_found(robot_name, item, hit)

    # ========== 未找到 ==========
    tts_manager.say_sync("I couldn't find it.")
    logger.info(f"[find] not found: {item}")
    return {"ok": True, "found": False, "blackboard_key": item}

def _on_found(robot_name: str, item: str, hit: Dict[str, Any]) -> Dict[str, Any]:
    record = {
        "class": item,
        "center_xy": hit.get("center_xy"),
        "bbox_xyxy": hit.get("bbox_xyxy"),
        "map_xy": hit.get("map_xy") if "map_xy" in hit else None,
        "conf": float(hit.get("conf", 0.0)),
        "timestamp": time.time(),
        "filter_used": True,
        "filter_score": hit.get("filter_score"),
        "filter_method": hit.get("filter_method", "single_frame_simple"),
        "detection_method": hit.get("detection_method", "single_frame_filtered"),
        "detection_count": 1,
        "successful_frames": 1,
    }
    bb_set(robot_name, item, record)
    tts_manager.say_sync(f"I found the {item}.")
    logger.info(f"[find] ✅ found {item} (key={item}, conf={record['conf']:.3f})")
    return {"ok": True, "found": True, "blackboard_key": item, "record": record}

# ========= LLM 重规划（与之前相同） =========
def create_llm_replanning_function() -> Callable[[Dict, List[Dict], str], Dict]:
    def llm_replanning_function(discovery_context: Dict, chat_history: List[Dict], robot_name: str) -> Dict:
        try:
            import os, json
            import openai
            from textwrap import dedent

            api_key = os.getenv("OPENAI_API_KEY")
            if not api_key:
                return {"success": False, "error": "No OpenAI API key"}

            client = openai.OpenAI(api_key=api_key)
            found_object = discovery_context["found_object"]
            blackboard_key = found_object["blackboard_key"]

            system_prompt = dedent(f"""
            You are an intelligent robot task planner. A robot has just found an object and you need to decide what to do next.

            Robot: {get_robot_id()}
            Master: {get_master_name()}
            Found Object: {found_object["class"]} (confidence: {found_object["confidence"]:.3f})
            Detection Quality: {found_object["detection_quality"]}
            Blackboard Key: {blackboard_key}

            If user asked to "bring/deliver to me":
              navigate (to blackboard key) -> pickup -> navigate (to {get_master_name()}) -> dropoff
            If user asked only "collect":
              navigate (to blackboard key) -> pickup
            If only "find":
              no further actions.

            Strict JSON:
            {{
              "action_needed": true/false,
              "tasks": [{{"action": "navigate|pickup|dropoff|wait", "parameters": {{...}} }}],
              "reasoning": "brief"
            }}
            """).strip()

            history_text = "No recent conversation history available."
            if chat_history:
                history_text = "Recent conversation:\n" + "\n".join(
                    f"{m.get('role','unknown')}: {m.get('content','')}" for m in chat_history[-6:]
                )

            user_prompt = f"""{history_text}

The robot has successfully found a {found_object["class"]}. What should it do next?
"""
            resp = client.chat.completions.create(
                model="gpt-4o",
                messages=[{"role": "system", "content": system_prompt},
                          {"role": "user", "content": user_prompt}],
                temperature=0.3,
                max_tokens=500
            )
            raw = resp.choices[0].message.content.strip()
            if "```json" in raw:
                s = raw.find("```json") + 7
                e = raw.find("```", s)
                raw = raw[s:e].strip()
            elif "{" in raw and "}" in raw:
                s = raw.find("{")
                e = raw.rfind("}") + 1
                raw = raw[s:e]

            parsed = json.loads(raw)
            tasks = []
            if parsed.get("action_needed") and isinstance(parsed.get("tasks"), list):
                for t in parsed["tasks"]:
                    if isinstance(t, dict) and "action" in t and "parameters" in t:
                        tasks.append(t)

            return {"success": True,
                    "tasks": tasks,
                    "reasoning": parsed.get("reasoning", ""),
                    "action_needed": bool(parsed.get("action_needed", False))}
        except Exception as e:
            logger.error(f"[find] LLM replanning function error: {e}")
            return {"success": False, "error": str(e)}
    return llm_replanning_function

def execute_find_with_llm_replanning(
    node: Any,
    robot_name: str,
    item: str,
    llm_replanning_fn: Optional[Callable[[Dict[str, Any], List[Dict[str, str]], str], Dict[str, Any]]] = None,
    history_store: Any = None
) -> Dict[str, Any]:
    basic = execute_find(node, robot_name, item)
    if not basic.get("found"):
        logger.info(f"[find] {robot_name} didn't find {item}, no follow-up actions.")
        return basic

    if llm_replanning_fn is None:
        llm_replanning_fn = create_llm_replanning_function()

    record = basic.get("record", {}) or {}
    discovery_context = {
        "found_object": {
            "class": item,
            "confidence": record.get("conf", 0.0),
            "position": record.get("center_xy", [0, 0]),
            "blackboard_key": basic.get("blackboard_key", item),
            "detection_quality": {
                "filter_used": record.get("filter_used", False),
                "filter_score": record.get("filter_score"),
                "detection_method": record.get("detection_method", "unknown"),
            }
        },
        "robot_name": robot_name
    }

    chat_history = []
    if history_store:
        try:
            chat_history = history_store.recent_chat_messages(max_turns=5)
        except Exception as e:
            logger.warning(f"[find] get chat history failed: {e}")

    logger.info(f"[find] Calling LLM replanning...")
    try:
        plan = llm_replanning_fn(discovery_context=discovery_context,
                                 chat_history=chat_history,
                                 robot_name=robot_name)
        if plan.get("success") and plan.get("action_needed") and plan.get("tasks"):
            basic["follow_up_tasks"] = plan["tasks"]
            basic["replanning_success"] = True
            basic["replanning_source"] = "llm"
            basic["replanning_reasoning"] = plan.get("reasoning", "")
            logger.info(f"[find] LLM generated {len(plan['tasks'])} follow-up tasks")
            return basic
        else:
            logger.warning(f"[find] LLM replanning not used or failed: {plan.get('error','no tasks')}")
    except Exception as e:
        logger.error(f"[find] LLM replanning error: {e}")

    basic["follow_up_tasks"] = []
    basic["replanning_success"] = False
    basic["replanning_source"] = "fallback"
    basic["replanning_reasoning"] = "No LLM plan available"
    return basic

# ========= 对外入口 =========
def run_find(node: Any, robot_name: str, item: str) -> Dict[str, Any]:
    # 自动注入历史：只读 memory.chattinglog.json；不存在就不用历史（不兜底）
    history_store = _FileHistoryStore(_HISTORY_PATH) if os.path.exists(_HISTORY_PATH) else None

    result = execute_find_with_llm_replanning(
        node, robot_name, item,
        history_store=history_store
    )

    if not result.get("found"):
        return result

    follow_ups = result.get("follow_up_tasks", []) or []
    if not follow_ups:
        return result

    logger.info(f"[find] Executing {len(follow_ups)} follow-up tasks for {robot_name}")
    tts_manager.say_sync("Now executing follow-up tasks")

    for i, task in enumerate(follow_ups, 1):
        action = task.get("action")
        params = task.get("parameters", {}) or {}
        logger.info(f"[find] 📋 Follow-up {i}/{len(follow_ups)} → action={action} params={params}")

        try:
            if action == "navigate":
                target = params.get("target")
                if target:
                    navigate_after_follow(node, robot_name, target)
                else:
                    logger.warning("[find] navigate missing target")
            elif action == "pickup":
                obj = params.get("item")
                if obj:
                    ok = pickup_item(obj)
                    if not ok:
                        logger.warning("[find] pickup failed")
                else:
                    logger.warning("[find] pickup missing item")
            elif action == "dropoff":
                obj = params.get("item")
                if obj:
                    ok = dropoff_item(obj)
                    if not ok:
                        logger.warning("[find] dropoff failed")
                else:
                    logger.warning("[find] dropoff missing item")
            else:
                logger.warning(f"[find] Unsupported follow-up action: {action}")
        except Exception as e:
            logger.exception(f"[find] Error during follow-up task: {e}")

    tts_manager.say_sync("All follow-up tasks completed")
    return result
