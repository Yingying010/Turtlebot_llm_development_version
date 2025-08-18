#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Simple Perception-Aware LLM with History:
抓一帧 → YOLO 感知 → 记录历史（感知+对话）→ 注入“当前感知 + 历史感知 + 历史对话” → LLM 返回统一 JSON
"""

import os, sys
import re
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)
import json
import time
from typing import Dict, List, Optional, Tuple, Any
import traceback

import cv2
import numpy as np
from ultralytics import YOLO
import openai
from textwrap import dedent
from pathlib import Path
from collections import Counter


from WhisperRepo.whisper_recognizer import recognize
from ttsRepo.stream_tts import tts_manager
from loguru import logger
from config import config
import robotControllerRepo.robot_scheduler as robot_scheduler
# ================== 配置 ==================
DEFAULT_MODEL = os.getenv("LLM_MODEL", "gpt-4o-mini")
YOLO_WEIGHTS = os.getenv("YOLO_WEIGHTS", "yolov8n.pt")
YOLO_CONF   = float(os.getenv("YOLO_CONF", "0.25"))
YOLO_IOU    = float(os.getenv("YOLO_IOU", "0.45"))
YOLO_DEVICE = os.getenv("YOLO_DEVICE", None)   # 例: "0"
DEFAULT_SOURCE = int(os.getenv("YOLO_SOURCE", "0"))  # 0 = 默认摄像头

SESSION_ID = os.getenv("SESSION_ID", "default")
MEMORY_DIR = Path("./memory"); MEMORY_DIR.mkdir(parents=True, exist_ok=True)
MEMORY_PATH = MEMORY_DIR / f"{SESSION_ID}.jsonl"

MAX_TURNS = int(os.getenv("MAX_TURNS", "8"))  # 注入最近对话轮数
PERCEPTION_HISTORY_DEPTH = int(os.getenv("PERCEPTION_HISTORY_DEPTH", "5"))  # 注入最近感知摘要条数

# ——统一输出提示词（在此基础上增加“可选历史输入”说明）——
SYSTEM_PROMPT = dedent("""
You are a specialized robot command interpreter that receives natural language input and converts it into a structured JSON object. 
Your primary responsibility is to identify only the executor robots and to determine their number, and parse the complete task list for each robot. 
The parsed tasks must be assigned to the corresponding robot, represented as an ordered list, and expressed strictly in the JSON schema described below. 
The output must respect task dependencies, including sequential, parallel, and synchronous execution, by correctly applying the `sequence` or `sync_group` fields when necessary.
**CRITICAL RULE: A robot is an executor ONLY if it is directly commanded to perform an action. Robots that are merely referenced as targets, destinations, or objects in parameters are NOT executors and must NOT be included in the output.**

Context:
- You will also receive a system message named CURRENT_PERCEPTION describing objects currently seen by the camera. When building the perception report (see below), you must use ONLY that message (no hallucination). If it is empty, return an empty report.

========================
Output JSON Schema
========================
{
    "perception_report": {
        "timestamp": "<iso8601>",
        "summary": { "<class>": <count>, ... },
        "objects": [
            { "id":"<id>", "class":"<class>",
              "center_xy":[<float>,<float>],
              "bbox_xyxy":[<float>,<float>,<float>,<float>],
              "conf": <float>
            }
        ]
    },
    "robots": {
        "<robot_name>": [ <task_object>, <task_object>, ... ],
        "<robot_name>": [ ... ]
    }
    "response": "<short natural-language reply>"
}
                       
Perception report rules:
- Base it ONLY on CURRENT_PERCEPTION. If IDs are not provided, assign per-class incremental IDs like "cup1","cup2"; if a class appears only once, using "cup" without a number is acceptable.
- Keep numeric values as valid JSON floats. Include at most 20 objects.


A task_object has the following structure:
{
    "action": "<action_name>",
    "parameters": { ... },
    "sequence": <non-negative integer>,  //Optional dependency: sequential dependency
    "sync_group": <non-negative integer> //Optional dependency: synchronous dependency
}
                       
========================
Supported Actions and Parameters
========================
1. navigate  
   - To a named target: {"target": "<target_name>"}  
   - To coordinates: {"position": {"x": <num>, "y": <num>, "heading_deg": <num_or_null>}}
2. follow  
   {"target": "<target_name>"}
3. face  
   {"target": "<target_name>"}
4. collect  
   - From a user or semantic location: {"item": "<item>", "target": "<target>"}  
   - From coordinates: {"item": "<item>", "position": {"x": <num>, "y": <num>, "heading_deg": <num_or_null>}}
5. deliver  
   - To a user or semantic location: {"item": "<item>", "target": "<target>"}  
   - To coordinates: {"item": "<item>", "position": {"x": <num>, "y": <num>, "heading_deg": <num_or_null>}}
6. wait  
   {"duration_sec": <number>}
7. move  
   {"direction": "forward" or "backward", "value": <number>, "unit": "meter" or "second"}
8. turn  
   {"direction": "left" or "right", "value": <number>, "unit": "degree" or "second"}

Note:
- Numeric values must match the text exactly (preserve signs)
- Semantics take precedence over literals: If the semantics of a command clearly correspond to an action (e.g., "look at" corresponds to `face`), the action with the closest semantic match must be selected, not a literal that partially matches (e.g., "turn").
- Specialized actions take precedence over general actions: For example, `face` (a specialized orientation action) takes precedence over `turn` (a general rotation action).
- Special Rules for Collect and Deliver (NOTE: This rule takes precedence over general navigation logic!): When the user specifies a collect or deliver action and clearly provides a target or a position, the task should be generated directly without inserting a preceding navigation step. If the instruction only mentions collect or deliver without specifying a location or target, then the `"target"` field should be assigned `null` to indicate the missing information.
- Strictly adhere to the principle of semantic precedence
- Do not add parameters not explicitly specified in the command
- Provide more accurate semantic analysis of compound verb phrases  

========================
Special Rules for Robot Names
========================     
All robot names must be converted to lowercase alphanumeric format without spaces. 
The same formatting rules apply to all target names.
For example: convert "robot one" to "robot1"
                       
========================
Executor Robot Identification Rules
========================
**Core Principle**: Only robots that are COMMANDED to perform actions are executors.

**Executor Identification**:
- Robot is explicitly addressed or named as command subject: "robot X, do Y" / "request robot X to Y"
- For compound commands ("and then"), the SAME robot continues as executor unless explicitly changed

**Parameter Identification**:  
- Robots after prepositions (to/from/at) are parameters: "deliver to robot Y" → robot Y is parameter
- Robots as targets/destinations/sources are parameters, NOT executors

**Key Rule**: In "robot X do A and then do B to robot Y" → robot X executes both A and B, robot Y is only a parameter
                       
========================
Rules for Cross-Robot Dependencies
========================
1. Parallel execution:
If no dependencies are specified, the robot will execute its tasks in parallel, do not add any sequence and sync_group fields.
2. Sequential execution (sequence):
If the user instruction explicitly specifies the temporal relationship between tasks, the dependency label sequence must be used. sequence is a globally incrementing number starting from 0, used to indicate the execution order of tasks. Don't ignore the “sequence”: 0
3. Synchronous execution (sync_group):
If the user explicitly uses synchronization keywords or implicitly indicates synchronization through semantics or context, synchronization dependencies must be applied. All tasks that must be executed simultaneously by multiple robots must be assigned the same sync_group ID.
sync_group is a globally incrementing number starting from 0. Tasks with the same sync_group value must start execution at the same time.

========================
Response
========================
Return a short natural-language reply (<=120 characters) in the same language as the user's last message. 
If tasks are generated, briefly acknowledge them; otherwise, briefly summarize the perception.

According to the above rules, the following instructions are parsed:       
""").strip()


def _now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S")

def generate_robot_aliases(robot_id: str):
    """
    根据 robot_id 生成变体，例如:
    robot1 -> ['robot1', 'robot 1', 'robot one', 'robotone']
    """
    # 英文数字映射
    num_words = {
        1: "one", 2: "two", 3: "three", 4: "four", 5: "five",
        6: "six", 7: "seven", 8: "eight", 9: "nine", 10: "ten"
    }
 
    # 提取前缀和数字
    match = re.match(r"([a-zA-Z]+)(\d+)", robot_id)
    if not match:
        raise ValueError(f"Invalid robot_id format: {robot_id}")
 
    prefix, num_str = match.groups()
    num_int = int(num_str)
 
    # 生成变体
    aliases = [
        f"{prefix}{num_str}",              # robot1
        f"{prefix} {num_str}",             # robot 1
        f"{prefix} {num_words.get(num_int, num_str)}",  # robot one
        f"{prefix}{num_words.get(num_int, num_str)}"    # robotone
    ]
    return aliases

def _clean(text: str) -> str:
    return re.sub(r'[^\w\s]', '', text).lower().strip()

# ================== 历史存储（对话 + 感知摘要） ==================
class HistoryStore:
    """
    JSONL 存储。每行一个事件：
    - {"type":"user","content":"文本","time":...}
    - {"type":"assistant","content":"统一JSON字符串","time":...}
    - {"type":"perception","content":{观测摘要dict},"time":...}
    """
    def __init__(self, path: Path):
        self.path = path
        self.events: List[Dict] = []
        if self.path.exists():
            with self.path.open("r", encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        obj = json.loads(line)
                        self.events.append(obj)
                    except Exception:
                        pass

    def append(self, etype: str, content):
        obj = {"type": etype, "content": content, "time": _now_iso()}
        self.events.append(obj)
        with self.path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(obj, ensure_ascii=False) + "\n")

    def recent_chat_messages(self, max_turns: int) -> List[Dict[str, str]]:
        """
        转为 OpenAI chat messages（只取最近 max_turns 轮 user/assistant）
        """
        chat_pairs: List[Dict[str, str]] = []
        msgs: List[Dict[str, str]] = []
        # 提取 user/assistant
        for e in self.events:
            if e.get("type") == "user":
                msgs.append({"role": "user", "content": e.get("content", "")})
            elif e.get("type") == "assistant":
                msgs.append({"role": "assistant", "content": e.get("content", "")})
        # 取最近 max_turns*2 条（粗略足够）
        return msgs[-max_turns*2:] if max_turns > 0 else []

    def recent_perception_summaries(self, depth: int) -> List[Dict]:
        """
        取最近 depth 条感知摘要（type=perception）
        """
        snaps = [e["content"] for e in self.events if e.get("type") == "perception"]
        return snaps[-depth:] if depth > 0 else []

# ================== YOLO 感知 ==================
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
        res = self.model.predict(
            source=frame, conf=self.conf, iou=self.iou, device=self.device, verbose=False
        )[0]
        annotated = res.plot()
        h, w = res.orig_shape
        detections: List[Dict] = []
        names = res.names  # id -> class name
        if result_boxes := getattr(res, "boxes", None):
            if len(result_boxes) > 0:
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

    @staticmethod
    def grab_one_frame(source: int = DEFAULT_SOURCE) -> np.ndarray:
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            raise RuntimeError(f"无法打开视频源：{source}")
        ok, frame = cap.read()
        cap.release()
        if not ok:
            raise RuntimeError("无法读取摄像头帧")
        return frame

# ================== 感知上下文构造 ==================
def _assign_ids(dets: list[dict], topk: int = 20):
    dets = dets[:topk]
    totals = Counter(d["class"] for d in dets)  # 每类总数
    seen   = Counter()                           # 已编号计数
    objs = []
    for d in dets:
        cls = d["class"]
        seen[cls] += 1
        # 只有一个同类时用类名作ID，有多个时才加编号：cup1、cup2...
        oid = cls if totals[cls] == 1 else f"{cls}{seen[cls]}"
        objs.append((oid, cls, d))  # ← 关键修正：追加一个三元组
    return objs

def build_perception_context(det_json: Dict, topk: int = 20) -> str:
    assigned = _assign_ids(det_json.get("detections", []), topk)
    objs = [{
        "id": oid,
        "class": cls,
        "center_xy": [round(float(d["center_xy"][0]), 1), round(float(d["center_xy"][1]), 1)],
        "bbox_xyxy": [round(float(v), 1) for v in d["bbox_xyxy"]],
        "conf": d["conf"],
    } for (oid, cls, d) in assigned]
    ctx = {"timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"), "objects": objs}
    return "CURRENT_PERCEPTION:\n" + json.dumps(ctx, ensure_ascii=False)

def build_perception_summary(det_json: Dict, topk: int = 20) -> Dict:
    assigned = _assign_ids(det_json.get("detections", []), topk)
    objs = [{
        "id": oid,
        "class": cls,
        "center_xy": [float(d["center_xy"][0]), float(d["center_xy"][1])],
        "bbox_xyxy": [float(v) for v in d["bbox_xyxy"]],
        "conf": float(d["conf"]),
    } for (oid, cls, d) in assigned]
    summary = {}
    for _, cls, _ in assigned:
        summary[cls] = summary.get(cls, 0) + 1
    return {"timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"), "summary": summary, "objects": objs}

def build_perception_history_text(summaries: List[Dict]) -> Optional[str]:
    if not summaries:
        return None
    return "PERCEPTION_HISTORY:\n" + json.dumps(summaries, ensure_ascii=False)

# ================== LLM 解析（感知 + 历史 + 用户输入） ==================
class PerceptionAwareLLM:
    def __init__(self, model: str = DEFAULT_MODEL):
        api_key = os.getenv("OPENAI_API_KEY")
        self.client = openai.OpenAI(api_key=api_key) if api_key else None
        self.model = model

    @staticmethod
    def extract_json(text: str) -> str:
        t = text.strip()
        if t.startswith("```"):
            lines = t.splitlines()
            if lines and lines[0].startswith("```"): lines = lines[1:]
            if lines and lines[-1].startswith("```"): lines = lines[:-1]
            t = "\n".join(lines).strip()
        return t

    def parse(self,
              user_text: str,
              perception_context: str,
              chat_history_messages: List[Dict[str, str]],
              perception_history_text: Optional[str]) -> str:
        if not self.client:
            raise RuntimeError("no OPEN_API_KEY")
        messages: List[Dict[str, str]] = [
            {"role": "system", "content": SYSTEM_PROMPT}
        ]
        if perception_history_text:
            messages.append({"role": "system", "content": perception_history_text})
        messages.append({"role": "system", "content": perception_context})
        # 注入最近对话
        messages.extend(chat_history_messages[-MAX_TURNS*2:])
        # 当前用户输入
        messages.append({"role": "user", "content": user_text})

        resp = self.client.chat.completions.create(
            model=self.model,
            messages=messages,
            temperature=0.1,
            max_tokens=512
        )
        return resp.choices[0].message.content.strip()

# ================== 一次性：感知 + 解析 + 历史写入 ==================
def perceive_and_parse(user_instruction: str,
                       source: int = DEFAULT_SOURCE,
                       show_window: bool = False,
                       save_annotated: Optional[str] = None) -> Dict:
    """
    1. 抓取一帧并做 YOLO 检测
    2. 构造 CURRENT_PERCEPTION 上下文
    3. 读取历史（对话+感知摘要）并注入
    4. 调用 LLM 返回统一 JSON；把此次对话与感知摘要写回历史
    """
    # --- 感知 ---
    perceiver = YOLOPerceiver()
    frame = YOLOPerceiver.grab_one_frame(source)
    annotated, det_json = perceiver.detect_frame(frame)

    if show_window:
        cv2.imshow("Perception", annotated)
        cv2.waitKey(800)
        cv2.destroyAllWindows()
    if save_annotated:
        cv2.imwrite(save_annotated, annotated)

    # --- 历史 ---
    store = HistoryStore(MEMORY_PATH)
    chat_hist = store.recent_chat_messages(MAX_TURNS)
    perception_hist_summaries = store.recent_perception_summaries(PERCEPTION_HISTORY_DEPTH)

    # 当前感知文本 & 历史感知文本
    perception_ctx = build_perception_context(det_json)
    perception_hist_text = build_perception_history_text(perception_hist_summaries)

    # --- LLM ---
    llm = PerceptionAwareLLM()
    raw = llm.parse(
        user_text=user_instruction,
        perception_context=perception_ctx,
        chat_history_messages=chat_hist,
        perception_history_text=perception_hist_text
    )

    # --- 解析输出，并写历史 ---
    try:
        parsed = json.loads(llm.extract_json(raw))
    except Exception:
        parsed = {"raw": raw, "note": "LLM 输出非严格 JSON，已原样返回在 raw 字段中"}

    # 写入历史（先写感知，然后对话）
    store.append("perception", build_perception_summary(det_json))
    store.append("user", user_instruction)
    # 为了便于人读/调试，把 assistant 写入统一 JSON 字符串（与 messages 同源）
    store.append("assistant", json.dumps(parsed, ensure_ascii=False))

    return {
        "command": parsed
    }

def run_conversation_loop() -> Optional[Dict[str, Any]]:
    robot_id = config.get("robot_id") 
    aliases = generate_robot_aliases(robot_id)
    logger.info(f"💡 Mode: {'Chat' if config.get('chat_or_instruct') else 'Control'}")

    while True:
        while True:
            try:
                raw_text = recognize(delay=3).strip()
            except Exception as e:
                logger.warning(f"🎙️ recognition failed: {e}")
                tts_manager.say("Sorry, could not hear you.")
                time.sleep(2)
                continue

            user_input = _clean(raw_text)
            if not user_input or user_input == "blank_audio" or not any(alias in user_input for alias in aliases):
                continue
            else:
                break
    
        exit_keywords = ["exit", "stop talking", "quit", "okay bye", "goodbye", "shut up", "i want to change the chat mode", "ending of this mode","ok finish", "ending this mode"]

        if any(kw in user_input.lower() for kw in exit_keywords):
            tts_manager.say("Exiting voice control.")
            time.sleep(1)
            break

        logger.info(f"🗣️ You said: {user_input}")
        tts_manager.say("User command has been received")
        time.sleep(0.5)
        tts_manager.say(f"Your cammand is {user_input}. Currently parsing it for you ....")
        time.sleep(0.5)

        try:
            out = perceive_and_parse(user_input, source=DEFAULT_SOURCE, show_window=True, save_annotated=None)

            print("✅ Parsed result:", json.dumps(out, indent=2))

            if out:
                # robot_scheduler.run(json_response)
                resp = out.get("command", {}).get("response")
                print(resp or "i can't give you any response")
                tts_manager.say(resp)
                time.sleep(1)
            else:
                tts_manager.say("Could not understand the command.")
                time.sleep(1)

        except Exception:
            logger.warning(f"⚠️ Error during LLM or parsing: \n{traceback.format_exc()}")
            tts_manager.say("Something went wrong.")


        # call scheduler
        execution_json = out["command"]["robots"]
        if execution_json:
            isSchedule = robot_scheduler.run(execution_json)
            if isSchedule == True:
                logger.info("✅ Command(s) executed successfully.")
                tts_manager.say("Command executed.")
                time.sleep(1)
            else:
                logger.warning("⚠️ Failed")
                tts_manager.say("Command execution failed, please re-give the command.")
                time.sleep(3)


# ================== 示例（可删） ==================
if __name__ == "__main__":
    # 示例：抓一帧 + “基于当前感知 + 历史感知 + 历史对话”解析为统一 JSON
    # instr = "firstly car 1 and Robot 2 start simultaneously: car 1 navigates to x is 1, y is 2 and robot 2 turns right 90 degrees secondly car 1 waits for 5 seconds"
    # out = perceive_and_parse(instr, source=DEFAULT_SOURCE, show_window=True, save_annotated=None)
    # print(json.dumps(out, indent=2, ensure_ascii=False))

    json_result = run_conversation_loop()
