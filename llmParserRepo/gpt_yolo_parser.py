import os, sys, re, json, time, traceback, threading
from typing import Dict, List, Optional, Tuple, Any
from textwrap import dedent
from pathlib import Path
from collections import Counter
 
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)
 
import cv2
import numpy as np
from ultralytics import YOLO
import openai
from loguru import logger
 
from WhisperRepo.whisper_recognizer import recognize
from ttsRepo.stream_tts import tts_manager
import config
import robotControllerRepo.robot_scheduler as robot_scheduler
 
# ================== 配置 ==================
DEFAULT_MODEL = os.getenv("LLM_MODEL", "gpt-4o-mini")
YOLO_WEIGHTS = os.getenv("YOLO_WEIGHTS", "yolov8n.pt")
YOLO_CONF   = float(os.getenv("YOLO_CONF", "0.25"))
YOLO_IOU    = float(os.getenv("YOLO_IOU", "0.45"))

YOLO_DEVICE = os.getenv("YOLO_DEVICE", None)
 
SESSION_ID = os.getenv("SESSION_ID", "chattinglog")
MEMORY_DIR = Path("./memory"); MEMORY_DIR.mkdir(parents=True, exist_ok=True)
MEMORY_PATH = MEMORY_DIR / f"{SESSION_ID}.jsonl"
 
MAX_TURNS = int(os.getenv("MAX_TURNS", "8"))
PERCEPTION_HISTORY_DEPTH = int(os.getenv("PERCEPTION_HISTORY_DEPTH", "5"))

DEFAULT_SOURCE = "udp://@:8888?fifo_size=1000000&overrun_nonfatal=1&buffer_size=1000000&probesize=32&analyzeduration=0"

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
        ],
        "relations": [
            { "subject":"<id>", "predicate":"on|left_of|right_of|near", "object":"<id>" }
        ]
    },
    "robots": {
        "<robot_name>": [ <task_object>, <task_object>, ... ],
        "<robot_name>": [ ... ]
    },
    "response": "<short natural-language reply>"
}
                       
========================
Perception report rules
========================
- Base it ONLY on CURRENT_PERCEPTION. ...
- Keep numeric values as valid JSON floats. Include at most 20 objects.
- Spatial relations (tiny rules, no hallucination):
  * Ignore objects with conf < 0.5. Let W,H be image width/height; for each object get (x1,y1,x2,y2,cx,cy,bottom_y,w,h).
  * left_of / right_of: if cx_a + 0.10*W < cx_b → left_of; if cx_a - 0.10*W > cx_b → right_of.
  * above / below: if cy_a + 0.10*H < cy_b → above; if cy_a - 0.10*H > cy_b → below.
  * on: if bottom_y_a ∈ [y1_b - 0.10*H, y2_b] AND horizontal_overlap(a,b) ≥ 0.30*min(w_a, w_b).
  * near: if distance(center_a, center_b) ≤ 0.25*sqrt(W^2 + H^2) and no stronger relation already chosen.
  * Priority: on > above/below > left_of/right_of > near. Max 5 relations total.


========================
A task_object has the following structure:
========================
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
If tasks are generated, briefly acknowledge them. If the user requests a scene description, describe both objects and relations if available. If the user is chatting with you, engage in free conversation with the user.

     
""").strip()


def _now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S")

def _clean(text: str) -> str:
    return re.sub(r'[^\w\s]', '', text).lower().strip()


def get_robot_id():
    return config.get("robot_id")

def get_master_name():
    return config.get("master_id")

import json
import re

def safe_json_parse(raw_str: str) -> dict:
    """尝试从 raw_str 中恢复出合法 JSON"""
    try:
        return json.loads(raw_str)
    except Exception:
        pass

    # 尝试去除 markdown ```json ``` 包裹
    raw_str = re.sub(r"^```(?:json)?\n?", "", raw_str.strip())
    raw_str = re.sub(r"\n?```$", "", raw_str.strip())

    # 尝试找出第一个和最后一个 { } 的位置，截取中间部分
    start = raw_str.find("{")
    end = raw_str.rfind("}")
    if start != -1 and end != -1:
        trimmed = raw_str[start:end+1]
        try:
            return json.loads(trimmed)
        except Exception:
            pass

    # 尝试补一个 }
    try:
        fixed = raw_str + "}"
        return json.loads(fixed)
    except Exception:
        pass

    # 最后失败就返回包装
    return {
        "raw": raw_str,
        "note": "Failed to parse as strict JSON. Original output preserved in raw."
    }


# ================== 历史存储 ==================
class HistoryStore:
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
        msgs: List[Dict[str, str]] = []
        for e in self.events:
            if e.get("type") == "user":
                msgs.append({"role": "user", "content": e.get("content", "")})
            elif e.get("type") == "assistant":
                msgs.append({"role": "assistant", "content": e.get("content", "")})
        return msgs[-max_turns*2:] if max_turns > 0 else []

    def recent_perception_summaries(self, depth: int) -> List[Dict]:
        snaps = [e["content"] for e in self.events if e.get("type") == "perception"]
        return snaps[-depth:] if depth > 0 else []
    
    def clear(self):
        self.events = []
        if self.path.exists():
            self.path.unlink()

# ================== OpenCV 统一视频源（关键修正） ==================
class GlobalVideoSource:
    def __init__(self, source: Any):
        self.source = source
        self.backend = cv2.CAP_FFMPEG if isinstance(source, str) else 0
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
                    raise RuntimeError(f"Can't open video source: {self.source}")
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
            m = float(frame.mean()); v = float(frame.var())
            h, w = frame.shape[:2]
            if h < 32 or w < 32:
                continue
            if m < 1.0 or v < 5.0:
                continue
            return frame
        raise RuntimeError("Reading video frame timeout/valid frame not arrived")
    
    def grab_one_frame_once(source: str,
                        warmup_frames: int = 8,
                        open_timeout_sec: float = 8.0) -> np.ndarray:
        backend = cv2.CAP_FFMPEG if isinstance(source, str) else 0
        t0 = time.time()
        cap = cv2.VideoCapture(source, backend)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        while time.time() - t0 < open_timeout_sec:
            ok, _ = cap.read()
            if ok:
                break
            time.sleep(0.05)

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
            raise RuntimeError("Unable to read valid frame")
        return frame


    def release(self):
        with self.lock:
            if self.cap is not None:
                self.cap.release()
                self.cap = None
                self.ready = False

# ================== YOLO 感知（关键：只接受 np.ndarray） ==================
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
        names = res.names

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

# ================== 感知上下文构造 ==================
def _assign_ids(dets: List[Dict], topk: int = 20):
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
    summary: Dict[str, int] = {}
    for _, cls, _ in assigned:
        summary[cls] = summary.get(cls, 0) + 1
    return {"timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"), "summary": summary, "objects": objs}

def build_perception_history_text(summaries: List[Dict]) -> Optional[str]:
    if not summaries:
        return None
    return "PERCEPTION_HISTORY:\n" + json.dumps(summaries, ensure_ascii=False)

# ================== LLM 解析 ==================
class PerceptionAwareLLM:
    def __init__(self, model: str = DEFAULT_MODEL):
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise RuntimeError("OPENAI_API_KEY no settings")
        self.client = openai.OpenAI(api_key=api_key)
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
        
        post_identity = dedent(f"""
        Context variables:
        Let me first define your identity. You are a highly intelligent indoor robot, and your name is {get_robot_id()}, and your master's name is {get_master_name()}. You can understand voice commands and assist users in completing various tasks. You can move freely, navigate, collect, and deliver items. You can also communicate freely with humans. In addition, you can collaborate with robot2. Next, here's what the user said to you:
        """).strip()
        
        messages: List[Dict[str, str]] = [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "system", "content": "You must only respond with valid JSON. Do not include any markdown, code block, or extra explanation."},
            {"role": "system", "content": post_identity}
        ]
        if perception_history_text:
            messages.append({"role": "system", "content": perception_history_text})
        messages.append({"role": "system", "content": perception_context})
        messages.extend(chat_history_messages[-MAX_TURNS*2:])
        messages.append({"role": "user", "content": user_text})

        resp = self.client.chat.completions.create(
            model=self.model,
            messages=messages,
            temperature=0.1,
            max_tokens=512
        )
        return resp.choices[0].message.content.strip()

# ================== 一次性：感知 + 解析 + 历史写入 ==================
VIDEO = GlobalVideoSource(DEFAULT_SOURCE)
def perceive_and_parse(user_instruction: str,
                       show_window: bool = False,
                       save_annotated: Optional[str] = None) -> Dict:
    
    store = HistoryStore(MEMORY_PATH)

    chat_hist = store.recent_chat_messages(MAX_TURNS)
    perception_hist_summaries = store.recent_perception_summaries(PERCEPTION_HISTORY_DEPTH)

    det_json = {"detections": [], "image": {}}
    perception_ctx = "CURRENT_PERCEPTION:\n" + json.dumps(
        {"timestamp": _now_iso(), "objects": []}, ensure_ascii=False
    )

    try:
        perceiver = YOLOPerceiver()

        try:
            frame = VIDEO.read(drop_n=3, timeout=5.0)
        except Exception as e:
            logger.warning(f"[VIDEO] GlobalVideoSource reading failed, using a one-time direct read as a fallback: {e}")

            frame = GlobalVideoSource.grab_one_frame_once(DEFAULT_SOURCE, warmup_frames=8, open_timeout_sec=8.0)

        annotated, det_json = perceiver.detect_frame(frame)

        print(f"[DEBUG] det_count={len(det_json.get('detections', []))}, image={det_json.get('image')}")
        cv2.imwrite("/tmp/percep_raw.jpg", frame)
        cv2.imwrite("/tmp/percep_annotated.jpg", annotated)

        if show_window:
            cv2.imshow("Perception", annotated)
            cv2.waitKey(800)
            cv2.destroyAllWindows()
        if save_annotated:
            cv2.imwrite(save_annotated, annotated)

        perception_ctx = build_perception_context(det_json)

    except Exception as e:
        logger.warning(f"⚠️ Camera or YOLO unavailable: {e}")
        det_json = {"detections": [], "image": {}}
        perception_ctx = "CURRENT_PERCEPTION:\n" + json.dumps(
            {"timestamp": _now_iso(), "objects": []}, ensure_ascii=False
        )

    perception_hist_text = build_perception_history_text(perception_hist_summaries)

    llm = PerceptionAwareLLM()
    raw = llm.parse(
        user_text=user_instruction,
        perception_context=perception_ctx,
        chat_history_messages=chat_hist,
        perception_history_text=perception_hist_text
    )

    try:
        parsed = json.loads(llm.extract_json(raw))
    except Exception:
        parsed = {"raw": raw, "note": "LLM output is not strict JSON and is returned as is in the raw field"}



    store.append("perception", build_perception_summary(det_json))
    store.append("user", user_instruction)
    store.append("assistant", json.dumps(parsed, ensure_ascii=False))

    return {"command": parsed}

def normalize_for_scheduler(cmd: dict) -> dict:
    if not isinstance(cmd, dict):
        return {"robots": {}}
    if "robots" in cmd and isinstance(cmd["robots"], dict):
        return cmd
    if all(isinstance(v, list) for v in cmd.values()):
        return {"robots": cmd}
    if "command" in cmd and isinstance(cmd["command"], dict):
        inner = cmd["command"]
        if "robots" in inner and isinstance(inner["robots"], dict):
            return inner
    return {"robots": {}}

def run_conversation_loop() -> Optional[Dict[str, Any]]:
    while True:
        while True:
            try:
                raw_text = recognize(delay=3).strip()
            except Exception as e:
                logger.warning(f"🎙️ recognition failed: {e}")
                tts_manager.say_sync("Sorry, could not hear you.")
                continue

            user_input = _clean(raw_text)
            if not user_input or user_input == "blank_audio":
                continue
            else:
                break

        exit_keywords = ["exit", "stop talking", "quit", "okay bye", "goodbye", "shut up",
                         "i want to change the chat mode", "ending of this mode", "ok finish", "ending this mode"]

        if any(kw in user_input.lower() for kw in exit_keywords):
            tts_manager.say_sync("Exiting this system")
            exit(0)

        logger.info(f"🗣️ You said: {user_input}")

        try:

            out = perceive_and_parse(user_input,
                                     show_window=True, save_annotated=None)

            print("✅ Parsed result:", json.dumps(out, indent=2, ensure_ascii=False))

            if out:
                resp = out.get("command", {}).get("response")
                print(resp or "i can't give you any response")
                tts_manager.say_sync(resp)
            else:
                tts_manager.say_sync("Could not understand the command.")

        except Exception:
            logger.warning(f"⚠️ Error during LLM or parsing: \n{traceback.format_exc()}")
            tts_manager.say_sync("Something went wrong.")

        execution_payload = normalize_for_scheduler(out.get("command", {}))
        if execution_payload["robots"]:
            isSchedule = robot_scheduler.run(execution_payload)
            if isSchedule is True:
                logger.info("✅ Command(s) executed successfully.")
                tts_manager.say_sync("Command executed.")
            else:
                logger.warning("⚠️ Failed")
                tts_manager.say_sync("Command execution failed, please re-give the command.")

# ================== 示例 ==================
if __name__ == "__main__":
    instr = "I want robot1 to move forward for 3 seconds and then turn left 90 degrees"
    out = perceive_and_parse(instr, show_window=True, save_annotated=None)
    print(json.dumps(out, indent=2, ensure_ascii=False))