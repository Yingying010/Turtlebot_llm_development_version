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
import subprocess 
 
from WhisperRepo.whisper_recognizer import recognize
from ttsRepo.stream_tts import tts_manager
import config
import robotControllerRepo.robot_scheduler as robot_scheduler



# ================== 配置 ==================
DEFAULT_MODEL = "gpt-4o"
YOLO_WEIGHTS = os.getenv("YOLO_WEIGHTS", "yolov8n.pt")
YOLO_CONF   = float(os.getenv("YOLO_CONF", "0.25"))
YOLO_IOU    = float(os.getenv("YOLO_IOU", "0.45"))

YOLO_DEVICE = os.getenv("YOLO_DEVICE", None)

SESSION_ID = os.getenv("SESSION_ID", "chattinglog")
MEMORY_DIR = Path("./memory"); MEMORY_DIR.mkdir(parents=True, exist_ok=True)
MEMORY_PATH = MEMORY_DIR / f"{SESSION_ID}.jsonl"
 
MAX_TURNS = int(os.getenv("MAX_TURNS", "8"))
PERCEPTION_HISTORY_DEPTH = int(os.getenv("PERCEPTION_HISTORY_DEPTH", "5"))

_video_env = os.getenv("VIDEO_SOURCE", "0")   # 树莓派默认用本地相机
if _video_env.isdigit():
    DEFAULT_SOURCE: Any = int(_video_env)
else:
    DEFAULT_SOURCE: Any = _video_env

# ——统一输出提示词（在此基础上增加"可选历史输入"说明）——
SYSTEM_PROMPT = dedent("""
You are a specialized robot command interpreter that receives natural language input and converts it into a structured JSON object. 
Your primary responsibility is to identify only the executor robots and to determine their number, and parse the complete task list for each robot. 
The parsed tasks must be assigned to the corresponding robot, represented as an ordered list, and expressed strictly in the JSON schema described below. 
The output must respect task dependencies, including sequential, parallel, and synchronous execution, by correctly applying the `sequence` or `sync_group` fields when necessary.

========================
Semantic Mapping Rules for Spatial References
========================
When processing spatial references in commands:
- "here" / "come here" / "过来" → refers to the master's current location, use master's name as target
- "there" / "go there" → requires specific location context
- "my location" / "where I am" → refers to the master's current location
- "your location" / "where you are" → refers to the robot's current location

                       
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
- Keep numeric values as valid JSON floats. Include at most 3 objects.
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
1. **Parallel execution**: 
   If no dependencies are specified, do not add sequence and sync_group fields.

2. **Sequential execution (sequence)**:
   For tasks that must happen in order within the same robot or across robots.
   - sequence is a globally incrementing number starting from 0
   - Tasks with sequence: 0 execute first, then sequence: 1, etc.

3. **Synchronous execution (sync_group)**:
   For tasks that must start simultaneously across multiple robots.
   - Identify synchronization phases in the command
   - Each synchronization phase gets a unique sync_group ID (0, 1, 2...)
   - **Key rule**: Different temporal phases require different sync_group values
   
   **Example**: "robot1 and robot2 collect books at the same time, then deliver them together"
   - Phase 1 (collect): sync_group: 0
   - Phase 2 (deliver): sync_group: 1

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

def detect_once() -> List[Dict[str, Any]]:
    """
    执行一次YOLO感知，返回 detections 列表（每个元素是一个 dict，包含 class / conf / center_xy / bbox_xyxy）
    用于被 find 动作调用
    """
    try:
        perceiver = YOLOPerceiver()

        ts = time.strftime("%Y%m%d-%H%M%S")
        img_path = f"/tmp/yolo_parser_frame_{ts}.jpg"
        subprocess.run([
            "rpicam-still", "-t", "1000",
            "--width", "1640", "--height", "1232",
            "-o", img_path
        ], check=True)

        _, det_json = perceiver.detect_photo(img_path)
        detections = det_json.get("detections", [])
        return detections
    except Exception as e:
        logger.warning(f"⚠️ detect_once failed: {e}")
        return []


def safe_json_parse(raw_str: str) -> dict:
    """增强版JSON解析器，处理各种格式问题"""
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
    if start != -1 and end != -1 and start < end:
        trimmed = raw_str[start:end+1]
        try:
            parsed = json.loads(trimmed)
            # 验证基本结构
            if _validate_json_structure(parsed):
                return parsed
        except Exception:
            pass

    # 尝试修复常见的JSON错误
    try:
        fixed_json = _fix_json_errors(raw_str)
        parsed = json.loads(fixed_json)
        if _validate_json_structure(parsed):
            return parsed
    except Exception:
        pass

    # 尝试修复截断的JSON
    try:
        repaired = _repair_truncated_json(raw_str)
        parsed = json.loads(repaired)
        if _validate_json_structure(parsed):
            return parsed
    except Exception:
        pass

    # 最后失败就返回包装，但确保有基本结构
    return {
        "perception_report": {
            "timestamp": _now_iso(),
            "summary": {},
            "objects": [],
            "relations": []
        },
        "robots": {},
        "response": "Sorry, I couldn't process that command properly.",
        "raw": raw_str,
        "note": "Enhanced parser attempted multiple fixes but failed. Basic structure provided."
    }

def _validate_json_structure(parsed: dict) -> bool:
    """验证解析结果的基本结构"""
    if not isinstance(parsed, dict):
        return False
    
    # 检查是否有主要字段
    main_fields = ["perception_report", "robots", "response"]
    has_main_structure = any(field in parsed for field in main_fields)
    
    return has_main_structure

def _fix_json_errors(json_str: str) -> str:
    """修复常见的JSON格式错误"""
    # 移除多余的逗号
    json_str = re.sub(r',\s*}', '}', json_str)
    json_str = re.sub(r',\s*]', ']', json_str)
    
    # 修复未闭合的字符串
    lines = json_str.split('\n')
    fixed_lines = []
    
    for line in lines:
        stripped = line.strip()
        if stripped and ':' in stripped:
            # 检查是否有未闭合的引号
            if stripped.count('"') % 2 == 1 and not stripped.endswith(('"', ',', '}', ']')):
                # 尝试在合适的位置添加引号
                if not stripped.endswith('"'):
                    stripped += '"'
        fixed_lines.append(stripped)
    
    return '\n'.join(fixed_lines)

def _repair_truncated_json(json_str: str) -> str:
    """修复被截断的JSON"""
    # 查找未闭合的括号并添加
    brace_count = json_str.count('{') - json_str.count('}')
    bracket_count = json_str.count('[') - json_str.count(']')
    
    if brace_count > 0:
        json_str += '}' * brace_count
    if bracket_count > 0:
        json_str += ']' * bracket_count
    
    # 特殊处理：如果response字段被截断
    if '"response"' in json_str and not json_str.strip().endswith(('}', '"')):
        # 查找response字段的位置并修复
        response_match = re.search(r'"response":\s*"([^"]*?)(?:"|\s*$)', json_str)
        if response_match and not response_match.group(0).endswith('"'):
            # 修复未闭合的response字段
            before_response = json_str[:response_match.start()]
            response_content = response_match.group(1)
            after_response = json_str[response_match.end():]
            
            # 重构JSON
            json_str = before_response + f'"response": "{response_content}"' + after_response
            
            # 确保JSON正确闭合
            if not json_str.strip().endswith('}'):
                json_str += '}'
    
    return json_str


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
        if isinstance(source, int):
            self.backend = cv2.CAP_V4L2
        elif isinstance(source, str) and "appsink" in source:
            self.backend = cv2.CAP_GSTREAMER
        else:
            self.backend = cv2.CAP_FFMPEG
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
    
    @staticmethod
    def grab_one_frame_once(source: Any, warmup_frames: int = 8, open_timeout_sec: float = 8.0) -> np.ndarray:
        if isinstance(source, int):
            backend = cv2.CAP_V4L2
        elif isinstance(source, str) and "appsink" in source:
            backend = cv2.CAP_GSTREAMER
        else:
            backend = cv2.CAP_FFMPEG
        cap = cv2.VideoCapture(source, backend)

        t0 = time.time()

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

    def detect_photo(self, img_path: str) -> Tuple[np.ndarray, Dict]:
        frame = cv2.imread(img_path)
        if frame is None:
            raise RuntimeError(f"Unable to read image: {img_path}")

        res = self.model.predict(
            source=frame, conf=self.conf, iou=self.iou, device=self.device, verbose=False
        )[0]

        annotated = res.plot()

        h, w = res.orig_shape
        detections: List[Dict] = []
        names = res.names

        for b in getattr(res, "boxes", []):
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            conf = float(b.conf[0])
            cls_id = int(b.cls[0])
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            cls_name = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else str(cls_id)
            detections.append({
                "class": cls_name.lower(),
                "conf": round(conf, 4),
                "bbox_xyxy": [x1, y1, x2, y2],
                "center_xy": [cx, cy]
            })

        return annotated, {"image": {"width": int(w), "height": int(h)},"detections": detections}



# ================== 感知上下文构造 ==================
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

def build_perception_context(det_json: Dict, topk: int = 3) -> str:
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

def build_perception_summary(det_json: Dict, topk: int = 3) -> Dict:
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
        Let me first define your identity. You are a highly intelligent indoor robot, and your name is {get_robot_id()}, and your master's name is {get_master_name()}. You can understand voice commands and assist users in completing various tasks. You can move freely, navigate, collect, and deliver items. You can also communicate freely with humans. In addition, you can collaborate with robot2.

        **IMPORTANT SEMANTIC RULE**: When your master says "come here" or similar commands, "here" always refers to your master's location. Use "{get_master_name()}" as the target, NOT the literal word "here".

        Next, here's what the user said to you:
        """).strip()

        print(post_identity)
        
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

        # 增加重试机制和更大的token限制
        max_attempts = 3
        for attempt in range(max_attempts):
            try:
                # 根据尝试次数调整参数
                if attempt == 0:
                    max_tokens = 1000  # 增加token限制
                    temperature = 0.1
                elif attempt == 1:
                    max_tokens = 1200
                    temperature = 0.15
                else:
                    max_tokens = 1500
                    temperature = 0.2

                resp = self.client.chat.completions.create(
                    model=self.model,
                    messages=messages,
                    temperature=temperature,
                    max_tokens=max_tokens
                )
                
                raw_output = resp.choices[0].message.content.strip()
                logger.info(f"🔄 Attempt {attempt + 1} - Raw output length: {len(raw_output)}")
                
                # 验证输出不为空且不是错误
                if not raw_output or "error" in raw_output.lower()[:100]:
                    if attempt < max_attempts - 1:
                        logger.warning(f"⚠️ Attempt {attempt + 1} produced invalid output, retrying...")
                        continue
                
                return raw_output
                
            except Exception as e:
                logger.error(f"❌ Attempt {attempt + 1} failed: {e}")
                if attempt == max_attempts - 1:
                    # 所有尝试都失败，返回基本的JSON结构
                    fallback_response = {
                        "perception_report": {
                            "timestamp": _now_iso(),
                            "summary": {},
                            "objects": [],
                            "relations": []
                        },
                        "robots": {},
                        "response": "Sorry, I encountered an error processing your command.",
                        "error": f"LLM call failed after {max_attempts} attempts: {str(e)}"
                    }
                    return json.dumps(fallback_response, ensure_ascii=False)
        
        # 理论上不应该到达这里
        return "{}"

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

        ts = time.strftime("%Y%m%d-%H%M%S")
        img_path = f"/tmp/yolo_parser_frame_{ts}.jpg"
        subprocess.run([
            "rpicam-still", "-t", "1000",
            "--width", "1640", "--height", "1232",
            "-o", img_path
        ], check=True)

        annotated, det_json = perceiver.detect_photo(img_path)

        print(f"[DEBUG] det_count={len(det_json.get('detections', []))}, image={det_json.get('image')}")
        cv2.imwrite("/tmp/percep_annotated.jpg", annotated)

        if show_window:
            try:
                cv2.imshow("Perception", annotated)
                cv2.waitKey(800)
                cv2.destroyAllWindows()
            except Exception as _e:
                logger.warning(f"GUI not available, skip imshow: {_e}")

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

    # 使用增强的JSON解析器
    try:
        extracted = llm.extract_json(raw)
        parsed = safe_json_parse(extracted)
        
        # 验证解析结果的完整性
        if not _validate_parsed_result(parsed):
            logger.warning("⚠️ Parsed result validation failed, using fallback structure")
            parsed = _create_fallback_structure(raw, user_instruction)
            
    except Exception as e:
        logger.error(f"❌ JSON parsing failed completely: {e}")
        parsed = _create_fallback_structure(raw, user_instruction)

    # 记录历史
    store.append("perception", build_perception_summary(det_json))
    store.append("user", user_instruction)
    store.append("assistant", json.dumps(parsed, ensure_ascii=False))

    return {"command": parsed}

def _validate_parsed_result(parsed: dict) -> bool:
    """验证解析结果是否包含所需的基本结构"""
    if not isinstance(parsed, dict):
        return False
    
    # 检查是否有错误标记
    if "error" in parsed and "failed after" in str(parsed.get("error", "")):
        return False
    
    # 检查基本结构
    required_fields = ["perception_report", "robots", "response"]
    missing_fields = [field for field in required_fields if field not in parsed]
    
    if len(missing_fields) > 1:  # 允许缺少一个字段
        return False
    
    # 检查robots字段格式
    if "robots" in parsed:
        robots = parsed["robots"]
        if not isinstance(robots, dict):
            return False
        
        # 检查每个机器人的任务格式
        for robot_id, tasks in robots.items():
            if not isinstance(tasks, list):
                return False
            for task in tasks:
                if not isinstance(task, dict) or "action" not in task:
                    return False
    
    return True

def _create_fallback_structure(raw_output: str, user_instruction: str) -> dict:
    """创建备用的JSON结构"""
    return {
        "perception_report": {
            "timestamp": _now_iso(),
            "summary": {},
            "objects": [],
            "relations": []
        },
        "robots": {},
        "response": "I'm sorry, I had trouble understanding that command. Could you please rephrase it?",
        "raw": raw_output,
        "note": "Fallback structure created due to parsing failure",
        "original_input": user_instruction
    }

def normalize_for_scheduler(cmd: dict) -> dict:
    """标准化调度器的输入格式"""
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
    """主对话循环，增加了更好的错误处理和状态管理"""
    logger.info("🎯 Starting enhanced conversation loop")
    
    consecutive_failures = 0
    max_consecutive_failures = 3
    
    while True:
        # 语音识别循环
        while True:
            try:
                raw_text = recognize(delay=3).strip()
                consecutive_failures = 0  # 重置失败计数
                break
            except Exception as e:
                consecutive_failures += 1
                logger.warning(f"🎙️ Recognition failed (attempt {consecutive_failures}): {e}")
                
                if consecutive_failures >= max_consecutive_failures:
                    tts_manager.say_sync("I'm having trouble hearing you. Please check the microphone.")
                    time.sleep(2)
                    consecutive_failures = 0  # 重置计数
                else:
                    tts_manager.say_sync("Sorry, could not hear you.")
                continue

        user_input = _clean(raw_text)
        if not user_input or user_input == "blank_audio":
            continue

        # 检查退出关键词
        exit_keywords = ["exit", "stop talking", "quit", "okay bye", "goodbye", "shut up",
                         "i want to change the chat mode", "ending of this mode", "ok finish", "ending this mode"]

        if any(kw in user_input.lower() for kw in exit_keywords):
            tts_manager.say_sync("Exiting voice control.")
            break

        logger.info(f"🗣️ You said: {user_input}")

        # 解析和执行
        out = {}
        try:
            out = perceive_and_parse(user_input, show_window=False, save_annotated=None)

            print("✅ Parsed result:", json.dumps(out, indent=2, ensure_ascii=False))

            if out and "command" in out:
                cmd = out["command"]
                resp = cmd.get("response", "")
                
                # 检查是否有有效的response
                if resp and resp != "i can't give you any response":
                    print(f"🤖 Response: {resp}")
                    tts_manager.say_sync(resp)
                else:
                    print("🤖 Response: Processing your command...")
                    tts_manager.say_sync("Processing your command.")
                
                # 尝试执行命令
                execution_payload = normalize_for_scheduler(cmd)
                if execution_payload.get("robots"):
                    try:
                        isSchedule = robot_scheduler.run(execution_payload)
                        if isSchedule is True:
                            logger.info("✅ Command(s) executed successfully.")
                            tts_manager.say_sync("Command executed successfully.")
                        else:
                            logger.warning("⚠️ Command execution failed")
                            tts_manager.say_sync("Command execution failed. Please try again.")
                    except Exception as exec_error:
                        logger.error(f"❌ Scheduler error: {exec_error}")
                        tts_manager.say_sync("Sorry, there was an error executing the command.")
                else:
                    # 没有机器人命令，可能是聊天
                    if resp:
                        logger.info("💬 Chat mode - no robot commands detected")
                    else:
                        tts_manager.say_sync("I didn't detect any robot commands in your request.")
            else:
                tts_manager.say_sync("Could not understand the command.")

        except Exception as e:
            logger.error(f"⚠️ Error during processing: \n{traceback.format_exc()}")
            tts_manager.say_sync("Something went wrong while processing your request.")
            out = {}

        # 添加小延迟以防止过于频繁的循环
        time.sleep(0.5)

# ================== 示例 ==================
if __name__ == "__main__":
    # 测试用例
    test_instructions = [
        "Can you help me to get my water bottle from the kitchen and then give it to me?"
    ]
    
    for instr in test_instructions:
        print(f"\n{'='*50}")
        print(f"Testing: {instr}")
        print('='*50)
        try:
            out = perceive_and_parse(instr, show_window=False, save_annotated=None)
            print(json.dumps(out, indent=2, ensure_ascii=False))
        except Exception as e:
            print(f"Test failed: {e}")
            traceback.print_exc()