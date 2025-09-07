import os, sys, re, json, time, traceback, threading
import os, sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from typing import Dict, List, Optional, Tuple, Any
from textwrap import dedent
from pathlib import Path
from collections import Counter
import openai
from loguru import logger
import config
from llmParserRepo.yolo_perception import detect_once as yolo_detect_once



# ================== 配置 ==================
DEFAULT_MODEL = "gpt-4o"

SESSION_ID = os.getenv("SESSION_ID", "chattinglog")
MEMORY_DIR = Path("./memory"); MEMORY_DIR.mkdir(parents=True, exist_ok=True)
MEMORY_PATH = MEMORY_DIR / f"{SESSION_ID}.jsonl"
 
MAX_TURNS = int(os.getenv("MAX_TURNS", "8"))
PERCEPTION_HISTORY_DEPTH = int(os.getenv("PERCEPTION_HISTORY_DEPTH", "5"))

# ——统一输出提示词（在此基础上增加"可选历史输入"说明）——
SYSTEM_PROMPT = dedent("""
You are a specialized robot command interpreter that receives natural language input and converts it into a structured JSON object. 
Your primary responsibility is to identify only the executor robots and to determine their number, and parse the complete task list for each robot. 
The parsed tasks must be assigned to the corresponding robot, represented as an ordered list, and expressed strictly in the JSON schema described below. 
The output must respect task dependencies, including sequential, parallel, and synchronous execution, by correctly applying the `sequence` or `sync_group` fields when necessary.

========================
IMPORTANT: Two-Stage Task Planning Strategy
========================
When users request actions that involve searching for objects followed by other actions (like "find my cup and bring it to me"):

**Stage 1 (Initial Planning - YOU ARE HERE):**
- Generate ONLY the search/find task for now
- Do NOT generate subsequent actions like collect/deliver yet
- The robot will dynamically plan follow-up actions after successfully finding the object

**Stage 2 (Dynamic Replanning - happens later):**
- After the object is found, the system will re-parse the user's original intent
- Based on current visual perception and the original command, it will generate appropriate follow-up actions

**Examples:**
- User: "Find my cup and bring it to me" → Generate ONLY: find task
- User: "Look for my keys and put them on the table" → Generate ONLY: find task  
- User: "Navigate to the kitchen" → Generate: navigate task (no find involved)
- User: "Move forward 2 meters" → Generate: move task (no find involved)

This approach ensures that follow-up actions are based on the actual object location and current environment state.

========================
Semantic Mapping Rules for Spatial References
========================
When processing spatial references in commands:
- "here" / "come here"  → refers to the master's current location, use master's name as target
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
2. pickup
   - Pick up an item : {"item": "<item>"}
3. dropoff  
   - Drop off an item : {"item": "<item>"}
4. find  
   - Search for an object: {"item": "<item>"}
5.contactless_transport
    {
      "item": "<object_name>",
      "start_position": "<named_location_or_person>",
      "goal_position": "<named_location_or_person>",
      "method": "acoustic" // Optional; include only if explicitly mentioned
    }
6. move  
   {"direction": "forward" or "backward", "value": <number>, "unit": "meter" or "second"}
7. turn  
   {"direction": "left" or "right", "value": <number>, "unit": "degree" or "second"}
8. follow  
   {"target": "<target_name>"}
9. face  
   {"target": "<target_name>"}
10. wait  
   {"duration_sec": <number>}

Note:
- Numeric values must match the text exactly (preserve signs)
- Semantics take precedence over literals: If the semantics of a command clearly correspond to an action (e.g., "look at" corresponds to `face`), the action with the closest semantic match must be selected, not a literal that partially matches (e.g., "turn").
- Specialized actions take precedence over general actions: For example, `face` (a specialized orientation action) takes precedence over `turn` (a general rotation action).
- The pickup and dropoff actions do not include any movement or navigation. If the robot is not already at the correct location (e.g., table, another robot, or the user), a separate navigate task must be added explicitly before the collect or deliver step.
- Use contactless_transport ONLY when explicitly mentioned by keywords like "contactless", "acoustic transport", "remote transport", or "without touching". 
- For regular "bring", "deliver", "transport" commands, use the standard pickup → navigate → dropoff sequence.
- **CRITICAL FOR FIND-BASED COMMANDS**: When user requests finding an object followed by other actions (collect, deliver, etc.), generate ONLY the find task. The system will dynamically generate follow-up actions after the object is found.
- Strictly adhere to the principle of semantic precedence
- Do not add parameters not explicitly specified in the command
- Provide more accurate semantic analysis of compound verb phrases 

========================
Note for find action
========================
- Use when user asks to "look for", "search for", "find", or "locate" objects
- The system automatically optimizes detection sensitivity and search patterns
- "save_as" creates an identifier for use in subsequent collect/deliver actions
- **IMPORTANT**: For compound commands like "find X and do Y", generate ONLY the find task. Follow-up actions will be planned dynamically after the object is found.

========================
Special Rules for Robot Names
========================     
- All robot names must be converted to lowercase alphanumeric format without spaces. 
- The same formatting rules apply to all target names.
- For example: convert "robot one" to "robot1"
                       
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
1. **Independent/Parallel execution (DEFAULT)**: 
   When robots work separately or independently, do NOT add sequence or sync_group fields.
   Each robot executes its own task list in natural order.

2. **Sequential execution (sequence)**:
   ONLY when tasks must happen in a specific order ACROSS robots.
   - Use only for cross-robot dependencies
   - Within a single robot, tasks naturally execute in list order
   - sequence is a globally incrementing number starting from 0

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
        # 使用独立的YOLO模块
        result = yolo_detect_once()
        
        if result and "result" in result:
            # 假设result["result"]包含检测结果
            # 需要根据yolo_perception.py的实际返回格式进行调整
            yolo_data = result["result"]
            detections = []
            
            # 🔥 修复：处理YOLO命令行返回的JSON格式
            if isinstance(yolo_data, list):
                # 如果直接是检测结果列表
                detections = yolo_data
            elif isinstance(yolo_data, dict):
                # 如果是包含检测结果的字典
                if "detections" in yolo_data:
                    detections = yolo_data["detections"]
                elif "predictions" in yolo_data:
                    detections = yolo_data["predictions"]
                else:
                    # 尝试从其他可能的字段获取
                    logger.warning(f"Unknown YOLO result format: {list(yolo_data.keys())}")
                    detections = []
            
            # 🔥 确保检测结果格式正确
            formatted_detections = []
            for det in detections:
                if isinstance(det, dict):
                    # 标准化检测结果格式
                    formatted_det = {
                        "class": det.get("class", det.get("name", "unknown")).lower(),
                        "conf": float(det.get("conf", det.get("confidence", 0.0))),
                        "center_xy": det.get("center_xy", det.get("center", [0, 0])),
                        "bbox_xyxy": det.get("bbox_xyxy", det.get("bbox", [0, 0, 0, 0]))
                    }
                    
                    # 如果没有center_xy，从bbox计算
                    if not formatted_det["center_xy"] or formatted_det["center_xy"] == [0, 0]:
                        bbox = formatted_det["bbox_xyxy"]
                        if len(bbox) == 4:
                            cx = (bbox[0] + bbox[2]) / 2.0
                            cy = (bbox[1] + bbox[3]) / 2.0
                            formatted_det["center_xy"] = [cx, cy]
                    
                    formatted_detections.append(formatted_det)
            
            logger.info(f"🔍 Processed {len(formatted_detections)} detections")
            return formatted_detections
        else:
            logger.warning("⚠️ No detection result from YOLO perceiver")
            return []
            
    except Exception as e:
        logger.warning(f"⚠️ detect_once failed: {e}")
        import traceback
        traceback.print_exc()
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
    """构造感知上下文文本"""
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
    """构造感知摘要（用于历史存储）"""
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
    """构造感知历史文本"""
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
                logger.info(f"📄 Attempt {attempt + 1} - Raw output length: {len(raw_output)}")
                
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
        # 使用独立的YOLO感知模块
        perception_result = yolo_detect_once()
        
        if perception_result and "result" in perception_result:
            # 根据yolo_perception.py的返回格式来处理
            yolo_data = perception_result["result"]
            
            # 构造兼容的det_json格式
            if isinstance(yolo_data, dict):
                det_json = yolo_data
            elif isinstance(yolo_data, list):
                # 如果是检测列表，包装成期望的格式
                det_json = {"detections": yolo_data, "image": {"width": 640, "height": 480}}
            
            logger.info(f"🔍 Perception successful: {len(det_json.get('detections', []))} objects detected")
            perception_ctx = build_perception_context(det_json)
            
            # 如果需要显示或保存图像，这里需要从perception_result中获取
            if "frame_path" in perception_result and (show_window or save_annotated):
                frame_path = perception_result["frame_path"]
                # 这里可能需要额外的处理来显示或保存图像
                logger.info(f"📸 Frame available at: {frame_path}")
                
        else:
            logger.warning("⚠️ YOLO perception failed or returned no data")
            
    except Exception as e:
        logger.warning(f"⚠️ Perception error: {e}")
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


# ================== 示例 ==================
if __name__ == "__main__":
    # 测试用例
    test_instructions = [
        "Robot1 and robot2, I'd like you both to help me get the pills off the table without contact."
    ]

    out = perceive_and_parse(test_instructions, show_window=False, save_annotated=None)

    print("✅ Parsed result:", json.dumps(out, indent=2, ensure_ascii=False))
    