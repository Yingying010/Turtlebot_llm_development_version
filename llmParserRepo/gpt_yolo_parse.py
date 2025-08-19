
"""
Simple Perception-Aware LLM with History:
抓一帧 → YOLO 感知 → 记录历史（感知+对话）→ 注入“当前感知 + 历史感知 + 历史对话” → LLM 返回统一 JSON
"""
 
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
from config import config
import robotControllerRepo.robot_scheduler as robot_scheduler
 
# ================== 配置 ==================
DEFAULT_MODEL = os.getenv("LLM_MODEL", "gpt-4o-mini")
YOLO_WEIGHTS = os.getenv("YOLO_WEIGHTS", "yolov8n.pt")
YOLO_CONF   = float(os.getenv("YOLO_CONF", "0.25"))
YOLO_IOU    = float(os.getenv("YOLO_IOU", "0.45"))
YOLO_DEVICE = os.getenv("YOLO_DEVICE", None)   # 例: "0"
# FIX: 仅使用 OpenCV 读 UDP 流，不再把 URL 直接传给 Ultralytics
DEFAULT_SOURCE = "udp://@:8888?fifo_size=1000000&overrun_nonfatal=1&buffer_size=1000000&probesize=32&analyzeduration=0"
 
SESSION_ID = os.getenv("SESSION_ID", "default")
MEMORY_DIR = Path("./memory"); MEMORY_DIR.mkdir(parents=True, exist_ok=True)
MEMORY_PATH = MEMORY_DIR / f"{SESSION_ID}.jsonl"
 
MAX_TURNS = int(os.getenv("MAX_TURNS", "8"))
PERCEPTION_HISTORY_DEPTH = int(os.getenv("PERCEPTION_HISTORY_DEPTH", "5"))

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


 
