#!/usr/bin/env python3
import os, sys
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)
import os, sys, json, re, time, requests
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)
from textwrap import dedent
from typing import Any, Dict, Tuple, Optional
from WhisperRepo.whisper_recognizer import recognize
from ttsRepo.stream_tts import tts_manager
from loguru import logger
from config import config


# ========== Prompt 与历史记忆 ================
SYSTEM_PROMPT: str = dedent("""
You are a task parser for multi-robot instructions.

OUTPUT FORMAT:
<robot_name>:
- <task_id> | <action> | <param_list> | sync_group=<name_or_null> | seq=<int>
...

SUPPORTED ACTIONS:
1. move     | direction=forward/backward, value=<num>, unit=meters/seconds
2. turn     | direction=left/right, value=<num>, unit=degrees/seconds
3. navigate | target=<place/person/robot> OR position(x=<num>, y=<num>, heading_deg=<num_or_null>)
4. follow   | target=<person/robot>
5. face     | target=<place/person/robot>
6. collect  | item=<item_name>, target=<...>
7. deliver  | item=<item_name>, target=<...>
8. wait     | duration_sec=<num>

COORDINATION RULES:
- sync_group: same name → synchronous; null → independent
- sequence: execution order

OUTPUT REQUIREMENT:
• Output ONLY plain-text lines, no JSON, no code fence, no explanations.
• Each line uses " | " to separate fields.

EXAMPLES:
robot1:
- t0 | navigate | position(x=100, y=200, heading_deg=null) | sync_group=null | seq=0
robot2:
- t1 | move | direction=forward, value=2, unit=seconds | sync_group=null | seq=0
""").strip()

HISTORY_PATH = "chat_history.json"

def load_history() -> list:
    if os.path.exists(HISTORY_PATH):
        with open(HISTORY_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    else:
        return [{"role": "system", "content": SYSTEM_PROMPT}]

def save_history(history: list):
    with open(HISTORY_PATH, "w", encoding="utf-8") as f:
        json.dump(history, f, ensure_ascii=False, indent=2)

history_messages = load_history()

def call_ollama(instruction: str, model: str = "llama3:8b") -> Tuple[str, float]:
    global history_messages
    history_messages.append({"role": "user", "content": instruction})

    url = "http://localhost:11434/api/chat"
    payload = {
        "model": model,
        "stream": False,
        "messages": history_messages,
    }

    start = time.perf_counter()
    response = requests.post(url, json=payload, timeout=300)
    response.raise_for_status()
    elapsed = time.perf_counter() - start

    data: Dict[str, Any] = response.json()
    content = data.get("message", {}).get("content", "")
    history_messages.append({"role": "assistant", "content": content})
    save_history(history_messages)

    return content, elapsed

def _parse_value(val: str) -> Any:
    val = val.strip()
    if val.lower() == "null": return None
    try:
        if re.fullmatch(r"-?\d+\.\d+", val): return float(val)
        if re.fullmatch(r"-?\d+", val): return int(val)
    except ValueError: pass
    return val

def _parse_param_list(param_str: str) -> Dict[str, Any]:
    param_str = param_str.strip()
    if param_str.startswith("position(") and param_str.endswith(")"):
        inside = param_str[len("position("):-1]
        kv_pairs = [kv.strip() for kv in inside.split(",")]
        return {"position": {k.strip(): _parse_value(v) for k, v in (kv.split("=", 1) for kv in kv_pairs)}}
    params = {}
    for kv in param_str.split(","):
        if "=" not in kv: raise ValueError(f"Invalid param: '{kv}'")
        k, v = kv.split("=", 1)
        params[k.strip()] = _parse_value(v)
    return params

def convert_to_json(text: str) -> Dict[str, Any]:
    robots: Dict[str, list] = {}
    current_robot = None
    for line in text.strip().splitlines():
        if not line.strip(): continue
        if not line.startswith("  -") and line.endswith(":"):
            current_robot = line[:-1].strip()
            robots[current_robot] = []
            continue
        if line.strip().startswith("-") and current_robot:
            task_part = line[line.find("-") + 1:].strip()
            fields = [f.strip() for f in task_part.split("|")]
            if len(fields) != 5: raise ValueError(f"Expected 5 fields: {line}")
            task_id, action, param_str, sync_part, seq_part = fields
            if not sync_part.startswith("sync_group="): raise ValueError(f"Invalid sync: {sync_part}")
            sync_group_raw = sync_part.split("=", 1)[1].strip()
            sync_group = None if sync_group_raw.lower() == "null" else sync_group_raw
            if not seq_part.startswith("seq="): raise ValueError(f"Invalid seq: {seq_part}")
            sequence = int(seq_part.split("=", 1)[1].strip())
            parameters = _parse_param_list(param_str)
            robots[current_robot].append({
                "task_id": task_id, "action": action,
                "parameters": parameters, "sync_group": sync_group, "sequence": sequence
            })
    return {"robots": robots}

def analyze_coordination(flexible_json: Dict[str, Any]) -> None:
    print("🔍 Coordination Analysis:")
    sequences = {}
    for robot_name, tasks in flexible_json["robots"].items():
        for task in tasks:
            seq = task["sequence"]
            sequences.setdefault(seq, {}).setdefault(task.get("sync_group", "independent"), []).append(f"{robot_name}:{task['action']}")
    for seq in sorted(sequences):
        print(f"\nSequence {seq}:")
        for sg, tasks in sequences[seq].items():
            label = "🔀 Independent" if sg == "independent" else f"🔄 Sync '{sg}'"
            print(f"  {label}: {', '.join(tasks)}")

def run_conversation_loop() -> Optional[Dict[str, Any]]:
    logger.info(f"💡 Mode: {'Chat' if config.get('chat_or_instruct') else 'Control'}")

    print("🎤 Entering continuous voice control mode. Say 'exit' to stop.")
    history_messages[:] = [{"role": "system", "content": SYSTEM_PROMPT}]
    save_history(history_messages)

    while True:
        try:
            user_input = recognize(delay=3).strip()  # 重新录音
        except Exception as e:
            logger.warning(f"🎙️ Speech recognition failed: {e}")
            tts_manager.say("Sorry, could not hear you.")
            time.sleep(1)
            continue

        if not user_input:
            tts_manager.say("Didn't catch that. Try again.")
            time.sleep(1)
            continue

        logger.info(f"🗣️ You said: {user_input}")

        exit_keywords = ["exit", "stop talking", "quit", "okay bye", "goodbye", "shut up"]

        if any(kw in user_input.lower() for kw in exit_keywords):
            tts_manager.say("Exiting voice control.")
            time.sleep(1)
            break

        # 调用 LLM 进行解析
        try:
            raw_response, _ = call_ollama(user_input)
            json_response = convert_to_json(raw_response)
            print("✅ Parsed result:", json.dumps(json_response, indent=2))

            if json_response:
                # robot_scheduler.run(json_response)
                tts_manager.say("Command executed.")
                time.sleep(1)
            else:
                tts_manager.say("Could not understand the command.")
                time.sleep(1)

        except Exception as e:
            logger.warning(f"⚠️ Error during LLM or parsing: {e}")
            tts_manager.say("Something went wrong.")

if __name__ == "__main__":
    response = run_conversation_loop()