#!/usr/bin/env python3
import os, sys
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)
import os, sys, json, re, time, requests
from textwrap import dedent
from typing import Any, Dict, Tuple, Optional
from WhisperRepo.whisper_recognizer import recognize
from ttsRepo.stream_tts import tts_manager
from loguru import logger
from config import config
from llama_cpp import Llama
import robotControllerRepo.robot_scheduler as robot_scheduler
import traceback



SYSTEM_PROMPT: str = dedent("""
        You are a specialized robot command interpreter that converts natural language instructions into structured JSON task sequences for robotic systems. Your role is to parse human commands and output precise, executable robot task definitions.

        Core Responsibilities
        1. Parse natural language input containing robot commands and instructions
        2. Extract robot identifiers from various naming formats
        3. Identify actions and their associated parameters
        4. Determine task sequencing and synchronization requirements
        5. Output structured JSON following the exact schema specification

        JSON STRUCTURE:
        {
        "robots": {
            "robot1": [task_array],
            "robot2": [task_array]
        }
        }

        TASK FORMAT:
        {
            "task_id": "tX",
            "action": "action_name",
            "parameters": {...},
            "sync_group": null or "sync_seq#",
            "sequence": number
        }
        
        SUPPORTED ACTIONS:
        1. move: {"direction": "forward/backward", "value": number, "unit": "meters/seconds"}
        2. turn: {"direction": "left/right", "value": number, "unit": "degrees/seconds"}  
        3. navigate: {"target": "place/person/robot"} OR {"position": {"x": num, "y": num, "heading_deg": num}}
            - If heading_deg is not mentioned, then it does not need to be written.
        4. follow: {"target": "person/robot"}
        5. face: {"target": "place/person/robot"}
        6. collect: {"item": "item_name", "target": "place/person/robot" OR "position": {"x": num, "y": num, "heading_deg": num}}
        7. deliver: {"item": "item_name", "target": "place/person/robot" OR "position": {"x": num, "y": num, "heading_deg": num}}
        8. wait: {"duration_sec": number}
                            
        KEY PROCESSING RULES:
        1. Sequence Management:
        - Tasks increment sequence (0,1,2...) in chronological order.
        - If described as parallel/simultaneous, different robots share the same sequence value.
        2. Synchronization Rules:
        - Sync groups: Use format "sync_seq#" where # matches the sequence number
        - Full sync: identical sequence, identical sync_group, and identical action (+ parameters) across robots. The tasks are executed as a single, lock-step operation.
        - Loose sync: identical sequence and sync_group, but different action/parameters. Robots merely start at the same time; what they do can differ.
        3. coordination rules:
        - sync_group: tasks with same sync_group name execute together (synchronously)
        - sync_group: null means independent task (no synchronization needed)
        - sequence: Defines execution order (0, 1, 2...). Lower numbers execute first
        - Within same sequence+sync_group, tasks run in parallel    
        5. task_id generation
        - Use unique task_ids globally (t0, t1, t2...)
""").strip()

SYSTEM_INSTRUCTION: str = dedent("""
    Parse the following natural language instructions into structured JSON task sequences for robotic systems.
""")


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


# ============== 加载模型 =================
print("⏳ Loading tokenizer and model...")
start = time.time()

model_path = "models/Qwen3_collaboration_v4_q4_k_m/Qwen3_collaboration_v4_q4_k_m.gguf"
llm = Llama(
    model_path=model_path,
    n_ctx=2048,
    n_threads=4,
    verbose=True,
)

print(f"✅ Model loaded in {time.time() - start:.2f} seconds")


#============== 推理过程 =================

def inference(user_input):
    print("🌀 Generating response...")
    start = time.time()

    messages = [
        {"role": "system", "content": SYSTEM_PROMPT},
        {"role": "user", "content": f"{SYSTEM_INSTRUCTION}\nInput: {user_input}"}
    ]
    output = llm.create_chat_completion(
        messages,
        temperature=0.3,
        max_tokens= 600
    )

    end = time.time()

    raw_response = output["choices"][0]["message"]["content"].strip()
    print("=== Raw Response ===\n", raw_response)

    json_result = extract_last_json(raw_response)

    print("\n=================== JSON Result ===================")
    print(json_result)
    print(f"✅ Generation completed in {end - start:.2f} seconds")

    return raw_response, json_result


def extract_last_json(text: str):
    end = text.rfind('}')
    while end != -1:
        stack = 0
        for i in range(end, -1, -1):  # 从end向前找起始的{
            if text[i] == '}':
                stack += 1
            elif text[i] == '{':
                stack -= 1
            if stack == 0:
                try:
                    return json.loads(text[i:end+1])
                except:
                    break
        end = text.rfind('}', 0, end)
    return None



# ============== 主控制函数 =================
# 支持更多数字的转换
# number_words = {
#     "0": "zero", "1": "one", "2": "two", "3": "three",
#     "4": "four", "5": "five", "6": "six", "7": "seven",
#     "8": "eight", "9": "nine", "10": "ten"
# }
 
# base_id = config.get("robot_id")  # e.g., "robot1"
 
# # 插入空格：robot1 → robot 1
# spaced_id = re.sub(r"(\d+)", r" \1", base_id)
 
# # 替换成英文单词：robot1 → robot one
# text_id = re.sub(r"(\d+)", lambda m: f" {number_words.get(m.group(1), m.group(1))}", base_id)
 
# # 创建 alias 集合
# robot_aliases = {base_id, spaced_id, text_id}
 
# # 可手动添加更多别名
# # robot_aliases.add("robo-one")
 
# def called_robot(text: str) -> bool:
#     t = text.lower()
#     return any(alias in t for alias in robot_aliases)


def _clean(text: str) -> str:
    return re.sub(r'[^\w\s]', '', text).lower().strip()

def run_conversation_loop() -> Optional[Dict[str, Any]]:
    logger.info(f"💡 Mode: {'Chat' if config.get('chat_or_instruct') else 'Control'}")

    print("🎤 Entering continuous voice control mode. Say 'exit' to stop.")
    history_messages[:] = [{"role": "system", "content": SYSTEM_PROMPT}]
    save_history(history_messages)
    robot_id = config.get("robot_id") 
    
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
            if not user_input or not "blank_audio":
                continue
            else:
                break

    
        # ② 判断是否叫到我
        # if not called_robot(user_input):
        #     tts_manager.say("You didn't call me, I am waiting for you")
        #     time.sleep(3)
        #     continue

        logger.info(f"🗣️ You said: {user_input}")
        tts_manager.say("User command has been received")
        time.sleep(0.5)
        tts_manager.say(f"Your cammand is {user_input}. Currently parsing it for you ....")
        time.sleep(0.5)

        exit_keywords = ["exit", "stop talking", "quit", "okay bye", "goodbye", "shut up", "i want to change the chat mode", "ending of this mode","ok finish", "ending this mode"]

        if any(kw in user_input.lower() for kw in exit_keywords):
            tts_manager.say("Exiting voice control.")
            time.sleep(1)
            break

        # 调用 LLM 进行解析
        try:
            _, json_result = inference(user_input)
            print("✅ Parsed result:", json.dumps(json_result, indent=2))

            if json_result:
                # robot_scheduler.run(json_response)
                tts_manager.say("Command parsing complete.")
                time.sleep(1)
            else:
                tts_manager.say("Could not understand the command.")
                time.sleep(1)

        except Exception:
            logger.warning(f"⚠️ Error during LLM or parsing: \n{traceback.format_exc()}")
            tts_manager.say("Something went wrong.")


        # call scheduler
        if json_result:
            isSchedule = robot_scheduler.run(json_result)
            if isSchedule == True:
                logger.info("✅ Command(s) executed successfully.")
                tts_manager.say("Command executed.")
                time.sleep(1)
            else:
                logger.warning("⚠️ Failed")
                tts_manager.say("Command execution failed, please re-give the command.")
                time.sleep(3)

if __name__ == "__main__":
    json_result = run_conversation_loop()