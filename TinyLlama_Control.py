from transformers import AutoTokenizer, AutoModelForCausalLM
from pathlib import Path
import torch
import time
import re
import json
from textwrap import dedent
from deepdiff import DeepDiff
from datetime import datetime

import torch, re, json, time
from textwrap import dedent
from stream_tts import tts_manager
from loguru import logger
from config import config
import control_turtlebot
from llama_cpp import Llama


#============== 工具函数 =================
def extract_json(text: str):
  # 找到 "Assistant:" 的位置
    assistant_idx = text.lower().find("assistant:")
    if assistant_idx == -1:
        print("❌ 没找到 'Assistant:' 标签")
        return None

    # 从 assistant 部分开始
    content = text[assistant_idx:]

    # 寻找第一个合法 JSON 字符串（使用大括号配对）
    start = content.find('{')
    while start != -1:
        stack = 0
        for i in range(start, len(content)):
            if content[i] == '{':
                stack += 1
            elif content[i] == '}':
                stack -= 1
                if stack == 0:
                    json_str = content[start:i+1]
                    try:
                        return json.loads(json_str)
                    except json.JSONDecodeError as e:
                        break  # 当前括号内容非法，换下一个 {
        start = content.find('{', start + 1)

    print("❌ 没找到合法 JSON 块")
    return None


#============== 加载模型 =================
print("⏳ Loading tokenizer and model...")
start = time.time()

model_path = "/home/ubuntu/Turtlebot_llm_development_version/models/Qwen3_base_instruction_q8/Qwen3_base_instruction_q8.gguf"  # 你量化后的模型路径
llm = Llama(
    model_path=model_path,
    n_ctx=2048,
    n_threads=4,  # 根据你 CPU 调整
    verbose=True,
)

print(f"✅ Model loaded in {time.time() - start:.2f} seconds")

# ===== Prompt 模板 =====
def inference(user_input):
    system_prompt = dedent('''
        You are a robot command parser. Given a natural language instruction, output ONLY a valid JSON object that maps each robot to a list of its actions.

        Each action must include:
        - "action": e.g. "move", "turn", "navigate", etc.
        - "direction": e.g. "left", "right", "forward", or null
        - "value": a number (e.g. 2.5) or null
        - "unit": e.g. "seconds", "meters", "degrees", or null
        - "target": either an object like {"x": ..., "y": ...}, or a name string like "user", or null

        Rules:
        - Output must be a valid JSON object.
        - Top-level keys must be robot identifiers in the form of <type><number>.
        - Each robot maps to a list of actions (1 or more).
        ❗ Output MUST start with '{' and end with '}'.
        ❗ Output MUST be a valid JSON object. No other text is allowed.
        ❌ Do NOT include plan, explanation, reasoning, thought, or any prefix like "Assistant:", "Plan:", "Structure:", "Here is".

    ''').strip()

    prompt = f"""<|system|>\n{system_prompt}\n<|user|>\n{user_input}\n<|assistant|>\n"""


    #============== 推理流程 =================

    print("\n🌀 Generating response...")
    gen_start = time.time()

    output = llm.create_chat_completion(
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_input}
        ],
        temperature=0.2,
        max_tokens=256
    )
    raw_response = output["choices"][0]["message"]["content"].strip()
    print("===================JSON raw response====================\n",raw_response)

    result = extract_json(response)
    print("===================JSON Result====================\n",result)

    gen_end = time.time()
    print(f"✅ Generation completed in {gen_end - gen_start:.2f} seconds")

    return raw_response, result


# ===== 主执行函数 =====
def run(user_input: str):
    logger.info(f"💡 Mode: {'Chat' if config.get('chat_or_instruct') else 'Control'}")
    logger.info(f"🧠 LLM Input: {user_input}")
    
    commands = inference(user_input)
    if commands:
        logger.info("\n✅ Parsed JSON:\n", commands)
        return commands
    else:
        print("\n❌ Failed to extract valid JSON.")
        logger.warning("⚠️ No command was executed due to invalid JSON.")
        return None 

# ===== 测试入口 =====
if __name__ == "__main__":
    user_input = "let robot 1 turn right 45 degrees"
    response = run(user_input)
    if response:
        control_turtlebot.run(response)
        logger.info("✅ Command(s) executed successfully.")
        tts_manager.say("Command executed.")
        tts_manager.wait_until_done()
    else:
        logger.warning("⚠️ No commands received from LLM.")
        tts_manager.say("Sorry, I couldn't understand the instruction.")
        tts_manager.wait_until_done()

