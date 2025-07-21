from pathlib import Path
import json
import time
from textwrap import dedent
from loguru import logger
from config import config
from stream_tts import tts_manager
import control_turtlebot
from llama_cpp import Llama


# ============== 工具函数 =================
def extract_json(text: str):
    # 找到 "Assistant:" 的位置（可选）
    assistant_idx = text.lower().find("assistant:")
    if assistant_idx != -1:
        text = text[assistant_idx:]

    # 寻找第一个合法 JSON 字符串（大括号匹配）
    start = text.find('{')
    while start != -1:
        stack = 0
        for i in range(start, len(text)):
            if text[i] == '{':
                stack += 1
            elif text[i] == '}':
                stack -= 1
                if stack == 0:
                    json_str = text[start:i+1]
                    try:
                        return json.loads(json_str)
                    except json.JSONDecodeError as e:
                        print("❌ JSON 解析失败:", e)
                        print("🚨 错误文本片段:", json_str)
                        break
        start = text.find('{', start + 1)

    print("❌ 没找到合法 JSON 块")
    return None


# ============== 加载模型 =================
print("⏳ Loading tokenizer and model...")
start = time.time()

model_path = "/home/ubuntu/Turtlebot_llm_development_version/models/Qwen3_base_instruction_q8.gguf"  # ⚠️ 请确认是 .gguf 文件！
llm = Llama(
    model_path=model_path,
    n_ctx=2048,
    n_threads=4,
    verbose=True,
)

print(f"✅ Model loaded in {time.time() - start:.2f} seconds")


# ============== 推理函数 =================
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
    print("=================== JSON Raw Response ===================\n", raw_response)

    result = extract_json(raw_response)
    print("=================== JSON Result ===================\n", result)

    print(f"✅ Generation completed in {time.time() - gen_start:.2f} seconds")
    return raw_response, result


# ============== 主控制函数 =================
def run(user_input: str):
    logger.info(f"💡 Mode: {'Chat' if config.get('chat_or_instruct') else 'Control'}")
    logger.info(f"🧠 LLM Input: {user_input}")

    raw, commands = inference(user_input)

    if commands:
        logger.info("✅ Parsed JSON:\n{}", commands)
        return commands
    else:
        logger.warning("⚠️ Failed to parse JSON.")
        return None


# ============== 主测试入口 =================
if __name__ == "__main__":
    user_input = "let robot 1 turn right 45 degrees"
    commands = run(user_input)

    if commands:
        control_turtlebot.run(commands)
        logger.info("✅ Command(s) executed successfully.")
        tts_manager.say("Command executed.")
        tts_manager.wait_until_done()
    else:
        logger.warning("⚠️ No commands received from LLM.")
        tts_manager.say("Sorry, I couldn't understand the instruction.")
        tts_manager.wait_until_done()
