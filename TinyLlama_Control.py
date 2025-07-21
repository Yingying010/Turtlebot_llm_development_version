from transformers import AutoTokenizer, AutoModelForCausalLM
import torch, re, json, time
from textwrap import dedent
from stream_tts import tts_manager
from loguru import logger
from config import config
import control_turtlebot
from llama_cpp import Llama

# ===== 模型加载 =====
print("⏳ Loading tokenizer and model...")
start = time.time()
model_path = "models/Qwen3_single_move_turn_q4_k_m.gguf"  # 你量化后的模型路径
llm = Llama(
    model_path=model_path,
    n_ctx=2048,
    n_threads=4,  # 根据你CPU核心数调整
    verbose=True,
)
print(f"✅ Model loaded in {time.time() - start:.2f} seconds")

# ===== Prompt 模板 =====
system_prompt = dedent('''
You are a robot command parser. Given a natural language instruction, output ONLY a valid JSON object that follows the format below.

JSON Format:
{
  "<robot_name>": [
    {
      "action": "move" | "turn",
      "direction": "left" | "right" | "forward" | "backward",
      "value": <number>,
      "unit": "seconds" | "meters" | "degrees"
    }
  ]
}

Rules:
 - Do NOT output any explanation or reasoning.
 - You MUST include the robot name as the top-level key.
 - Only output a valid object followed by the format.
''').strip()

def build_prompt(instruction: str) -> str:
    return f"<|system|>\n{system_prompt}\n<|user|>\n{instruction}\n<|assistant|>\n"

# ===== JSON 提取函数 =====
def extract_json(text: str):
    start = text.find('{')
    while start != -1:
        stack = 0
        for i in range(start, len(text)):
            if text[i] == '{':
                stack += 1
            elif text[i] == '}':
                stack -= 1
            if stack == 0:
                try:
                    return json.loads(text[start:i+1])
                except json.JSONDecodeError:
                    break
        start = text.find('{', start + 1)
    return None

# ===== LLM响应生成 =====
def generate_response(user_input: str, max_new_tokens=256):
    print("🌀 Generating response...")
    prompt = build_prompt(user_input)
    start = time.time()
    output = llm(prompt, max_tokens=max_new_tokens, stop=["<|user|>", "<|system|>"])
    end = time.time()
    
    response = output["choices"][0]["text"].strip()
    print("=== Raw Response ===\n", response)
    
    result = extract_json(response)
    print("\n=================== JSON Result ===================")
    print(result)
    print(f"✅ Generation completed in {end - start:.2f} seconds")
    return result

# ===== 主执行函数 =====
def run(user_input: str):
    logger.info(f"💡 Mode: {'Chat' if config.get('chat_or_instruct') else 'Control'}")
    logger.info(f"🧠 LLM Input: {user_input}")
    
    commands = generate_response(user_input)
    if commands:
        logger.info("\n✅ Parsed JSON:\n", commands)
        return commands
    else:
        print("\n❌ Failed to extract valid JSON.")
        logger.warning("⚠️ No command was executed due to invalid JSON.")
        return None 

# ===== 测试入口 =====
if __name__ == "__main__":
    user_input = "let robot 1 turn left 90 degrees"
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

