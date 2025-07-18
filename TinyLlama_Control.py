from transformers import AutoTokenizer, AutoModelForCausalLM
import torch, re, ast, json, time
from textwrap import dedent
from stream_tts import tts_manager
from loguru import logger
from config import config
import control_turtlebot
from llama_cpp import Llama

print("⏳ Loading tokenizer and model...")
start = time.time()
model_path = "models/Qwen3_single_move_turn_q4_k_m.gguf"  # 你量化后的模型路径
llm = Llama(
    model_path=model_path,
    n_ctx=2048,
    n_threads=4,  # 根据你 CPU 调整
    verbose=True,
)
print(f"✅ Model loaded in {time.time() - start:.2f} seconds")

# ===== Prompt 构建模板 =====
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
    return """<|system|>\n{system_prompt}\n<|user|>\n{instruction}\n<|assistant|>\n"""

# ===== 生成 JSON =====
def generate_response(user_input, max_new_tokens=256):
    print("🌀 Generating response...")
    start = time.time()
    output = llm(prompt, max_tokens=256, stop=["<|user|>", "<|system|>"])
    end = time.time()

    response = output["choices"][0]["text"].strip()
    print("=== Raw Response ===\n", response)

    result = extract_json(response)
    print("\n=================== JSON Result ===================")
    print(result)
    print(f"✅ Generation completed in {end - start:.2f} seconds")
    return result

# === JSON 提取 ===
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
                except:
                    break
        start = text.find('{', start + 1)
    return None


# ===== 主执行函数 =====
def run(user_input):
    logger.info(f"💡 Mode: {'Chat' if config.get('chat_or_instruct') else 'Control'}")
    logger.info(f"🧠 LLM Input: {user_input}")
    response = generate_response(user_input)
    # print("\n🧠 Raw LLM Output:\n", response)
    commands = extract_json(response)
    if commands:
        print("\n✅ Parsed JSON:\n", commands)
    else:
        print("\n❌ Failed to extract valid JSON.")
    return commands

# ===== 测试入口 =====
if __name__ == "__main__":
    user_input = "let turtlebot1 forward 2 meters and turn left 45 degrees"
    command = run(user_input)
    if command:
        control_turtlebot.run(command)
        tts_manager.say("Command executed.")
        logger.info("✅ Command(s) executed successfully.")
