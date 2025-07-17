from llama_cpp import Llama
import json
import time
from textwrap import dedent

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

instruction = "make turtlebot 2 move backward 3 meters"

prompt = f"""<|system|>\n{system_prompt}\n<|user|>\n{instruction}\n<|assistant|>\n"""

 
# === 推理 ===
print("🌀 Generating response...")
start = time.time()
output = llm(prompt, max_tokens=256, stop=["<|user|>", "<|system|>"])
end = time.time()

response = output["choices"][0]["text"].strip()
print("=== Raw Response ===\n", response)

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

result = extract_json(response)
print("\n=================== JSON Result ===================")
print(result)
print(f"✅ Generation completed in {end - start:.2f} seconds")