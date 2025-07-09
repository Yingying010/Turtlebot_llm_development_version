from transformers import AutoTokenizer, AutoModelForCausalLM
import torch, re, ast, json, time
from textwrap import dedent
from stream_tts import tts_manager
from loguru import logger
from config import config

# ===== 模型路径与设备 =====
model_path = "./lora-tinyllama-turtlebot"
device = "cuda" if torch.cuda.is_available() else "cpu"

# ===== 加载模型与 Tokenizer =====
print("⏳ Loading tokenizer and model...")
load_start = time.time()
tokenizer = AutoTokenizer.from_pretrained(model_path)
model = AutoModelForCausalLM.from_pretrained(model_path).to(device)
model.eval()
load_end = time.time()
print(f"✅ Model loaded in {load_end - load_start:.2f} seconds")

# ===== Prompt 构建模板 =====
SYSTEM_PROMPT = dedent("""
    You are **TurtleBotCommandParser**, an expert JSON generator for multi-robot control.
    ■ Output **only** a JSON object that conforms to <SCHEMA>.
    ■ Before generating JSON, map any alias in <ALIASES> to its canonical robot_id.
    ■ Do NOT output any text other than the JSON.
    <ALIASES>
      "robot1"  ->  "turtlebot1"
      "robot2"  ->  "turtlebot2"
      "turtlebot one" -> "turtlebot1"
      "turtlebot two" -> "turtlebot2"
    </ALIASES>
    <SCHEMA>
    {
      "$root": {
        "<robot_id>": [
          {
            "action"   : "move | turn | navigation_to",
            "direction": "forward | backward | left | right",
            "value"    : <number>,
            "unit"     : "meters | seconds | degrees",
            "target"   : "self" | {"x": <number>, "y": <number>}
          }
        ]
      }
    }
    </SCHEMA>
    Think step-by-step *silently*, then output the JSON only.
""").strip()

EXAMPLES = [
    {
        "instruction": "let turtlebot1 forward 2 meters and turn left 45 degrees, and let turtlebot2 backward 10 meters and turn right 10 degrees.",
        "response": {
            "turtlebot1": [
                {"action": "move", "direction": "forward", "value": 2, "unit": "meters"},
                {"action": "turn", "direction": "left", "value": 45, "unit": "degrees", "target": "self"}
            ],
            "turtlebot2": [
                {"action": "move", "direction": "backward", "value": 10, "unit": "meters"},
                {"action": "turn", "direction": "right", "value": 10, "unit": "degrees", "target": "self"}
            ]
        }
    },
    {
        "instruction": "robot1, go find robot2",
        "response": {
            "turtlebot1": [
                {"action": "navigate_to", "target": "turtlebot2"}
            ]
        }
    }
]

def build_prompt(instruction: str) -> str:
    example_blocks = "\n\n".join([
        f"<EXAMPLE>\n### Instruction:\n{ex['instruction']}\n### Response:\n{json.dumps(ex['response'], indent=2, ensure_ascii=False)}\n</EXAMPLE>"
        for ex in EXAMPLES
    ])
    user_block = f"<TASK>\n### Instruction:\n{instruction}\n### Response:"
    return f"{SYSTEM_PROMPT}\n\n{example_blocks}\n\n{user_block}"

# ===== 生成 JSON =====
def generate_response(user_input, max_new_tokens=256):
    prompt = build_prompt(user_input)
    print("\n🌀 Generating response...")
    gen_start = time.time()
    inputs = tokenizer(prompt, return_tensors="pt").to(device)

    with torch.no_grad():
        outputs = model.generate(
            **inputs,
            max_new_tokens=max_new_tokens,
            do_sample=False,  # ⚠️ 改为 deterministic，避免慢
            pad_token_id=tokenizer.eos_token_id
        )
    gen_end = time.time()
    print(f"✅ Generation completed in {gen_end - gen_start:.2f} seconds")

    decoded = tokenizer.decode(outputs[0], skip_special_tokens=True)
    return decoded[len(prompt):].strip()

# ===== 提取 JSON =====
def extract_json(text: str):
    match = re.search(r"```(?:json)?\s*([\s\S]+?)\s*```", text)
    if match:
        try:
            return json.loads(match.group(1).strip())
        except Exception:
            pass
    start = text.find('{')
    while start != -1:
        stack = 0
        for idx in range(start, len(text)):
            if text[idx] == '{':
                stack += 1
            elif text[idx] == '}':
                stack -= 1
                if stack == 0:
                    try:
                        return json.loads(text[start:idx+1])
                    except Exception:
                        break
        start = text.find('{', start + 1)
    return None

# ===== 主执行函数 =====
def run(user_input):
    logger.info(f"💡 Mode: {'Chat' if config.get('chat_or_instruct') else 'Control'}")
    logger.info(f"🧠 LLM Input: {user_input}")
    response = generate_response(user_input)
    print("\n🧠 Raw LLM Output:\n", response)
    commands = extract_json(response)
    if commands:
        print("\n✅ Parsed JSON:\n", commands)
    else:
        print("\n❌ Failed to extract valid JSON.")
    return commands

# ===== 测试入口 =====
if __name__ == "__main__":
    user_input = "let turtlebot1 forward 2 meters and turn left 45 degrees"
    run(user_input)
