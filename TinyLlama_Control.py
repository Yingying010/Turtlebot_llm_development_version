from transformers import AutoTokenizer, AutoModelForCausalLM
import torch
import re
import ast
import json
from textwrap import dedent
from stream_tts import tts_manager
import threading
from loguru import logger
from config import config

 
# ========== 加载模型和 Tokenizer ==========
model_path = "./lora-tinyllama-turtlebot"  # 你的模型保存路径
device = "cuda" if torch.cuda.is_available() else "cpu"
 
tokenizer = AutoTokenizer.from_pretrained(model_path)
model = AutoModelForCausalLM.from_pretrained(model_path).to(device)
 
# ========== 构造 Prompt ==========
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
      // add more if needed
    </ALIASES>
    • Map aliases BOTH when used as robot IDs **and** when they appear as "target" values.
 
    <SCHEMA>
    {
      "$root": {
        "<robot_id>": [
          {
            "action"   : "move | turn | navigation_to",
            "direction": "forward | backward | left | right",   // optional for navigation_to
            "value"    : <number>,                              // distance (m), duration (s) or angle (deg)
            "unit"     : "meters | seconds | degrees",
            "target"   : "self" | {"x": <number>, "y": <number>} // navigation target or self
          }
        ]
      }
    }
</SCHEMA>
 
    Think step-by-step *silently*, then output the JSON only.
""").strip()
 
EXAMPLES = [
    {
        "instruction": "let turtlebot1 forward 2 meters and turn left 45 degrees, "
                       "and let turtlebot2 backward 10 meters and turn right 10 degrees.",
        "response": {
            "turtlebot1": [
                {"action": "move", "direction": "forward", "value": 2,  "unit": "meters"},
                {"action": "turn", "direction": "left", "target": "self", "value": 45, "unit": "degrees", }
            ],
            "turtlebot2": [
                {"action": "move", "direction": "backward","value": 10, "unit": "meters"},
                {"action": "turn", "direction": "right", "target": "self", "value": 10, "unit": "degrees", }
            ]
        }
    },
    {
        "instruction": "let turtlebot1 forward 2 seconds and go to position (2.5, 3.0), "
                       "and let turtlebot2 turn right 45 degrees and go to position (100, 350)",
        "response": {
            "turtlebot1": [
                {"action": "move", "direction": "forward", "value": 2,  "unit": "seconds"},
                {"action": "navigation_to", "target": {
                    "x": 2.5,
                    "y": 3.0
                } }
            ],
            "turtlebot2": [
                {"action": "turn", "direction": "right", "target": "self", "value": 45, "unit": "degrees", },
                {"action": "navigation_to", "target": {
                    "x": 100,
                    "y": 350
                } }
            ]
        }
    },
    {
        "instruction": "let robot1 approach me",
        "response": {
            "turtlebot1": [
                {"action": "navigate_to","target": "user"}
            ],
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
    # 👉 继续在这里添加更多 few-shot 样例（保持相同格式）...
]

def build_prompt(instruction: str) -> str:
    """
    构造高质量 prompt，引导模型生成符合 schema 的 JSON。
    """
    # Few-shot 示例块
    few_shot_blocks = []
    for ex in EXAMPLES:
        few_shot_blocks.append(
            dedent(f"""
            <EXAMPLE>
            ### Instruction:
            {ex['instruction']}
            ### Response:
            {json.dumps(ex['response'], indent=2, ensure_ascii=False)}
            </EXAMPLE>
            """).strip()
        )
    few_shot_section = "\n\n".join(few_shot_blocks)
 
    # 用户真实指令块
    user_block = dedent(f"""
        <TASK>
        ### Instruction:
        {instruction}
        ### Response:
    """).strip()
 
    # 最终 prompt
    prompt = f"{SYSTEM_PROMPT}\n\n{few_shot_section}\n\n{user_block}"
    return prompt
 
# ========== 生成函数 ==========
def generate_response(user_input, max_new_tokens=256):
    prompt = build_prompt(user_input)
    inputs = tokenizer(prompt, return_tensors="pt").to(device)
 
    with torch.no_grad():
        outputs = model.generate(
            **inputs,
            max_new_tokens=max_new_tokens,
            do_sample=True,
            temperature=0.7,
            top_p=0.9,
            pad_token_id=tokenizer.eos_token_id
        )
 
    decoded = tokenizer.decode(outputs[0], skip_special_tokens=True)
    return decoded[len(prompt):].strip()

# ========== 提取 JSON ==========
def extract_json(text: str):
    fence_match = re.search(r"```(?:json)?\s*([\s\S]+?)\s*```", text, re.IGNORECASE)
    if fence_match:
        candidate = fence_match.group(1).strip()
        try:
            return json.loads(candidate)
        except Exception as e:
            print("⚠️ fenced JSON 解析失败:", e)
    start = text.find('{')
    while start != -1:
        stack = 0
        for idx in range(start, len(text)):
            if text[idx] == '{':
                stack += 1
            elif text[idx] == '}':
                stack -= 1
                if stack == 0:          # 找到平衡
                    candidate = text[start:idx+1]
                    try:
                        return json.loads(candidate)
                    except Exception:
                        # 解析失败，继续在后面找
                        break
        start = text.find('{', start + 1)
    return None 

def run(user_input):
    logger.info(f"💡 Current Mode: {'Chat' if config.get('chat_or_instruct') else 'Control'}")
    logger.info(f"🧠 LLM Input: {user_input}")
    response = generate_response(user_input)
    print("\n🧠 原始模型输出:\n", response)
    commands = extract_json(response)
    if commands:
        print("\n✅ 成功提取的 JSON:\n", commands)
    else:
        print("\n❌ 无法提取 JSON，请检查输出格式。")
    return commands
 
# ========== 主程序 ==========
if __name__ == "__main__":
    user_input = "let turtlebot1 forward 2 meters and turn left 45 degrees"
    # test_instruction = "let turtlebot1 forward 2 meters and turn left 45 degrees, and let turtlebot2 backward 10 meters and turn right 10 degrees."
    # test_instruction = "hey, robot1! go find robot2"
    # test_instruction = "let turtlebot1 go to position (2.0, 3.0)"
    response = run(user_input)


