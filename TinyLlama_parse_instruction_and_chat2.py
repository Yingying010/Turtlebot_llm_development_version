import json
import re
import time
from loguru import logger
from const_config import param_types
from config import config
from transformers import AutoTokenizer, AutoModelForCausalLM
import torch
from control_turtlebot import move_robot
from control_speaker import set_speaker_volume
import threading
from textwrap import dedent
from stream_tts import tts_manager

stop_generation_flag = threading.Event()  # 中断标志

# ========== 加载模型和 Tokenizer ==========
model_path = "lora-tinyllama-turtlebot"
device = "cuda" if torch.cuda.is_available() else "cpu"
tokenizer = AutoTokenizer.from_pretrained(model_path)
model = AutoModelForCausalLM.from_pretrained(model_path).to(device)
 
# ========== 构造 Prompt (control version)==========
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
            "action"   : "move | turn | navigate_to",
            "direction": "forward | backward | left | right",   // optional for navigate_to
            "value"    : <number>,                              // distance (m), duration (s) or angle (deg)
            "unit"     : "meters | seconds | degrees",
            "target"   : "self" | {"x": <number>, "y": <number>} // navigate target or self
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
                {"action": "navigate_to", "target": {
                    "x": 2.5,
                    "y": 3.0
                } }
            ],
            "turtlebot2": [
                {"action": "turn", "direction": "right", "target": "self", "value": 45, "unit": "degrees", },
                {"action": "navigate_to", "target": {
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


def get_system_prompt(user_input):
    is_chat = config.get("chat_or_instruct")
    if is_chat: 
        prompt = (
            "You are a warm and empathetic AI companion. Respond like a friend.\n\n"
            f"User: {user_input}\nAssistant:"
        )
    else:
        prompt = build_prompt(user_input)
    return prompt


# ========== 生成response ==========
def generate_response(prompt: str, result_container: list[str]):
    inputs = tokenizer(prompt, return_tensors="pt").to(model.device)
    tts_manager.say("Generating response")
    logger.info("⏳ Generating response...")
    start_time = time.time()
    try:
        with torch.no_grad():
            outputs = model.generate(
                **inputs,
                max_new_tokens=256,
                do_sample=True,
                temperature=0.7,
                top_p=0.9,
                pad_token_id=tokenizer.eos_token_id
            )
        decoded = tokenizer.decode(outputs[0], skip_special_tokens=True)
        decoded = decoded[len(prompt):].lstrip()
        duration = time.time() - start_time
        logger.info(f"✅ Response generated in {duration:.2f} seconds.")
        result_container.append(decoded)
    except Exception as e:
        logger.error(f"❌ Generation failed: {e}")

# ========== 提取 JSON (control version)==========
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
 

# ========== 记录日志==========
def log_chat_to_file(user_input, response_text, filepath="log.txt"):
    """将聊天输入输出记录到 log.txt"""
    with open(filepath, "a") as f:
        f.write(f"\n🧑 User: {user_input}\n🤖 Assistant: {response_text}\n{'='*40}\n")

# ========== 主运行 ==========
def send(user_input):
    logger.info(f"💡 Current Mode: {'Chat' if config.get('chat_or_instruct') else 'Control'}")
    logger.info(f"🧠 LLM Input: {user_input}")
    prompt = get_system_prompt(user_input)

    # 清除中断标志
    stop_generation_flag.clear()

    result_container: list[str] = []

    # 启动子线程生成
    gen_thread = threading.Thread(target=generate_response, args=(prompt, result_container))
    gen_thread.start()

    # 检查是否中断
    while gen_thread.is_alive():
        if stop_generation_flag.is_set():
            logger.warning("🛑 Generation interrupted by user.")
            return ""  # 或者 return None
        time.sleep(0.1)

    if not result_container:
        logger.warning("❌ No response generated.")
        return ""

    output_text = result_container[0]
    logger.info(f"🧠 LLM Output:\n{output_text}")

    log_chat_to_file(user_input, output_text)

    if config.get("chat_or_instruct"):
        # 提取第一个 Assistant 回复
        match = re.findall(r"Assistant:\s*(.+)", output_text)
        first_response = match[0].strip() if match else output_text.strip()
        logger.info(f"🗣️ Final chat reply: {first_response}")
        return first_response
    else:
        command = extract_json(output_text)
        if command:
            logger.info("\nJSON Command: \n", command)
            return command
        else:
            print("\nUnable to extract JSON, please check the output format。")

# ========== 执行命令 ==========
def execute_commands(result):
    tts_manager.say("Executing command")
    return ""
    