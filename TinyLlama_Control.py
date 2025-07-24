from pathlib import Path
import json
import time
from textwrap import dedent
from loguru import logger
from config import config
from stream_tts import tts_manager
import control_turtlebot
from llama_cpp import Llama


# ============== 加载模型 =================
print("⏳ Loading tokenizer and model...")
start = time.time()

model_path = "/home/ubuntu/Turtlebot_llm_development_version/models/Qwen3_instructions_BaseCommands_q4_k_m/Qwen3_instructions_BaseCommands-q4_k_m.gguf"
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

    sys_prompt = "You are a professional robot control assistant. Parse the user's natural language instructions and convert them into structured JSON commands for robot control."
    user_instruction = "Parse the following robot control instruction and convert it to structured JSON format."

    messages = [
        {"role": "system", "content": sys_prompt},
        {"role": "user", "content": f"{user_instruction}\nInput: {user_input}"}
    ]
    output = llm.create_chat_completion(
        messages,
        temperature=0.3,
        max_tokens= 256
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
    user_input = "robot one go to x is 100 and y is 200"
    commands = run(user_input)

    if commands:
        control_turtlebot.controller(commands)
        logger.info("✅ Command(s) executed successfully.")
        tts_manager.say("Command executed.")
        tts_manager.wait_until_done()
    else:
        logger.warning("⚠️ No commands received from LLM.")
        tts_manager.say("Sorry, I couldn't understand the instruction.")
        tts_manager.wait_until_done()
