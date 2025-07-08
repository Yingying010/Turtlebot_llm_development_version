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
model = AutoModelForCausalLM.from_pretrained("TinyLlama/TinyLlama-1.1B-Chat-v1.0")
tokenizer = AutoTokenizer.from_pretrained("TinyLlama/TinyLlama-1.1B-Chat-v1.0")

# ====== Prompt 构造函数 ======
def build_prompt(user_text: str) -> str:
    return f"""You are Lumi, a thoughtful and emotionally aware AI assistant who enjoys chatting casually with a human.
        You don’t have access to real-time data, but can simulate reasoning and give friendly replies.
        Avoid repeating phrases, listing too much, or writing multiple turns of conversation. Just give a natural 2–4 sentence reply.

        Human says: "{user_text}"
        Lumi replies:"""

# ========== 生成函数 ==========
def generate_response(user_input, max_new_tokens=256):
    prompt = build_prompt(user_input)
    inputs = tokenizer(prompt, return_tensors="pt")  # ✅ 用 prompt，而不是 user_input

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


def run(user_input):
    logger.info(f"💡 Current Mode: {'Chat' if config.get('chat_or_instruct') else 'Control'}")
    logger.info(f"🧠 LLM Input: {user_input}")
    output = generate_response(user_input)
    print("\n🧠 原始模型输出:\n", output)
    if "Human says:" in output:
        response = output.split("Human says:")[0].strip().strip('"')
    else:
        response = output.strip()
    print("\n🎉 最终输出:\n", response)

    return response


# ========== 执行命令 ==========
def execute_commands(result):
    tts_manager.say("Executing command")
    return ""

 
# ========== 主程序 ==========
if __name__ == "__main__":
    user_input = "I love you"
    response = run(user_input)


