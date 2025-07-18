import requests
import json
import time
from textwrap import dedent
from stream_tts import tts_manager
from loguru import logger
from config import config
import threading

# ========== Prompt 构造函数 ==========
# def build_prompt(user_text: str) -> str:
#     return dedent(f"""
#     You are Lumi, a thoughtful and emotionally aware AI assistant who enjoys chatting casually with a human.
#     You don’t have access to real-time data, but can simulate reasoning and give friendly replies.
#     Avoid repeating phrases, listing too much, or writing multiple turns of conversation. Just give a natural 2–4 sentence reply.

#     Human says: "{user_text}"
#     Lumi replies:
#     """)

# ========== 生成函数（使用 Ollama） ==========
def generate_response(user_input: str, max_new_tokens=256) -> str:
    prompt = user_input
    print("\n🌀 Generating response...")
    gen_start = time.time()

    try:
        response = requests.post(
            "http://localhost:11434/api/generate",
            json={
                "model": "tinyllama",
                "prompt": prompt,
                "stream": False,
                "options": {
                    "num_predict": max_new_tokens,
                    "temperature": 0.7,
                    "top_p": 0.9
                }
            }
        )
        response.raise_for_status()
        result_text = response.json()['response']
        gen_end = time.time()
        print(f"✅ Generation completed in {gen_end - gen_start:.2f} seconds")
        return result_text.strip()
    except Exception as e:
        print(f"❌ Error during generation: {e}")
        return "Sorry, I couldn't generate a response."

# ========== 主执行函数 ==========
def run(user_input: str) -> str:
    logger.info(f"💡 Current Mode: {'Chat' if config.get('chat_or_instruct') else 'Control'}")
    logger.info(f"🧠 LLM Input: {user_input}")
    output = generate_response(user_input)

    # 清洗模型回复
    if "Human says:" in output:
        response = output.split("Human says:")[0].strip().strip('"')
    else:
        response = output.strip()

    print("\n🎉 Response:\n", response)
    return response

# ========== 命令执行（可选） ==========
def execute_commands(result):
    tts_manager.say("Executing command")
    return ""

# ========== 主程序 ==========
if __name__ == "__main__":
    user_input = "What is 1 plus 1?"
    response = run(user_input)

    # ✅ 用线程包装 say，并等待它播放完
    tts_thread = threading.Thread(target=tts_manager.say, args=(response,))
    tts_thread.start()
    tts_thread.join()

    print("🎉 Program ended after TTS finished.")

