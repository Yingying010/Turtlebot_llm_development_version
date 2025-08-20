import sys
import os
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)
from config import config
from WhisperRepo.whisper_recognizer import conversation_active, recognize
# from llmParserRepo.qwen3_parser import run_conversation_loop
# from llmParserRepo.gpt_yolo_parser import run_conversation_loop
from llmParserRepo.gpt_yolo_localParser import run_conversation_loop
# import TinyLlama_Chat
import time
import re
from ttsRepo.stream_tts import tts_manager
from loguru import logger
from loguru import logger
import traceback
import argparse
from pathlib import Path
from llmParserRepo.gpt_yolo_localParser import HistoryStore

running = False
actived = 0
allow_running = True

def run_conversation():
    logger.info("🎤 Recording...")
    tts_manager.say_sync("I'm listening.")

    try:
        response = run_conversation_loop()

    except Exception:
        logger.error(f"LLM error: \n{traceback.format_exc()}")
        tts_manager.say_sync("Something went wrong")
        conversation_active.clear()
        return
    
    # -------- 对话结束 ----------------------------------------------------
    logger.info("Conversation finished.")
    conversation_active.clear()

# ✅ 启动欢迎语
def startchat():
    os.system("afplay beep.wav")
    logger.info("📢 Starting chat system")
    tts_manager.say_sync("Welcome! You can start speaking after the beep")

def _clean(text: str) -> str:
    return re.sub(r'[^\w\s]', '', text).lower().strip()


# ✅ 启动入口
if __name__ == "__main__":
    if len(sys.argv)<2:
        print("Usage: python3 main.py <robot_id> <master_id>")
        sys.exit(1)
 
    robot_id = sys.argv[1]
    master_id = sys.argv[2]
    config.set(robot_id=robot_id)
    config.set(master_id=master_id)

    startchat()

    while True:
        # 唤醒词监听
        while True:
            try:
                raw_text = recognize(delay=3).strip()
            except Exception as e:
                logger.warning(f"🎙️ recognition failed: {e}")
                tts_manager.say_sync("Sorry, could not hear you.")
                continue

            wakeup_word = _clean(raw_text)
            if not wakeup_word:
                continue
            else:
                break

        if "open robot system" in wakeup_word:
            config.set(chat_or_instruct=False)
            logger.info("🎮 Switched to CONTROL mode.")
            tts_manager.say_sync("Okay, I'm now in control mode. If you want to exit this mode, just say ending this mode.")
        elif wakeup_word in {"i want to chat with you", "open chat system"}:
            config.set(chat_or_instruct=True)
            logger.info("💬 Switched to CHAT mode.")
            tts_manager.say_sync("Sure, I'm now in chat mode.")
        elif wakeup_word in {"ok bye", "okay bye", "ok byebye", "okay byebye","finish system"}:
            tts_manager.say_sync("Goodbye!")
            time.sleep(1)
            # 建议用 sys.exit(0) 更优雅
            sys.exit(0)
        else:
            continue

        # ✅ 把同一个 history 传进去（与你之前的函数签名一致）
        run_conversation_loop()

