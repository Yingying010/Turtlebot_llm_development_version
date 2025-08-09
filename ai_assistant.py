import sys
import os
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)
from config import config
from WhisperRepo.whisper_recognizer import conversation_active, recognize
from llmParserRepo.qwen3_parser import run_conversation_loop
# import TinyLlama_Chat
import time
import re
from ttsRepo.stream_tts import tts_manager
from loguru import logger
from loguru import logger
import traceback

running = False
actived = 0
allow_running = True

def run_conversation():
    logger.info("🎤 Recording...")
    tts_manager.say("I'm listening.")
    time.sleep(0.5)

    # -------- Chat / Control ----------------------------------
    is_chat = config.get("chat_or_instruct")
    try:
        if is_chat:
            # response = TinyLlama_Chat.run()
            response = None
        else:
            response = run_conversation_loop()

    except Exception:
        logger.error(f"🧠 LLM error: \n{traceback.format_exc()}")
        tts_manager.say("Something went wrong while thinking")
        time.sleep(0.5)
        conversation_active.clear()
        return

    # -------- 根据模式反馈 ------------------------------------------------
    if config.get("chat_or_instruct"):  # Chat 模式
        if isinstance(response, str):
            tts_manager.say(response)
            logger.info("✅ Chat response delivered.")
        else:
            logger.warning("⚠️ LLM chat mode returned unexpected format.")
            tts_manager.say("Sorry, something went wrong.")
    else:
            conversation_active.clear()
            return


    # -------- 对话结束 ----------------------------------------------------
    logger.info("✅ Conversation finished.")
    conversation_active.clear()

# ✅ 启动欢迎语
def startchat():
    os.system("afplay beep.wav")
    logger.info("📢 Starting chat system")
    tts_manager.say("Welcome! You can start speaking after the beep.")
    time.sleep(0.5)

def _clean(text: str) -> str:
    return re.sub(r'[^\w\s]', '', text).lower().strip()


# ✅ 启动入口
if __name__ == "__main__":
    if len(sys.argv)<2:
        print("Usage: python3 main.py <robot_id>")
        sys.exit(1)

    robot_id = sys.argv[1]
    config.set(robot_id=robot_id)
    print(config.get("robot_id"))

    startchat()
    time.sleep(4)

    while True:
        while True:
            try:
                raw_text = recognize(delay=3).strip()
            except Exception as e:
                logger.warning(f"🎙️ recognition failed: {e}")
                tts_manager.say("Sorry, could not hear you.")
                time.sleep(2)
                continue
        
            wakeup_word = _clean(raw_text)
            if not wakeup_word:
                continue
            else:
                break

        if "open robot system" in wakeup_word:
            config.set(chat_or_instruct=False)
            logger.info("🎮 Switched to CONTROL mode.")
            tts_manager.say("Okay, I'm now in control mode. If you want to exit this mode, just say ending this mode.")
        elif wakeup_word in {"i want to chat with you", "open chat system"}:
            config.set(chat_or_instruct=True)
            logger.info("💬 Switched to CHAT mode.")
            tts_manager.say("Sure, I'm now in chat mode.")
        elif wakeup_word in {"ok bye", "okay bye", "ok byebye", "okay byebye","finish system"}:
            tts_manager.say("Goodbye!")
            time.sleep(1)
            os._exit(0)
        else:
            continue

        run_conversation()

