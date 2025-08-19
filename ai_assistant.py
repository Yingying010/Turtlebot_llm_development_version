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
        tts_manager.say_sync("Something went wrong while thinking")
        conversation_active.clear()
        return

    # -------- 根据模式反馈 ------------------------------------------------
    if config.get("chat_or_instruct"):  # Chat 模式
        if isinstance(response, str):
            tts_manager.say_sync(response)
            logger.info("✅ Chat response delivered.")
        else:
            logger.warning("⚠️ LLM chat mode returned unexpected format.")
            tts_manager.say_sync("Sorry, something went wrong.")
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
    tts_manager.say_sync("Welcome! You can start speaking after the beep")

def _clean(text: str) -> str:
    return re.sub(r'[^\w\s]', '', text).lower().strip()


# ✅ 启动入口
if __name__ == "__main__":
    import argparse
    from pathlib import Path

    parser = argparse.ArgumentParser()
    parser.add_argument("robot_id", help="robot name")
    parser.add_argument("master_id", help="master name")
    parser.add_argument("--clear-memory", action="store_true",
                        help="在程序结束后清空memory")
    args = parser.parse_args()

    # 本地创建 HistoryStore（不放进 config）
    memory_path = Path(f"./memory_{args.robot_id}.jsonl")
    history = HistoryStore(memory_path)

    # 仍然用 config 存“可序列化配置”
    config.set(robot_id=args.robot_id)
    config.set(master_id=args.master_id)

    print(config.get("robot_id"))   # robot1
    # print(config.get("history"))  # ❌ 不再从 config 取

    try:
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
                import sys
                sys.exit(0)
            else:
                continue

            # ✅ 把同一个 history 传进去（与你之前的函数签名一致）
            run_conversation_loop(history)

    finally:
        # ✅ 用本地变量清理
        if args.clear_memory:
            try:
                history.clear()
                logger.info("🧹 Memory cleared after program exit.")
            except Exception as e:
                logger.warning(f"⚠️ Failed to clear memory: {e}")

