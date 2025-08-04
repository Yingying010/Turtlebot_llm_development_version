import sys
import os
import os, sys
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)
from config import config
from WhisperRepo.whisper_recognizer import Whisper_run, conversation_active
import llmParserRepo.qwen3_parser as llama8_Control
# import TinyLlama_Chat
import robotControllerRepo.robot_scheduler as robot_scheduler
import time
from ttsRepo.stream_tts import tts_manager
from loguru import logger
from loguru import logger

running = False
actived = 0
allow_running = True

def hwcallback():


    global running, actived, allow_running
    logger.info('🟡 HotWord triggered')

    if running and not allow_running:
        actived = 3 
        logger.warning("❌ Repeated wakeword detected but not allowed to interrupt.")
        return False

    if running:
        actived = 2 
        logger.info("⚠️ Conversation interrupted by new wakeword.")
        tts_manager.stop()  # 🛑 停止当前播放
        tts_manager.say("Okay, I stopped. You can speak again.")
    else:
        actived = 1  # 正常唤醒
        logger.info("✅ Wakeword detected. Starting new interaction.")

    return True


# ✅ 主对话管理器
def dialog_manager():
    global running, actived
    while True:
        if actived == 1:
            running = True
            actived = 0
            run_conversation()
            running = False
        elif actived == 2:
            tts_manager.stop()
            running = False
            actived = 1
        time.sleep(0.2)



def run_conversation():
    logger.info("🎤 Recording...")
    tts_manager.say("I'm listening.")
    tts_manager.wait_until_done()

    # -------- Chat / Control ----------------------------------
    is_chat = config.get("chat_or_instruct")
    try:
        if is_chat:
            # response = TinyLlama_Chat.run()
            response = None
        else:
            response = llama8_Control.run_conversation_loop()

    except Exception as e:
        logger.error(f"🧠 LLM error: {e}")
        tts_manager.say("Something went wrong while thinking.")
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

    else:                               # Control 模式
        if response:
            isSchedule = robot_scheduler.run(response)
            if isSchedule == True:
                logger.info("✅ Command(s) executed successfully.")
                tts_manager.say("Command executed.")
                time.sleep(1)
            else:
                logger.warning("⚠️ Failed")
                tts_manager.say("Parsing failed so the command cannot be executed.")
                time.sleep(1)

        else:
            logger.warning("⚠️ No commands received from LLM.")
            tts_manager.say("Sorry, I couldn't understand the instruction.")
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



# ✅ 启动入口
if __name__ == "__main__":
    if len(sys.argv)<2:
        print("Usage: python3 main.py <robot_id>")
        sys.exit(1)

    robot_id = sys.argv[1]
    config.set(robot_id=robot_id)
    print(config.get("robot_id"))

    startchat()
    time.sleep(4)  # ✅ 给用户准备说话时间，避免误触
    # Vosk_run(hwcallback)    # 热词检测循环（另起线程）
    Whisper_run(hwcallback)
    dialog_manager()        # 主对话处理循环