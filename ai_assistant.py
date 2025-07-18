import sys
import os
from config import config
# from VoskReco.Vosk_run import Vosk_run, recognize
from WhisperReco.whisper_run import Whisper_run, recognize
import TinyLlama_Control
import TinyLlama_Chat
import control_turtlebot
# from Phi_parse_instruction_and_chat import send, execute_commands, stop_generation_flag
import time
from stream_tts import tts_manager
from loguru import logger
import if_exit, if_time
from loguru import logger
from WhisperReco.whisper_run import conversation_active
import play


'''
✅ 日志记录
✅ 合理状态管理：running, actived, allow_running
✅ 唤醒时打断语音播放（tts_manager.stop()）
✅ 唤醒时说一句话提示用户（tts_manager.say(...)）
'''
running = False
actived = 0
allow_running = True

# ✅ 唤醒词触发回调
def hwcallback():


    global running, actived, allow_running
    logger.info('🟡 HotWord triggered')

    if running and not allow_running:
        actived = 3  # 多次唤醒错误状态
        logger.warning("❌ Repeated wakeword detected but not allowed to interrupt.")
        return False

    if running:
        actived = 2  # 正在对话中再次唤醒
        TinyLlama_Control.stop_generation_flag.set() # # 在 actived = 2 的判断中添加
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
    """
    一次完整的语音对话 / 指令控制流程：
    1. 录音并语音转文字  recognize()
    2. 处理退出 / 定时 等本地指令
    3. 调用 LLM (chat 或 control)
    4. 返回结果（TTS 或执行设备指令）
    5. 对话结束 -> conversation_active.clear()
    """
    logger.info("🎤 Recording...")
    tts_manager.say("I'm listening.")
    time.sleep(3)

    # -------- 录音 + 识别 --------------------------------------------------
    try:
        user_text = recognize(delay=3)          # 新版 recognize 无 duration
    except Exception as e:
        logger.error(f"🎙️ Speech recognition error: {e}")
        tts_manager.say("Sorry, I couldn't hear you.")
        conversation_active.clear()
        return

    if not user_text.strip():
        tts_manager.say("Sorry, I didn't catch that.")
        conversation_active.clear()
        return

    logger.info(f"🧑 User said: {user_text}")

    # -------- 本地退出 / 定时等指令 ---------------------------------------
    if if_exit.ifend(user_text):
        tts_manager.say("Okay, ending the conversation.")
        conversation_active.clear()
        return

    if if_exit.ifexit(user_text):
        tts_manager.say("Goodbye.")
        logger.info("🔚 Exit triggered by user.")
        conversation_active.clear()
        exit(0)

    if if_time.timedetect(user_text):
        tts_manager.say("Timer has been set.")
        conversation_active.clear()
        return

    # -------- 调用 LLM (Chat / Control) ----------------------------------
    is_chat = config.get("chat_or_instruct")
    try:
        if is_chat:
            response = TinyLlama_Chat.run(user_text)
        else:
            response = TinyLlama_Control.run(user_text)

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
            control_turtlebot.run(response)
            tts_manager.say("Command executed.")
            logger.info("✅ Command(s) executed successfully.")
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
    logger.info("📢 Starting chat system")
    tts_manager.say("Welcome! You can start speaking after the beep.")



# ✅ 启动入口
if __name__ == "__main__":
    startchat()
    time.sleep(4)  # ✅ 给用户准备说话时间，避免误触程）
    play.play_beep_aplay("soundRepo/beep.wav")
    Whisper_run(hwcallback)
    dialog_manager()        # 主对话处理循环