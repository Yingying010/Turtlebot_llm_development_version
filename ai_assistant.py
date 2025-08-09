#!/usr/bin/env python3
import os, sys, time, traceback
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)

from config import config
from llmParserRepo.qwen3_parser import run_conversation_loop
from ttsRepo.stream_tts import tts_manager
from loguru import logger

# === 原有全局 ===
running = False
actived = 0
allow_running = True

# === 这里开始新增：ROS 订阅 /speech_text 作为热词/指令入口 ===
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import threading

SPEECH_TOPIC = "/speech_text"

# 你之前 whisper 中的语义搬过来即可
WAKE_CONTROL  = {"open robot system"}                     # 进入控制模式
WAKE_CHAT     = {"i want to chat with you", "open chat system"}  # 进入聊天模式
EXIT_WORDS    = {"ok bye", "okay bye", "ok byebye", "okay byebye", "finish system"}

# ========== 你原有的会话状态 ==========
from WhisperRepo.whisper_recognizer import conversation_active  # 继续复用

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
        tts_manager.stop()
        tts_manager.say("Okay, I stopped. You can speak again.")
    else:
        actived = 1
        logger.info("✅ Wakeword detected. Starting new interaction.")
    return True

# ========= ROS 订阅节点：将转录文本 -> 驱动对话状态 =========
class SpeechGateway(Node):
    def __init__(self):
        super().__init__("speech_gateway")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, SPEECH_TOPIC, self.on_text, qos)
        self.get_logger().info(f"🎧 Listening on {SPEECH_TOPIC}")

    def on_text(self, msg: String):
        try:
            text = (msg.data or "").strip().lower()
            if not text:
                return

            if any(kw in text for kw in WAKE_CONTROL):
                config.set(chat_or_instruct=False)
                logger.info("🎮 Switched to CONTROL mode.")
                tts_manager.say("Okay, I'm now in control mode.")
                conversation_active.set()
                _ = hwcallback()
                return

            if any(kw in text for kw in WAKE_CHAT):
                config.set(chat_or_instruct=True)
                logger.info("💬 Switched to CHAT mode.")
                tts_manager.say("Sure, I'm now in chat mode.")
                conversation_active.set()
                _ = hwcallback()
                return

            if any(kw in text for kw in EXIT_WORDS):
                tts_manager.say("Goodbye!")
                time.sleep(1)
                os._exit(0)

            # 其它普通文本：这里不处理（留给各自业务模块去订阅）
        except Exception:
            logger.error(f"speech callback error:\n{traceback.format_exc()}")

# ========= 你原有的对话管理逻辑（保持不变）=========
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
    time.sleep(0.5)
    is_chat = config.get("chat_or_instruct")
    try:
        if is_chat:
            response = None  # 你的聊天模型入口
        else:
            response = run_conversation_loop()
    except Exception:
        logger.error(f"🧠 LLM error: \n{traceback.format_exc()}")
        tts_manager.say("Something went wrong while thinking")
        time.sleep(0.5)
        conversation_active.clear()
        return

    if config.get("chat_or_instruct"):
        if isinstance(response, str):
            tts_manager.say(response)
            logger.info("✅ Chat response delivered.")
        else:
            logger.warning("⚠️ LLM chat mode returned unexpected format.")
            tts_manager.say("Sorry, something went wrong.")
    else:
        conversation_active.clear()
        return

    logger.info("✅ Conversation finished.")
    conversation_active.clear()

def startchat():
    os.system("afplay beep.wav")
    logger.info("📢 Starting chat system")
    tts_manager.say("Welcome! You can start speaking after the beep.")
    time.sleep(0.5)

# ========= 入口 =========
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 main.py <robot_id>")
        sys.exit(1)

    robot_id = sys.argv[1]
    config.set(robot_id=robot_id)
    print(config.get("robot_id"))

    startchat()
    time.sleep(1)

    # 1) 初始化 ROS（兼容“已在别处 init”的场景）
    try:
        rclpy.init()
    except RuntimeError:
        pass

    # 2) 创建节点并后台 spin
    node = SpeechGateway()
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    # 3) 主对话管理循环（阻塞）
    dialog_manager()
