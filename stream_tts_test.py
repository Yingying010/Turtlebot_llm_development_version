from stream_tts import tts_manager
import time

tts_manager.say("Hello! I am your espeak based assistant.")
time.sleep(1)  # 模拟中途打断
tts_manager.stop()
