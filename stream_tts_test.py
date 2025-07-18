from stream_tts import tts_manager
import time

tts_manager.say("Hello! I am your assistant.")
time.sleep(5)  # 模拟中途打断
tts_manager.say("Good Bye")
time.sleep(5)  # 模拟中途打断
tts_manager.stop()
