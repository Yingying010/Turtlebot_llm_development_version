import os, sys, time
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from ttsRepo.stream_tts import tts_manager


def pickup_item(item:str):
    is_successful = False
    print(f"🗣️ Speaking: I am picking up {item}")
    tts_manager.say(f"I am picking up {item}")
    time.sleep(3)  # 模拟收集时间
    tts_manager.say(f"{item} picked up successfully")

    return is_successful