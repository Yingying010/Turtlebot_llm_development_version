import os, sys, time
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from ttsRepo.stream_tts import tts_manager


def pickup_item(item:str):
    print(f"🗣️ Speaking: I am picking up {item}")
    tts_manager.say_sync(f"I am picking up {item}")
    time.sleep(10)  # 模拟收集时间
    print("---------------------------------------")
    tts_manager.say_sync(f"{item} picked up successfully")

    return True