import os, sys, time
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from ttsRepo.stream_tts import tts_manager


def dropoff_item(item: str):
    is_successful = False
    print(f"🗣️ Speaking: I am dropping off {item}")
    tts_manager.say(f"I am dropping off {item}")
    time.sleep(3)  # 模拟收集时间
    tts_manager.say(f"{item} dropped off successfully")

    return is_successful