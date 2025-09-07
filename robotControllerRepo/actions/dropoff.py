import os, sys, time
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from ttsRepo.stream_tts import tts_manager


def dropoff_item(item: str):
    print(f"🗣️ Speaking: I am dropping off {item}")
    tts_manager.say_sync(f"I am dropping off {item}")
    time.sleep(3)
    tts_manager.say_sync(f"{item} dropped off successfully")

    return True