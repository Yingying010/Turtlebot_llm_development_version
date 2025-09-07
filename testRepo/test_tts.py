# stream_tts_test.py
import os,sys
import time
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)
from ttsRepo.stream_tts import tts_manager

def main():
    tts_manager.say_sync("test")

if __name__ == "__main__":
    main()