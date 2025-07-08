import platform
import os
import time
from loguru import logger

def set_speaker_volume(volume):
    """
    volume: float between 0.0 ~ 1.0
    """
    percent = int(volume * 100)
    if platform.system() == "Linux":
        os.system(f"amixer -D pulse sset Master {percent}%")
    elif platform.system() == "Darwin":  # macOS
        os.system(f"osascript -e 'set volume output volume {percent}'")
    else:
        logger.warning("⚠️ set_speaker_volume not supported on this OS")

    logger.info("✅ Speaker volume has been set to {volume}.")
