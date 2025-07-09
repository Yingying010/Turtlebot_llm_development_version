import sys, os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

import tempfile, threading, time, warnings, re, string, subprocess
from typing import Final
import numpy as np
from loguru import logger
from config import config
from stream_tts import tts_manager

conversation_active: Final[threading.Event] = threading.Event()

# === 录音参数 ===
SAMPLERATE = 48000
DURATION = 5  # 秒
DEVICE = "plughw:1,0"
SILENCE_THRESHOLD = 20
SILENCE_DURATION = 1.0
MAX_DURATION = 10

# === 工具函数 ===
def _clean(text: str) -> str:
    return re.sub(r'[^\w\s]', '', text).lower().strip()

# === 使用 arecord 命令录音到临时文件 ===
def record_with_arecord(duration=DURATION, device=DEVICE) -> str:
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        wav_path = f.name

    logger.info(f"🎙️ Recording {duration}s with arecord...")
    cmd = [
        "arecord",
        "-D", device,
        "-r", str(SAMPLERATE),
        "-f", "S16_LE",
        "-c", "1",
        "-t", "wav",
        "-d", str(duration),
        wav_path
    ]
    subprocess.run(cmd)
    logger.success(f"✅ Recording finished: {wav_path}")
    return wav_path

# === 使用 whisper-cli 转录语音 ===
def transcribe_audio(wav_path: str, delay: float = 0.0) -> str:
    model_path = os.path.expanduser("~/ggml-tiny.en.bin")
    cli_path   = os.path.expanduser("~/whisper.cpp/build/bin/whisper-cli")

    cmd = [cli_path, "-m", model_path, "-f", wav_path]
    result = subprocess.run(cmd, capture_output=True, text=True)

    output = result.stdout.strip()
    os.unlink(wav_path)  # 删除录音文件

    lines = output.strip().splitlines()
    text_lines = [line for line in lines if line and not line.startswith("###")]
    text = text_lines[-1] if text_lines else ""

    logger.success(f"📝 Transcribed Text: {text or '<EMPTY>'}")
    if delay:
        time.sleep(delay)
    return text

# === 一次性录音并转录 ===
def recognize(delay: float = 0.0) -> str:
    wav_path = record_with_arecord()
    return transcribe_audio(wav_path, delay)

# === 热词识别后台线程 ===
def Whisper_run(callback_func):
    def loop():
        print("🟢 Whisper hotword loop started")
        while True:
            if conversation_active.is_set():
                time.sleep(0.2)
                continue

            raw_text   = recognize(delay=3)
            clean_text = _clean(raw_text)
            if not clean_text:
                continue

            if "hi controller" in clean_text:
                config.set(chat_or_instruct=False)
                logger.info("🎮 Switched to CONTROL mode.")
                tts_manager.say("Okay, I'm now in control mode.")
                conversation_active.set()
                callback_func()

            elif "hi assistant" in clean_text:
                config.set(chat_or_instruct=True)
                logger.info("💬 Switched to CHAT mode.")
                tts_manager.say("Sure, I'm now in chat mode.")
                conversation_active.set()
                callback_func()

            elif clean_text in {"ok bye", "okay bye", "ok byebye", "okay byebye"}:
                tts_manager.say("Goodbye!")
                time.sleep(1)
                os._exit(0)

    threading.Thread(target=loop, daemon=True).start()

# === 单独运行测试 ===
if __name__ == "__main__":
    logger.info("🎤 Start single recognition test...")
    result = recognize()
    print("🗣️ You said:", result or "<nothing>")
