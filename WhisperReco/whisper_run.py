import sys, os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

import threading, time, subprocess, tempfile, re
from typing import Final
from loguru import logger
from config import config
from stream_tts import tts_manager

# === 运行状态控制 ===
conversation_active: Final[threading.Event] = threading.Event()

# === 参数配置 ===
DEVICE = "plughw:1,0"                     # 根据你的麦克风设备编号设置
SAMPLERATE = 48000
DURATION_MAX = 10                         # 最大录音时长（秒）
SILENCE_THRESHOLD_DB = 5                 # 静音判定阈值（dB）
SILENCE_DURATION = 1.0                   # 静音持续时长（秒）
MODEL_PATH = os.path.expanduser("~/ggml-tiny.en.bin")
CLI_PATH = os.path.expanduser("~/whisper.cpp/build/bin/whisper-cli")

# === 清理文本 ===
def _clean(text: str) -> str:
    return re.sub(r'[^\w\s]', '', text).lower().strip()

# === 用 arecord 录音直到静音 ===
def record_until_silence_arecord() -> str:
    tmp_wav = tempfile.NamedTemporaryFile(suffix=".wav", delete=False).name
    logger.info("🎙️ Recording until silence (via arecord)...")

    cmd = [
        "sox", "-t", "alsa", DEVICE, "-r", str(SAMPLERATE), "-c", "1", "-b", "16",
        tmp_wav, "silence", "1", f"{int(SILENCE_DURATION)}", f"{SILENCE_THRESHOLD_DB}%",
        "1", f"{int(SILENCE_DURATION)}", f"{SILENCE_THRESHOLD_DB}%"
    ]

    try:
        subprocess.run(cmd, check=True)
        logger.info("🔇 Silence detected. Recording stopped.")
    except subprocess.CalledProcessError:
        logger.warning("⚠️ sox recording failed or was too short.")

    return tmp_wav

# === Whisper CLI 进行转录 ===
def transcribe_audio(wav_path: str, delay: float = 0.0) -> str:
    cmd = [CLI_PATH, "-m", MODEL_PATH, "-f", wav_path]
    result = subprocess.run(cmd, capture_output=True, text=True)

    output = result.stdout.strip()
    os.unlink(wav_path)

    lines = output.strip().splitlines()
    text_lines = [line for line in lines if line and not line.startswith("###")]
    text = text_lines[-1] if text_lines else ""

    logger.success(f"📝 Transcribed Text: {text or '<EMPTY>'}")
    if delay: time.sleep(delay)
    return text

# === 主识别函数 ===
def recognize(delay: float = 0.0) -> str:
    wav_path = record_until_silence_arecord()
    return transcribe_audio(wav_path, delay)

# === 后台线程：识别热词 ===
def Whisper_run(callback_func):
    def loop():
        print("🟢 Whisper hotword loop started")
        while True:
            if conversation_active.is_set():
                time.sleep(0.2)
                continue

            raw_text = recognize(delay=2)
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

# === 测试运行 ===
if __name__ == "__main__":
    logger.info("🎤 Start single recognition test...")
    result = recognize()
    print("🗣️ You said:", result or "<nothing>")
