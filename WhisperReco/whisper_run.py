import sys, os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

import queue, tempfile, threading, time, warnings, re, string, subprocess
from typing import Final
import numpy as np
import sounddevice as sd
from scipy.io.wavfile import write
from loguru import logger
from config import config
from stream_tts import tts_manager



conversation_active: Final[threading.Event] = threading.Event()

# === 录音参数 ===
SAMPLERATE = 48000
BLOCKSIZE = 1024
CHANNELS = 1
SILENCE_THRESHOLD = 20
SILENCE_DURATION = 1.0
MAX_DURATION = 10

# === 工具函数 ===
def _clean(text: str) -> str:
    return re.sub(r'[^\w\s]', '', text).lower().strip()

# === 录音直到静音结束 ===
def record_until_silence(threshold=SILENCE_THRESHOLD,
                         silence_duration=SILENCE_DURATION,
                         max_duration=MAX_DURATION) -> np.ndarray:
    q_local = queue.Queue()
    silence_blocks = int(silence_duration * SAMPLERATE / BLOCKSIZE)
    max_blocks = int(max_duration * SAMPLERATE / BLOCKSIZE)

    def cb(indata, frames, time_info, status):
        if status:
            logger.warning(f"⚠️ Audio status: {status}")
        q_local.put(indata.copy())

    logger.info("🎙️ Waiting for speech...")
    audio_blocks, silence_counter = [], 0
    is_recording = False

    with sd.InputStream(samplerate=SAMPLERATE, channels=1, blocksize=BLOCKSIZE, callback=cb):
        while True:
            try:
                block = q_local.get(timeout=1)
            except queue.Empty:
                continue

            volume = np.abs(block).mean() * 1000
            logger.debug(f"📊 Vol: {volume:.1f}")

            if not is_recording and volume > threshold:
                logger.info("🔴 Voice detected. Start recording...")
                is_recording = True
                audio_blocks.append(block)
                continue
            elif not is_recording:
                continue

            audio_blocks.append(block)
            silence_counter = silence_counter + 1 if volume < threshold else 0

            if silence_counter >= silence_blocks:
                logger.info("🔇 Silence detected. Stopping.")
                break
            if len(audio_blocks) >= max_blocks:
                logger.info("⏰ Max length reached.")
                break

    pcm_f32 = np.concatenate(audio_blocks).flatten()
    pcm_i16 = (pcm_f32 * 32767).clip(-32768, 32767).astype(np.int16)
    return pcm_i16

# === 使用 whisper.cpp 的 whisper-cli 来转录语音 ===
def transcribe_audio(audio: np.ndarray, delay: float = 0.0) -> str:
    if audio.dtype != np.int16:
        audio = (audio * 32767).clip(-32768, 32767).astype(np.int16)

    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        write(f.name, SAMPLERATE, audio)
        wav_path = f.name

    model_path = os.path.expanduser("~/ggml-tiny.en.bin")  # 你可修改为自己的路径
    cli_path   = os.path.expanduser("~/whisper.cpp/build/bin/whisper-cli")

    cmd = [cli_path, "-m", model_path, "-f", wav_path, "-nt", "-l", "en"]
    result = subprocess.run(cmd, capture_output=True, text=True)

    os.unlink(wav_path)
    output = result.stdout.strip()

    # 提取最后转录文本（跳过多余行）
    lines = output.strip().splitlines()
    text_lines = [line for line in lines if line and not line.startswith("###")]
    text = text_lines[-1] if text_lines else ""

    logger.success(f"📝 Transcribed Text: {text or '<EMPTY>'}")
    if delay: time.sleep(delay)
    return text

def recognize(delay: float = 0.0) -> str:
    audio = record_until_silence()
    return transcribe_audio(audio, delay)

# === 热词后台线程 ===
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
