import sys, os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

import queue, threading, time, re, subprocess
import numpy as np
import sounddevice as sd
from scipy.io.wavfile import write
from loguru import logger
from config import config
from stream_tts import tts_manager
from typing import Final
import wave

conversation_active: Final[threading.Event] = threading.Event()

# === 参数 ===
SAMPLERATE = 48000
BLOCKSIZE = 1024
SILENCE_THRESHOLD = 10.0
SILENCE_DURATION  = 1.0
MAX_DURATION      = 10
FIXED_WAV_PATH    = "/tmp/voice_input.wav"

# === 清理文本 ===
def _clean(text: str) -> str:
    return re.sub(r'[^\w\s]', '', text).lower().strip()

# === 标准写入 wav 文件 ===
def save_wav_standard(wav_path, audio_int16, samplerate=48000):
    with wave.open(wav_path, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)  # 16-bit PCM
        wf.setframerate(samplerate)
        wf.writeframes(audio_int16.tobytes())

# === 录音直到静音结束 ===
def record_until_silence(threshold=SILENCE_THRESHOLD,
                         silence_duration=SILENCE_DURATION,
                         max_duration=MAX_DURATION) -> str:
    q_local         = queue.Queue()
    silence_blocks  = int(silence_duration * SAMPLERATE / BLOCKSIZE)
    max_blocks      = int(max_duration * SAMPLERATE / BLOCKSIZE)

    def cb(indata, frames, time_info, status):
        if status:
            logger.warning(f"\u26a0\ufe0f Audio status: {status}")
        q_local.put(indata.copy())

    logger.info("\ud83c\udfa7 Waiting for speech to start...")

    audio_blocks    = []
    volume_list     = []
    silence_counter = 0
    is_recording    = False
    has_valid_speech = False

    with sd.InputStream(samplerate=SAMPLERATE, channels=1,
                        blocksize=BLOCKSIZE, callback=cb):
        while True:
            try:
                block = q_local.get(timeout=1)
            except queue.Empty:
                continue

            volume = np.abs(block).mean() * 1000
            logger.debug(f"\ud83d\udcca Vol: {volume:.1f}")

            if not is_recording:
                if volume > 6.5:  # 低队值开始录音
                    logger.info("\ud83d\udd34 Voice detected. Start recording...")
                    is_recording = True
                    audio_blocks.append(block)
                    volume_list.append(volume)
                continue

            audio_blocks.append(block)
            volume_list.append(volume)

            if volume > 10.0:
                has_valid_speech = True

            if volume < threshold:
                silence_counter += 1
                if silence_counter >= silence_blocks:
                    logger.info("\ud83d\udd07 Silence detected. Stopping recording.")
                    break
            else:
                silence_counter = 0

            if len(audio_blocks) >= max_blocks:
                logger.info("\u23f0 Max recording length reached. Forcing stop.")
                break

    if not has_valid_speech:
        logger.warning("\u26a0\ufe0f No valid speech detected during recording. Skipping save.")
        return None

    # === 从第一个 vol > 10 的位置切割 ===
    start_idx = next((i for i, v in enumerate(volume_list) if v > 10.0), 0)
    trimmed_blocks = audio_blocks[start_idx:]
    if not trimmed_blocks:
        logger.warning("\u26a0\ufe0f No audio above 10.0 dB, using full recording.")
        trimmed_blocks = audio_blocks

    # 保存为 WAV 文件
    pcm_f32 = np.concatenate(trimmed_blocks).flatten()
    pcm_i16 = (pcm_f32 * 32767).clip(-32768, 32767).astype(np.int16)

    save_wav_standard(FIXED_WAV_PATH, pcm_i16, SAMPLERATE)
    logger.success(f"\ud83d\udcc2 Saved trimmed recording to {FIXED_WAV_PATH}")
    return FIXED_WAV_PATH

# === 调用 whisper-cli 转录 ===
def transcribe_audio(wav_path: str, delay: float = 0.0) -> str:
    model_path = os.path.expanduser("~/whisper.cpp/models/ggml-base.en.bin")
    cli_path   = os.path.expanduser("~/whisper.cpp/build/bin/whisper-cli")
    cmd = [cli_path, "-m", model_path, "-f", wav_path]

    result = subprocess.run(cmd, capture_output=True, text=True)
    output = result.stdout.strip()

    # 提取识别文本行：形如 "[00:00:00.000 --> 00:00:00.840]   - Hello, hello."
    lines = output.splitlines()
    text_lines = [
        line.split("]", 1)[-1].strip(" -\t") for line in lines
        if "-->" in line and "]" in line
    ]
    raw_text = " ".join(text_lines).strip()

    # === 删除标点符号（小写、去空格）===
    clean_text = _clean(raw_text)

    logger.success(f"\ud83d\udcdd Transcribed Text: {clean_text or '<EMPTY>'}")
    if delay:
        time.sleep(delay)
    return clean_text

# === 识别函数 ===
def recognize(delay: float = 0.0) -> str:
    wav_path = record_until_silence()
    if wav_path is None:
        return ""
    return transcribe_audio(wav_path, delay)

# === 后台热词识别线程 ===
def Whisper_run(callback_func):
    def loop():
        print("\ud83d\udfe2 Whisper hotword loop started")
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
                logger.info("\ud83c\udfae Switched to CONTROL mode.")
                tts_manager.say("Okay, I'm now in control mode.")
                conversation_active.set()
                callback_func()

            elif "hi assistant" in clean_text:
                config.set(chat_or_instruct=True)
                logger.info("\ud83d\udcac Switched to CHAT mode.")
                tts_manager.say("Sure, I'm now in chat mode.")
                conversation_active.set()
                callback_func()

            elif clean_text in {"ok bye", "okay bye", "ok byebye", "okay byebye"}:
                tts_manager.say("Goodbye!")
                time.sleep(1)
                os._exit(0)

    threading.Thread(target=loop, daemon=True).start()

# === 测试入口 ===
if __name__ == "__main__":
    logger.info("\ud83c\udfa4 Start single recognition test...")
    result = recognize()
    print("\ud83d\udde3\ufe0f You said:", result or "<nothing>")
