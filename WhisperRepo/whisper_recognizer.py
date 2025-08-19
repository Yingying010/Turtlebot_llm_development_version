import os, sys
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)

import queue, threading, time, re, subprocess
import numpy as np
import sounddevice as sd
from scipy.io.wavfile import write
from loguru import logger
from config import config
from ttsRepo.stream_tts import tts_manager
from typing import Final
import wave

conversation_active: Final[threading.Event] = threading.Event()

# === 参数 ===
SAMPLERATE = 48000
BLOCKSIZE = 1024
SILENCE_THRESHOLD = 20.0
SILENCE_DURATION  = 1.5
MAX_DURATION      = 30
FIXED_WAV_PATH    = "/tmp/voice_input.wav"
MODEL_PATH = os.path.expanduser("~/whisper.cpp/models/ggml-tiny.en.bin")

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

    pre_speech_buffer = []  # 保存最近的几个块
    pre_speech_maxlen = 24  # ← 前两个block（可以调整成更多）
    audio_blocks      = []
    silence_counter   = 0
    is_recording      = False

    def cb(indata, frames, time_info, status):
        if status:
            logger.warning(f"⚠️ Audio status: {status}")
        q_local.put(indata.copy())

    logger.info("🎙️ Waiting for speech to start...")

    with sd.InputStream(samplerate=SAMPLERATE, channels=1,
                blocksize=BLOCKSIZE, callback=cb,
                device=("pulse", None)):  # 指定使用 PulseAudio 的远程输入设备
        while True:
            try:
                block = q_local.get(timeout=1)
            except queue.Empty:
                continue

            volume = np.abs(block).mean() * 1000
            logger.debug(f"📊 Vol: {volume:.1f}")

            # 始终维护前2个block的缓存
            pre_speech_buffer.append(block)
            if len(pre_speech_buffer) > pre_speech_maxlen:
                pre_speech_buffer.pop(0)

            if not is_recording:
                if volume > threshold:
                    logger.info("🔴 Voice detected. Start recording...")
                    is_recording = True
                    audio_blocks.extend(pre_speech_buffer)  # 加上前面的缓存
                    audio_blocks.append(block)
                continue

            audio_blocks.append(block)

            if volume < threshold:
                silence_counter += 1
                if silence_counter >= silence_blocks:
                    logger.info("🔇 Silence detected. Stopping recording.")
                    break
            else:
                silence_counter = 0

            if len(audio_blocks) >= max_blocks:
                logger.info("⏰ Max recording length reached. Forcing stop.")
                break

    # === 保存为固定路径 wav 文件 ===
    pcm_f32 = np.concatenate(audio_blocks).flatten()
    pcm_i16 = (pcm_f32 * 32767).clip(-32768, 32767).astype(np.int16)

    save_wav_standard(FIXED_WAV_PATH, pcm_i16, SAMPLERATE)
    logger.success(f"💾 Saved recording to {FIXED_WAV_PATH}")
    return FIXED_WAV_PATH

# === 调用 whisper-cli 转录 ===
def transcribe_audio(wav_path: str, delay: float = 0.0) -> str:
    model_path = MODEL_PATH
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

    logger.success(f"📝 Transcribed Text: {clean_text or '<EMPTY>'}")
    if delay:
        time.sleep(delay)
    return clean_text

def recognize(delay: float = 0.0) -> str:
    wav_path = record_until_silence()
    return transcribe_audio(wav_path, delay)
