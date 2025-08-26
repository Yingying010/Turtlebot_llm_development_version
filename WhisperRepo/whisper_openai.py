import os, sys
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)

import queue, threading, time, re, subprocess
import numpy as np
import sounddevice as sd
from scipy.io.wavfile import write
from loguru import logger
import config
from ttsRepo.stream_tts import tts_manager
from typing import Final
import wave

# === 新增：OpenAI 客户端 ===
from openai import OpenAI

conversation_active: Final[threading.Event] = threading.Event()

# === 参数 ===
SAMPLERATE = 48000
BLOCKSIZE = 1024
SILENCE_THRESHOLD = 20.0
SILENCE_DURATION  = 1.5
MAX_DURATION      = 30
FIXED_WAV_PATH    = "/tmp/voice_input.wav"

# 选择 OpenAI 的 STT 模型（可改成 "gpt-4o-transcribe" 追求更高精度）
OPENAI_STT_MODEL = os.getenv("OPENAI_STT_MODEL", "gpt-4o-mini-transcribe")

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
    pre_speech_maxlen = 24
    audio_blocks      = []
    silence_counter   = 0
    is_recording      = False

    def cb(indata, frames, time_info, status):
        if status:
            logger.warning(f"⚠️ Audio status: {status}")
        q_local.put(indata.copy())

    logger.info("🎙️ Waiting for speech to start...")

    # 如果你在远程 PulseAudio，用 device=("pulse", None)；本地默认可去掉 device 参数
    with sd.InputStream(samplerate=SAMPLERATE, channels=1,
                        blocksize=BLOCKSIZE, callback=cb,
                        device=("pulse", None)):
        while True:
            try:
                block = q_local.get(timeout=1)
            except queue.Empty:
                continue

            volume = np.abs(block).mean() * 1000
            logger.debug(f"📊 Vol: {volume:.1f}")

            pre_speech_buffer.append(block)
            if len(pre_speech_buffer) > pre_speech_maxlen:
                pre_speech_buffer.pop(0)

            if not is_recording:
                if volume > threshold:
                    logger.info("🔴 Voice detected. Start recording...")
                    is_recording = True
                    audio_blocks.extend(pre_speech_buffer)
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

# === 使用 OpenAI Audio API 转录 ===
def transcribe_audio(wav_path: str, delay: float = 0.0) -> str:
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        logger.error("❌ OPENAI_API_KEY 未设置。请先在环境变量中配置。")
        return ""

    client = OpenAI(api_key=api_key)

    try:
        with open(wav_path, "rb") as f:
            # 直接文件转写：返回对象含 text 字段
            # 支持的模型：gpt-4o-transcribe / gpt-4o-mini-transcribe
            # 也可传 response_format="text" 来直接拿纯文本
            resp = client.audio.transcriptions.create(
                model=OPENAI_STT_MODEL,
                file=f,
                language="en",
                # 可选参数：
                # language="zh",        # 明确语言可能提升准确率
                # temperature=0,        # 更稳定输出
                # response_format="text"  # 纯文本；默认会有 resp.text
            )
        # SDK 通常提供 resp.text；为兼容性再兜底一下：
        raw_text = getattr(resp, "text", None) or str(resp)
        clean_text = _clean(raw_text or "")

        logger.success(f"📝 Transcribed Text: {clean_text or '<EMPTY>'}")
        if delay:
            time.sleep(delay)
        return clean_text
    except Exception as e:
        logger.exception(f"❌ OpenAI 转写失败: {e}")
        return ""

def recognize(delay: float = 0.0) -> str:
    wav_path = record_until_silence()
    return transcribe_audio(wav_path, delay)
