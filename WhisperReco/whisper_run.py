"""
WhisperReco/whisper_run.py
--------------------------
后台线程：监听麦克风，识别唤醒词并切换聊天 / 控制模式
"""

import os, queue, tempfile, threading, time, warnings, re, string
from typing import Final

import numpy as np
import sounddevice as sd
from scipy.io.wavfile import write
import whisper
from loguru import logger

from config import config                 # 全局配置管理
from stream_tts import tts_manager        # TTS
# run_conversation() 里需要用到的对话事件
from threading import Event
conversation_active: Final[Event] = Event()

# ─────────────────── 音频 / 录音参数 ───────────────────
SAMPLERATE        = 16_000
BLOCKSIZE         = 1024
SILENCE_THRESHOLD = 20        # 音量阈值（越低越敏感）
SILENCE_DURATION  = 1.0       # 判定静音的持续秒数
MAX_DURATION      = 10        # 单段最长录音秒数
# ──────────────────────────────────────────────────────

warnings.filterwarnings("ignore",
                        message="FP16 is not supported on CPU; using FP32 instead")

model       = whisper.load_model("small.en")
model_lock  = threading.Lock()   # 互斥跑 Whisper

# ──────────────────────── 工具函数 ─────────────────────
def _clean(text: str) -> str:
    """去标点、转小写、收尾空格"""
    return re.sub(r'[^\w\s]', '', text).lower().strip()

# ────────────────────── 录音直到静音 ───────────────────
def record_until_silence(threshold=SILENCE_THRESHOLD,
                         silence_duration=SILENCE_DURATION,
                         max_duration=MAX_DURATION) -> np.ndarray:
    """
    1. 先等待“有人说话”（volume > threshold）  
    2. 录音，直到连续 `silence_duration` 秒音量 < threshold  
    3. 或者超过 `max_duration` 秒就强制停止  
    返回：int16 PCM 一维数组
    """
    q_local       = queue.Queue()
    silence_blocks = int(silence_duration * SAMPLERATE / BLOCKSIZE)
    max_blocks     = int(max_duration   * SAMPLERATE / BLOCKSIZE)

    def cb(indata, frames, time_info, status):
        if status:
            logger.warning(f"⚠️ Audio status: {status}")
        q_local.put(indata.copy())

    logger.info("🎙️ Waiting for speech to start...")
    audio_blocks    = []
    silence_counter = 0
    is_recording    = False

    with sd.InputStream(samplerate=SAMPLERATE,
                        channels=1,
                        blocksize=BLOCKSIZE,
                        callback=cb):
        while True:
            try:
                block = q_local.get(timeout=1)
            except queue.Empty:
                continue

            volume = np.abs(block).mean() * 1000
            logger.debug(f"📊 Vol: {volume:.1f}")

            if not is_recording:
                if volume > threshold:
                    logger.info("🔴 Voice detected. Start recording...")
                    is_recording = True
                    audio_blocks.append(block)
                continue  # 继续读下一块

            # 录音阶段
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

    pcm_f32 = np.concatenate(audio_blocks).flatten()        # float32 [-1,1]
    pcm_i16 = (pcm_f32 * 32767).clip(-32768, 32767).astype(np.int16)
    return pcm_i16

# ───────────────────── Whisper 转录 ─────────────────────
def transcribe_audio(audio: np.ndarray, delay: float = 0.0) -> str:
    if audio.dtype != np.int16:
        audio = (audio * 32767).clip(-32768, 32767).astype(np.int16)

    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        write(f.name, SAMPLERATE, audio)
        wav_path = f.name

    with model_lock:
        result = model.transcribe(wav_path, language="en")

    os.unlink(wav_path)
    text = result["text"].lower().strip()
    logger.success(f"📝 Transcribed Text: {text or '<EMPTY>'}")

    if delay:
        time.sleep(delay)
    return text

def recognize(delay: float = 0.0) -> str:
    audio = record_until_silence()
    return transcribe_audio(audio, delay)

# ──────────────────── 启动热词后台线程 ──────────────────
def Whisper_run(callback_func):
    def loop():
        print("🟢 Whisper hotword loop started")
        while True:
            # 如果主线程正在对话，暂停监听
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
