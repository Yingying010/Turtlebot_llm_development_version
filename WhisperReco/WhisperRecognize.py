import sys
import os
import queue
import threading
import time
import re
import subprocess
import tempfile
import numpy as np
import sounddevice as sd
from scipy.io.wavfile import write
from loguru import logger
from config import config
from stream_tts import tts_manager
from typing import Final
import wave

conversation_active: Final[threading.Event] = threading.Event()

# === 参数设置 ===
SAMPLERATE = 48000  # 保持48000Hz采样率
BLOCKSIZE = 1024    # 适合48000Hz的块大小
SILENCE_THRESHOLD = 15.0
SILENCE_DURATION = 1.0
MAX_DURATION = 10
FIXED_WAV_PATH = "/tmp/voice_input.wav"

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

# === 录音直到静音结束 (优化版) ===
def record_until_silence(threshold=SILENCE_THRESHOLD,
                         silence_duration=SILENCE_DURATION,
                         max_duration=MAX_DURATION) -> str:
    q_local = queue.Queue()
    silence_blocks = int(silence_duration * SAMPLERATE / BLOCKSIZE)
    max_blocks = int(max_duration * SAMPLERATE / BLOCKSIZE)

    pre_speech_buffer = []
    pre_speech_maxlen = 24
    silence_counter = 0
    is_recording = False

    # 使用临时文件而不是内存存储
    temp_wav = tempfile.NamedTemporaryFile(suffix='.wav', delete=False)
    temp_path = temp_wav.name
    temp_wav.close()

    def cb(indata, frames, time_info, status):
        nonlocal is_recording, silence_counter
        if status:
            logger.warning(f"⚠️ Audio status: {status}")
        
        block = indata.copy()
        volume = np.abs(block).mean() * 1000
        logger.debug(f"📊 Vol: {volume:.1f}")

        # 实时音频处理
        block = block * 1.2  # 增益控制
        block = np.clip(block, -1.0, 1.0)

        q_local.put((block, volume))

    logger.info("🎙️ Waiting for speech to start...")

    with sd.InputStream(samplerate=SAMPLERATE, channels=1,
                      blocksize=BLOCKSIZE, callback=cb):
        with wave.open(temp_path, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(SAMPLERATE)

            while True:
                try:
                    block, volume = q_local.get(timeout=1)
                except queue.Empty:
                    continue

                # 维护前几个block的缓存
                pre_speech_buffer.append(block)
                if len(pre_speech_buffer) > pre_speech_maxlen:
                    pre_speech_buffer.pop(0)

                if not is_recording:
                    if volume > threshold:
                        logger.info("🔴 Voice detected. Start recording...")
                        is_recording = True
                        # 写入缓存块
                        for buf_block in pre_speech_buffer:
                            pcm_i16 = (buf_block * 32767).clip(-32768, 32767).astype(np.int16)
                            wf.writeframes(pcm_i16.tobytes())
                        # 写入当前块
                        pcm_i16 = (block * 32767).clip(-32768, 32767).astype(np.int16)
                        wf.writeframes(pcm_i16.tobytes())
                    continue

                # 写入当前块
                pcm_i16 = (block * 32767).clip(-32768, 32767).astype(np.int16)
                wf.writeframes(pcm_i16.tobytes())

                if volume < threshold:
                    silence_counter += 1
                    if silence_counter >= silence_blocks:
                        logger.info("🔇 Silence detected. Stopping recording.")
                        break
                else:
                    silence_counter = 0

                if wf.getnframes() >= max_duration * SAMPLERATE:
                    logger.info("⏰ Max recording length reached. Forcing stop.")
                    break

    logger.success(f"💾 Saved recording to {temp_path}")
    return temp_path

# === 调用 whisper-cli 转录 (优化版) ===
def transcribe_audio(wav_path: str, delay: float = 0.0) -> str:
    model_path = os.path.expanduser("~/whisper.cpp/models/ggml-base.en.bin")
    cli_path = os.path.expanduser("~/whisper.cpp/build/bin/whisper-cli")
    cmd = [
        cli_path,
        "-m", model_path,
        "-f", wav_path,
        "-t", "4",  # 使用4个线程
        "--language", "en",  # 明确指定语言
        "--translate",  # 如果只需要英文
        "--speed-up"  # 启用加速模式(如果whisper版本支持)
    ]

    try:
        # 添加超时防止卡死
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        output = result.stdout.strip()
    except subprocess.TimeoutExpired:
        logger.error("Whisper transcription timed out")
        return ""

    # 提取识别文本行
    lines = output.splitlines()
    text_lines = [
        line.split("]", 1)[-1].strip(" -\t") for line in lines
        if "-->" in line and "]" in line
    ]
    raw_text = " ".join(text_lines).strip()

    clean_text = _clean(raw_text)
    logger.success(f"📝 Transcribed Text: {clean_text or '<EMPTY>'}")
    
    if delay:
        time.sleep(delay)
    
    # 删除临时文件
    try:
        os.unlink(wav_path)
    except:
        pass
    
    return clean_text

# === 并行识别函数 ===
def recognize(delay: float = 0.0) -> str:
    transcription_queue = queue.Queue()
    
    def record_and_transcribe():
        wav_path = record_until_silence()
        result = transcribe_audio(wav_path, delay)
        transcription_queue.put(result)
    
    # 启动录音和转录线程
    threading.Thread(target=record_and_transcribe, daemon=True).start()
    
    # 等待结果，可以在这里添加超时
    try:
        return transcription_queue.get(timeout=MAX_DURATION + 15)  # 最大录音时间+15秒缓冲
    except queue.Empty:
        logger.warning("Recognition timed out")
        return ""

# === 后台热词识别线程 (优化版) ===
def Whisper_run(callback_func):
    def loop():
        print("🟢 Whisper hotword loop started")
        while True:
            if conversation_active.is_set():
                time.sleep(0.2)
                continue

            clean_text = recognize(delay=3)
            if not clean_text:
                continue

            if "open robot system" in clean_text:
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

# === 测试入口 ===
if __name__ == "__main__":
    logger.info("🎤 Start single recognition test...")
    result = recognize()
    print("🗣️ You said:", result or "<nothing>")