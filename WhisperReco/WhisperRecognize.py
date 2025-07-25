import sys, os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

import queue, threading, time, re, subprocess
import numpy as np
import sounddevice as sd
from scipy.io.wavfile import write
from scipy.signal import butter, lfilter
from loguru import logger
from config import config
from stream_tts import tts_manager
from typing import Final
import wave

conversation_active: Final[threading.Event] = threading.Event()

# === 优化后的参数 ===
# 首先检测设备支持的采样率
try:
    default_samplerate = sd.query_devices(sd.default.device[0], 'input')['default_samplerate']
    if default_samplerate < 16000:
        SAMPLERATE = 44100  # 回退到常见采样率
    else:
        SAMPLERATE = 16000  # 优先使用16000
except:
    SAMPLERATE = 44100  # 默认回退值

BLOCKSIZE = 1024
SILENCE_THRESHOLD = 20.0
SILENCE_DURATION = 0.8
MAX_DURATION = 8
FIXED_WAV_PATH = "/dev/shm/voice_input.wav"
PRE_SPEECH_BUFFER_SIZE = 24

# 打印实际使用的采样率
logger.info(f"🔊 Using sample rate: {SAMPLERATE}Hz")

# === 音频滤波函数 ===
def butter_highpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return b, a

def highpass_filter(data, cutoff=100, fs=SAMPLERATE, order=5):
    b, a = butter_highpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

# === 清理文本 ===
def _clean(text: str) -> str:
    return re.sub(r'[^\w\s]', '', text).lower().strip()

# === 标准写入 wav 文件 ===
def save_wav_standard(wav_path, audio_int16, samplerate):
    with wave.open(wav_path, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)  # 16-bit PCM
        wf.setframerate(samplerate)
        wf.writeframes(audio_int16.tobytes())

# === 录音直到静音结束 ===
def record_until_silence(threshold=SILENCE_THRESHOLD,
                         silence_duration=SILENCE_DURATION,
                         max_duration=MAX_DURATION) -> str:
    q_local = queue.Queue()
    silence_blocks = int(silence_duration * SAMPLERATE / BLOCKSIZE)
    max_blocks = int(max_duration * SAMPLERATE / BLOCKSIZE)

    pre_speech_buffer = []
    audio_blocks = []
    silence_counter = 0
    is_recording = False

    def cb(indata, frames, time_info, status):
        if status:
            logger.warning(f"⚠️ Audio status: {status}")
        q_local.put(indata.copy())

    logger.info("🎙️ Waiting for speech to start...")

    try:
        with sd.InputStream(samplerate=SAMPLERATE, channels=1,
                          blocksize=BLOCKSIZE, callback=cb, dtype='float32'):
            while True:
                try:
                    block = q_local.get(timeout=1)
                except queue.Empty:
                    continue

                volume = np.abs(block).mean() * 1000
                logger.debug(f"📊 Vol: {volume:.1f}")

                pre_speech_buffer.append(block)
                if len(pre_speech_buffer) > PRE_SPEECH_BUFFER_SIZE:
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

    except Exception as e:
        logger.error(f"❌ Audio input error: {str(e)}")
        raise

    pcm_f32 = np.concatenate(audio_blocks).flatten()
    pcm_f32 = highpass_filter(pcm_f32)
    pcm_i16 = (pcm_f32 * 32767).clip(-32768, 32767).astype(np.int16)

    save_wav_standard(FIXED_WAV_PATH, pcm_i16, SAMPLERATE)
    logger.success(f"💾 Saved recording to {FIXED_WAV_PATH}")
    return FIXED_WAV_PATH

# === 调用 whisper-cli 转录 ===
def transcribe_audio(wav_path: str, delay: float = 0.0) -> str:
    model_path = os.path.expanduser("~/whisper.cpp/models/ggml-tiny.en.bin")
    cli_path = os.path.expanduser("~/whisper.cpp/build/bin/whisper-cli")
    
    # 如果采样率不是16000，需要添加--rate参数
    if SAMPLERATE != 16000:
        cmd = [cli_path, "-m", model_path, "-f", wav_path, "-t", "2", "--step", "500", "--length", "500", "--rate", str(SAMPLERATE)]
    else:
        cmd = [cli_path, "-m", model_path, "-f", wav_path, "-t", "2", "--step", "500", "--length", "500"]

    result = subprocess.run(cmd, capture_output=True, text=True)
    output = result.stdout.strip()

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
    return clean_text

# === 识别函数 ===
def recognize(delay: float = 0.0) -> str:
    wav_path = record_until_silence()
    return transcribe_audio(wav_path, delay)

# === 后台热词识别线程 ===
def Whisper_run(callback_func):
    def loop():
        print("🟢 Whisper hotword loop started")
        while True:
            if conversation_active.is_set():
                time.sleep(0.2)
                continue

            try:
                raw_text = recognize(delay=3)
                clean_text = _clean(raw_text)
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
                    
            except Exception as e:
                logger.error(f"❌ Recognition error: {str(e)}")
                time.sleep(1)  # 防止快速重试导致CPU占用过高

    threading.Thread(target=loop, daemon=True).start()

# === 测试入口 ===
if __name__ == "__main__":
    # 打印音频设备信息
    print("🔊 Available audio devices:")
    print(sd.query_devices())
    print(f"🔊 Default input device: {sd.default.device[0]}")
    
    logger.info("🎤 Start single recognition test...")
    result = recognize()
    print("🗣️ You said:", result or "<nothing>")