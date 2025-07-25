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

# === 基于设备信息的优化参数 ===
SAMPLERATE = 48000  # 设备支持48000Hz
BLOCKSIZE = 1024
SILENCE_THRESHOLD = 25.0  # 适当提高阈值
SILENCE_DURATION = 0.7    # 缩短静音检测时间
MAX_DURATION = 6          # 缩短最大录音时长
FIXED_WAV_PATH = "/dev/shm/voice_input.wav"  # 使用内存文件系统
PRE_SPEECH_BUFFER_SIZE = 24  # 预录音缓冲区大小

logger.info(f"🔊 Using sample rate: {SAMPLERATE}Hz (device supported rate)")

# === 音频滤波函数 ===
def butter_highpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return b, a

def highpass_filter(data, cutoff=150, fs=SAMPLERATE, order=5):
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
        # 明确指定设备参数
        with sd.InputStream(
            samplerate=SAMPLERATE, 
            channels=1,
            blocksize=BLOCKSIZE, 
            callback=cb, 
            dtype='float32',
            device=sd.default.device[0]  # 明确使用默认输入设备
        ):
            start_time = time.time()
            while True:
                try:
                    block = q_local.get(timeout=1)
                except queue.Empty:
                    # 检查是否超时
                    if time.time() - start_time > 30:
                        logger.warning("🕒 No audio input for 30 seconds. Restarting...")
                        return None
                    continue

                volume = np.abs(block).mean() * 1000
                logger.debug(f"📊 Vol: {volume:.1f}")

                # 维护预录音缓冲区
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
        # 尝试重新初始化音频设备
        sd._terminate()
        sd._initialize()
        return None

    if not audio_blocks:
        logger.warning("⚠️ No audio recorded. Returning empty.")
        return None

    pcm_f32 = np.concatenate(audio_blocks).flatten()
    pcm_f32 = highpass_filter(pcm_f32, cutoff=150)  # 使用更高的截止频率
    pcm_i16 = (pcm_f32 * 32767).clip(-32768, 32767).astype(np.int16)

    save_wav_standard(FIXED_WAV_PATH, pcm_i16, SAMPLERATE)
    logger.success(f"💾 Saved recording to {FIXED_WAV_PATH}")
    return FIXED_WAV_PATH

# === 调用 whisper-cli 转录 ===
def transcribe_audio(wav_path: str, delay: float = 0.0) -> str:
    model_path = os.path.expanduser("~/whisper.cpp/models/ggml-tiny.en.bin")
    cli_path = os.path.expanduser("~/whisper.cpp/build/bin/whisper-cli")
    
    # 添加 --rate 参数指定采样率
    cmd = [
        cli_path, 
        "-m", model_path, 
        "-f", wav_path, 
        "-t", "2", 
        "--step", "500", 
        "--length", "500",
        "--rate", str(SAMPLERATE)  # 明确指定采样率
    ]

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        output = result.stdout.strip()
    except subprocess.TimeoutExpired:
        logger.error("⌛ Transcription timed out after 15 seconds")
        return ""
    except Exception as e:
        logger.error(f"❌ Transcription failed: {str(e)}")
        return ""

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
    if not wav_path:
        return ""
    return transcribe_audio(wav_path, delay)

# === 后台热词识别线程 ===
def Whisper_run(callback_func):
    def loop():
        print("🟢 Whisper hotword loop started")
        error_count = 0
        
        while True:
            if conversation_active.is_set():
                time.sleep(0.2)
                continue

            try:
                raw_text = recognize(delay=3)
                clean_text = _clean(raw_text)
                
                if not clean_text:
                    error_count = 0  # 重置错误计数
                    continue
                
                error_count = 0  # 成功识别后重置错误计数
                
                # 热词检测
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
                error_count += 1
                logger.error(f"❌ Recognition error ({error_count}): {str(e)}")
                
                if error_count > 5:
                    logger.critical("🛑 Too many consecutive errors. Restarting audio system...")
                    sd._terminate()
                    sd._initialize()
                    error_count = 0
                
                time.sleep(1)  # 错误后等待

    threading.Thread(target=loop, daemon=True).start()

# === 测试入口 ===
if __name__ == "__main__":
    # 打印音频设备信息
    print("🔊 Audio device information:")
    print(sd.query_devices())
    print(f"🔊 Default input device: {sd.default.device[0]}")
    
    logger.info("🎤 Starting audio system test...")
    
    # 测试录音
    logger.info("🔊 Testing audio recording...")
    wav_path = record_until_silence()
    if wav_path:
        logger.success(f"✅ Recording test successful: {wav_path}")
        
        # 测试转录
        logger.info("🔊 Testing transcription...")
        result = transcribe_audio(wav_path)
        print("🗣️ You said:", result or "<nothing>")
    else:
        logger.error("❌ Recording test failed")