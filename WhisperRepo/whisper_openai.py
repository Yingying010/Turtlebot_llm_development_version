import os, sys
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)

import queue, threading, time, re, subprocess
import numpy as np
import sounddevice as sd
from loguru import logger
from typing import Final
import wave
from openai import OpenAI

# 初始化OpenAI客户端
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

conversation_active: Final[threading.Event] = threading.Event()

# === 参数 ===
SAMPLERATE = 48000
BLOCKSIZE = 1024
SILENCE_THRESHOLD = 20.0
SILENCE_DURATION  = 1.5
MAX_DURATION      = 30
FIXED_WAV_PATH    = "/tmp/voice_input.wav"

# === 音频设备检测和配置 ===
def list_audio_devices():
    """列出可用的音频设备"""
    logger.info("🔍 Available audio devices:")
    devices = sd.query_devices()
    for i, device in enumerate(devices):
        device_type = "🎤" if device['max_input_channels'] > 0 else "🔊"
        if device['max_input_channels'] > 0:  # 只显示输入设备
            logger.info(f"  {device_type} {i}: {device['name']} (channels: {device['max_input_channels']})")
    return devices

def get_default_input_device():
    """获取默认输入设备"""
    try:
        default_device = sd.query_devices(kind='input')
        logger.info(f"🎤 Using default input device: {default_device['name']}")
        return None  # None表示使用默认设备
    except Exception as e:
        logger.warning(f"⚠️ Could not get default input device: {e}")
        return None

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
                         max_duration=MAX_DURATION,
                         device_id=None) -> str:
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
                device=device_id):  # 使用指定的设备ID或默认设备
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

# === 使用OpenAI API转录音频 ===
def transcribe_audio(wav_path: str, delay: float = 0.0) -> str:
    """使用OpenAI Whisper API转录音频文件"""
    
    if not client.api_key:
        logger.error("❌ OpenAI API key not found. Please set OPENAI_API_KEY environment variable.")
        return ""
    
    try:
        logger.info("🔄 Transcribing with OpenAI API...")
        
        with open(wav_path, "rb") as audio_file:
            # 使用新版OpenAI库的API
            transcript = client.audio.transcriptions.create(
                model="whisper-1",
                file=audio_file,
                language="en",  # 根据需要修改语言，可以设为None让API自动检测
                response_format="text",
                temperature=0.2,  # 较低的temperature获得更稳定的结果
                prompt=""  # 可以添加上下文提示来提高准确性
            )
        
        # 新版API直接返回文本字符串
        raw_text = transcript.strip()
        
        # === 删除标点符号（小写、去空格）===
        clean_text = _clean(raw_text)
        
        logger.success(f"📝 Raw Text: {raw_text}")
        logger.success(f"🧹 Clean Text: {clean_text or '<EMPTY>'}")
        
        if delay:
            time.sleep(delay)
            
        return clean_text
        
    except Exception as e:
        # 处理各种可能的错误
        error_msg = str(e)
        if "rate_limit" in error_msg.lower():
            logger.error("❌ OpenAI rate limit exceeded. Please wait and try again.")
        elif "invalid" in error_msg.lower():
            logger.error(f"❌ Invalid request: {e}")
        elif "api" in error_msg.lower():
            logger.error(f"❌ OpenAI API error: {e}")
        elif "file" in error_msg.lower():
            logger.error(f"❌ Audio file error: {wav_path} - {e}")
        else:
            logger.error(f"❌ Transcription failed: {e}")
        return ""

def recognize(delay: float = 0.0, device_id=None) -> str:
    """完整的语音识别流程：录音 -> 转录"""
    wav_path = record_until_silence(device_id=device_id)
    return transcribe_audio(wav_path, delay)

# === 测试函数 ===
def test_recognition():
    """测试语音识别功能"""
    logger.info("🚀 Starting voice recognition test...")
    
    if not client.api_key:
        logger.error("Please set your OpenAI API key:")
        logger.info("export OPENAI_API_KEY='your-api-key-here'")
        return
    
    # 列出可用的音频设备
    list_audio_devices()
    
    # 获取默认输入设备
    device_id = get_default_input_device()
    
    try:
        while True:
            logger.info("🎤 Say something... (Ctrl+C to quit)")
            text = recognize(device_id=device_id)
            
            if text:
                logger.success(f"✅ Recognized: '{text}'")
            else:
                logger.warning("⚠️ No speech detected or transcription failed")
                
    except KeyboardInterrupt:
        logger.info("👋 Voice recognition stopped by user")
    except Exception as e:
        logger.error(f"❌ Error during recognition: {e}")
        logger.info("💡 Try running with sudo if you get permission errors")
        logger.info("💡 Make sure your microphone permissions are enabled in System Preferences")

if __name__ == "__main__":
    test_recognition()