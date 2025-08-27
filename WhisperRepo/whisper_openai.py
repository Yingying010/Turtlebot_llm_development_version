import os, sys
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)

import queue, threading, time, re, subprocess
import numpy as np
import sounddevice as sd
from scipy.io.wavfile import write
from loguru import logger
from typing import Final
import wave

# === 新增：OpenAI 客户端 ===
from openai import OpenAI

conversation_active: Final[threading.Event] = threading.Event()

# === 优化后的参数 ===
SAMPLERATE = 48000
BLOCKSIZE = 1024
SILENCE_THRESHOLD = 30.0  # 🔧 根据你的环境调整：你的背景噪音在24左右
SPEECH_THRESHOLD = 35.0   # 🔧 新增：语音检测阈值，需要明显高于背景噪音
SILENCE_DURATION = 2.5    # 🔧 增加到2.5秒，避免正常停顿被误判
CONFIRMATION_DURATION = 1.5  # 🔧 新增：确认阶段的等待时间
CALIBRATION_BLOCKS = 20   # 🔧 校准阶段采集的块数
MAX_DURATION = 30
FIXED_WAV_PATH = "/tmp/voice_input.wav"

# 选择 OpenAI 的 STT 模型（可改成 "gpt-4o-transcribe" 追求更高精度）
OPENAI_STT_MODEL = os.getenv("OPENAI_STT_MODEL", "gpt-4o-transcribe")

# === 清理文本 ===
def _clean(text: str) -> str:
    return re.sub(r'[^\w\s]', '', text).lower().strip()

def is_english(text: str) -> bool:
    """判断字符串是否主要由英文单词组成"""
    return bool(re.fullmatch(r"[a-zA-Z0-9\s]+", text)) and len(text.strip()) > 3

# === 标准写入 wav 文件 ===
def save_wav_standard(wav_path, audio_int16, samplerate=48000):
    with wave.open(wav_path, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)  # 16-bit PCM
        wf.setframerate(samplerate)
        wf.writeframes(audio_int16.tobytes())

# === 🔧 优化后的录音函数，增加背景噪音校准 ===
def record_until_silence(threshold=SILENCE_THRESHOLD,
                         silence_duration=SILENCE_DURATION,
                         max_duration=MAX_DURATION) -> str:
    q_local = queue.Queue()
    silence_blocks = int(silence_duration * SAMPLERATE / BLOCKSIZE)
    confirmation_blocks = int(CONFIRMATION_DURATION * SAMPLERATE / BLOCKSIZE)
    calibration_blocks = CALIBRATION_BLOCKS
    max_blocks = int(max_duration * SAMPLERATE / BLOCKSIZE)

    pre_speech_buffer = []  # 保存最近的几个块
    pre_speech_maxlen = 24
    audio_blocks = []
    silence_counter = 0
    confirmation_counter = 0
    is_recording = False
    in_confirmation = False  # 🔧 新增：确认阶段标志
    is_calibrating = True    # 🔧 新增：校准阶段标志
    calibration_counter = 0
    
    # 🔧 背景噪音校准
    background_volumes = []
    speech_threshold_dynamic = SPEECH_THRESHOLD
    silence_threshold_dynamic = SILENCE_THRESHOLD
    
    def cb(indata, frames, time_info, status):
        if status:
            logger.warning(f"⚠️ Audio status: {status}")
        q_local.put(indata.copy())

    logger.info("🔧 Calibrating background noise... Please stay quiet for 2 seconds.")

    with sd.InputStream(samplerate=SAMPLERATE, channels=1,
                        blocksize=BLOCKSIZE, callback=cb,
                        device=("pulse", None)):
        while True:
            try:
                block = q_local.get(timeout=1)
            except queue.Empty:
                continue

            volume = np.abs(block).mean() * 1000
            
            # 🔧 校准阶段：收集背景噪音数据
            if is_calibrating:
                background_volumes.append(volume)
                calibration_counter += 1
                logger.debug(f"🔧 Calibrating... Vol: {volume:.1f} ({calibration_counter}/{calibration_blocks})")
                
                if calibration_counter >= calibration_blocks:
                    # 计算背景噪音统计
                    bg_avg = np.mean(background_volumes)
                    bg_std = np.std(background_volumes)
                    bg_max = np.max(background_volumes)
                    
                    # 🔧 更保守的动态阈值设置
                    silence_threshold_dynamic = bg_avg + 3 * bg_std  # 背景噪音 + 3个标准差
                    speech_threshold_dynamic = bg_avg + 6 * bg_std   # 背景噪音 + 6个标准差
                    
                    # 确保最小阈值差异和绝对最小值
                    min_speech_gap = max(8, bg_std * 3)  # 至少8的差距，或3倍标准差
                    if speech_threshold_dynamic - silence_threshold_dynamic < min_speech_gap:
                        speech_threshold_dynamic = silence_threshold_dynamic + min_speech_gap
                    
                    # 🔧 对于低噪音环境，设置最小语音阈值
                    if speech_threshold_dynamic < bg_avg + 10:
                        speech_threshold_dynamic = bg_avg + 10
                    
                    logger.info(f"🎯 Background noise calibrated:")
                    logger.info(f"   Avg: {bg_avg:.1f}, Std: {bg_std:.1f}, Max: {bg_max:.1f}")
                    logger.info(f"   Speech threshold: {speech_threshold_dynamic:.1f}")
                    logger.info(f"   Silence threshold: {silence_threshold_dynamic:.1f}")
                    logger.info("🎙️ Waiting for speech to start...")
                    
                    is_calibrating = False
                continue

            logger.debug(f"📊 Vol: {volume:.1f}, Speech: {speech_threshold_dynamic:.1f}, Silence: {silence_threshold_dynamic:.1f}")

            pre_speech_buffer.append(block)
            if len(pre_speech_buffer) > pre_speech_maxlen:
                pre_speech_buffer.pop(0)

            if not is_recording:
                # 🔧 更严格的语音检测：需要连续几个块都超过阈值
                if volume > speech_threshold_dynamic:
                    # 检查最近3个块是否都比较高
                    recent_high = sum(1 for v in pre_speech_buffer[-3:] if np.abs(v).mean() * 1000 > speech_threshold_dynamic * 0.8)
                    if recent_high >= 2 or volume > speech_threshold_dynamic * 1.2:  # 要么连续高，要么单次很高
                        logger.info("🔴 Voice detected. Start recording...")
                        is_recording = True
                        audio_blocks.extend(pre_speech_buffer)
                        audio_blocks.append(block)
                continue

            audio_blocks.append(block)

            # 🔧 使用动态静音阈值检测结束
            current_threshold = silence_threshold_dynamic * 0.8 if in_confirmation else silence_threshold_dynamic
            
            if volume < current_threshold:
                silence_counter += 1
                
                # 🔧 第一阶段：初始静音检测
                if not in_confirmation and silence_counter >= silence_blocks:
                    logger.info("🤔 Potential end detected. Entering confirmation phase...")
                    in_confirmation = True
                    confirmation_counter = 0
                    silence_counter = 0  # 重置计数器
                
                # 🔧 第二阶段：确认阶段
                elif in_confirmation:
                    confirmation_counter += 1
                    if confirmation_counter >= confirmation_blocks:
                        logger.info("🔇 Confirmed silence. Stopping recording.")
                        break
            else:
                # 🔧 检测到声音，重置所有计数器
                silence_counter = 0
                if in_confirmation:
                    logger.info("🔊 Voice detected during confirmation. Continuing recording...")
                    in_confirmation = False
                    confirmation_counter = 0

            if len(audio_blocks) >= max_blocks:
                logger.info("⏰ Max recording length reached. Forcing stop.")
                break

    # === 保存为固定路径 wav 文件 ===
    pcm_f32 = np.concatenate(audio_blocks).flatten()
    pcm_i16 = (pcm_f32 * 32767).clip(-32768, 32767).astype(np.int16)
    save_wav_standard(FIXED_WAV_PATH, pcm_i16, SAMPLERATE)
    
    # 🔧 增加录音统计信息
    duration = len(audio_blocks) * BLOCKSIZE / SAMPLERATE
    logger.success(f"💾 Saved recording to {FIXED_WAV_PATH} (Duration: {duration:.1f}s)")
    return FIXED_WAV_PATH

# === 🔧 增强的转录函数 ===
def transcribe_audio(wav_path: str, delay: float = 0.0) -> str:
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        logger.error("❌ OPENAI_API_KEY 未设置。请先在环境变量中配置。")
        return ""

    client = OpenAI(api_key=api_key)

    try:
        with open(wav_path, "rb") as f:
            # 🔧 优化转录参数
            resp = client.audio.transcriptions.create(
                model=OPENAI_STT_MODEL,
                file=f,
                language="en",
                temperature=0.2,  # 🔧 稍微降低随机性，提高一致性
                prompt="This is a robot command or conversation. Please transcribe accurately including technical terms and robot names."  # 🔧 添加上下文提示
            )
        
        raw_text = getattr(resp, "text", None) or str(resp)
        logger.info(f"📄 Raw transcript: '{raw_text}'")

        # 🔧 改进文本处理：保留更多信息
        # 移除过度的清理，保留标点符号以便更好地理解语句结构
        processed_text = raw_text.strip() if raw_text else ""
        
        if not processed_text:
            logger.warning("⚠️ 转录结果为空")
            return ""
        
        # 🔧 基本的英文检查（放宽条件）
        if not re.search(r'[a-zA-Z]', processed_text):
            logger.warning("❌ 转录结果不包含英文字符，可能是噪音")
            return ""

        clean_text = _clean(processed_text)  # 仍然提供清理版本用于后续处理
        
        logger.success(f"📝 Transcribed Text: '{processed_text}' -> Clean: '{clean_text}'")

        if delay:
            time.sleep(delay)
        return clean_text
        
    except Exception as e:
        logger.exception(f"❌ OpenAI 转写失败: {e}")
        return ""

def recognize(delay: float = 0.0) -> str:
    wav_path = record_until_silence()
    return transcribe_audio(wav_path, delay)