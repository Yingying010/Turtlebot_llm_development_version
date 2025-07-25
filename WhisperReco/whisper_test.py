import sys
import os
import queue
import threading
import time
import wave
import numpy as np
import sounddevice as sd
from scipy.io.wavfile import write
from loguru import logger
import subprocess

# ===== 配置参数 =====
SAMPLE_RATE = 48000       # 采样率 (Hz)
BLOCK_SIZE = 1024         # 音频块大小
SILENCE_THRESHOLD = 10.0  # 静音检测阈值 (0-100)
SILENCE_DURATION = 2.0    # 静音持续时间 (秒)
MAX_DURATION = 15.0       # 最大录音时长 (秒)
GAIN = 2.0                # 音频增益
MODEL_PATH = os.path.expanduser("~/whisper.cpp/models/ggml-base.en.bin")
CLI_PATH = os.path.expanduser("~/whisper.cpp/build/bin/whisper-cli")

# ===== 音频处理函数 =====
def save_wav(filepath, audio_data, samplerate):
    """保存WAV文件"""
    with wave.open(filepath, 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)  # 16-bit
        wf.setframerate(samplerate)
        wf.writeframes(audio_data.tobytes())

def process_audio_block(block, gain):
    """处理音频块"""
    block = block * gain  # 应用增益
    return np.clip(block, -1.0, 1.0)  # 限幅防止削波

# ===== 录音功能 =====
def record_audio():
    """录音直到检测到静音"""
    q = queue.Queue()
    audio_blocks = []
    silence_blocks = int(SILENCE_DURATION * SAMPLE_RATE / BLOCK_SIZE)
    silence_counter = 0
    is_recording = False
    
    def callback(indata, frames, time_info, status):
        if status:
            logger.warning(f"Audio status: {status}")
        block = process_audio_block(indata.copy(), GAIN)
        volume = np.abs(block).mean() * 1000
        q.put((block, volume))
    
    logger.info("🎤 准备录音... (说话开始)")
    
    try:
        with sd.InputStream(samplerate=SAMPLE_RATE, channels=1,
                          blocksize=BLOCK_SIZE, callback=callback):
            start_time = time.time()
            
            while True:
                # 超时检查
                if time.time() - start_time > MAX_DURATION:
                    logger.warning("⏰ 达到最大录音时长")
                    break
                
                # 获取音频块
                try:
                    block, volume = q.get(timeout=1)
                except queue.Empty:
                    continue
                
                logger.debug(f"音量: {volume:.1f}")
                
                # 语音活动检测
                if not is_recording:
                    if volume > SILENCE_THRESHOLD:
                        logger.info("🔴 检测到语音，开始录音...")
                        is_recording = True
                        audio_blocks.append(block)
                else:
                    audio_blocks.append(block)
                    if volume < SILENCE_THRESHOLD:
                        silence_counter += 1
                        if silence_counter >= silence_blocks:
                            logger.info("🔇 检测到静音，停止录音")
                            break
                    else:
                        silence_counter = 0
    
    except KeyboardInterrupt:
        logger.info("录音被用户中断")
        return None
    
    if not audio_blocks:
        logger.error("没有录到音频")
        return None
    
    # 合并音频数据
    audio_data = np.concatenate(audio_blocks)
    audio_data = (audio_data * 32767).astype(np.int16)  # 转换为16位PCM
    
    # 保存临时文件
    temp_file = "test_recording.wav"
    save_wav(temp_file, audio_data, SAMPLE_RATE)
    logger.success(f"💾 录音已保存: {temp_file}")
    
    return temp_file

# ===== 转录功能 =====
def transcribe_audio(wav_path):
    """使用Whisper转录音频"""
    if not os.path.exists(wav_path):
        logger.error(f"文件不存在: {wav_path}")
        return ""
    
    if not os.path.exists(MODEL_PATH):
        logger.error(f"模型文件不存在: {MODEL_PATH}")
        return ""
    
    if not os.path.exists(CLI_PATH):
        logger.error(f"Whisper CLI不存在: {CLI_PATH}")
        return ""
    
    cmd = [
        CLI_PATH,
        "-m", MODEL_PATH,
        "-f", wav_path,
        "-t", "4",        # 使用4个线程
        "--language", "en"
    ]
    
    logger.info("🔄 开始转录...")
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        output = result.stdout.strip()
        
        # 提取识别文本
        lines = output.splitlines()
        text_lines = [
            line.split("]", 1)[-1].strip(" -\t") for line in lines
            if "-->" in line and "]" in line
        ]
        transcription = " ".join(text_lines).strip()
        
        if transcription:
            logger.success(f"转录结果: {transcription}")
        else:
            logger.warning("转录返回空结果")
        
        return transcription
    
    except subprocess.TimeoutExpired:
        logger.error("转录超时")
    except Exception as e:
        logger.error(f"转录失败: {e}")
    
    return ""

# ===== 主测试函数 =====
def run_test():
    """运行完整测试流程"""
    # 1. 录音
    audio_file = record_audio()
    if not audio_file:
        return
    
    # 2. 转录
    transcription = transcribe_audio(audio_file)
    
    # 3. 显示结果
    print("\n=== 测试结果 ===")
    print(f"音频文件: {audio_file}")
    print(f"转录内容: {transcription or '(空)'}")
    
    # 4. 清理临时文件
    try:
        os.remove(audio_file)
    except:
        pass

# ===== 主程序 =====
if __name__ == "__main__":
    # 配置日志
    logger.remove()
    logger.add(sys.stdout, 
              format="<green>{time:YYYY-MM-DD HH:mm:ss}</green> | <level>{level: <8}</level> | <level>{message}</level>",
              level="INFO")
    
    print("=== 语音录音和转录测试 ===")
    print(f"采样率: {SAMPLE_RATE}Hz | 静音阈值: {SILENCE_THRESHOLD} | 最大时长: {MAX_DURATION}秒")
    print("按Ctrl+C可随时中断测试\n")
    
    try:
        run_test()
    except Exception as e:
        logger.error(f"测试出错: {e}")
    finally:
        print("\n测试结束")