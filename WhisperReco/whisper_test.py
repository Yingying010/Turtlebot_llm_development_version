import numpy as np
import sounddevice as sd
import scipy.signal
from scipy.io.wavfile import write
import matplotlib.pyplot as plt
from loguru import logger
import queue
import time
import os
import sys

# 测试参数配置
TEST_SAMPLERATE = 48000  # 保持48000Hz
TEST_BLOCKSIZE = 1024
TEST_THRESHOLD = 10.0    # 初始阈值
TEST_SILENCE_DURATION = 2.0
TEST_GAIN = 2.0          # 初始增益值
TEST_DURATION = 10       # 最大测试时长(秒)

# 创建测试目录
os.makedirs("test_recordings", exist_ok=True)

def visualize_audio(audio_data, title="Audio Waveform"):
    """可视化音频波形"""
    plt.figure(figsize=(12, 4))
    plt.plot(audio_data)
    plt.title(title)
    plt.xlabel("Samples")
    plt.ylabel("Amplitude")
    plt.grid()
    plt.show()

def audio_preprocess(block, samplerate, gain=1.0):
    """音频预处理流水线"""
    # 高通滤波 (去除低频噪声)
    b, a = scipy.signal.butter(4, 100/(samplerate/2), 'highpass')
    processed = scipy.signal.filtfilt(b, a, block.flatten()).reshape(-1,1)
    
    # 增益控制
    processed = processed * gain
    
    # 限幅防止削波
    processed = np.clip(processed, -1.0, 1.0)
    
    return processed

def test_recording(threshold, gain, silence_duration):
    """执行一次测试录音"""
    q = queue.Queue()
    audio_blocks = []
    silence_blocks = int(silence_duration * TEST_SAMPLERATE / TEST_BLOCKSIZE)
    silence_counter = 0
    is_recording = False
    start_time = time.time()
    
    def callback(indata, frames, time_info, status):
        nonlocal is_recording, silence_counter
        if status:
            logger.warning(f"Audio status: {status}")
        
        # 原始音频数据
        raw_block = indata.copy()
        raw_volume = np.abs(raw_block).mean() * 1000
        
        # 处理后的音频数据
        processed_block = audio_preprocess(raw_block, TEST_SAMPLERATE, gain)
        processed_volume = np.abs(processed_block).mean() * 1000
        
        q.put((raw_block, processed_block, raw_volume, processed_volume))
    
    logger.info(f"🎤 Starting test with threshold={threshold}, gain={gain}, silence_duration={silence_duration}")
    
    try:
        with sd.InputStream(samplerate=TEST_SAMPLERATE, channels=1,
                           blocksize=TEST_BLOCKSIZE, callback=callback):
            logger.info("Recording started... Speak now!")
            
            while True:
                # 超时检查
                if time.time() - start_time > TEST_DURATION:
                    logger.warning("Test duration reached")
                    break
                
                # 获取音频块
                try:
                    raw_block, processed_block, raw_vol, proc_vol = q.get(timeout=1)
                except queue.Empty:
                    continue
                
                logger.debug(f"Raw vol: {raw_vol:.1f} | Processed vol: {proc_vol:.1f}")
                
                # 语音活动检测
                if not is_recording:
                    if proc_vol > threshold:
                        logger.info("🔴 Voice detected! Start recording...")
                        is_recording = True
                        audio_blocks.append(processed_block)
                else:
                    audio_blocks.append(processed_block)
                    if proc_vol < threshold:
                        silence_counter += 1
                        if silence_counter >= silence_blocks:
                            logger.info("🔇 Silence detected. Stopping recording.")
                            break
                    else:
                        silence_counter = 0
    
    except KeyboardInterrupt:
        logger.info("Test interrupted by user")
    
    # 保存录音
    if audio_blocks:
        audio_data = np.concatenate(audio_blocks)
        filename = f"test_recordings/test_{time.strftime('%Y%m%d_%H%M%S')}.wav"
        write(filename, TEST_SAMPLERATE, (audio_data * 32767).astype(np.int16))
        logger.success(f"Saved test recording: {filename}")
        
        # 可视化
        visualize_audio(audio_data, "Processed Audio Waveform")
        
        return filename
    else:
        logger.error("No audio was recorded!")
        return None

def interactive_test():
    """交互式测试工具"""
    print("\n" + "="*50)
    print("麦克风录音测试工具")
    print("="*50)
    
    # 显示音频设备信息
    print("\n可用音频设备:")
    print(sd.query_devices())
    
    current_threshold = TEST_THRESHOLD
    current_gain = TEST_GAIN
    current_silence = TEST_SILENCE_DURATION
    
    while True:
        print("\n当前参数:")
        print(f"1. 音量阈值: {current_threshold} (建议范围5-20)")
        print(f"2. 增益值: {current_gain} (建议范围1.0-3.0)")
        print(f"3. 静音持续时间: {current_silence}秒 (建议1.0-3.0)")
        print("4. 开始测试")
        print("5. 退出")
        
        choice = input("请选择操作[1-5]: ").strip()
        
        if choice == "1":
            try:
                new_thresh = float(input("输入新的音量阈值: "))
                if 0 < new_thresh < 50:
                    current_threshold = new_thresh
                else:
                    print("阈值必须在0-50之间")
            except ValueError:
                print("请输入有效数字")
        
        elif choice == "2":
            try:
                new_gain = float(input("输入新的增益值: "))
                if 0.5 <= new_gain <= 5.0:
                    current_gain = new_gain
                else:
                    print("增益必须在0.5-5.0之间")
            except ValueError:
                print("请输入有效数字")
        
        elif choice == "3":
            try:
                new_silence = float(input("输入静音持续时间(秒): "))
                if 0.5 <= new_silence <= 5.0:
                    current_silence = new_silence
                else:
                    print("持续时间必须在0.5-5.0秒之间")
            except ValueError:
                print("请输入有效数字")
        
        elif choice == "4":
            print("\n开始测试... (说话吧！)")
            test_recording(current_threshold, current_gain, current_silence)
        
        elif choice == "5":
            print("退出测试工具")
            break
        
        else:
            print("无效选择，请重新输入")

if __name__ == "__main__":
    # 配置日志
    logger.remove()
    logger.add(sys.stdout, format="<green>{time:YYYY-MM-DD HH:mm:ss}</green> | <level>{level: <8}</level> | <level>{message}</level>")
    
    # 运行交互式测试
    interactive_test()