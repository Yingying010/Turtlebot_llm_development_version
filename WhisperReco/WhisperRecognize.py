import os
import sys
import queue
import threading
import time
import re
import numpy as np
import sounddevice as sd
import soundfile as sf
from loguru import logger
from typing import Final, Optional
import whisper

# === 加载 Whisper base.en 模型（会自动下载并缓存） ===
model = whisper.load_model("base.en")
conversation_active: Final[threading.Event] = threading.Event()

# === 参数优化 ===
# SAMPLERATE = 16000
BLOCKSIZE = 2048
SILENCE_THRESHOLD = 10.0
SILENCE_DURATION = 1.5
MIN_ACTIVATION_DURATION = 0.3
TEMP_WAV_PATH = "/tmp/current_audio.wav"

class WhisperRecognizer:
    def __init__(self):
        # 用于存放音频片段
        self.audio_buffer: list[np.ndarray] = []
        # 先设置一个占位，实际采样率在 start() 里覆盖
        self.samplerate: float = None  
        # 用于对外返回转录结果
        self.result_queue: queue.Queue[str] = queue.Queue()
        self.stream: Optional[sd.InputStream] = None
        self.speech_start_time = 0.0
        self.last_audio_time = 0.0
        self.is_speaking = False

        # 热词
        self.key_phrases = {
            "activate control": ["open robot system", "activate robot", "robot mode"],
            "activate chat":    ["hi assistant", "hey assistant", "start chat"],
            "shutdown":         ["ok bye", "okay bye", "goodbye", "exit"],
        }

        logger.success("🟢 Whisper 识别器初始化完成")

    def _audio_callback(self, indata, frames, time_info, status):
        if status:
            logger.warning(f"音频状态: {status}")

        self.audio_buffer.append(indata.copy())
        volume = np.abs(indata).mean() * 1000
        now = time.time()

        if volume > SILENCE_THRESHOLD:
            self.last_audio_time = now
            if not self.is_speaking:
                self.speech_start_time = now
                self.is_speaking = True
                logger.info("🔴 检测到语音")
        elif self.is_speaking and (now - self.last_audio_time > SILENCE_DURATION):
            self.is_speaking = False
            logger.info("🔇 静音检测，开始处理…")
            # 异步处理，防止阻塞回调
            threading.Thread(target=self._process_audio, daemon=True).start()

    def start(self):
        """启动音频流"""
       # 动态探测默认输入设备的采样率
        device_info = sd.query_devices(kind='input')
        self.samplerate = int(device_info['default_samplerate'])
        logger.info(f"🛠 侦测到输入设备默认采样率: {self.samplerate} Hz")

        self.stream = sd.InputStream(
            samplerate=self.samplerate,
            channels=1,
            blocksize=BLOCKSIZE,
            dtype='float32',
            callback=self._audio_callback
        )
        self.stream.start()
        logger.info("🎙️ 音频流已启动")

    def _save_audio(self) -> bool:
        if not self.audio_buffer:
            return False
        audio = np.concatenate(self.audio_buffer, axis=0)
        # 保存时使用实际的采样率，保证一致
        sf.write(TEMP_WAV_PATH, audio, self.samplerate, format='WAV', subtype='PCM_16')
        self.audio_buffer = []
        return True

    def _process_audio(self):
        if not self._save_audio():
            return

        if time.time() - self.speech_start_time < MIN_ACTIVATION_DURATION:
            logger.debug("🌓 语音太短，跳过转录")
            return

        try:
            result = model.transcribe(TEMP_WAV_PATH, language="en", fp16=False)
            transcript = result.get("text", "").strip()
        except Exception as e:
            logger.error(f"❌ 转录失败: {e}")
            return

        if transcript:
            logger.info(f"📝 转录结果: {transcript}")
            # 放入队列，供外部获取
            self.result_queue.put(transcript)
            self._check_for_keywords(transcript)

    def _check_for_keywords(self, text: str):
        t = text.lower()
        if conversation_active.is_set():
            if any(p in t for p in self.key_phrases["shutdown"]):
                logger.info("🛑 检测到关闭命令")
                self._shutdown()
            return

        if any(p in t for p in self.key_phrases["activate control"]):
            logger.info("🎮 切换到控制模式")
            conversation_active.set()
        elif any(p in t for p in self.key_phrases["activate chat"]):
            logger.info("💬 切换到聊天模式")
            conversation_active.set()
        elif any(p in t for p in self.key_phrases["shutdown"]):
            logger.info("🛑 检测到关闭命令")
            self._shutdown()

    def get_result(self, timeout: Optional[float] = None) -> str:
        """
        从内部队列获取下一条转录结果。
        :param timeout: 阻塞等待时间（秒），None 则无限等待
        :raises queue.Empty: 如果超时未得到结果
        """
        return self.result_queue.get(timeout=timeout)

    def _shutdown(self):
        if self.stream:
            self.stream.stop()
            self.stream.close()
        logger.success("✅ 系统已安全关闭")
        sys.exit(0)


# === 模块测试或示例用法 ===
if __name__ == "__main__":
    logger.info("🚀 启动 Whisper 语音识别模块")
    recognizer = WhisperRecognizer()
    recognizer.start()
    logger.info("🎧 系统就绪，等待语音输入…")

    try:
        while True:
            # 阻塞等待下一条转录文本，最多等 5 秒
            try:
                text = recognizer.get_result(timeout=5.0)
                print(f"主程序拿到转录: {text}")
                # 这里可以进一步处理 text，比如发给聊天机器人等
            except queue.Empty:
                # 5 秒无输入，继续等待
                continue

    except KeyboardInterrupt:
        logger.info("🛑 用户中断，系统关闭")
        recognizer._shutdown()
