# stream_tts.py
import os
import platform
import threading
from loguru import logger
import subprocess
import time

class TTSManager:
    def __init__(self, voice="en-us+f3", speed=140, pitch=60):
        self.voice = voice
        self.speed = speed
        self.pitch = pitch
        self._current_process = None
        self._lock = threading.Lock()
        self._playing = threading.Event()
        self._thread = None

    def _build_command(self, text: str, wav_path: str):
        return [
            "espeak-ng", "-v", self.voice,
            "-s", str(self.speed), "-p", str(self.pitch),
            text, "-w", wav_path
        ]

    def say(self, text: str):
        self.stop()  # 终止上一次播放

        def _speak():
            try:
                logger.info(f"🔊 Speaking: {text}")
                with self._lock:
                    wav_path = "/tmp/tts_output.wav"
                    subprocess.run(self._build_command(text, wav_path), check=True)

                    self._playing.set()
                    self._current_process = subprocess.Popen(["aplay", "-D", "plughw:2,0", wav_path])
                
                self._current_process.wait()
            except Exception as e:
                logger.error(f"TTS error: {e}")
            finally:
                self._playing.clear()
                with self._lock:
                    self._current_process = None

        self._thread = threading.Thread(target=_speak, daemon=True)
        self._thread.start()

    def stop(self):
        with self._lock:
            if self._current_process and self._current_process.poll() is None:
                logger.info("🛑 Stopping TTS playback...")
                self._current_process.terminate()
                self._current_process = None
        self._playing.clear()

    def wait_until_done(self):
        """等待播放完成（非阻塞主线程调用者，只等待播放线程结束）"""
        self._playing.wait()  # 等待播放开始
        while self._playing.is_set():
            time.sleep(0.1)

# ✅ 创建全局单例
tts_manager = TTSManager()
