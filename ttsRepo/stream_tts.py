# stream_tts.py
import os
import platform
import threading
from loguru import logger
import subprocess


class TTSManager:
    def __init__(self, voice="en-us+f2", speed=160, pitch=50):
        self.voice = voice
        self.speed = speed
        self.pitch = pitch
        self._current_process = None
        self._lock = threading.Lock()

    def _build_command(self, text: str):
        if platform.system() == "Darwin":  # macOS
            return ["espeak-ng", "-v", self.voice, "-s", str(self.speed), "-p", str(self.pitch), text]
        elif platform.system() == "Linux":
            return ["espeak-ng", "-v", self.voice, "-s", str(self.speed), "-p", str(self.pitch), text]
        else:
            raise NotImplementedError("This TTS manager only supports macOS and Linux (Raspberry Pi)")

    def say(self, text: str):
        self.stop()  # 停止前一次任务

        def _speak():
            try:
                logger.info(f"🔊 Speaking: {text}")
                with self._lock:
                    wav_path = "/tmp/tts_output.wav"
                    subprocess.run([
                        "espeak-ng", "-v", self.voice,
                        "-s", str(self.speed), "-p", str(self.pitch),
                        text, "-w", wav_path
                    ])
                    self._current_process = subprocess.Popen(["paplay", wav_path])
                    self._current_process.wait()
            except Exception as e:
                logger.error(f"TTS error: {e}")

        threading.Thread(target=_speak).start()

    def stop(self):
        with self._lock:
            if self._current_process and self._current_process.poll() is None:
                logger.info("🛑 Stopping TTS playback...")
                self._current_process.terminate()
                self._current_process = None

# ✅ 创建全局单例
tts_manager = TTSManager()
