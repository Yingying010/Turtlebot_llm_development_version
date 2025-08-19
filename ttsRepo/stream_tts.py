# stream_tts.py
import os
import platform
import threading
import subprocess
import shutil
import tempfile
from typing import Optional, List
from loguru import logger


def _which(cmd: str) -> Optional[str]:
    return shutil.which(cmd)


def _split_text(text: str, max_len: int = 180) -> List[str]:
    """
    将长文本切片，优先按句号/逗号/空格切，避免过长一口气合成导致卡顿或丢音。
    """
    text = (text or "").strip()
    if len(text) <= max_len:
        return [text] if text else []
    parts, buf = [], []
    length = 0
    for ch in text:
        buf.append(ch)
        length += 1
        if length >= max_len and ch in ".。！？!?,，;； ":
            parts.append("".join(buf).strip())
            buf, length = [], 0
    if buf:
        parts.append("".join(buf).strip())
    return [p for p in parts if p]


class SpeechHandle:
    """一个可等待的句柄，用于等待播报完成或取消。"""
    def __init__(self):
        self._done = threading.Event()
        self._canceled = threading.Event()

    def wait(self, timeout: Optional[float] = None) -> bool:
        """阻塞直到播报完成或超时。返回是否完成（未超时）。"""
        return self._done.wait(timeout)

    def cancel(self):
        self._canceled.set()

    def _mark_done(self):
        self._done.set()

    def _is_canceled(self) -> bool:
        return self._canceled.is_set()


class TTSManager:
    def __init__(self, voice: str = "en-us+f2", speed: int = 160, pitch: int = 50):
        self.voice = voice
        self.speed = speed
        self.pitch = pitch

        self._lock = threading.RLock()
        self._current_process: Optional[subprocess.Popen] = None
        self._playing = False
        self._current_handle: Optional[SpeechHandle] = None

        # 选择播放器
        system = platform.system()
        if system == "Darwin":
            self._player = _which("afplay")
        elif system == "Linux":
            self._player = _which("paplay") or _which("aplay")
        else:
            raise NotImplementedError("This TTS manager only supports macOS and Linux (Raspberry Pi).")

        if not _which("espeak-ng"):
            logger.warning("⚠️ 'espeak-ng' not found in PATH. Please install it: sudo apt install espeak-ng")

        if not self._player:
            logger.warning("⚠️ No suitable audio player found (afplay/paplay/aplay). "
                           "Install one to enable playback.")

    # -------------------- 对外接口 --------------------

    def say(self, text: str, *, block: bool = False, interrupt: bool = True) -> SpeechHandle:
        """
        播报一段文本。
        - block=True：阻塞直到播放结束
        - interrupt=True：开始前先打断上一次播放
        返回 SpeechHandle，可手动 wait() 或 cancel()。
        """
        handle = SpeechHandle()
        if not text or not text.strip():
            handle._mark_done()
            return handle

        if interrupt:
            self.stop()

        t = threading.Thread(target=self._speak_worker, args=(text, handle), daemon=True)
        t.start()
        if block:
            handle.wait()
        return handle

    def say_sync(self, text: str) -> None:
        """同步播报（阻塞直到播放完成）。"""
        self.say(text, block=True)

    def is_playing(self) -> bool:
        with self._lock:
            return self._playing

    def stop(self) -> None:
        """中断当前播放。"""
        with self._lock:
            if self._current_handle:
                self._current_handle.cancel()

            if self._current_process and self._current_process.poll() is None:
                try:
                    logger.info("🛑 Stopping TTS playback...")
                    self._current_process.terminate()
                except Exception as e:
                    logger.error(f"Terminate player failed: {e}")
            self._current_process = None
            self._playing = False

    # -------------------- 内部实现 --------------------

    def _speak_worker(self, text: str, handle: SpeechHandle):
        chunks = _split_text(text)
        if not chunks:
            handle._mark_done()
            return

        with self._lock:
            self._playing = True
            self._current_handle = handle

        try:
            for part in chunks:
                if handle._is_canceled():
                    # logger.info("🔇 TTS canceled before synthesis.")
                    break

                wav_path = None
                try:
                    # 1) 合成到临时 wav
                    with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as tmpf:
                        wav_path = tmpf.name

                    es_cmd = [
                        "espeak-ng",
                        "-v", self.voice,
                        "-s", str(self.speed),
                        "-p", str(self.pitch),
                        part,
                        "-w", wav_path
                    ]
                    synth = subprocess.run(es_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    if synth.returncode != 0:
                        logger.error(f"espeak-ng failed: {synth.stderr.decode(errors='ignore')}")
                        break

                    if handle._is_canceled():
                        # logger.info("🔇 TTS canceled after synthesis, before playback.")
                        break

                    # 2) 播放（阻塞到结束）
                    if not self._player:
                        logger.error("No audio player available (afplay/paplay/aplay).")
                        break

                    play_cmd = [self._player, wav_path]
                    logger.info(f"🔊 {part}")
                    with self._lock:
                        # 双重检查取消/竞争
                        if handle._is_canceled():
                            break
                        self._current_process = subprocess.Popen(play_cmd)

                    self._current_process.wait()
                    with self._lock:
                        self._current_process = None

                finally:
                    if wav_path and os.path.exists(wav_path):
                        try:
                            os.remove(wav_path)
                        except Exception:
                            pass

                if handle._is_canceled():
                    # logger.info("🔇 TTS canceled during playback.")
                    break

        except Exception as e:
            logger.error(f"TTS error: {e}")

        finally:
            with self._lock:
                self._playing = False
                self._current_process = None
                self._current_handle = None
            handle._mark_done()


# ✅ 创建全局单例
tts_manager = TTSManager()
