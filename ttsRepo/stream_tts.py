# stream_tts.py（只使用 gTTS）
import os
import threading
import tempfile
import platform
import subprocess
from typing import Optional, List
from loguru import logger
import shutil
from gtts import gTTS

def _split_text(text: str, max_len: int = 180) -> List[str]:
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
    def __init__(self):
        self._done = threading.Event()
        self._canceled = threading.Event()

    def wait(self, timeout: Optional[float] = None) -> bool:
        return self._done.wait(timeout)

    def cancel(self):
        self._canceled.set()

    def _mark_done(self):
        self._done.set()

    def _is_canceled(self) -> bool:
        return self._canceled.is_set()

class TTSManager:
    def __init__(self, lang: str = "en", tld: str = "com"):
        self.lang = lang
        self.tld = tld

        self._lock = threading.RLock()
        self._current_process: Optional[subprocess.Popen] = None
        self._playing = False
        self._current_handle: Optional[SpeechHandle] = None

        system = platform.system()
        if system == "Darwin":
            self._player = shutil.which("afplay")
        elif system == "Linux":
            self._player = shutil.which("mpg123") or shutil.which("ffplay")
        else:
            raise NotImplementedError("Only macOS/Linux supported.")

        if not self._player:
            logger.warning("No mp3 player found. Please install mpg123 or ffplay.")

    def say(self, text: str, *, block: bool = False, interrupt: bool = True) -> SpeechHandle:
        handle = SpeechHandle()
        if not text.strip():
            handle._mark_done()
            return handle

        if interrupt:
            self.stop()

        t = threading.Thread(target=self._speak_worker, args=(text, handle), daemon=True)
        t.start()
        if block:
            handle.wait()
        return handle

    def say_sync(self, text: str):
        self.say(text, block=True)

    def is_playing(self) -> bool:
        with self._lock:
            return self._playing

    def stop(self):
        with self._lock:
            if self._current_handle:
                self._current_handle.cancel()
            if self._current_process and self._current_process.poll() is None:
                try:
                    self._current_process.terminate()
                except Exception:
                    pass
            self._current_process = None
            self._playing = False

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
                    break

                mp3_path = None
                try:
                    # 合成 mp3
                    fd, mp3_path = tempfile.mkstemp(suffix=".mp3")
                    os.close(fd)
                    gTTS(part, lang=self.lang, tld=self.tld).save(mp3_path)

                    if handle._is_canceled():
                        break

                    # 播放 mp3
                    play_cmd = [self._player, mp3_path] if "mpg123" in self._player else [self._player, "-nodisp", "-autoexit", mp3_path]
                    logger.info(f"🔊 {part}")
                    with self._lock:
                        self._current_process = subprocess.Popen(play_cmd)
                    self._current_process.wait()

                finally:
                    if mp3_path and os.path.exists(mp3_path):
                        try:
                            os.remove(mp3_path)
                        except Exception:
                            pass

        except Exception as e:
            logger.error(f"TTS error: {e}")

        finally:
            with self._lock:
                self._playing = False
                self._current_process = None
                self._current_handle = None
            handle._mark_done()

# ✅ 创建全局单例：使用美式英语
tts_manager = TTSManager(lang="en", tld="com")
