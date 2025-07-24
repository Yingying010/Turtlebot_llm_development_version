# WhisperReco/whisper_run.py

import queue
import threading
from loguru import logger
from typing import Optional

from .WhisperRecognize import WhisperRecognizer, conversation_active

# 单例识别器对象
_recognizer: Optional[WhisperRecognizer] = None

def Whisper_run(hwcallback):
    """
    启动 Whisper 识别器并注册唤醒回调（与 Vosk_run 接口一致）。
    这里假设 hwcallback() 会在热词检测到时被调用，
    你可以在这个函数里启动你的热词检测逻辑，触发 hwcallback() 后主程序才会调用 recognize()。
    """
    global _recognizer

    if _recognizer is None:
        _recognizer = WhisperRecognizer()
        _recognizer.start()
        logger.info("✅ Whisper_run: 识别器已启动")
    else:
        logger.warning("⚠️ Whisper_run 已经启动，无需重复调用")

    # 如果你还需要在这里启动热词检测线程，请在此处调用，例如：
    # threading.Thread(target=你的热词检测函数, args=(hwcallback,), daemon=True).start()


def recognize(delay: float = 3.0) -> str:
    """
    等待并返回下一条识别结果（阻塞）。
    :param delay: 最大等待时间（秒），超时后返回空字符串。
    """
    if _recognizer is None:
        raise RuntimeError("请先调用 Whisper_run() 启动识别器")

    try:
        # 从内部队列获取一条转录文本
        text = _recognizer.get_result(timeout=delay)
        return text
    except queue.Empty:
        logger.debug(f"recognize: {delay}s 内无识别结果，返回空字符串")
        return ""
