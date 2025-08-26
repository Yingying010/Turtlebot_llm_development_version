# === 新增：全局单例 ROS 发布器（只发布，不 spin） ===
import os, sys
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)

import queue, threading, time, re, subprocess
import numpy as np
import sounddevice as sd
from loguru import logger
import wave
import threading
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# === 全局单例 ROS 发布器（只发布，不 spin） ===
import threading
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

_ros_lock = threading.Lock()
_ros_node: Node | None = None
_ros_pub = None
_ros_owned_init = False   # ← 我们是否自己调用了 rclpy.init()

def _ensure_ros_publisher():
    """只创建 publisher，不启动 spin（发布不需要 spin）。"""
    global _ros_node, _ros_pub, _ros_owned_init
    with _ros_lock:
        try:
            rclpy.init()
            _ros_owned_init = True
        except:
            pass

        if _ros_node is None:
            _ros_node = Node("whisper_publisher")

        # 若 publisher 不存在或已被销毁，重新创建
        if _ros_pub is None:
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            _ros_pub = _ros_node.create_publisher(String, "/speech_text", qos)

        return _ros_pub  # 可选：返回句柄
    
def _teardown_ros_publisher():
    """销毁节点并把句柄清空；不调用 rclpy.shutdown()，以便同进程其他节点继续工作。"""
    global _ros_node, _ros_pub
    with _ros_lock:
        try:
            if _ros_node is not None:
                _ros_node.destroy_node()
        except Exception:
            pass
        _ros_node = None
        _ros_pub = None

def _publish_text(text: str):
    if not text or not text.strip():
        return
    _ensure_ros_publisher()
    from std_msgs.msg import String
    msg = String(); msg.data = text.strip()
    print(f"📤 publishing to /speech_text: {msg.data}")   # <— 新增：可见的stdout
    _ros_pub.publish(msg)


#----------------

# === 参数 ===
SAMPLERATE = 48000
BLOCKSIZE = 1024
SILENCE_THRESHOLD = 20.0
SILENCE_DURATION  = 1.5
MAX_DURATION      = 30
FIXED_WAV_PATH    = "/tmp/voice_input.wav"

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
                         max_duration=MAX_DURATION) -> str:
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
                device=("pulse", None)):  # 指定使用 PulseAudio 的远程输入设备
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

# === 调用 whisper-cli 转录 ===
def transcribe_audio(wav_path: str, delay: float = 0.0) -> str:
    model_path = os.path.expanduser("~/whisper.cpp/models/ggml-tiny.en.bin")
    cli_path   = os.path.expanduser("~/whisper.cpp/build/bin/whisper-cli")
    cmd = [cli_path, "-m", model_path, "-f", wav_path]

    result = subprocess.run(cmd, capture_output=True, text=True)
    output = result.stdout.strip()

    # 提取识别文本行：形如 "[00:00:00.000 --> 00:00:00.840]   - Hello, hello."
    lines = output.splitlines()
    text_lines = [
        line.split("]", 1)[-1].strip(" -\t") for line in lines
        if "-->" in line and "]" in line
    ]
    raw_text = " ".join(text_lines).strip()

    # === 删除标点符号（小写、去空格）===
    clean_text = _clean(raw_text)

    logger.success(f"📝 Transcribed Text: {clean_text or '<EMPTY>'}")
    if delay:
        time.sleep(delay)
    return clean_text


# === 识别函数 ===
def recognize(delay: float = 0.0) -> str:
    wav_path = record_until_silence()
    return transcribe_audio(wav_path, delay)


# === 新增：持续监听主循环 ===
# === 持续监听主循环（修正 transcribe_file -> transcribe_audio；加入“归属 shutdown”）===
# 支持外部停止信号
def run_continuous_listen(stop_event: threading.Event | None = None,
                          sleep_after_publish: float = 0.05):
    _ensure_ros_publisher()
    print("🎧 Whisper continuous listening started.")
    try:
        while rclpy.ok() and not (stop_event and stop_event.is_set()):
            wav_path = record_until_silence()
            if not wav_path:
                continue
            text = transcribe_audio(wav_path)
            if not text:
                continue
            _publish_text(text)  # 内部会 _ensure_ros_publisher()
            if sleep_after_publish:
                time.sleep(sleep_after_publish)
    except KeyboardInterrupt:
        pass
    finally:
        _teardown_ros_publisher()

if __name__ == "__main__":
    run_continuous_listen()
 
