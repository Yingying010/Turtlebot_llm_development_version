import os, sys
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)

import queue, threading, time, re, subprocess
import numpy as np
import sounddevice as sd
from loguru import logger
import wave
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

_ros_lock = threading.Lock()
_ros_node: Node | None = None
_ros_pub = None
_ros_owned_init = False

def _ensure_ros_publisher():
    global _ros_node, _ros_pub, _ros_owned_init
    with _ros_lock:
        try:
            rclpy.init()
            _ros_owned_init = True
        except:
            pass

        if _ros_node is None:
            _ros_node = Node("whisper_publisher")

        if _ros_pub is None:
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            _ros_pub = _ros_node.create_publisher(String, "/speech_text", qos)

        return _ros_pub

def _teardown_ros_publisher():
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
    msg = String(); msg.data = text.strip()
    print(f"publishing to /speech_text: {msg.data}")
    _ros_pub.publish(msg)


# Parameters
SAMPLERATE = 48000
BLOCKSIZE = 1024
SILENCE_THRESHOLD = 20.0
SILENCE_DURATION  = 1.5
MAX_DURATION      = 30
FIXED_WAV_PATH    = "/tmp/voice_input.wav"

def _clean(text: str) -> str:
    return re.sub(r'[^\w\s]', '', text).lower().strip()

def save_wav_standard(wav_path, audio_int16, samplerate=48000):
    with wave.open(wav_path, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)  # 16-bit PCM
        wf.setframerate(samplerate)
        wf.writeframes(audio_int16.tobytes())

def record_until_silence(threshold=SILENCE_THRESHOLD,
                         silence_duration=SILENCE_DURATION,
                         max_duration=MAX_DURATION) -> str:
    q_local         = queue.Queue()
    silence_blocks  = int(silence_duration * SAMPLERATE / BLOCKSIZE)
    max_blocks      = int(max_duration * SAMPLERATE / BLOCKSIZE)

    pre_speech_buffer = []
    pre_speech_maxlen = 24
    audio_blocks      = []
    silence_counter   = 0
    is_recording      = False

    def cb(indata, frames, time_info, status):
        if status:
            logger.warning(f"Audio status: {status}")
        q_local.put(indata.copy())

    logger.info("Waiting for speech to start...")

    with sd.InputStream(samplerate=SAMPLERATE, channels=1,
                        blocksize=BLOCKSIZE, callback=cb,
                        device=("pulse", None)):
        while True:
            try:
                block = q_local.get(timeout=1)
            except queue.Empty:
                continue

            volume = np.abs(block).mean() * 1000
            logger.debug(f"Vol: {volume:.1f}")

            pre_speech_buffer.append(block)
            if len(pre_speech_buffer) > pre_speech_maxlen:
                pre_speech_buffer.pop(0)

            if not is_recording:
                if volume > threshold:
                    logger.info("Voice detected. Start recording...")
                    is_recording = True
                    audio_blocks.extend(pre_speech_buffer)
                    audio_blocks.append(block)
                continue

            audio_blocks.append(block)

            if volume < threshold:
                silence_counter += 1
                if silence_counter >= silence_blocks:
                    logger.info("Silence detected. Stopping recording.")
                    break
            else:
                silence_counter = 0

            if len(audio_blocks) >= max_blocks:
                logger.info("Max recording length reached. Forcing stop.")
                break

    pcm_f32 = np.concatenate(audio_blocks).flatten()
    pcm_i16 = (pcm_f32 * 32767).clip(-32768, 32767).astype(np.int16)

    save_wav_standard(FIXED_WAV_PATH, pcm_i16, SAMPLERATE)
    logger.success(f"Saved recording to {FIXED_WAV_PATH}")
    return FIXED_WAV_PATH

def transcribe_audio(wav_path: str, delay: float = 0.0) -> str:
    model_path = os.path.expanduser("~/whisper.cpp/models/ggml-tiny.en.bin")
    cli_path   = os.path.expanduser("~/whisper.cpp/build/bin/whisper-cli")
    cmd = [cli_path, "-m", model_path, "-f", wav_path]

    result = subprocess.run(cmd, capture_output=True, text=True)
    output = result.stdout.strip()

    lines = output.splitlines()
    text_lines = [
        line.split("]", 1)[-1].strip(" -\t") for line in lines
        if "-->" in line and "]" in line
    ]
    raw_text = " ".join(text_lines).strip()

    clean_text = _clean(raw_text)

    logger.success(f"Transcribed Text: {clean_text or '<EMPTY>'}")
    if delay:
        time.sleep(delay)
    return clean_text

def recognize(delay: float = 0.0) -> str:
    wav_path = record_until_silence()
    return transcribe_audio(wav_path, delay)

def run_continuous_listen(stop_event: threading.Event | None = None,
                          sleep_after_publish: float = 0.05):
    _ensure_ros_publisher()
    print("Whisper continuous listening started.")
    try:
        while rclpy.ok() and not (stop_event and stop_event.is_set()):
            wav_path = record_until_silence()
            if not wav_path:
                continue
            text = transcribe_audio(wav_path)
            if not text:
                continue
            _publish_text(text)
            if sleep_after_publish:
                time.sleep(sleep_after_publish)
    except KeyboardInterrupt:
        pass
    finally:
        _teardown_ros_publisher()