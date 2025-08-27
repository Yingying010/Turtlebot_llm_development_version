"""
Optimized Speech Recognition System with Advanced Voice Activity Detection
Enhanced features:
- Adaptive silence detection
- Intelligent noise filtering  
- Configurable parameters
- Better error handling
- Performance optimizations
"""

import os
import sys
import queue
import threading
import time
import re
import subprocess
from pathlib import Path
from typing import Final, Optional, Dict, Any, Tuple
from dataclasses import dataclass
from contextlib import contextmanager

import numpy as np
import sounddevice as sd
from scipy.io.wavfile import write
from loguru import logger
import wave
from openai import OpenAI

PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.append(PROJECT_ROOT)

# Global conversation state
conversation_active: Final[threading.Event] = threading.Event()

@dataclass
class SpeechConfig:
    """Configuration for speech recognition parameters"""
    # Audio parameters
    samplerate: int = 48000
    blocksize: int = 1024
    channels: int = 1
    
    # Voice Activity Detection
    silence_threshold: float = 15.0  # Lowered from 20.0
    silence_duration: float = 3.0    # Increased from 1.5
    max_duration: float = 45.0       # Increased from 30
    
    # Advanced VAD parameters
    enable_adaptive_threshold: bool = True
    noise_gate_factor: float = 0.3
    volume_history_size: int = 20
    confirmation_duration: float = 2.0
    
    # Audio device
    audio_device: Optional[str] = None  # None = default, "pulse" for remote
    
    # OpenAI API
    openai_model: str = "whisper-1"  # More reliable than gpt-4o-transcribe
    language: str = "en"
    temperature: float = 0.0
    
    # File paths
    temp_wav_path: str = "/tmp/voice_input.wav"
    
    @classmethod
    def from_env(cls) -> 'SpeechConfig':
        """Load configuration from environment variables"""
        return cls(
            silence_threshold=float(os.getenv("SPEECH_SILENCE_THRESHOLD", "15.0")),
            silence_duration=float(os.getenv("SPEECH_SILENCE_DURATION", "3.0")),
            max_duration=float(os.getenv("SPEECH_MAX_DURATION", "45.0")),
            openai_model=os.getenv("OPENAI_STT_MODEL", "whisper-1"),
            language=os.getenv("SPEECH_LANGUAGE", "en"),
            audio_device=os.getenv("SPEECH_AUDIO_DEVICE"),
        )

class AdaptiveVoiceDetector:
    """Advanced Voice Activity Detection with adaptive thresholding"""
    
    def __init__(self, config: SpeechConfig):
        self.config = config
        self.volume_history = []
        self.noise_floor = 0.0
        self.adaptive_threshold = config.silence_threshold
        self.is_calibrated = False
        
    def update_volume_history(self, volume: float):
        """Update volume history for adaptive thresholding"""
        self.volume_history.append(volume)
        if len(self.volume_history) > self.config.volume_history_size:
            self.volume_history.pop(0)
    
    def calibrate_noise_floor(self, initial_samples: list):
        """Calibrate noise floor from initial samples"""
        if len(initial_samples) >= 10:
            self.noise_floor = np.mean(initial_samples) + np.std(initial_samples)
            self.adaptive_threshold = max(
                self.noise_floor * 2.0,
                self.config.silence_threshold * 0.7
            )
            self.is_calibrated = True
            logger.debug(f"🎛️ Noise floor calibrated: {self.noise_floor:.2f}, "
                        f"Adaptive threshold: {self.adaptive_threshold:.2f}")
    
    def is_speech(self, volume: float) -> bool:
        """Determine if current volume indicates speech"""
        self.update_volume_history(volume)
        
        # Use adaptive threshold if enabled and calibrated
        if self.config.enable_adaptive_threshold and self.is_calibrated:
            threshold = self.adaptive_threshold
        else:
            threshold = self.config.silence_threshold
        
        return volume > threshold
    
    def is_sustained_silence(self, silent_blocks: int) -> Tuple[bool, bool]:
        """Check for sustained silence with confirmation logic"""
        required_blocks = int(self.config.silence_duration * 
                             self.config.samplerate / self.config.blocksize)
        
        # Initial silence detection
        initial_silence = silent_blocks >= required_blocks
        
        # Extended confirmation for longer utterances
        if initial_silence and len(self.volume_history) > required_blocks:
            # Check if this might be mid-sentence pause
            recent_avg = np.mean(self.volume_history[-required_blocks:])
            overall_avg = np.mean(self.volume_history)
            
            # If recent volume is significantly higher than noise floor,
            # this might be a natural pause
            extended_silence = (silent_blocks >= 
                               required_blocks + int(self.config.confirmation_duration * 
                                                   self.config.samplerate / self.config.blocksize))
            
            return initial_silence, extended_silence
        
        return initial_silence, initial_silence

class OptimizedSpeechRecognizer:
    """Main speech recognition class with optimizations"""
    
    def __init__(self, config: Optional[SpeechConfig] = None):
        self.config = config or SpeechConfig.from_env()
        self.vad = AdaptiveVoiceDetector(self.config)
        self.openai_client = None
        self._setup_openai_client()
        
    def _setup_openai_client(self):
        """Initialize OpenAI client with error handling"""
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            logger.warning("⚠️ OPENAI_API_KEY not set. OpenAI transcription will fail.")
            return
        
        try:
            self.openai_client = OpenAI(api_key=api_key)
            logger.debug("✅ OpenAI client initialized")
        except Exception as e:
            logger.error(f"❌ Failed to initialize OpenAI client: {e}")

    @staticmethod
    def clean_text(text: str) -> str:
        """Clean transcribed text"""
        if not text:
            return ""
        # Remove punctuation but keep basic structure
        cleaned = re.sub(r'[^\w\s\-\']', '', text.lower().strip())
        # Remove extra whitespace
        cleaned = re.sub(r'\s+', ' ', cleaned)
        return cleaned

    @staticmethod
    def is_valid_english(text: str, min_length: int = 3) -> bool:
        """Validate if text is meaningful English"""
        if not text or len(text.strip()) < min_length:
            return False
        
        # Check for reasonable English character distribution
        alpha_chars = sum(1 for c in text if c.isalpha())
        total_chars = len(text.replace(' ', ''))
        
        if total_chars == 0:
            return False
            
        alpha_ratio = alpha_chars / total_chars
        return alpha_ratio > 0.7  # At least 70% alphabetic characters

    def save_wav_optimized(self, audio_data: np.ndarray, filepath: str):
        """Save audio data to WAV file with optimizations"""
        try:
            # Ensure audio is in correct format
            if audio_data.dtype != np.int16:
                # Convert float32 to int16
                audio_int16 = (audio_data * 32767).clip(-32768, 32767).astype(np.int16)
            else:
                audio_int16 = audio_data
            
            with wave.open(filepath, "wb") as wf:
                wf.setnchannels(self.config.channels)
                wf.setsampwidth(2)  # 16-bit
                wf.setframerate(self.config.samplerate)
                wf.writeframes(audio_int16.tobytes())
                
            logger.debug(f"💾 Audio saved: {filepath} ({len(audio_int16)} samples)")
            
        except Exception as e:
            logger.error(f"❌ Failed to save audio: {e}")
            raise

    @contextmanager
    def audio_stream(self):
        """Context manager for audio stream"""
        device_config = {}
        if self.config.audio_device == "pulse":
            device_config["device"] = ("pulse", None)
        elif self.config.audio_device:
            device_config["device"] = self.config.audio_device

        stream = None
        try:
            stream = sd.InputStream(
                samplerate=self.config.samplerate,
                channels=self.config.channels,
                blocksize=self.config.blocksize,
                **device_config
            )
            stream.start()
            yield stream
        except Exception as e:
            logger.error(f"❌ Audio stream error: {e}")
            raise
        finally:
            if stream:
                stream.stop()
                stream.close()

    def record_with_vad(self) -> Optional[str]:
        """Record audio with advanced Voice Activity Detection"""
        audio_queue = queue.Queue()
        max_blocks = int(self.config.max_duration * self.config.samplerate / self.config.blocksize)
        
        pre_speech_buffer = []
        pre_speech_maxlen = 30  # Increased buffer
        audio_blocks = []
        silence_counter = 0
        is_recording = False
        
        # Noise calibration samples
        calibration_samples = []
        calibration_blocks = 20
        
        def audio_callback(indata, frames, time_info, status):
            if status:
                logger.warning(f"⚠️ Audio callback status: {status}")
            audio_queue.put(indata.copy())

        logger.info("🎙️ Initializing speech detection...")
        
        try:
            with self.audio_stream() as stream:
                stream.callback = audio_callback
                
                while True:
                    try:
                        block = audio_queue.get(timeout=2.0)
                    except queue.Empty:
                        if is_recording:
                            logger.warning("⏰ Audio stream timeout during recording")
                            break
                        continue

                    volume = np.abs(block).mean() * 1000
                    logger.debug(f"📊 Volume: {volume:.2f}")

                    # Calibration phase
                    if len(calibration_samples) < calibration_blocks:
                        calibration_samples.append(volume)
                        if len(calibration_samples) == calibration_blocks:
                            self.vad.calibrate_noise_floor(calibration_samples)
                        continue

                    # Maintain pre-speech buffer
                    pre_speech_buffer.append(block)
                    if len(pre_speech_buffer) > pre_speech_maxlen:
                        pre_speech_buffer.pop(0)

                    # Check for speech start
                    if not is_recording:
                        if self.vad.is_speech(volume):
                            logger.info("🔴 Speech detected - Starting recording")
                            is_recording = True
                            # Add buffered audio
                            audio_blocks.extend(pre_speech_buffer)
                            audio_blocks.append(block)
                            silence_counter = 0
                        continue

                    # Recording phase
                    audio_blocks.append(block)

                    # Voice Activity Detection during recording
                    if self.vad.is_speech(volume):
                        silence_counter = 0
                    else:
                        silence_counter += 1
                        
                        # Check for sustained silence
                        initial_silence, extended_silence = self.vad.is_sustained_silence(silence_counter)
                        
                        if extended_silence:
                            logger.info("🔇 Extended silence detected - Stopping recording")
                            break
                        elif initial_silence:
                            logger.debug("🤔 Initial silence detected - Waiting for confirmation")

                    # Max duration check
                    if len(audio_blocks) >= max_blocks:
                        logger.info("⏰ Maximum recording duration reached")
                        break

        except Exception as e:
            logger.error(f"❌ Recording failed: {e}")
            return None

        if not audio_blocks:
            logger.warning("⚠️ No audio recorded")
            return None

        # Process and save audio
        try:
            pcm_data = np.concatenate(audio_blocks).flatten()
            self.save_wav_optimized(pcm_data, self.config.temp_wav_path)
            logger.success(f"✅ Recording completed: {len(pcm_data)} samples, "
                          f"{len(pcm_data)/self.config.samplerate:.1f}s")
            return self.config.temp_wav_path
            
        except Exception as e:
            logger.error(f"❌ Failed to process recorded audio: {e}")
            return None

    def transcribe_with_openai(self, wav_path: str) -> str:
        """Transcribe audio using OpenAI Whisper API"""
        if not self.openai_client:
            logger.error("❌ OpenAI client not available")
            return ""

        if not os.path.exists(wav_path):
            logger.error(f"❌ Audio file not found: {wav_path}")
            return ""

        try:
            file_size = os.path.getsize(wav_path)
            logger.debug(f"📁 Transcribing audio file: {wav_path} ({file_size} bytes)")
            
            with open(wav_path, "rb") as audio_file:
                response = self.openai_client.audio.transcriptions.create(
                    model=self.config.openai_model,
                    file=audio_file,
                    language=self.config.language,
                    temperature=self.config.temperature,
                    response_format="text"
                )
            
            # Handle different response formats
            if hasattr(response, 'text'):
                raw_text = response.text
            else:
                raw_text = str(response)
            
            logger.info(f"📄 Raw transcription: '{raw_text}'")
            
            # Clean and validate
            clean_text = self.clean_text(raw_text)
            
            if not self.is_valid_english(clean_text):
                logger.warning(f"⚠️ Invalid or non-English content detected: '{clean_text}'")
                return ""
            
            logger.success(f"✅ Transcription completed: '{clean_text}'")
            return clean_text
            
        except Exception as e:
            logger.exception(f"❌ OpenAI transcription failed: {e}")
            return ""

    def recognize_speech(self, delay: float = 0.0) -> str:
        """Main speech recognition method"""
        logger.info("🎤 Starting speech recognition...")
        
        # Record audio
        wav_path = self.record_with_vad()
        if not wav_path:
            logger.error("❌ Recording failed")
            return ""
        
        # Transcribe
        result = self.transcribe_with_openai(wav_path)
        
        # Optional delay
        if delay > 0:
            time.sleep(delay)
            
        # Cleanup
        try:
            if os.path.exists(wav_path):
                os.remove(wav_path)
        except Exception as e:
            logger.warning(f"⚠️ Failed to cleanup temp file: {e}")
        
        return result

# Global instance for backward compatibility
_default_recognizer = None

def get_recognizer() -> OptimizedSpeechRecognizer:
    """Get or create default recognizer instance"""
    global _default_recognizer
    if _default_recognizer is None:
        _default_recognizer = OptimizedSpeechRecognizer()
    return _default_recognizer

# Backward compatibility functions
def recognize(delay: float = 0.0) -> str:
    """Legacy interface for speech recognition"""
    return get_recognizer().recognize_speech(delay)

def _clean(text: str) -> str:
    """Legacy text cleaning function"""
    return OptimizedSpeechRecognizer.clean_text(text)

# Testing and diagnostics
def test_audio_levels(duration: float = 10.0):
    """Test and display audio input levels for calibration"""
    recognizer = get_recognizer()
    
    print(f"🎛️ Testing audio levels for {duration} seconds...")
    print("📊 Speak normally to see volume levels:")
    
    audio_queue = queue.Queue()
    volumes = []
    
    def callback(indata, frames, time_info, status):
        audio_queue.put(indata.copy())
    
    try:
        with recognizer.audio_stream() as stream:
            stream.callback = callback
            start_time = time.time()
            
            while time.time() - start_time < duration:
                try:
                    block = audio_queue.get(timeout=0.1)
                    volume = np.abs(block).mean() * 1000
                    volumes.append(volume)
                    
                    # Real-time volume display
                    bar_length = int(volume / 5)  # Scale for display
                    bar = "█" * min(bar_length, 50)
                    print(f"\r📊 Volume: {volume:6.2f} {bar:<50}", end="", flush=True)
                    
                except queue.Empty:
                    continue
    
    except Exception as e:
        print(f"\n❌ Audio test failed: {e}")
        return
    
    print(f"\n\n📈 Audio Level Statistics:")
    print(f"   Average: {np.mean(volumes):.2f}")
    print(f"   Maximum: {np.max(volumes):.2f}")
    print(f"   Minimum: {np.min(volumes):.2f}")
    print(f"   Std Dev: {np.std(volumes):.2f}")
    
    recommended_threshold = np.mean(volumes) + 2 * np.std(volumes)
    print(f"\n💡 Recommended silence threshold: {recommended_threshold:.1f}")

if __name__ == "__main__":
    # Command line interface for testing
    import argparse
    
    parser = argparse.ArgumentParser(description="Optimized Speech Recognition System")
    parser.add_argument("--test-audio", action="store_true", help="Test audio input levels")
    parser.add_argument("--test-recognition", action="store_true", help="Test speech recognition")
    parser.add_argument("--duration", type=float, default=10.0, help="Test duration in seconds")
    
    args = parser.parse_args()
    
    if args.test_audio:
        test_audio_levels(args.duration)
    elif args.test_recognition:
        print("🎤 Testing speech recognition (speak after the prompt):")
        result = recognize()
        print(f"📝 Result: '{result}'")
    else:
        print("Use --test-audio or --test-recognition to test the system")