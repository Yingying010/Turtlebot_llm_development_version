import os
from ttsRepo.stream_tts import tts_manager

# 设置远程 PulseAudio 服务器地址（树莓派 IP）
# os.environ["PULSE_SERVER"] = "tcp:192.168.0.208"  # ⚠️ 记得替换成你的 IP，例如 192.168.1.42

# 测试播报
tts_manager.say("Hello! This is a test")
