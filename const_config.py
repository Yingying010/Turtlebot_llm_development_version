# const_config.py

# 每个设备的 value 参数期望的类型
param_types = {
    "turtlebot1": "float",
    "turtlebot2": "float",
    "speaker": "float"
}

# ✅ 是否启用本地语音识别模块（如 Vosk）
use_online_recognize = False   # 若改为 True，表示使用 Azure 等在线识别

# ✅ 是否启用 OpenAI 模型（如 ChatGPT）作为对话系统
chat_or_instruct = True

# ✅ 是否启用 MQTT / WiFi 控制功能（若启用远程控制 TurtleBot）
wlan_enable = False  # 你之后接入 MQTT 时可以改为 True

# ✅ 热词唤醒设置
porcupine_enable = True
porcupinepath="/Users/heartfillialucy/UCL/IOT Project/随便写写的代码库/Porcupine"
porcupine_keywords = ["picovoice", "bumblebee", "hey google"]
porcupine_sensitivities = [0.65, 0.65, 0.65]
porcupine_access_key = "m6bCeuPahAqtAXgoL7cjx4ANi0lLGXsy//mEy9m+asqn5igPT7LQNg=="

