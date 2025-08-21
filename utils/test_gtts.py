from gtts import gTTS
import os

# 生成语音
tts = gTTS("Hello, how are you?", lang="en", tld="com")
tts.save("output.mp3")

# 播放
os.system("mpg123 output.mp3")
