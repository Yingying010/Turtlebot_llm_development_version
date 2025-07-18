import subprocess

def play_beep_aplay():
    try:
        subprocess.run(["aplay", "-D", "plughw:2,0", "soundRepo/beep.wav"], check=True)
    except Exception as e:
        print(f"播放失败: {e}")