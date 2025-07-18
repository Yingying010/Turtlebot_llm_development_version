import subprocess

def play_beep_aplay(path):
    try:
        subprocess.run(["aplay", "-D", "plughw:2,0",path], check=True)
    except Exception as e:
        print(f"播放失败: {e}")


if __name__ == "__main__":
    path = "soundRepo/beep.wav"
    play_beep_aplay(path)