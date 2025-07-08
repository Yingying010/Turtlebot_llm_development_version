# if_exit.py (English + with sound)

from play import play

def ifend(text: str) -> bool:
    """
    End current conversation, not entire system.
    """
    end_keywords = [
        "stop", "that's all", "i'm done", "no more", "nothing else", "end the conversation"
    ]
    text = text.lower()
    if any(kw in text for kw in end_keywords):
        play('Sound/ding.wav')
        play('Sound/quit.wav')
        return True
    return False

def ifexit(text: str) -> bool:
    """
    Exit the full assistant system.
    """
    exit_keywords = [
        "exit program", "shut down", "terminate the system", "close assistant", "quit assistant"
    ]
    text = text.lower()
    if any(kw in text for kw in exit_keywords):
        play('Sound/exit.wav')
        return True
    return False
