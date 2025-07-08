# if_time.py

def timedetect(text: str) -> bool:
    """
    Detect if the user is asking for current time.
    Returns True if a time-related phrase is detected.
    """
    time_keywords = [
        "what time is it",
        "current time",
        "tell me the time",
        "can you give me the time",
        "do you know the time"
    ]
    text = text.lower()
    return any(kw in text for kw in time_keywords)
