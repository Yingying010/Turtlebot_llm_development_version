# config.py
import json
from pathlib import Path

CONFIG_PATH = Path("config.json")

# 默认参数（首次生成时使用）
DEFAULT_CONFIG = {
    "robot_id": "robot1",
    "master_id": "lucy",
    "isConversation": True,
    "chat_or_instruct": False
}

def load_config():
    if not CONFIG_PATH.exists():
        save_config(DEFAULT_CONFIG)
    with CONFIG_PATH.open("r", encoding="utf-8") as f:
        return json.load(f)

def save_config(data: dict):
    with CONFIG_PATH.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

def get(key, default=None):
    cfg = load_config()
    return cfg.get(key, default)

def set(**kwargs):
    cfg = load_config()
    cfg.update(kwargs)
    save_config(cfg)
