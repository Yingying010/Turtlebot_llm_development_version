# config.py
import json
from pathlib import Path

CONFIG_PATH = Path("config.json")

DEFAULT_CONFIG = {
    "robot_id": "robot1",
    "master_id": "lucy",
    "isConversation": True,
    "chat_or_instruct": False,

    # 👇👇 新增：find 搜索相关配置
    "find_use_rotate_scan": True,       # 找不到时是否做旋转扫描
    "find_rotate_step_deg": 30,         # 每步转多少度
    "find_rotate_max_deg": 360,         # 最大总转角
    "find_rotate_pause_s": 0.5,         # 每步后暂停秒数
    "find_rotate_speed_deg_s": 25.0,    # 旋转角速度（度/秒）
    "find_waypoints": ["table", "shelf", "corner"],  # 多点搜索顺序（名称对应 semantic_locations）

    "semantic_locations": {
        "lucy":  {"x": -500, "y": -900, "heading": None},
        "table": {"x":    0, "y":    0, "heading": None},
        "robot2":{"x": -500, "y": -500, "heading": None},
        "corner":{"x": 1000, "y": 1000, "heading": None},
        "shelf": {"x": 1000, "y": -1000,"heading": None},
    },
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

def update_nested(parent_key: str, subkey: str, value: dict):
    """
    用于更新嵌套字段，例如 semantic_locations["robot3"] = {x: ..., y: ...}
    """
    cfg = load_config()
    if parent_key not in cfg or not isinstance(cfg[parent_key], dict):
        cfg[parent_key] = {}
    cfg[parent_key][subkey] = value
    save_config(cfg)

def delete_nested(parent_key: str, subkey: str):
    cfg = load_config()
    if parent_key in cfg and subkey in cfg[parent_key]:
        del cfg[parent_key][subkey]
        save_config(cfg)

def reset_to_default():
    save_config(DEFAULT_CONFIG)
