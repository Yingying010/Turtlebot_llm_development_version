import json
from datetime import datetime
from loguru import logger


# 状态参数（可用于控制 TurtleBot）
params = {
    "robot_id":"robot1",
    "robot_id2":"robot2",
    "master_id": "lucy",
    "isConversation": True,
}

semantic_locations = {
    "lucy": {"x": 800, "y": 800},
    "table": {"x": 0, "y": 0},
    "robot2": {"x": 0, "y": 500},  # 可选 heading
    "corner": {"x": 500, "y": -500},
    "shelf": {"x": -1000, "y": -1000}
}

class ConfigManager:
    def __init__(self, params):
        self.params = params
        self.tracked_params = {"robot_id", "master_id", "chat_or_instruct", "isConversation"}

    def set(self, **kwargs):
        changed_params = {}
        for key, value in kwargs.items():
            if key in self.tracked_params and self.params.get(key) != value:
                changed_params[key] = value
            self.params[key] = value
        # if changed_params:
        #     logger.info(f'✅ Changed_params: {changed_params}')

    def get(self, key):
        return self.params.get(key)


# 单例配置对象
config = ConfigManager(params)
