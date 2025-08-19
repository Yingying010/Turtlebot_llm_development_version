import json
from datetime import datetime
from loguru import logger


# 状态参数（可用于控制 TurtleBot）
params = {
    "robot_id":"robot1",
    "robot_id2":"robot2",
    "master_id": "lucy",
    "isConversation": True,
    "chat_or_instruct": True,  # 默认聊天模式
}

semantic_locations = {
    "lucy": {"x": 500, "y": 500},
    "amy": {"x": 100, "y": 100, "heading":180},
    "table": {"x": 0, "y": -1500},
    "robot2": {"x": 0, "y": 0},  # 可选 heading
    "corner": {"x": 0, "y": 0},
    "shelf": {"x": -1000, "y": -1000}
}

class ConfigManager:
    def __init__(self, params):
        self.params = params

    def set(self, **kwargs):
        """更新参数值"""
        changed_params = {}
        for key, value in kwargs.items():
            if key in self.params:
                if (key in self.tracked_params) and self.params[key] != value:
                    changed_params[key] = value
                self.params[key] = value
            elif key in self.device_params:
                if key in self.tracked_params and self.device_params[key] != value:
                    changed_params[key] = value
                self.device_params[key] = value
            else:
                logger.warning(f"⚠️ Unknown config key: {key}")
        if changed_params:
            self.write_to_file(changed_params)
            logger.info(f'✅ Changed_params: {changed_params}')

    def get(self, key):
        """获取参数值"""
        return self.params.get(key) if key in self.params else self.device_params.get(key, None)


# 单例配置对象
config = ConfigManager(params)
