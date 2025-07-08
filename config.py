import json
from datetime import datetime
from loguru import logger

# 状态参数（可用于控制 TurtleBot）
params = {
    # 运行状态锁
    "notify_enable": False,
    "chat_enable": False,
    "rec_enable": False,  # 是否启用远程控制模块（如 WiFi 控制）

    # 核心控制参数
    "command": "",             # 一般指令字符串
    "answer": "turtlebot系统已启动，请下达指令。",
    "turtlebot_command": "",   # 小车控制指令（如前进、转弯）
    "turtlebot_mode": "standby",  # 控制模式：standby, manual, follow, explore
    "robot_speed": 0.3,           # 控制移动速度（0~1）
    
    # 控制选项
    "Noticenotify": True, # 是否启用“通知播报”功能。

    "chat_or_instruct": True,  # 默认聊天模式
}

# 哪些参数需要记录变更日志
tracked_params = [
    "command",
    "turtlebot_command",
    "turtlebot_mode",
    "robot_speed",
    "Noticenotify"
]

# 外设控制参数
device_params = {
    "turtlebot1": "robot",
    "turtlebot2": "robot",
    "speaker": "audio"
}

class ConfigManager:
    def __init__(self, params, tracked_params):
        self.params = params
        self.device_params = device_params
        self.tracked_params = tracked_params

    def write_to_file(self, changed_params):
        """将变更记录写入日志文件"""
        with open("Log/config_state.txt", "a") as file:
            current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            formatted_data = {
                "timestamp": current_time,
                "changed_params": changed_params
            }
            file.write(json.dumps(formatted_data, indent=4) + "\n\n")

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
config = ConfigManager(params, tracked_params)
