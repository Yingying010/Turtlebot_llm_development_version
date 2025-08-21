import subprocess
import time

# robot1: forward 0.01
cmd_robot1 = [
    "ros2", "topic", "pub",
    "/robot1/cmd_vel",
    "geometry_msgs/msg/Twist",
    "{linear: {x: 0.01}}",
    "-r", "10"
]

# robot2: backward 0.01
cmd_robot2 = [
    "ros2", "topic", "pub",
    "/robot2/cmd_vel",
    "geometry_msgs/msg/Twist",
    "{linear: {x: -0.01}}",
    "-r", "10"
]

# 启动两个发布进程（非阻塞）
print("🚀 启动 robot1 和 robot2 的持续发布...")
proc1 = subprocess.Popen(cmd_robot1)
proc2 = subprocess.Popen(cmd_robot2)

try:
    # 持续运行直到手动终止
    print("✅ 正在运行中，按 Ctrl+C 停止...")
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("\n🛑 捕获到 Ctrl+C，正在终止...")
    proc1.terminate()
    proc2.terminate()
    proc1.wait()
    proc2.wait()
    print("✅ 已全部停止")
