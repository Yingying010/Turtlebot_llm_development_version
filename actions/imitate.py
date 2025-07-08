# actions/imitate.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import time

# 全局 imitation 控制器（记录哪个机器人在模仿）
imitation_threads = {}
imitation_flags = {}

def imitate_robot(robot_id, target_robot_id):
    """
    启动 imitation 线程，让 robot_id 模仿 target_robot_id 的速度指令
    """
    if robot_id in imitation_threads:
        print(f"⚠️ {robot_id} is already imitating.")
        return


    def imitation_loop():
        node = rclpy.create_node(f"{robot_id}_imitator")
        pub = node.create_publisher(Twist, f"/{robot_id}/cmd_vel", 10)
        latest_cmd = Twist()

        def callback(msg):
            nonlocal latest_cmd
            latest_cmd = msg

        sub = node.create_subscription(
            Twist,
            f"/{target_robot_id}/cmd_vel",
            callback,
            10
        )

        imitation_flags[robot_id] = True
        print(f"🎭 {robot_id} started imitating {target_robot_id}...")

        while imitation_flags[robot_id]:
            pub.publish(latest_cmd)
            rclpy.spin_once(node, timeout_sec=0.1)

        pub.publish(Twist())  # 停止运动
        node.destroy_node()
        print(f"🛑 {robot_id} stopped imitating {target_robot_id}.")

    t = threading.Thread(target=imitation_loop)
    t.start()
    imitation_threads[robot_id] = t

def stop_imitation(robot_id):
    """
    停止指定机器人的 imitation 行为
    """
    if robot_id not in imitation_threads:
        print(f"⚠️ {robot_id} is not imitating.")
        return

    imitation_flags[robot_id] = False
    imitation_threads[robot_id].join()
    del imitation_threads[robot_id]
    del imitation_flags[robot_id]
