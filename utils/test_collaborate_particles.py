# multi_robot_move.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import time

class MultiRobotController(Node):
    def __init__(self):
        super().__init__('multi_robot_controller')

    def move(self, robot_id, direction, speed, duration_sec):
        publisher = self.create_publisher(Twist, f'/{robot_id}/cmd_vel', 10)
        twist = Twist()

        # 设置方向
        if direction == "forward":
            twist.linear.x = abs(speed)
        elif direction == "backward":
            twist.linear.x = -abs(speed)
        else:
            self.get_logger().error(f"Unknown direction: {direction}")
            return

        time.sleep(0.2)  # 等待 publisher 初始化

        start = time.time()
        while time.time() - start < duration_sec:
            publisher.publish(twist)
            time.sleep(0.1)  # 相当于 -r 10

        publisher.publish(Twist())  # 停止
        self.get_logger().info(f"{robot_id} finished moving {direction}.")

def main():
    rclpy.init()
    node = MultiRobotController()

    # 设置移动参数
    duration = 3  # 秒
    speed = 0.01  # m/s

    # 启动两个线程控制两个机器人
    t1 = threading.Thread(target=node.move, args=("robot1", "forward", speed, duration))
    t2 = threading.Thread(target=node.move, args=("robot2", "backward", speed, duration))

    t1.start()
    t2.start()

    t1.join()
    t2.join()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
