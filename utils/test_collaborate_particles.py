import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import time

class ContinuousMover(Node):
    def __init__(self):
        super().__init__('continuous_mover')
        self.robot_publishers = {}
        self.stop_flags = {}

    def start_moving(self, robot_id, direction, speed):
        pub = self.create_publisher(Twist, f'/{robot_id}/cmd_vel', 10)
        self.robot_publishers[robot_id] = pub
        self.stop_flags[robot_id] = False

        twist = Twist()
        if direction == "forward":
            twist.linear.x = abs(speed)
        elif direction == "backward":
            twist.linear.x = -abs(speed)
        else:
            self.get_logger().error(f"Unknown direction: {direction}")
            return

        def move_loop():
            self.get_logger().info(f"🚗 {robot_id} starts moving {direction} at {speed} m/s")
            time.sleep(0.2)  # 给 publisher 建立时间

            while not self.stop_flags[robot_id]:
                pub.publish(twist)
                time.sleep(0.1)  # 10Hz

            # 停止
            pub.publish(Twist())
            time.sleep(0.1)
            self.get_logger().info(f"🛑 {robot_id} stopped.")

        thread = threading.Thread(target=move_loop)
        thread.start()

    def stop_all(self):
        for robot_id in self.stop_flags:
            self.stop_flags[robot_id] = True
        time.sleep(0.5)

def main():
    rclpy.init()
    node = ContinuousMover()

    try:
        node.start_moving("robot1", "forward", 0.01)
        node.start_moving("robot2", "backward", 0.01)
        print("🚀 Both robots are moving. Press Ctrl+C to stop.")

        # ✅ 使用多线程 executor（关键！）
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()

    except KeyboardInterrupt:
        print("\n🛑 KeyboardInterrupt detected. Stopping robots...")
        node.stop_all()

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
