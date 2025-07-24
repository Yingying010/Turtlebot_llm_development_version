import rclpy
from rclpy.node import Node
# from std_msgs.msg import String
import json
from phasespace_msgs.msg import Rigid


class RigidTracker(Node):
    def __init__(self, position_cache, robot_name):
        super().__init__('rigid_tracker')
        self.subscription = self.create_subscription(
            Rigid,
            f'/phasespace_body_{robot_name}',
            self.listener_callback,
            10
        )
        self.position_cache = position_cache
        self.robot_name = robot_name

    def listener_callback(self, msg):
        try:
            cond = msg.cond
            if cond > 0.8:
                self.position_cache[self.robot_name] = {
                    "x": msg.x,
                    "y": msg.y,
                    "z": msg.z,
                    "qx": msg.qx,
                    "qy": msg.qy,
                    "qz": msg.qz,
                    "qw": msg.qw,
                    "heading_y": msg.heading_y,
                    "cond": cond
                }
                # self.get_logger().info(
                #     f"📍 {self.robot_name} position: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}, "
                #     f"qx={msg.qx:.2f}, qy={msg.qy:.2f}, qz={msg.qz:.2f}, qw={msg.qw:.2f}, "
                #     f"heading_y={msg.heading_y:.2f}, cond={cond:.2f}"
                # )
        except Exception as e:
            self.get_logger().error(f"💥 Error reading Rigid message: {e}")


def main(args=None):
    rclpy.init(args=args)
    position_cache = {}
    robot_name = "robot1"
    node = RigidTracker(position_cache,robot_name)

    # ✅ 打印缓存
    # print("📦 Current position cache:", position_cache)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
