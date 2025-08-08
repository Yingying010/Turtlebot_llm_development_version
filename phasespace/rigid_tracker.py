import rclpy
from rclpy.node import Node
from phasespace_msgs.msg import Rigid
import math
from scipy.spatial.transform import Rotation as R
 
 
class RigidTracker(Node):
    def __init__(self, position_cache, robot_name, position_lock=None):
        print(f"✅ RigidTracker initialized for {robot_name}")
        super().__init__('rigid_tracker')
        self.subscription = self.create_subscription(
            Rigid,
            f'/phasespace_body_{robot_name}',
            self.listener_callback,
            10
        )
        self.position_cache = position_cache
        self.position_lock = position_lock
        self.robot_name = robot_name

    def listener_callback(self, msg):
        # self.get_logger().info("📩 Received Rigid message")
        try:
            cond = msg.cond
            if cond > 0.8:
                heading_y = yaw_from_quat(msg.qx, msg.qy, msg.qz, msg.qw)
                data = {
                    "x": msg.x,
                    "y": msg.y,
                    "z": msg.z,
                    "qx": msg.qx,
                    "qy": msg.qy,
                    "qz": msg.qz,
                    "qw": msg.qw,
                    "heading_y": heading_y,
                    "cond": cond
                }
                if self.position_lock:
                    with self.position_lock:
                        self.position_cache[self.robot_name] = data
                else:
                    self.position_cache[self.robot_name] = data

                # self.get_logger().info(
                #     f"📍 {self.robot_name} position: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}, "
                #     f"qx={msg.qx:.2f}, qy={msg.qy:.2f}, qz={msg.qz:.2f}, qw={msg.qw:.2f}, "
                #     f"heading_y={heading_y:.2f}, cond={cond:.2f}"
                # )

        except Exception as e:
            self.get_logger().error(f"💥 Error reading Rigid message: {e}")
 
 
def yaw_from_quat(qw, qx, qy, qz):
    f = R.from_quat([qx, qy, qz, qw]).apply([0, 0, 1])
    return (math.degrees(math.atan2(f[0], f[2])) + 360) % 360
 
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
 
