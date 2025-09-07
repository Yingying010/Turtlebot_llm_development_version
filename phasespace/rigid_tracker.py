import rclpy
from rclpy.node import Node
from phasespace_msgs.msg import Rigid
import math
from scipy.spatial.transform import Rotation as R
from rclpy.executors import MultiThreadedExecutor
import threading
import time
 
 
class RigidTracker(Node):
    def __init__(self, position_cache, name, position_lock=None):
        print(f"RigidTracker initialized for {name}")
        super().__init__(f'rigid_tracker_{name}')
        self.subscription = self.create_subscription(
            Rigid,
            f'/phasespace_body_{name}',
            self.listener_callback,
            10
        )
        self.position_cache = position_cache
        self.position_lock = position_lock
        self.name = name

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
                        self.position_cache[self.name] = data
                else:
                    self.position_cache[self.name] = data

                # self.get_logger().info(
                #     f"📍 {self.robot_name} position: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}, "
                #     f"qx={msg.qx:.2f}, qy={msg.qy:.2f}, qz={msg.qz:.2f}, qw={msg.qw:.2f}, "
                #     f"heading_y={heading_y:.2f}, cond={cond:.2f}"
                # )

        except Exception as e:
            self.get_logger().error(f"Error reading Rigid message: {e}")
 
 
def yaw_from_quat(qw, qx, qy, qz):
    f = R.from_quat([qx, qy, qz, qw]).apply([0, 0, 1])
    return (math.degrees(math.atan2(f[0], f[2])) + 360) % 360
 
def main():
    rclpy.init()
 
    # 用来存储位置信息
    position_cache = {}
    position_lock = threading.Lock()
 
    # 创建 robot1 的跟踪器
    tracker = RigidTracker(position_cache, "robot1", position_lock)
 
    executor = MultiThreadedExecutor()
    executor.add_node(tracker)
 
    try:
        # 启一个线程运行 ROS2 executor
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
 
        print("Listening for robot1's position updates... (Ctrl+C to stop)")
 
        # 循环打印 robot1 位置信息
        while rclpy.ok():
            with position_lock:
                if "robot1" in position_cache:
                    data = position_cache["robot1"]
                    print(
                        f"[robot1] x={data['x']:.2f}, y={data['y']:.2f}, z={data['z']:.2f}, "
                        f"heading_y={data['heading_y']:.2f}, cond={data['cond']:.2f}"
                    )
            time.sleep(1.0)
 
    except KeyboardInterrupt:
        print("\n🛑 Stopping...")
    finally:
        executor.shutdown()
        tracker.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
