import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from phasespace_msgs.msg import Rigids  # ✅ 确保你的本地有这个自定义消息类型
import json
import math

class RigidsJsonBridge(Node):
    def __init__(self):
        super().__init__('rigids_json_bridge')
        self.rigid_id_to_name = {
            1: "robot1",
            2: "robot2",
            # 可以继续扩展更多
        }

        self.subscription = self.create_subscription(
            Rigids,
            '/phasespace_rigids',  # ⚠️ 要确认这个 topic 是否真的在发布
            self.listener_callback,
            10
        )

        self.rigid_publishers = {}  # ✅ 初始化 publishers 字典

        for name in self.rigid_id_to_name.values():
            topic = f"/rigids_position/{name}"
            self.rigid_publishers[name] = self.create_publisher(String, topic, 10)
        self.get_logger().info('Rigids JSON bridge started.')
    


    def quaternion_to_yaw(self, qx, qy, qz, qw):
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy**2 + qz**2)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw_rad)
        if yaw_deg<0:
                yaw_deg += 360
        return yaw_deg

    


    def listener_callback(self, msg):
        for rigid in msg.rigids:
            if rigid.cond < 0:
                continue

            robot_name = self.rigid_id_to_name.get(rigid.id, None)
            if not robot_name:
                self.get_logger().warn(f"Unknown rigid id: {rigid.id}")
                continue

            rigid_dict = {
                "id": rigid.id,
                "x": rigid.x,
                "y": rigid.y,
                "z": rigid.z,
                "qx": rigid.qx,
                "qy": rigid.qy,
                "qz": rigid.qz,
                "qw": rigid.qw,
                "heading_y": rigid.heading_y,
                "cond": rigid.cond
            }

            json_obj = {"rigids": [rigid_dict]}
            json_str = json.dumps(json_obj, indent=2)
            print(f"\n📤 Publishing for {robot_name}:\n", json_str)

            ros_msg = String()
            ros_msg.data = json.dumps(json_obj)
            self.rigid_publishers[robot_name].publish(ros_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RigidsJsonBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
