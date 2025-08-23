#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Collaborative Contactless Transport (Decentralized, Formation+Sync)
- 两车在微粒点 P 背靠背成队，整体朝向指向目标 T
- 分布式握手与微秒级同步起跑
- 运输阶段：robot1 前进(+v)，robot2 后退(-v)，保持基线不变直达目标

运行示例：
  robot1 终端：
    python3 test_collaborate_particles_decentralized.py --robot_id robot1 --peer_id robot2 \
      --particle 0 0 --target 500 -500 --baseline 0.38 --speed 0.06
  robot2 终端：
    python3 test_collaborate_particles_decentralized.py --robot_id robot2 --peer_id robot1 \
      --particle 0 0 --target 500 -500 --baseline 0.38 --speed 0.06
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import math, time, json, argparse, threading

# ======= 工具函数 =======

def angle_wrap(a):
    """wrap to [-pi, pi)"""
    while a >= math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

def plan_formation(particle_xy, target_xy, baseline):
    """
    基于微粒点P与目标点T规划队形：
    - 方向 φ = atan2(Ty-Py, Tx-Px)
    - 中心在 P
    - robot1 在 -baseline/2 * u 处、朝向 φ
    - robot2 在 +baseline/2 * u 处、朝向 φ+π
    返回：
      phi, r1_pose(x,y,theta), r2_pose(x,y,theta), path_len
    """
    px, py = particle_xy
    tx, ty = target_xy
    phi = math.atan2(ty - py, tx - px)         # 整体朝向
    ux, uy = math.cos(phi), math.sin(phi)      # 单位方向
    half = 0.5 * baseline
    r1 = (px - half*ux, py - half*uy, phi)     # 背靠背：r1 朝 φ
    r2 = (px + half*ux, py + half*uy, phi + math.pi)  # r2 朝 φ+π
    path_len = math.hypot(tx - px, ty - py)    # 中心到目标的直线距离
    return phi, r1, r2, path_len

# ======= 简化的姿态/控制占位（可接你的里程计与导航） =======
class SimpleMotion:
    """
    仅用于最小可行测试：
    - rotate_to: 固定时间旋转（无里程计闭环）
    - drive_distance: 固定时间直行
    真实系统建议替换为：你的 /odom 或 PhaseSpace 姿态闭环。
    """
    def __init__(self, node: Node, robot_ns: str):
        self.node = node
        self.pub = node.create_publisher(Twist, f'/{robot_ns}/cmd_vel', 10)

    def stop(self):
        tw = Twist()
        self.pub.publish(tw)

    def rotate_to(self, target_yaw: float, ang_speed: float = 0.6, max_time: float = 2.0):
        """
        无里程计版本：仅朝目标方向粗略旋转 max_time 秒
        有里程计时，请改为闭环 turn-to-heading。
        """
        tw = Twist()
        # 简化：按目标 yaw 的符号给出旋转方向（只为占位，不精准）
        tw.angular.z = ang_speed if angle_wrap(target_yaw) >= 0 else -ang_speed
        t0 = time.time()
        while time.time() - t0 < max_time:
            self.pub.publish(tw)
            rclpy.spin_once(self.node, timeout_sec=0.0)
            time.sleep(0.02)
        self.stop()

    def drive_distance(self, forward: bool, dist: float, speed: float = 0.06):
        """
        无里程计版本：按距离/速度估算时间开环直行
        forward=True: 线速度 +speed；False: -speed
        """
        dur = max(0.0, dist / max(1e-6, speed))
        tw = Twist()
        tw.linear.x = speed if forward else -speed
        t0 = time.time()
        while time.time() - t0 < dur:
            self.pub.publish(tw)
            rclpy.spin_once(self.node, timeout_sec=0.0)
            time.sleep(0.02)
        self.stop()

# ======= 协调消息类型 =======
MSG_SYN   = "transport_syn"     # 发起
MSG_ACK   = "transport_ack"     # 对端确认
MSG_READY = "transport_ready"   # 双方准备完成（对齐队形结束）
MSG_GO    = "transport_go"      # 给出绝对起跑时刻
MSG_HB    = "heartbeat"
MSG_ESTOP = "emergency_stop"

class TransportNode(Node):
    def __init__(self, args):
        super().__init__('collab_transport_node_'+args.robot_id)
        self.robot_id = args.robot_id
        self.peer_id  = args.peer_id
        self.speed    = float(args.speed)
        self.baseline = float(args.baseline)
        self.particle = (float(args.particle[0]), float(args.particle[1]))
        self.target   = (float(args.target[0]), float(args.target[1]))

        # 规划队形
        self.phi, self.r1_pose, self.r2_pose, self.path_len = plan_formation(self.particle, self.target, self.baseline)
        # 本机器人是 r1 还是 r2
        self.is_r1 = (self.robot_id.lower().endswith("1"))

        # 通信
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.bus_pub = self.create_publisher(String, '/transport_coordination', qos)
        self.bus_sub = self.create_subscription(String, '/transport_coordination', self.on_bus, qos)

        # 运动控制（示例）
        self.motion = SimpleMotion(self, self.robot_id)

        # 状态
        self.have_ack = False
        self.peer_ready = False
        self.self_ready = False
        self.start_at = None
        self.transport_distance = self.path_len  # 中心到目标的距离
        self.last_hb = time.time()

        # 心跳定时器
        self.create_timer(1.0, self._heartbeat)

        # 启动主线程
        self.worker = threading.Thread(target=self._main, daemon=True)
        self.worker.start()

    # ====== 总流程 ======
    def _main(self):
        self.get_logger().info(f"🧭 Formation plan: phi={math.degrees(self.phi):.1f}°, baseline={self.baseline:.3f} m")
        self.get_logger().info(f"   r1 target pose=({self.r1_pose[0]:.2f},{self.r1_pose[1]:.2f},{math.degrees(self.r1_pose[2]):.1f}°)")
        self.get_logger().info(f"   r2 target pose=({self.r2_pose[0]:.2f},{self.r2_pose[1]:.2f},{math.degrees(self.r2_pose[2]):.1f}°)")
        self.get_logger().info(f"   path_len (center→T)={self.transport_distance:.2f} m")

        # 1) 发起握手
        self._publish(MSG_SYN, {"who": self.robot_id, "phi": self.phi, "baseline": self.baseline})
        # 等 ACK
        t0 = time.time()
        while not self.have_ack and time.time() - t0 < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        # 2) 对齐队形（示例：开环）
        yaw = self.r1_pose[2] if self.is_r1 else self.r2_pose[2]
        self.motion.rotate_to(yaw, ang_speed=0.6, max_time=1.2)
        # 简化：直达自己在队形中的位置（无里程计版本用占位的时间推进）
        self.motion.drive_distance(forward=True, dist=0.25, speed=0.08)  # 这里只是让车“挪一挪”，验证消息流；有里程计时请替换为真正的导航到 (x,y)

        self.self_ready = True
        self._publish(MSG_READY, {"who": self.robot_id})

        # 3) 等双方 ready 后，由 robot1 下发 GO（绝对时间）
        while not (self.self_ready and self.peer_ready):
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.is_r1:
            self.start_at = time.time() + 0.6  # 600ms 预留
            self._publish(MSG_GO, {"start_at": self.start_at, "distance": self.transport_distance, "speed": self.speed})
            self.get_logger().info(f"🚦 GO broadcast: start_at={self.start_at:.6f}")
        else:
            # 等 GO
            while self.start_at is None:
                rclpy.spin_once(self, timeout_sec=0.1)

        # 4) 微秒级等待：忙等至 start_at
        while True:
            now = time.time()
            remain = self.start_at - now
            if remain <= 0:
                break
            # 最后2ms 内忙等；更早时轻睡眠
            time.sleep(0.0005 if remain < 0.002 else 0.005)

        # 5) 同步运输直线段
        # robot1 前进(+v)，robot2 后退(-v)，全球向量同向，保持基线不变
        forward = True if self.is_r1 else False
        self.get_logger().info(f"🏁 START transport | forward={forward} | dist={self.transport_distance:.3f} m | v={self.speed:.3f} m/s")
        self.motion.drive_distance(forward=forward, dist=self.transport_distance, speed=self.speed)
        self.get_logger().info("✅ Transport done.")

    # ====== 总线处理 ======
    def on_bus(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        t = data.get("type")
        if t == MSG_SYN and data.get("who") != self.robot_id:
            # 回 ACK
            self._publish(MSG_ACK, {"who": self.robot_id})
        elif t == MSG_ACK and data.get("who") == self.peer_id:
            self.have_ack = True
            self.get_logger().info("🤝 got ACK")
        elif t == MSG_READY and data.get("who") == self.peer_id:
            self.peer_ready = True
            self.get_logger().info("✅ peer READY")
        elif t == MSG_GO:
            self.start_at = float(data["start_at"])
            self.transport_distance = float(data["distance"])
            self.speed = float(data["speed"])
            self.get_logger().info(f"📨 GO received: start_at={self.start_at:.6f}")
        elif t == MSG_ESTOP:
            self.get_logger().warning("🚨 E-STOP received!")
            self.motion.stop()

    def _publish(self, typ, payload):
        payload = {"type": typ, **payload, "ts": time.time()}
        out = String()
        out.data = json.dumps(payload)
        self.bus_pub.publish(out)

    def _heartbeat(self):
        hb = {"type": MSG_HB, "who": self.robot_id, "ts": time.time()}
        out = String(); out.data = json.dumps(hb)
        self.bus_pub.publish(out)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot_id", required=True, help="robot1 / robot2（用于决定前进/后退）")
    parser.add_argument("--peer_id", required=True)
    parser.add_argument("--particle", nargs=2, required=True, metavar=("PX","PY"), help="微粒点坐标（m）")
    parser.add_argument("--target", nargs=2, required=True, metavar=("TX","TY"), help="目标点坐标（m）")
    parser.add_argument("--baseline", type=float, default=0.38, help="两车中心间距（m）")
    parser.add_argument("--speed", type=float, default=0.06, help="运输线速度（m/s）")
    args = parser.parse_args()

    rclpy.init()
    node = TransportNode(args)
    try:
        rclpy.spin(node)  # 主线程在 _main，spin 仅保持 ROS 生命周期
    except KeyboardInterrupt:
        pass
    finally:
        node.motion.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
