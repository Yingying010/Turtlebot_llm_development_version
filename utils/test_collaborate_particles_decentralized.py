#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Collaborative Transport with PhaseSpace (using navigate.py)

流程:
  Phase1: 根据微粒点与目标点计算两车编队位置和整体朝向
  Phase2: 两车分别调用 navigate.py 导航到编队位置并调整角度
  Phase3: 两车背靠背同步直线运输，robot1 前进、robot2 后退

运行:
  robot1:
    python3 test_collaborate_particles_decentralized.py --robot_id robot1
  robot2:
    python3 test_collaborate_particles_decentralized.py --robot_id robot2
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import math, time, json, argparse, threading

# === 导入你现有的导航函数 ===
from navigate import navigate_to_target

# === 全局参数（直接改这里就行） ===
PARTICLE = (0.0, 0.0)     # 微粒点
TARGET   = (5.0, -5.0)    # 目标点
BASELINE = 0.38           # 两车间距
SPEED    = 0.06           # 运输速度

# ======= 工具函数 =======
def plan_formation(particle_xy, target_xy, baseline):
    px, py = particle_xy
    tx, ty = target_xy
    phi = math.atan2(ty - py, tx - px)
    ux, uy = math.cos(phi), math.sin(phi)
    half = 0.5 * baseline
    r1 = (px - half*ux, py - half*uy, phi)
    r2 = (px + half*ux, py + half*uy, phi + math.pi)
    path_len = math.hypot(tx - px, ty - py)
    return phi, r1, r2, path_len

# ======= 简单直行控制 =======
class Motion:
    def __init__(self, node: Node, robot_ns: str):
        self.node = node
        self.pub = node.create_publisher(Twist, f'/{robot_ns}/cmd_vel', 10)

    def stop(self):
        self.pub.publish(Twist())

    def drive_constant(self, forward: bool, dist: float, speed: float):
        dur = dist / max(speed,1e-6)
        tw = Twist(); tw.linear.x = speed if forward else -speed
        t0 = time.time()
        while time.time()-t0 < dur:
            self.pub.publish(tw)
            time.sleep(0.02)
        self.stop()

# ======= 协调协议 =======
MSG_SYN,MSG_ACK,MSG_READY,MSG_GO="syn","ack","ready","go"

class TransportManager:
    """协作运输管理器，依赖外部 node"""
    def __init__(self, node: Node, robot_id: str):
        self.node = node
        self.robot_id = robot_id
        self.peer_id = "robot1" if robot_id == "robot2" else "robot2"
        self.motion = Motion(node, robot_id)

        # Phase1: 队形规划
        self.phi,self.r1,self.r2,self.path_len = plan_formation(PARTICLE,TARGET,BASELINE)
        self.is_r1 = (robot_id=="robot1")

        # 通信
        qos = QoSProfile(depth=10)
        qos.reliability=ReliabilityPolicy.RELIABLE
        self.pub = node.create_publisher(String,'/transport_coordination',qos)
        self.sub = node.create_subscription(String,'/transport_coordination',self.on_bus,qos)

        # 状态
        self.have_ack=False; self.peer_ready=False; self.self_ready=False; self.start_at=None

        # 开启主流程线程
        threading.Thread(target=self._main,daemon=True).start()

    # ===== 主流程 =====
    def _main(self):
        self.node.get_logger().info(f"🧭 Formation phi={math.degrees(self.phi):.1f}° | path={self.path_len:.2f}m")

        # Phase1: 握手
        self._publish(MSG_SYN,{"who":self.robot_id})
        t0=time.time()
        while not self.have_ack and time.time()-t0<3: rclpy.spin_once(self.node,0.1)

        # Phase2: 导航到编队位置 (调用 navigate.py)
        if self.is_r1:
            x_goal, y_goal, yaw_goal = self.r1
        else:
            x_goal, y_goal, yaw_goal = self.r2

        target = {"x": x_goal, "y": y_goal, "heading_deg": math.degrees(yaw_goal)}
        self.node.get_logger().info(f"🚗 {self.robot_id} navigating to {target} ...")
        navigate_to_target(self.node, None, self.robot_id, target)  # executor传None，因为你集成时会传入controller的executor

        self.self_ready=True; self._publish(MSG_READY,{"who":self.robot_id})
        while not(self.self_ready and self.peer_ready): rclpy.spin_once(self.node,0.1)

        # Phase3: 同步运输
        if self.is_r1:
            self.start_at=time.time()+0.6
            self._publish(MSG_GO,{"start_at":self.start_at,"dist":self.path_len,"speed":SPEED})
        else:
            while self.start_at is None: rclpy.spin_once(self.node,0.1)

        while self.start_at>time.time(): time.sleep(0.001)
        forward=self.is_r1
        self.node.get_logger().info(f"🏁 START transport | forward={forward} | dist={self.path_len:.2f}m | v={SPEED}")
        self.motion.drive_constant(forward,self.path_len,SPEED)
        self.node.get_logger().info("✅ Transport done")

    # ===== 总线处理 =====
    def on_bus(self,msg:String):
        try: d=json.loads(msg.data)
        except: return
        t=d.get("type"); who=d.get("who")
        if t==MSG_SYN and who!=self.robot_id: self._publish(MSG_ACK,{"who":self.robot_id})
        elif t==MSG_ACK and who==self.peer_id: self.have_ack=True
        elif t==MSG_READY and who==self.peer_id: self.peer_ready=True
        elif t==MSG_GO: self.start_at=d["start_at"]; self.path_len=d["dist"]

    def _publish(self,typ,payload):
        out=String(); out.data=json.dumps({"type":typ,**payload,"ts":time.time()})
        self.pub.publish(out)

# ===== 主入口 =====
def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--robot_id",required=True,choices=["robot1","robot2"])
    args=parser.parse_args()

    rclpy.init()
    node = Node("collab_transport")
    manager = TransportManager(node, args.robot_id)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        manager.motion.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__": main()
