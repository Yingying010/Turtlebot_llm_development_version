#!/usr/bin/env python3

# deliver.py —— 独立导航 + 语音反馈 + 模拟送达 (修复版本)
 
import os, sys, time, math
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy._rclpy_pybind11 import InvalidHandle
from typing import Dict, List, Tuple, Optional
from geometry_msgs.msg import Twist
import threading
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import config
from ttsRepo.stream_tts import tts_manager
from phasespace.rigid_tracker import RigidTracker
from rclpy.executors import MultiThreadedExecutor
from robotControllerRepo.actions.navigate import navigate_to_target

semantic_locations = config.get("semantic_locations")

# === 执行 deliver 行为 ===
def deliver_item(node: Node, robot_name: str, item: str, executor):
    """
    送达物品 - 自动导航到主人位置并送达
    
    Args:
        node: ROS节点
        robot_name: 机器人名称  
        item: 要送达的物品名称
        executor: ROS执行器
    """
    print(f"\n🚚 Starting deliver_item for {robot_name}")
    print(f"📦 Item: {item}")
    
    # === 自动解析送达目标（默认送给主人） ===
    master_name = config.get("master_id", "master")
    print(f"🎯 Auto-resolving delivery target: {master_name}")
    
    # 检查主人是否在语义位置中定义
    target = None
    if master_name in semantic_locations:
        target = semantic_locations[master_name]
        print(f"📍 Found master location in semantic_locations: {target}")
    else:
        # 默认位置 - 主人位置
        target = {"x": 500, "y": 500, "heading_deg": None}
        print(f"⚠️ Master location not found, using default: {target}")
    
    # === 阶段 1：导航到目标位置 ===
    print(f"\n🧭 Phase 1: Navigation to delivery target")
    is_successful = navigate_to_target(node, executor, robot_name, target)
    
    if not is_successful:
        print(f"❌ Navigation failed!")
        tts_manager.say(f"Sorry, I could not reach the delivery location")
        return False
        
    print(f"✅ Navigation completed! Reached delivery location")
    print(f"🎯 Ready to deliver {item}...")
 
    # === 阶段 2：送达 ===
    print(f"\n📦 Phase 2: Delivery process")
    print(f"🗣️ Speaking: I am delivering {item}")
    tts_manager.say(f"I am delivering {item}")
    
    print(f"⏳ Delivering {item}... (10 seconds)")
    time.sleep(10)  # 模拟送达时间
    
    print(f"✅ Delivery completed!")
    tts_manager.say(f"{item} delivered successfully")
    
    return True

# === 🧪 独立测试主函数 ===
def run_deliver_test():
    """运行 deliver 独立测试"""
    print("🤖 DELIVER.PY STANDALONE TEST")
    print("=" * 50)
    
    # 解析命令行参数
    robot_name = sys.argv[1] if len(sys.argv) > 1 else "robot1"
    item_name = sys.argv[2] if len(sys.argv) > 2 else "bottle"
    
    print(f"🎯 Test Parameters:")
    print(f"   🤖 Robot: {robot_name}")
    print(f"   📦 Item: {item_name}")
    print("")
    
    try:
        # 初始化ROS
        print(f"\n🚀 Initializing ROS...")
        rclpy.init()
        
        # 创建节点和执行器
        node = rclpy.create_node(f'deliver_test_{robot_name}')
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        # 启动执行器线程
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        print(f"✅ ROS initialized successfully")
        
        print(f"\n⚠️ Make sure your robot is ready!")
        input("Press Enter when robot is ready, or Ctrl+C to cancel...")
        
        # 🔥 执行真实的deliver测试
        print(f"\n🧪 Starting deliver test...")
        result = deliver_item(
            node=node,
            robot_name=robot_name,
            item=item_name,
            executor=executor
        )
        
        # 显示结果
        print(f"\n" + "=" * 50)
        print(f"🎉 DELIVER TEST RESULT:")
        print(f"   ✅ Success: {result}")
        print(f"   🤖 Robot: {robot_name}")
        print(f"   📦 Item: {item_name}")
        print("=" * 50)
        
        return result
        
    except KeyboardInterrupt:
        print(f"\n🛑 Test cancelled by user")
        return False
        
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        return False
        
    finally:
        # 清理ROS资源
        try:
            print(f"\n📚 Cleaning up ROS resources...")
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()
            print(f"✅ ROS cleanup completed")
        except:
            pass

# === 主入口 ===
if __name__ == "__main__":
    print("🤖 Deliver.py Standalone Test")
    print("Usage: python3 deliver.py [robot_name] [item_name]")
    print("")
    print("Examples:")
    print("  python3 deliver.py                    # Test robot1 delivering bottle")
    print("  python3 deliver.py robot1 cup         # Test robot1 delivering cup") 
    print("  python3 deliver.py robot2 book        # Test robot2 delivering book")
    print("")
    print("Auto-delivery logic:")
    print("  - Automatically delivers to master's location from config")
    print("  - Falls back to default location if master not in semantic_locations")
    print("")
    print("Prerequisites:")
    print("  1. Make sure robot is ready and ROS topics are active")
    print("  2. Configure master_id in config.json")
    print("")
    
    success = run_deliver_test()
    exit(0 if success else 1)