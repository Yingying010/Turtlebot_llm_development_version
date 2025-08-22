#!/usr/bin/env python3

# collect.py —— 独立导航 + 语音反馈 + 模拟收集
 
import os, sys, time, math
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy._rclpy_pybind11 import InvalidHandle
from typing import Dict, List, Tuple, Optional
from geometry_msgs.msg import Twist
import threading
# 添加项目根路径
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import config
from ttsRepo.stream_tts import tts_manager
from phasespace.rigid_tracker import RigidTracker
from rclpy.executors import MultiThreadedExecutor
from robotControllerRepo.actions.navigate import navigate_to_target
from robotControllerRepo.utils.coord_convert import resolve_object_position

semantic_locations = config.get("semantic_locations")

# === 执行 collect 行为 ===
def collect_item(node: Node, robot_name: str, item: str, target, executor):
    print(f"\n🚗 Starting collect_item for {robot_name}")
    print(f"📦 Item: {item}")
    print(f"🎯 Target: {target} (type: {type(target).__name__})")
    
    if isinstance(target, str):
        print(f"🔍 Target is blackboard key, resolving position...")
        
        # 获取机器人位置
        from robotControllerRepo.actions.navigate import get_current_position
        robot_x, robot_y, robot_heading = get_current_position(robot_name)
        robot_pos = {"x": robot_x, "y": robot_y, "heading_y": robot_heading}
        print(f"📍 Robot position: ({robot_x:.2f}, {robot_y:.2f}, {robot_heading:.1f}°)")
        
        # 🔥 解析物体位置
        map_position = resolve_object_position(robot_name, target, robot_pos)
        if map_position:
            print(f"✅ Object position resolved:")
            print(f"   📍 Map coordinates: ({map_position['x']:.2f}, {map_position['y']:.2f})")
            print(f"   📏 Estimated distance: {map_position.get('estimated_distance', 0):.2f}m")
            print(f"   🎯 Confidence: {map_position.get('confidence', 0):.2f}")
            target = map_position
        else:
            print(f"❌ Failed to resolve position for '{target}'")
            print(f"💡 Make sure blackboard data exists for this object")
            return False

    # === 阶段 1：导航到目标位置 ===
    print(f"\n🧭 Phase 1: Navigation to target")
    is_successful = navigate_to_target(node, executor, robot_name, target)
    
    if not is_successful:
        print(f"❌ Navigation failed!")
        return False
        
    print(f"✅ Navigation completed! Reached target location")
    print(f"🎯 Ready to collect {item}...")
 
    # === 阶段 2：收集 ===
    print(f"\n📦 Phase 2: Collection process")
    print(f"🗣️ Announcing collection...")
    tts_manager.say(f"I am collecting {item}")
    
    print(f"⏳ Collecting {item}... (10 seconds)")
    time.sleep(10)  # 模拟收集时间
    
    print(f"✅ Collection completed!")
    tts_manager.say(f"{item} collected successfully")

    return True

# === 🧪 独立测试主函数 ===
def run_collect_test():
    """运行 collect 独立测试"""
    print("🤖 COLLECT.PY STANDALONE TEST")
    print("=" * 50)
    
    # 解析命令行参数
    robot_name = sys.argv[1] if len(sys.argv) > 1 else "robot1"
    item_name = sys.argv[2] if len(sys.argv) > 2 else "cup"
    target_key = sys.argv[3] if len(sys.argv) > 3 else f"{item_name}_target"
    
    print(f"🎯 Test Parameters:")
    print(f"   🤖 Robot: {robot_name}")
    print(f"   📦 Item: {item_name}")
    print(f"   🔑 Blackboard key: {target_key}")
    print("")
    
    # 检查黑板数据是否存在
    bb_file = f"/tmp/robot_blackboard_{robot_name}.json"
    if not os.path.exists(bb_file):
        print(f"⚠️ Warning: No blackboard data found!")
        print(f"📂 Expected file: {bb_file}")
        print(f"💡 Suggestion: Run find.py first to generate blackboard data:")
        print(f"   python3 robotControllerRepo/actions/find.py {robot_name} {item_name}")
        print("")
        
        user_input = input("Continue anyway? (y/N): ").lower().strip()
        if user_input != 'y':
            print("🛑 Test cancelled")
            return False
    else:
        print(f"✅ Blackboard data found: {bb_file}")
    
    try:
        # 初始化ROS
        print(f"\n🚀 Initializing ROS...")
        rclpy.init()
        
        # 创建节点和执行器
        node = rclpy.create_node(f'collect_test_{robot_name}')
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        # 启动执行器线程
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        print(f"✅ ROS initialized successfully")
        
        print(f"\n⚠️ Make sure your robot is ready!")
        input("Press Enter when robot is ready, or Ctrl+C to cancel...")
        
        # 🔥 执行真实的collect测试
        print(f"\n🧪 Starting collect test...")
        result = collect_item(
            node=node,
            robot_name=robot_name,
            item=item_name,
            target=target_key,  # 使用黑板key
            executor=executor
        )
        
        # 显示结果
        print(f"\n" + "=" * 50)
        print(f"🎉 COLLECT TEST RESULT:")
        print(f"   ✅ Success: {result}")
        print(f"   🤖 Robot: {robot_name}")
        print(f"   📦 Item: {item_name}")
        print(f"   🔑 Target: {target_key}")
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
            print(f"\n🔚 Cleaning up ROS resources...")
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()
            print(f"✅ ROS cleanup completed")
        except:
            pass

# === 主入口 ===
if __name__ == "__main__":
    print("🤖 Collect.py Standalone Test")
    print("Usage: python3 collect.py [robot_name] [item_name] [blackboard_key]")
    print("")
    print("Examples:")
    print("  python3 collect.py                    # Test robot1 collecting cup from 'cup_target'")
    print("  python3 collect.py robot1 bottle      # Test robot1 collecting bottle from 'bottle_target'")
    print("  python3 collect.py robot1 cup my_cup  # Test robot1 collecting cup from 'my_cup' key")
    print("")
    print("Prerequisites:")
    print("  1. Run find.py first to generate blackboard data")
    print("  2. Make sure robot is ready and ROS topics are active")
    print("")
    
    success = run_collect_test()
    exit(0 if success else 1)