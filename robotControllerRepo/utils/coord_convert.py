# coordinate_utils.py
# -*- coding: utf-8 -*-
"""
独立的坐标转换工具模块
- 像素坐标 → 地图坐标转换
- 距离估计
- 物体位置解析
- 可独立测试和调试
"""

import os
import sys
import json
import math
import time
from typing import Dict, List, Tuple, Optional

# 简单日志
class SimpleLogger:
    def info(self, msg): print(f"[INFO] {msg}")
    def warning(self, msg): print(f"[WARN] {msg}")
    def error(self, msg): print(f"[ERROR] {msg}")
    def debug(self, msg): print(f"[DEBUG] {msg}")

logger = SimpleLogger()

# -------- 物体尺寸数据库 --------
OBJECT_SIZE_DATABASE = {
    # 日常小物品（厘米）
    "cup": {"width": 8, "height": 10},
    "bottle": {"width": 6, "height": 20},
    "phone": {"width": 7, "height": 15},
    "remote": {"width": 5, "height": 18},
    "mouse": {"width": 6, "height": 10},
    "keys": {"width": 8, "height": 3},
    
    # 中等物品
    "book": {"width": 15, "height": 20},
    "laptop": {"width": 35, "height": 25},
    "keyboard": {"width": 45, "height": 15},
    "tablet": {"width": 25, "height": 18},
    
    # 大物品
    "chair": {"width": 50, "height": 80},
    "table": {"width": 120, "height": 75},
    "person": {"width": 40, "height": 170},
    "sofa": {"width": 180, "height": 85},
    "bed": {"width": 200, "height": 50},
    
    # 默认值
    "unknown": {"width": 10, "height": 10}
}

# -------- 距离估计 --------
def estimate_distance_from_bbox(bbox_xyxy: List[float], 
                               object_class: str,
                               image_width: int = 640,
                               image_height: int = 480) -> float:
    """
    根据边界框大小估计物体距离
    
    Args:
        bbox_xyxy: [x1, y1, x2, y2] 边界框坐标
        object_class: 物体类别
        image_width: 图像宽度
        image_height: 图像高度
    
    Returns:
        估计的距离（米）
    """
    x1, y1, x2, y2 = bbox_xyxy
    bbox_width = x2 - x1
    bbox_height = y2 - y1
    
    # 获取物体真实尺寸
    size_info = OBJECT_SIZE_DATABASE.get(object_class, OBJECT_SIZE_DATABASE["unknown"])
    real_width_cm = size_info["width"]
    real_height_cm = size_info["height"]
    
    # 计算像素尺寸占图像的比例
    width_ratio = bbox_width / image_width
    height_ratio = bbox_height / image_height
    
    # 使用更准确的距离估计公式
    # 基于相似三角形原理：distance = (real_size * focal_length) / pixel_size
    # 这里使用经验公式，focal_length ≈ image_width / (2 * tan(fov/2))
    # 假设相机水平视野角为60度
    fov_rad = math.radians(60)
    focal_length_pixels = image_width / (2 * math.tan(fov_rad / 2))
    
    # 使用宽度估计距离（通常比高度更准确）
    distance_from_width = (real_width_cm / 100) * focal_length_pixels / bbox_width
    distance_from_height = (real_height_cm / 100) * focal_length_pixels / bbox_height
    
    # 取两个估计的平均值
    estimated_distance = (distance_from_width + distance_from_height) / 2
    
    # 🔥 添加经验校准系数（基于实际测试调整）
    # 如果估计距离偏小，增大这个系数
    CALIBRATION_FACTOR = 2.5  # 将0.4m校准到1.0m: 1.0/0.4 = 2.5
    estimated_distance *= CALIBRATION_FACTOR
    
    # 限制距离范围：0.2m - 5.0m
    estimated_distance = max(0.2, min(5.0, estimated_distance))
    
    logger.debug(f"Distance estimation for {object_class}:")
    logger.debug(f"  - Bbox: {bbox_width:.1f}x{bbox_height:.1f} pixels")
    logger.debug(f"  - Real size: {real_width_cm}x{real_height_cm} cm")
    logger.debug(f"  - Raw distance: {estimated_distance/CALIBRATION_FACTOR:.2f}m")
    logger.debug(f"  - Calibrated distance: {estimated_distance:.2f}m")
    
    return estimated_distance

# -------- 坐标转换 --------
def pixel_to_map_coordinates(pixel_xy: List[float], 
                           robot_position: Dict[str, float],
                           estimated_distance: float,
                           camera_params: Optional[Dict] = None) -> Dict[str, float]:
    """
    将像素坐标转换为地图坐标
    
    Args:
        pixel_xy: [center_x, center_y] 在图像中的像素坐标
        robot_position: {"x": robot_x, "y": robot_y, "heading_y": robot_heading_deg}
        estimated_distance: 估计的物体距离（米）
        camera_params: 摄像头参数字典
    
    Returns:
        {"x": map_x, "y": map_y, "heading_deg": None} 地图坐标
    """
    # 默认摄像头参数
    if camera_params is None:
        camera_params = {
            "image_width": 640,
            "image_height": 480,
            "fov_horizontal_deg": 60,  # 水平视野角
            "camera_offset_x": 0,      # 摄像头相对机器人中心的x偏移
            "camera_offset_y": 0       # 摄像头相对机器人中心的y偏移
        }
    
    center_x, center_y = pixel_xy
    robot_x = robot_position["x"]
    robot_y = robot_position["y"] 
    robot_heading_deg = robot_position["heading_y"]
    robot_heading_rad = math.radians(robot_heading_deg)
    
    # 摄像头参数
    image_width = camera_params["image_width"]
    image_height = camera_params["image_height"]
    fov_horizontal_rad = math.radians(camera_params["fov_horizontal_deg"])
    
    # 计算物体相对于图像中心的角度偏移
    center_offset_x = center_x - (image_width / 2)
    angle_per_pixel = fov_horizontal_rad / image_width
    relative_angle_rad = center_offset_x * angle_per_pixel
    
    # 物体在全局坐标系中的角度
    object_angle_global_rad = robot_heading_rad + relative_angle_rad
    
    # 考虑摄像头偏移（如果有）
    camera_x = robot_x + camera_params["camera_offset_x"]
    camera_y = robot_y + camera_params["camera_offset_y"]
    
    # 计算物体的全局坐标
    # 注意：这里假设y轴指向北，x轴指向东
    object_x = camera_x + estimated_distance * math.sin(object_angle_global_rad)
    object_y = camera_y + estimated_distance * math.cos(object_angle_global_rad)
    
    result = {
        "x": object_x,
        "y": object_y,
        "heading_deg": None
    }
    
    logger.debug(f"Coordinate conversion:")
    logger.debug(f"  - Pixel: ({center_x:.1f}, {center_y:.1f})")
    logger.debug(f"  - Robot pos: ({robot_x:.2f}, {robot_y:.2f}, {robot_heading_deg:.1f}°)")
    logger.debug(f"  - Distance: {estimated_distance:.2f}m")
    logger.debug(f"  - Relative angle: {math.degrees(relative_angle_rad):.1f}°")
    logger.debug(f"  - Map coordinates: ({object_x:.2f}, {object_y:.2f})")
    
    return result

# -------- 黑板读取 --------
def _bb_path(robot_name: str) -> str:
    return f"/tmp/robot_blackboard_{robot_name}.json"

def bb_get(robot_name: str, key: str) -> Optional[Dict]:
    """从黑板读取物体信息"""
    bb_file = _bb_path(robot_name)
    if not os.path.exists(bb_file):
        logger.warning(f"Blackboard file not found: {bb_file}")
        return None
    
    try:
        with open(bb_file, "r") as f:
            data = json.load(f)
        
        objects = data.get("objects", {})
        return objects.get(key)
        
    except Exception as e:
        logger.error(f"Failed to read blackboard: {e}")
        return None

# -------- 主要接口函数 --------
def resolve_object_position(robot_name: str, 
                          blackboard_key: str,
                          robot_position: Dict[str, float],
                          camera_params: Optional[Dict] = None) -> Optional[Dict[str, float]]:
    """
    从黑板解析物体的地图位置
    
    Args:
        robot_name: 机器人名称
        blackboard_key: 黑板中的物体key
        robot_position: 当前机器人位置
        camera_params: 摄像头参数（可选）
    
    Returns:
        物体的地图坐标 {"x": float, "y": float, "heading_deg": None} 或 None
    """
    logger.info(f"🔍 Resolving object position: {blackboard_key}")
    
    # 从黑板读取物体信息
    object_info = bb_get(robot_name, blackboard_key)
    if not object_info:
        logger.error(f"❌ Object {blackboard_key} not found in blackboard")
        return None
    
    logger.info(f"📦 Object info: {object_info}")
    
    # 检查必要的字段
    required_fields = ["center_xy", "bbox_xyxy", "class"]
    missing_fields = [f for f in required_fields if f not in object_info]
    if missing_fields:
        logger.error(f"❌ Missing fields in object info: {missing_fields}")
        return None
    
    # 估计距离
    estimated_distance = estimate_distance_from_bbox(
        object_info["bbox_xyxy"],
        object_info["class"]
    )
    
    # 转换坐标
    map_position = pixel_to_map_coordinates(
        object_info["center_xy"],
        robot_position,
        estimated_distance,
        camera_params
    )
    
    # 添加额外信息
    map_position["estimated_distance"] = estimated_distance
    map_position["confidence"] = object_info.get("conf", 0.0)
    map_position["object_class"] = object_info["class"]
    map_position["timestamp"] = object_info.get("timestamp", time.time())
    
    logger.info(f"✅ Position resolved: ({map_position['x']:.2f}, {map_position['y']:.2f})")
    return map_position

# -------- 测试和调试功能 --------
def test_coordinate_conversion():
    """测试坐标转换功能"""
    print("🧪 Testing coordinate conversion...")
    
    # 模拟数据
    robot_pos = {"x": 0.0, "y": 0.0, "heading_y": 0.0}  # 机器人在原点，朝北
    pixel_coords = [320, 240]  # 图像中心
    distance = 1.0  # 1米距离
    
    # 测试1：正前方物体
    result1 = pixel_to_map_coordinates(pixel_coords, robot_pos, distance)
    print(f"Test 1 - Front center: {result1}")
    # 预期：(0.0, 1.0) - 正前方1米
    
    # 测试2：右侧物体
    pixel_coords_right = [480, 240]  # 图像右侧
    result2 = pixel_to_map_coordinates(pixel_coords_right, robot_pos, distance)
    print(f"Test 2 - Right side: {result2}")
    # 预期：x > 0 - 右侧
    
    # 测试3：机器人旋转90度朝东
    robot_pos_east = {"x": 0.0, "y": 0.0, "heading_y": 90.0}
    result3 = pixel_to_map_coordinates(pixel_coords, robot_pos_east, distance)
    print(f"Test 3 - Robot facing east: {result3}")
    # 预期：(1.0, 0.0) - 东方1米

def test_distance_estimation():
    """测试距离估计功能"""
    print("🧪 Testing distance estimation...")
    
    # 测试不同物体的距离估计
    test_objects = [
        {"bbox": [200, 200, 280, 260], "class": "cup"},      # 小杯子
        {"bbox": [100, 100, 200, 300], "class": "bottle"},   # 瓶子
        {"bbox": [50, 50, 350, 400], "class": "person"},     # 人
    ]
    
    for obj in test_objects:
        distance = estimate_distance_from_bbox(obj["bbox"], obj["class"])
        print(f"  {obj['class']}: {distance:.2f}m")

def test_blackboard_integration():
    """测试黑板集成功能"""
    print("🧪 Testing blackboard integration...")
    
    # 创建测试黑板数据
    test_data = {
        "objects": {
            "test_cup": {
                "class": "cup",
                "center_xy": [350, 250],
                "bbox_xyxy": [300, 200, 400, 300],
                "conf": 0.85,
                "timestamp": time.time()
            }
        }
    }
    
    # 写入测试数据
    bb_file = "/tmp/robot_blackboard_test_robot.json"
    with open(bb_file, "w") as f:
        json.dump(test_data, f)
    
    # 测试解析
    robot_pos = {"x": 1.0, "y": 2.0, "heading_y": 45.0}
    result = resolve_object_position("test_robot", "test_cup", robot_pos)
    
    if result:
        print(f"✅ Blackboard test successful: {result}")
    else:
        print("❌ Blackboard test failed")
    
    # 清理测试文件
    if os.path.exists(bb_file):
        os.remove(bb_file)

# -------- 主函数 --------
if __name__ == "__main__":
    print("🔧 Coordinate Utils Test Suite")
    print("=" * 50)
    
    print("\n1. Testing coordinate conversion...")
    test_coordinate_conversion()
    
    print("\n2. Testing distance estimation...")
    test_distance_estimation()
    
    print("\n3. Testing blackboard integration...")
    test_blackboard_integration()
    
    print("\n✅ All tests completed!")
    print("\nUsage examples:")
    print("  from coordinate_utils import resolve_object_position")
    print("  position = resolve_object_position('robot1', 'cup_target', robot_pos)")