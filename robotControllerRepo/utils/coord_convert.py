import os
import sys
import json
import math
import time
from typing import Dict, List, Tuple, Optional

class SimpleLogger:
    def info(self, msg): print(f"[INFO] {msg}")
    def warning(self, msg): print(f"[WARN] {msg}")
    def error(self, msg): print(f"[ERROR] {msg}")
    def debug(self, msg): print(f"[DEBUG] {msg}")

logger = SimpleLogger()

OBJECT_SIZE_DATABASE = {
    "cup": {"width": 8, "height": 10},
    "bottle": {"width": 5, "height": 16},
    "phone": {"width": 7, "height": 15},
    "remote": {"width": 5, "height": 18},
    "mouse": {"width": 6, "height": 10},
    "keys": {"width": 8, "height": 3},

    "book": {"width": 15, "height": 20},
    "laptop": {"width": 35, "height": 25},
    "keyboard": {"width": 45, "height": 15},
    "tablet": {"width": 25, "height": 18},

    "chair": {"width": 50, "height": 80},
    "table": {"width": 120, "height": 75},
    "person": {"width": 40, "height": 170},
    "sofa": {"width": 180, "height": 85},
    "bed": {"width": 200, "height": 50},

    "unknown": {"width": 10, "height": 10}
}

def estimate_distance_from_bbox(bbox_xyxy: List[float], 
                               object_class: str,
                               image_width: int = 640,
                               image_height: int = 480) -> float:

    x1, y1, x2, y2 = bbox_xyxy
    bbox_width = x2 - x1
    bbox_height = y2 - y1

    size_info = OBJECT_SIZE_DATABASE.get(object_class, OBJECT_SIZE_DATABASE["unknown"])
    real_width_cm = size_info["width"]
    real_height_cm = size_info["height"]

    width_ratio = bbox_width / image_width
    height_ratio = bbox_height / image_height

    fov_rad = math.radians(60)
    focal_length_pixels = image_width / (2 * math.tan(fov_rad / 2))
 
    distance_from_width = (real_width_cm / 100) * focal_length_pixels / bbox_width
    distance_from_height = (real_height_cm / 100) * focal_length_pixels / bbox_height

    estimated_distance = (distance_from_width + distance_from_height) / 2

    CALIBRATION_FACTOR = 3.2
    estimated_distance *= CALIBRATION_FACTOR

    estimated_distance = max(0.2, min(5.0, estimated_distance))
    
    logger.debug(f"Distance estimation for {object_class}:")
    logger.debug(f"  - Bbox: {bbox_width:.1f}x{bbox_height:.1f} pixels")
    logger.debug(f"  - Real size: {real_width_cm}x{real_height_cm} cm")
    logger.debug(f"  - Raw distance: {estimated_distance/CALIBRATION_FACTOR:.2f}m")
    logger.debug(f"  - Calibrated distance: {estimated_distance:.2f}m")
    
    return estimated_distance

def pixel_to_map_coordinates(pixel_xy: List[float], 
                           robot_position: Dict[str, float],
                           estimated_distance: float,
                           camera_params: Optional[Dict] = None) -> Dict[str, float]:

    if camera_params is None:
        camera_params = {
            "image_width": 640,
            "image_height": 480,
            "fov_horizontal_deg": 62.2,
            "camera_offset_x": 0.0,
            "camera_offset_y": 0.0
        }
    
    center_x, center_y = pixel_xy
    robot_x = robot_position["x"]
    robot_y = robot_position["y"] 
    robot_heading_deg = robot_position["heading_y"]
    robot_heading_rad = math.radians(robot_heading_deg)

    image_width = camera_params["image_width"]
    image_height = camera_params["image_height"]
    fov_horizontal_rad = math.radians(camera_params["fov_horizontal_deg"])

    center_offset_x = center_x - (image_width / 2)
    angle_per_pixel = fov_horizontal_rad / image_width

    relative_angle_rad = -center_offset_x * angle_per_pixel

    object_angle_global_rad = robot_heading_rad + relative_angle_rad
 
    camera_x = robot_x + camera_params["camera_offset_x"]
    camera_y = robot_y + camera_params["camera_offset_y"]

    object_x = camera_x + estimated_distance * math.sin(object_angle_global_rad)
    object_y = camera_y + estimated_distance * math.cos(object_angle_global_rad)
    
    result = {
        "x": object_x,
        "y": object_y,
        "heading_deg": None
    }
    
    logger.debug(f"🔧 FIXED Coordinate conversion:")
    logger.debug(f"  - Pixel: ({center_x:.1f}, {center_y:.1f})")
    logger.debug(f"  - Image center: ({image_width/2}, {image_height/2})")
    logger.debug(f"  - Pixel offset: {center_offset_x:.1f} (negative=left, positive=right)")
    logger.debug(f"  - Robot pos: ({robot_x:.2f}, {robot_y:.2f}, {robot_heading_deg:.1f}°)")
    logger.debug(f"  - Distance: {estimated_distance:.2f}m")
    logger.debug(f"  - Relative angle: {math.degrees(relative_angle_rad):.1f}° (positive=left turn, negative=right turn)")
    logger.debug(f"  - Global angle: {math.degrees(object_angle_global_rad):.1f}°")
    logger.debug(f"  - Map coordinates: ({object_x:.2f}, {object_y:.2f})")
    
    return result

def _bb_path(robot_name: str) -> str:
    return f"robot_blackboard_{robot_name}.json"

def bb_get(robot_name: str, key: str) -> Optional[Dict]:
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

def resolve_object_position(robot_name: str, 
                          blackboard_key: str,
                          robot_position: Dict[str, float],
                          camera_params: Optional[Dict] = None) -> Optional[Dict[str, float]]:
    logger.info(f"resolve the position in blackboard: {blackboard_key}")

    object_info = bb_get(robot_name, blackboard_key)
    if not object_info:
        logger.error(f"can't find object in blackboard {blackboard_key}")
        return None
    
    logger.info(f"object info: {object_info}")

    required_fields = ["center_xy", "bbox_xyxy", "class"]
    missing_fields = [f for f in required_fields if f not in object_info]
    if missing_fields:
        logger.error(f"miss field: {missing_fields}")
        return None

    bbox_xyxy = object_info["bbox_xyxy"]
    center_xy = object_info["center_xy"]

    max_x = max(bbox_xyxy[2], center_xy[0])
    max_y = max(bbox_xyxy[3], center_xy[1])
    
    logger.info(f"original max_x={max_x:.1f}, max_y={max_y:.1f}")

    detected_width, detected_height = 640, 480

    if camera_params is None:
        camera_params = {}
    
    camera_params.update({
        "image_width": detected_width,
        "image_height": detected_height,
        "fov_horizontal_deg": camera_params.get("fov_horizontal_deg", 62.2),
        "camera_offset_x": camera_params.get("camera_offset_x", 0.0),
        "camera_offset_y": camera_params.get("camera_offset_y", 0.0)
    })
 
    estimated_distance = estimate_distance_from_bbox(
        object_info["bbox_xyxy"],
        object_info["class"],
        image_width=detected_width,
        image_height=detected_height
    )

    map_position = pixel_to_map_coordinates(
        object_info["center_xy"],
        robot_position,
        estimated_distance,
        camera_params
    )

    map_position["estimated_distance"] = estimated_distance
    map_position["confidence"] = object_info.get("conf", 0.0)
    map_position["object_class"] = object_info["class"]
    map_position["timestamp"] = object_info.get("timestamp", time.time())
    map_position["detected_image_size"] = f"{detected_width}x{detected_height}"
    
    logger.info(f"resolve position succussful: ({map_position['x']:.2f}, {map_position['y']:.2f})")
    logger.info(f"distance: {estimated_distance:.2f}m, image: {detected_width}x{detected_height}")
    return map_position

def calibrate_distance_estimation(expected_distance: float, measured_distance: float):
    if measured_distance > 0:
        new_factor = expected_distance / measured_distance
        logger.info(f"Distance calibration recommendation:")
        logger.info(f"   Expected distance: {expected_distance:.2f} m")
        logger.info(f"   Current measurement: {measured_distance:.2f} m")
        logger.info(f"   Suggested calibration factor: {new_factor:.2f}")
        logger.info(f"   Set CALIBRATION_FACTOR = {new_factor:.2f} in estimate_distance_from_bbox")
        return new_factor
    return 1.0

def test_coordinate_conversion():
    print("Testing coordinate conversion...")

    robot_pos = {"x": 0.0, "y": 0.0, "heading_y": 0.0} 
    pixel_coords = [320, 240]
    distance = 1.0

    result1 = pixel_to_map_coordinates(pixel_coords, robot_pos, distance)
    print(f"Test 1 - Front center: {result1}")

    pixel_coords_right = [480, 240]
    result2 = pixel_to_map_coordinates(pixel_coords_right, robot_pos, distance)
    print(f"Test 2 - Right side: {result2}")

    robot_pos_east = {"x": 0.0, "y": 0.0, "heading_y": 90.0}
    result3 = pixel_to_map_coordinates(pixel_coords, robot_pos_east, distance)
    print(f"Test 3 - Robot facing east: {result3}")

def test_distance_estimation():
    print("Testing distance estimation...")

    test_objects = [
        {"bbox": [200, 200, 280, 260], "class": "cup"},
        {"bbox": [100, 100, 200, 300], "class": "bottle"},
        {"bbox": [50, 50, 350, 400], "class": "person"},
    ]
    
    for obj in test_objects:
        distance = estimate_distance_from_bbox(obj["bbox"], obj["class"])
        print(f"  {obj['class']}: {distance:.2f}m")

def test_blackboard_integration():
    print("Testing blackboard integration...")

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

    bb_file = "/tmp/robot_blackboard_test_robot.json"
    with open(bb_file, "w") as f:
        json.dump(test_data, f)

    robot_pos = {"x": 1.0, "y": 2.0, "heading_y": 45.0}
    result = resolve_object_position("test_robot", "test_cup", robot_pos)
    
    if result:
        print(f"Blackboard test successful: {result}")
    else:
        print("Blackboard test failed")

    if os.path.exists(bb_file):
        os.remove(bb_file)
