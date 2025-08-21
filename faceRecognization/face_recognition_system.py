#!/usr/bin/env python3
"""
YOLOv8 + 人脸识别集成系统
结合YOLOv8的人体检测和专门的人脸识别模型
"""

import cv2
import numpy as np
import face_recognition
import pickle
import os
from pathlib import Path
from typing import List, Tuple, Dict, Optional
from ultralytics import YOLO
import logging

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class FaceRecognitionSystem:
    def __init__(self, 
                 yolo_model_path: str = "yolov8n.pt",
                 face_encodings_path: str = "face_encodings.pkl",
                 confidence_threshold: float = 0.5):
        """
        初始化人脸识别系统
        
        Args:
            yolo_model_path: YOLOv8模型路径
            face_encodings_path: 人脸编码数据库路径
            confidence_threshold: YOLO检测置信度阈值
        """
        self.yolo_model = YOLO(yolo_model_path)
        self.face_encodings_path = face_encodings_path
        self.confidence_threshold = confidence_threshold
        
        # 加载已知人脸编码
        self.known_face_encodings = []
        self.known_face_names = []
        self.load_face_database()
        
        logger.info(f"系统初始化完成，已加载 {len(self.known_face_names)} 个已知人脸")

    def load_face_database(self):
        """加载人脸编码数据库"""
        if os.path.exists(self.face_encodings_path):
            try:
                with open(self.face_encodings_path, 'rb') as f:
                    data = pickle.load(f)
                    self.known_face_encodings = data['encodings']
                    self.known_face_names = data['names']
                logger.info(f"已加载人脸数据库: {len(self.known_face_names)} 个人脸")
            except Exception as e:
                logger.error(f"加载人脸数据库失败: {e}")
        else:
            logger.warning("人脸数据库不存在，将创建新的数据库")

    def save_face_database(self):
        """保存人脸编码数据库"""
        try:
            data = {
                'encodings': self.known_face_encodings,
                'names': self.known_face_names
            }
            with open(self.face_encodings_path, 'wb') as f:
                pickle.dump(data, f)
            logger.info("人脸数据库已保存")
        except Exception as e:
            logger.error(f"保存人脸数据库失败: {e}")

    def add_person_from_folder(self, person_name: str, folder_path: str):
        """
        从文件夹中添加一个人的多张照片
        
        Args:
            person_name: 人名
            folder_path: 包含该人照片的文件夹路径
        """
        folder_path = Path(folder_path)
        if not folder_path.exists():
            logger.error(f"文件夹不存在: {folder_path}")
            return
        
        image_extensions = {'.jpg', '.jpeg', '.png', '.bmp'}
        image_files = [f for f in folder_path.iterdir() 
                      if f.suffix.lower() in image_extensions]
        
        encodings_added = 0
        for image_file in image_files:
            try:
                # 读取图像
                image = face_recognition.load_image_file(str(image_file))
                
                # 获取人脸编码
                face_encodings = face_recognition.face_encodings(image)
                
                if face_encodings:
                    # 添加第一个检测到的人脸
                    self.known_face_encodings.append(face_encodings[0])
                    self.known_face_names.append(person_name)
                    encodings_added += 1
                    logger.info(f"已添加 {person_name} 的人脸: {image_file.name}")
                else:
                    logger.warning(f"在 {image_file.name} 中未检测到人脸")
                    
            except Exception as e:
                logger.error(f"处理图像 {image_file.name} 时出错: {e}")
        
        logger.info(f"为 {person_name} 添加了 {encodings_added} 个人脸编码")
        self.save_face_database()

    def detect_people_yolo(self, image: np.ndarray) -> List[Tuple[int, int, int, int]]:
        """
        使用YOLOv8检测图像中的人体
        
        Args:
            image: 输入图像
            
        Returns:
            人体边界框列表 [(x1, y1, x2, y2), ...]
        """
        results = self.yolo_model(image, verbose=False)
        people_boxes = []
        
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    # 检查是否为人类 (class_id = 0 在COCO数据集中表示person)
                    if int(box.cls) == 0 and float(box.conf) > self.confidence_threshold:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                        people_boxes.append((x1, y1, x2, y2))
        
        return people_boxes

    def recognize_faces_in_region(self, image: np.ndarray, 
                                 region: Tuple[int, int, int, int]) -> List[Dict]:
        """
        在指定区域内进行人脸识别
        
        Args:
            image: 输入图像
            region: 区域坐标 (x1, y1, x2, y2)
            
        Returns:
            识别结果列表 [{"name": str, "confidence": float, "location": tuple}, ...]
        """
        x1, y1, x2, y2 = region
        
        # 确保坐标在图像范围内
        h, w = image.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        
        # 提取区域
        region_image = image[y1:y2, x1:x2]
        
        if region_image.size == 0:
            return []
        
        # 检测人脸位置
        face_locations = face_recognition.face_locations(region_image)
        
        if not face_locations:
            return []
        
        # 获取人脸编码
        face_encodings = face_recognition.face_encodings(region_image, face_locations)
        
        results = []
        for face_encoding, face_location in zip(face_encodings, face_locations):
            # 与已知人脸进行比较
            matches = face_recognition.compare_faces(
                self.known_face_encodings, face_encoding, tolerance=0.6)
            face_distances = face_recognition.face_distance(
                self.known_face_encodings, face_encoding)
            
            name = "Unknown"
            confidence = 0.0
            
            if len(face_distances) > 0:
                best_match_index = np.argmin(face_distances)
                if matches[best_match_index]:
                    name = self.known_face_names[best_match_index]
                    confidence = 1 - face_distances[best_match_index]
            
            # 转换坐标到原图像坐标系
            top, right, bottom, left = face_location
            face_location_global = (
                left + x1, top + y1, 
                right + x1, bottom + y1
            )
            
            results.append({
                "name": name,
                "confidence": confidence,
                "location": face_location_global
            })
        
        return results

    def process_image(self, image: np.ndarray) -> Dict:
        """
        处理单张图像，返回人体检测和人脸识别结果
        
        Args:
            image: 输入图像
            
        Returns:
            处理结果字典
        """
        # 1. 使用YOLO检测人体
        people_boxes = self.detect_people_yolo(image)
        
        # 2. 在每个人体区域内进行人脸识别
        all_face_results = []
        for person_box in people_boxes:
            face_results = self.recognize_faces_in_region(image, person_box)
            all_face_results.extend(face_results)
        
        return {
            "people_boxes": people_boxes,
            "face_recognition_results": all_face_results,
            "total_people": len(people_boxes),
            "recognized_faces": len([r for r in all_face_results if r["name"] != "Unknown"])
        }

    def draw_results(self, image: np.ndarray, results: Dict) -> np.ndarray:
        """
        在图像上绘制检测和识别结果
        
        Args:
            image: 输入图像
            results: 处理结果
            
        Returns:
            标注后的图像
        """
        annotated_image = image.copy()
        
        # 绘制人体检测框
        for i, (x1, y1, x2, y2) in enumerate(results["people_boxes"]):
            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated_image, f"Person {i+1}", (x1, y1-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 绘制人脸识别结果
        for face_result in results["face_recognition_results"]:
            left, top, right, bottom = face_result["location"]
            name = face_result["name"]
            confidence = face_result["confidence"]
            
            # 选择颜色
            color = (0, 255, 255) if name != "Unknown" else (0, 0, 255)
            
            # 绘制人脸框
            cv2.rectangle(annotated_image, (left, top), (right, bottom), color, 2)
            
            # 绘制标签
            label = f"{name}"
            if name != "Unknown":
                label += f" ({confidence:.2f})"
            
            # 计算文本位置
            (text_width, text_height), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            
            cv2.rectangle(annotated_image, (left, top - text_height - 10),
                         (left + text_width, top), color, -1)
            cv2.putText(annotated_image, label, (left, top - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # 添加统计信息
        stats_text = f"People: {results['total_people']}, Recognized: {results['recognized_faces']}"
        cv2.putText(annotated_image, stats_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        return annotated_image

    def process_video(self, video_source=0, save_output: str = None):
        """
        处理视频流（摄像头或视频文件）
        
        Args:
            video_source: 视频源（摄像头编号或视频文件路径）
            save_output: 保存输出视频的路径（可选）
        """
        cap = cv2.VideoCapture(video_source)
        
        if not cap.isOpened():
            logger.error("无法打开视频源")
            return
        
        # 设置视频写入器（如果需要保存）
        writer = None
        if save_output:
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            fps = int(cap.get(cv2.CAP_PROP_FPS))
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            writer = cv2.VideoWriter(save_output, fourcc, fps, (width, height))
        
        logger.info("开始处理视频流，按 'q' 退出")
        
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                
                # 处理帧
                results = self.process_image(frame)
                annotated_frame = self.draw_results(frame, results)
                
                # 显示结果
                cv2.imshow('Face Recognition System', annotated_frame)
                
                # 保存帧（如果需要）
                if writer:
                    writer.write(annotated_frame)
                
                # 检查退出条件
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        except KeyboardInterrupt:
            logger.info("用户中断")
        finally:
            cap.release()
            if writer:
                writer.release()
            cv2.destroyAllWindows()


def main():
    """主函数示例"""
    # 初始化系统
    face_system = FaceRecognitionSystem()
    
    # 添加已知人脸（从文件夹）
    # face_system.add_person_from_folder("张三", "faces/zhangsan/")
    # face_system.add_person_from_folder("李四", "faces/lisi/")
    
    # 处理单张图像
    # image = cv2.imread("test_image.jpg")
    # results = face_system.process_image(image)
    # annotated_image = face_system.draw_results(image, results)
    # cv2.imshow("Result", annotated_image)
    # cv2.waitKey(0)
    
    # 处理视频流
    face_system.process_video(video_source=0)  # 使用摄像头
    # face_system.process_video(video_source="video.mp4", save_output="output.avi")  # 处理视频文件


if __name__ == "__main__":
    main()