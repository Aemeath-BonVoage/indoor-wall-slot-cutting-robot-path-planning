#!/usr/bin/env python3
"""
黄色方块扫描器
通过上下移动机械臂并使用相机检测黄色方块位置
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_point


class MarkerScanner(Node):
    def __init__(self):
        super().__init__('marker_scanner')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 相机订阅
        self.image_sub = self.create_subscription(
            Image,
            '/front_camera/image_raw',  # 修改为正确的话题名
            self.image_callback,
            10
        )
        
        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 检测结果
        self.marker_detected = False
        self.marker_position_camera = None  # 相机坐标系
        self.marker_position_base = None    # base_link坐标系
        
        # 黄色HSV范围
        self.yellow_lower = np.array([20, 100, 100])
        self.yellow_upper = np.array([30, 255, 255])
        
        # 相机参数（从URDF获取）
        self.camera_fov = 1.3962634  # 80度
        self.image_width = 640
        self.image_height = 480
        
        # 统计信息
        self.image_count = 0
        
        self.get_logger().info('黄色方块扫描器已初始化')
        self.get_logger().info(f'订阅话题: /front_camera/image_raw')
    
    def image_callback(self, msg):
        """处理相机图像"""
        self.image_count += 1
        
        # 每10帧打印一次日志
        if self.image_count % 10 == 0:
            self.get_logger().debug(f'已接收 {self.image_count} 帧图像')
        
        try:
            # 转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 检测黄色方块
            self.detect_yellow_marker(cv_image)
            
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {e}')
    
    def detect_yellow_marker(self, image):
        """检测黄色方块"""
        # 转换到HSV色彩空间
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 创建黄色掩码
        mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        
        # 形态学操作去噪
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # 找到最大轮廓
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            # 面积阈值（避免误检）
            if area > 100:  # 至少100像素
                # 计算中心点
                M = cv2.moments(largest_contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    
                    # 估算距离（简单的相似三角形方法）
                    # 假设方块实际大小为5cm
                    marker_real_size = 0.05  # 米
                    focal_length = (self.image_width / 2) / np.tan(self.camera_fov / 2)
                    
                    # 从轮廓估算方块在图像中的大小
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    marker_image_size = max(w, h)
                    
                    # 计算距离
                    distance = (marker_real_size * focal_length) / marker_image_size
                    
                    # 计算相对于相机光轴的偏移
                    # 图像中心
                    img_center_x = self.image_width / 2
                    img_center_y = self.image_height / 2
                    
                    # 像素偏移
                    pixel_offset_x = cx - img_center_x
                    pixel_offset_y = cy - img_center_y
                    
                    # 转换为实际偏移（米）
                    offset_x = (pixel_offset_x / focal_length) * distance
                    offset_y = (pixel_offset_y / focal_length) * distance
                    
                    # 相机坐标系中的位置
                    # 相机坐标系：X右，Y下，Z前
                    self.marker_position_camera = {
                        'x': offset_x,
                        'y': offset_y,
                        'z': distance
                    }
                    
                    self.marker_detected = True
                    
                    # 只在首次检测时打印详细信息
                    if self.image_count % 30 == 0:  # 每30帧打印一次
                        self.get_logger().info(
                            f'检测到黄色方块！'
                            f'图像位置: ({cx}, {cy}), '
                            f'面积: {area:.0f}px, '
                            f'距离: {distance:.3f}m'
                        )
    
    def get_marker_in_base_link(self):
        """
        获取方块在base_link坐标系中的位置
        返回: (x, y, z) 或 None
        """
        if not self.marker_detected or self.marker_position_camera is None:
            return None
        
        try:
            # 创建相机坐标系中的点
            point_camera = PointStamped()
            point_camera.header.frame_id = 'camera_optical_link'
            point_camera.header.stamp = self.get_clock().now().to_msg()  # 使用正确的时间戳格式
            point_camera.point.x = self.marker_position_camera['z']  # 相机Z轴是前方
            point_camera.point.y = -self.marker_position_camera['x']  # 相机X轴是右，转换为左
            point_camera.point.z = -self.marker_position_camera['y']  # 相机Y轴是下，转换为上
            
            # 尝试多个可能的目标坐标系
            target_frames = ['base_link', 'base_footprint', 'odom']
            
            for target_frame in target_frames:
                try:
                    # 等待TF变换（短超时）
                    transform = self.tf_buffer.lookup_transform(
                        target_frame,
                        'camera_optical_link',
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.5)
                    )
                    
                    # 转换到目标坐标系
                    point_target = do_transform_point(point_camera, transform)
                    
                    self.marker_position_base = {
                        'x': point_target.point.x,
                        'y': point_target.point.y,
                        'z': point_target.point.z,
                        'frame': target_frame
                    }
                    
                    self.get_logger().info(
                        f'成功转换到 {target_frame} 坐标系: '
                        f'X={point_target.point.x:.4f}, Y={point_target.point.y:.4f}, Z={point_target.point.z:.4f}'
                    )
                    
                    return (point_target.point.x, point_target.point.y, point_target.point.z)
                    
                except Exception as e:
                    # 尝试下一个坐标系
                    continue
            
            # 所有坐标系都失败
            self.get_logger().debug('无法找到有效的TF变换，尝试的坐标系: ' + ', '.join(target_frames))
            
            # 使用简化的估算方法（基于相机位置）
            # 相机安装在link1上，位置大约在 (0.11, 0.0215, 0.5) 相对于link1
            # link1相对于base_link的初始位置约为 (0.039, 0, 0.048)
            # 简化估算：假设相机朝前，距离约0.8m
            estimated_x = 0.15 + self.marker_position_camera['z']  # 相机前方
            estimated_y = -self.marker_position_camera['x']  # 左右偏移
            estimated_z = 0.55 - self.marker_position_camera['y']  # 高度
            
            self.get_logger().debug(
                f'使用简化估算: X={estimated_x:.4f}, Y={estimated_y:.4f}, Z={estimated_z:.4f}'
            )
            
            return (estimated_x, estimated_y, estimated_z)
            
        except Exception as e:
            self.get_logger().error(f'坐标转换失败: {e}')
            return None
    
    def reset_detection(self):
        """重置检测状态"""
        self.marker_detected = False
        self.marker_position_camera = None
        self.marker_position_base = None


def main(args=None):
    rclpy.init(args=args)
    scanner = MarkerScanner()
    
    try:
        rclpy.spin(scanner)
    except KeyboardInterrupt:
        pass
    
    scanner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
