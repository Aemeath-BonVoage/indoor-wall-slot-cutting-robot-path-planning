#!/usr/bin/env python3
"""
轨迹可视化器
在Gazebo中动态显示切割轨迹
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose
import math


class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')
        
        # Gazebo服务客户端
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        
        self.spawned_models = []
        
        self.get_logger().info('轨迹可视化器已初始化（Gazebo模式）')
    
    def visualize_in_map_frame(self, points, color='red', frame_id='map'):
        """
        在Gazebo中可视化路径
        
        参数:
            points: 路径点列表 [(x, y, z, rotation), ...]
            color: 颜色
            frame_id: 坐标系（忽略，Gazebo使用world坐标系）
        """
        if len(points) < 4:
            self.get_logger().warn('点数不足，无法生成边框')
            return
        
        # 等待服务
        if not self.spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Gazebo spawn服务不可用')
            return
        
        self.get_logger().info('开始在Gazebo中生成红色边框...')
        
        # 生成4条边框线
        self._spawn_border_lines(points, color)
        
        self.get_logger().info(f'轨迹可视化已发布到Gazebo')
    
    def _spawn_border_lines(self, points, color):
        """生成边框线条"""
        # 提取4个顶点
        p1, p2, p3, p4 = points[:4]
        
        # 计算4条边
        edges = [
            ('bottom', p1, p2),  # 下边（沿X轴）
            ('right', p2, p3),   # 右边（沿Z轴）
            ('top', p3, p4),     # 上边（沿X轴）
            ('left', p4, p1),    # 左边（沿Z轴）
        ]
        
        for name, start, end in edges:
            # 计算中点
            mid_x = (start[0] + end[0]) / 2
            mid_y = (start[1] + end[1]) / 2
            mid_z = (start[2] + end[2]) / 2
            
            # 计算长度和方向
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            dz = end[2] - start[2]
            length = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            # 判断是水平线还是垂直线
            is_horizontal = abs(dz) < 0.001  # Z变化很小，说明是水平线
            
            # 生成SDF
            model_name = f'cutting_border_{name}'
            sdf = self._create_line_sdf_for_wall(length, color, is_horizontal)
            
            # 设置位置
            pose = Pose()
            pose.position.x = mid_x
            pose.position.y = mid_y
            pose.position.z = mid_z
            
            # 设置旋转
            # 墙面在X-Z平面，Y=-3.9
            # Box默认沿X轴，我们需要：
            # - 水平线：沿X轴，不需要额外旋转
            # - 垂直线：沿Z轴，需要绕Y轴旋转90度
            
            if is_horizontal:
                # 水平线，box默认就是沿X轴的
                pose.orientation.w = 1.0
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = 0.0
            else:
                # 垂直线，绕Y轴旋转90度
                # 使box从沿X轴变成沿Z轴
                angle = math.pi / 2
                pose.orientation.w = math.cos(angle / 2)
                pose.orientation.x = 0.0
                pose.orientation.y = math.sin(angle / 2)
                pose.orientation.z = 0.0
            
            # 生成模型
            self._spawn_model(model_name, sdf, pose)
    
    def _create_line_sdf_for_wall(self, length, color, is_horizontal):
        """创建贴墙线条的SDF描述
        
        Box的本地坐标系：
        - X轴：长度方向（线条延伸方向）
        - Y轴：厚度方向（垂直于墙面，很薄）
        - Z轴：宽度方向（线条粗细）
        """
        # 颜色映射
        colors = {
            'red': (1, 0, 0),
            'green': (0, 1, 0),
            'blue': (0, 0, 1),
            'yellow': (1, 1, 0),
        }
        r, g, b = colors.get(color, (1, 0, 0))
        
        # 线条尺寸
        line_width = 0.002  # 2mm粗（Z方向）
        line_thickness = 0.001  # 1mm厚（Y方向，垂直于墙面）
        
        sdf = f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="line_model">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <box>
            <size>{length} {line_thickness} {line_width}</size>
          </box>
        </geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
          <emissive>{r*0.8} {g*0.8} {b*0.8} 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''
        return sdf
    
    def _spawn_model(self, model_name, sdf, pose):
        """在Gazebo中生成模型"""
        request = SpawnEntity.Request()
        request.name = model_name
        request.xml = sdf
        request.initial_pose = pose
        request.reference_frame = 'world'
        
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            self.spawned_models.append(model_name)
            self.get_logger().info(f'已生成模型: {model_name}')
        else:
            self.get_logger().error(f'生成模型失败: {model_name}')
    
    def clear_markers(self):
        """清除所有生成的模型"""
        if not self.delete_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Gazebo delete服务不可用')
            return
        
        for model_name in self.spawned_models:
            request = DeleteEntity.Request()
            request.name = model_name
            
            future = self.delete_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        self.spawned_models.clear()
        self.get_logger().info('已清除所有边框模型')


def main(args=None):
    rclpy.init(args=args)
    visualizer = TrajectoryVisualizer()
    
    # 示例：可视化一个矩形
    points = [
        (8.385, -3.9, 0.565, 0),  # 左下
        (8.455, -3.9, 0.565, 0),  # 右下
        (8.455, -3.9, 0.635, 0),  # 右上
        (8.385, -3.9, 0.635, 0),  # 左上
    ]
    
    visualizer.visualize_in_map_frame(points, color='red', frame_id='map')
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    
    visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
