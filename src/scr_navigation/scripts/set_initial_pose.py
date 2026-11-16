#!/usr/bin/env python3
"""
自动设置机器人初始位姿
在导航启动时自动发布初始位姿到AMCL
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time


class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('initial_pose_setter')
        
        # use_sim_time会被ROS自动声明，不需要手动声明
        
        # 声明参数
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', 0.0)
        
        # 获取参数
        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        z = self.get_parameter('z').value
        yaw = self.get_parameter('yaw').value
        
        # 创建发布者
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # 等待一段时间确保AMCL已启动和TF准备好
        self.get_logger().info('等待3秒确保系统准备就绪...')
        time.sleep(3.0)
        
        # 发布初始位姿
        self.publish_initial_pose(x, y, z, yaw)
        
        self.get_logger().info(f'初始位姿已设置: x={x}, y={y}, yaw={yaw}')
        
        # 再发布几次确保收到
        for i in range(3):
            time.sleep(0.5)
            self.publish_initial_pose(x, y, z, yaw)
        
        self.get_logger().info('初始位姿设置完成')
    
    def publish_initial_pose(self, x, y, z, yaw):
        """发布初始位姿"""
        msg = PoseWithCovarianceStamped()
        # 使用当前时间戳（对于仿真很重要）
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        
        # 将yaw转换为四元数
        import math
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # 设置协方差
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909
        ]
        
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseSetter()
    
    # 等待一段时间后退出
    time.sleep(1.0)
    
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
