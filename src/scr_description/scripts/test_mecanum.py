#!/usr/bin/env python3
"""
麦克纳姆轮测试脚本
测试四个方向的移动：前后、左右、斜向、旋转
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class MecanumTester(Node):
    def __init__(self):
        super().__init__('mecanum_tester')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('麦克纳姆轮测试节点已启动')

    def send_velocity(self, linear_x, linear_y, angular_z, duration=2.0):
        """发送速度命令"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(msg)
            time.sleep(0.1)
        
        # 停止
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        time.sleep(0.5)

    def run_tests(self):
        """运行所有测试"""
        self.get_logger().info('开始测试...')
        time.sleep(1.0)
        
        # 测试1: 前进
        self.get_logger().info('测试1: 前进')
        self.send_velocity(0.3, 0.0, 0.0, 2.0)
        
        # 测试2: 后退
        self.get_logger().info('测试2: 后退')
        self.send_velocity(-0.3, 0.0, 0.0, 2.0)
        
        # 测试3: 左平移
        self.get_logger().info('测试3: 左平移')
        self.send_velocity(0.0, 0.3, 0.0, 2.0)
        
        # 测试4: 右平移
        self.get_logger().info('测试4: 右平移')
        self.send_velocity(0.0, -0.3, 0.0, 2.0)
        
        # 测试5: 左前斜向移动
        self.get_logger().info('测试5: 左前斜向移动')
        self.send_velocity(0.3, 0.3, 0.0, 2.0)
        
        # 测试6: 右后斜向移动
        self.get_logger().info('测试6: 右后斜向移动')
        self.send_velocity(-0.3, -0.3, 0.0, 2.0)
        
        # 测试7: 原地左转
        self.get_logger().info('测试7: 原地左转')
        self.send_velocity(0.0, 0.0, 0.5, 2.0)
        
        # 测试8: 原地右转
        self.get_logger().info('测试8: 原地右转')
        self.send_velocity(0.0, 0.0, -0.5, 2.0)
        
        # 测试9: 前进+旋转
        self.get_logger().info('测试9: 前进+旋转')
        self.send_velocity(0.3, 0.0, 0.3, 2.0)
        
        self.get_logger().info('所有测试完成！')


def main(args=None):
    rclpy.init(args=args)
    tester = MecanumTester()
    
    try:
        tester.run_tests()
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
