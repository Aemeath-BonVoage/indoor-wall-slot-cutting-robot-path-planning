#!/usr/bin/env python3
"""
麦克纳姆轮运动学控制器
使用planar_move插件控制底盘运动，同时驱动轮子旋转作为视觉效果
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class MecanumController(Node):
    def __init__(self):
        super().__init__('mecanum_controller')
        
        # 订阅cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 发布到planar_move插件（实际控制底盘运动）
        self.planar_pub = self.create_publisher(
            Twist,
            '/cmd_vel_direct',
            10
        )
        
        # 发布轮子速度命令到ros2_control（视觉效果）
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, 
            '/wheel_controller/commands', 
            10
        )
        
        # 机器人参数
        self.wheel_radius = 0.05  # 轮子半径 (m)
        self.lx = 0.29658  # 前后轮距的一半 (m)
        self.ly = 0.2469   # 左右轮距的一半 (m)
        
        self.get_logger().info('麦克纳姆轮控制器已启动 (planar_move + 轮子视觉)')
        
        # 启动时发送零速度
        self.init_timer = self.create_timer(0.5, self.send_zero_velocity)
        self.init_count = 0

    def send_zero_velocity(self):
        """启动时发送零速度"""
        wheel_cmd = Float64MultiArray()
        wheel_cmd.data = [0.0, 0.0, 0.0, 0.0]
        self.wheel_pub.publish(wheel_cmd)
        
        zero_twist = Twist()
        self.planar_pub.publish(zero_twist)
        
        self.init_count += 1
        if self.init_count >= 10:
            self.init_timer.cancel()

    def cmd_vel_callback(self, msg):
        """
        接收cmd_vel，转发到planar_move并计算轮子速度
        """
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        
        # 直接转发到planar_move插件控制底盘
        self.planar_pub.publish(msg)
        
        # 计算轮子速度（用于视觉效果）
        # 左前轮 (lf)
        v_lf = (vx - vy - (self.lx + self.ly) * wz) / self.wheel_radius
        # 右前轮 (rf)
        v_rf = (vx + vy + (self.lx + self.ly) * wz) / self.wheel_radius
        # 左后轮 (lb)
        v_lb = (vx + vy - (self.lx + self.ly) * wz) / self.wheel_radius
        # 右后轮 (rb)
        v_rb = (vx - vy + (self.lx + self.ly) * wz) / self.wheel_radius
        
        # 发布轮子速度命令
        wheel_cmd = Float64MultiArray()
        wheel_cmd.data = [v_lf, v_rf, v_lb, v_rb]
        self.wheel_pub.publish(wheel_cmd)
        
        # 调试信息
        if abs(vx) > 0.01 or abs(vy) > 0.01 or abs(wz) > 0.01:
            self.get_logger().info(
                f'cmd_vel: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    controller = MecanumController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
