#!/usr/bin/env python3
"""
机械臂演示 - 使用JointTrajectoryController平滑控制
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class ArmDemo(Node):
    def __init__(self):
        super().__init__('arm_demo')
        
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        # Action客户端
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('等待arm_controller action服务...')
        self._action_client.wait_for_server()
        self.get_logger().info('连接成功!')

    def move_to(self, positions, duration=2.0):
        """平滑移动到目标位置"""
        goal_msg = FollowJointTrajectory.Goal()
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 4
        point.time_from_start = Duration(
            sec=int(duration), 
            nanosec=int((duration % 1) * 1e9)
        )
        
        trajectory.points = [point]
        goal_msg.trajectory = trajectory
        
        self.get_logger().info(f'移动到: {[f"{p:.2f}" for p in positions]}, 时间: {duration}s')
        
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        return True

    def run_demo(self):
        self.get_logger().info('='*50)
        self.get_logger().info('开始机械臂演示')
        self.get_logger().info('='*50)
        
        # 回到初始位置
        self.get_logger().info('\n>>> 回到初始位置')
        self.move_to([0.0, 0.0, 0.0, 0.0], 3.0)
        time.sleep(0.5)
        
        # joint1 升降 (prismatic, Z轴, 范围 -0.35 ~ 0.2)
        self.get_logger().info('\n>>> joint1 升降')
        self.move_to([0.15, 0.0, 0.0, 0.0], 2.0)
        self.move_to([-0.2, 0.0, 0.0, 0.0], 3.0)
        self.move_to([0.0, 0.0, 0.0, 0.0], 2.0)
        
        # joint2 伸缩 (prismatic, 范围 -0.08 ~ 0.06)
        self.get_logger().info('\n>>> joint2 伸缩')
        self.move_to([0.0, 0.05, 0.0, 0.0], 2.0)
        self.move_to([0.0, -0.06, 0.0, 0.0], 2.0)
        self.move_to([0.0, 0.0, 0.0, 0.0], 2.0)
        
        # joint3 伸缩 (prismatic, 范围 -0.045 ~ 0.02)
        self.get_logger().info('\n>>> joint3 伸缩')
        self.move_to([0.0, 0.0, 0.015, 0.0], 2.0)
        self.move_to([0.0, 0.0, -0.04, 0.0], 2.0)
        self.move_to([0.0, 0.0, 0.0, 0.0], 2.0)
        
        # joint4 旋转 (revolute, 范围 -1.57 ~ 1.57)
        self.get_logger().info('\n>>> joint4 旋转')
        self.move_to([0.0, 0.0, 0.0, 1.2], 2.0)
        self.move_to([0.0, 0.0, 0.0, -1.2], 3.0)
        self.move_to([0.0, 0.0, 0.0, 0.0], 2.0)
        
        # 组合动作
        self.get_logger().info('\n>>> 组合动作')
        self.move_to([0.1, 0.03, -0.02, 0.8], 3.0)
        self.move_to([-0.15, -0.05, 0.01, -0.8], 4.0)
        self.move_to([0.0, 0.0, 0.0, 0.0], 3.0)
        
        self.get_logger().info('='*50)
        self.get_logger().info('演示完成!')
        self.get_logger().info('='*50)


def main(args=None):
    rclpy.init(args=args)
    demo = ArmDemo()
    
    try:
        demo.run_demo()
    except KeyboardInterrupt:
        pass
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
