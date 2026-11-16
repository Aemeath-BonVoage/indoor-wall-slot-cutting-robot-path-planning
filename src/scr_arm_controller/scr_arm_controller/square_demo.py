#!/usr/bin/env python3
"""
用joint1(上下)和joint2(左右)画5cm正方形
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class SquareDemo(Node):
    def __init__(self):
        super().__init__('square_demo')

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
        """移动到目标位置"""
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

        self.get_logger().info(f'移动到: j1={positions[0]:.3f}, j2={positions[1]:.3f}, j3={positions[2]:.3f}, j4={positions[3]:.2f}')

        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        return True

    def draw_square(self):
        """
        用joint1(上下)和joint2(左右)画5cm正方形

        joint1: Z轴上下, 范围 -0.35 ~ 0.2
        joint2: X轴左右, 范围 -0.08 ~ 0.06

        顺序: 右→上→左→下
        """
        self.get_logger().info('='*60)
        self.get_logger().info('开始画5cm正方形')
        self.get_logger().info('使用 joint1(上下) 和 joint2(左右)')
        self.get_logger().info('='*60)

        # 回到初始位置
        self.get_logger().info('\n>>> 回到初始位置')
        self.move_to([0.0, 0.0, 0.0, 0.0], 2.0)
        time.sleep(0.5)

        # 定义正方形的四个角点
        half_size = 0.025  # 2.5cm

        # 角点: [joint1, joint2, joint3, joint4]
        corner1 = [-half_size, -half_size, 0.0, 0.0]  # 下左
        corner2 = [-half_size,  half_size, 0.0, 0.0]  # 下右
        corner3 = [ half_size,  half_size, 0.0, 0.0]  # 上右
        corner4 = [ half_size, -half_size, 0.0, 0.0]  # 上左

        self.get_logger().info('\n>>> 下左角')
        self.move_to(corner1, 2.0)
        time.sleep(0.3)

        self.get_logger().info('\n>>> 下右角（右移）')
        self.move_to(corner2, 2.0)
        time.sleep(0.3)

        self.get_logger().info('\n>>> 上右角（上移）')
        self.move_to(corner3, 2.0)
        time.sleep(0.3)

        self.get_logger().info('\n>>> 上左角（左移）')
        self.move_to(corner4, 2.0)
        time.sleep(0.3)

        self.get_logger().info('\n>>> 回到下左角（下移，闭合）')
        self.move_to(corner1, 2.0)
        time.sleep(0.3)

        self.get_logger().info('\n>>> 回到原点')
        self.move_to([0.0, 0.0, 0.0, 0.0], 2.0)

        self.get_logger().info('='*60)
        self.get_logger().info('完成!')
        self.get_logger().info('='*60)


def main(args=None):
    rclpy.init(args=args)
    demo = SquareDemo()

    try:
        demo.draw_square()
    except KeyboardInterrupt:
        pass
    finally:
        demo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
