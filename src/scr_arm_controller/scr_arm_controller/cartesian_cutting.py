#!/usr/bin/env python3
"""
笛卡尔空间切割控制 - 指定相对于base_link的坐标进行切割
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class CartesianCuttingController(Node):
    def __init__(self):
        super().__init__('cartesian_cutting_controller')

        # 创建Action客户端
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        self.get_logger().info('等待action服务器...')
        self._action_client.wait_for_server()
        self.get_logger().info('已连接到action服务器')

        # 关节名称
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        # 关节限位
        self.joint_limits = {
            'joint1': (-0.35, 0.2),
            'joint2': (-0.08, 0.06),
            'joint3': (-0.045, 0.02),
            'joint4': (-1.57, 1.57)
        }

        # 计算笛卡尔空间的工作范围（相对于base_link）
        # 初始位置: X=0.537m, Y≈0m, Z=0.507m
        self.x0 = 0.53677
        self.y0 = -0.00015
        self.z0 = 0.50679

        # 笛卡尔空间限位（基于关节限位计算）
        self.cartesian_limits = {
            'x': (self.x0 - (-self.joint_limits['joint3'][0]),  # joint3控制X，反向
                  self.x0 - (-self.joint_limits['joint3'][1])),
            'y': (self.y0 - (-self.joint_limits['joint2'][0]),  # joint2控制Y，反向
                  self.y0 - (-self.joint_limits['joint2'][1])),
            'z': (self.z0 + self.joint_limits['joint1'][0],     # joint1控制Z
                  self.z0 + self.joint_limits['joint1'][1])
        }

        self.get_logger().info('笛卡尔空间工作范围（相对于base_link）:')
        self.get_logger().info(f'  X: [{self.cartesian_limits["x"][0]:.4f}, {self.cartesian_limits["x"][1]:.4f}] m')
        self.get_logger().info(f'  Y: [{self.cartesian_limits["y"][0]:.4f}, {self.cartesian_limits["y"][1]:.4f}] m')
        self.get_logger().info(f'  Z: [{self.cartesian_limits["z"][0]:.4f}, {self.cartesian_limits["z"][1]:.4f}] m')

    def inverse_kinematics(self, x, y, z, rotation=0.0):
        """
        逆运动学求解
        输入: 相对于base_link的目标位置 (x, y, z) 和旋转角度
        输出: 关节值 [j1, j2, j3, j4] 或 None（如果无解）

        根据正向运动学分析：
        - joint1 影响 Z 轴
        - joint2 影响 Y 轴
        - joint3 影响 X 轴
        - joint4 是旋转
        """
        # 计算需要的关节位移
        # joint3 控制 X 轴（关节值增加 → X减少）
        j3 = -(x - self.x0)

        # joint2 控制 Y 轴（关节值增加 → Y减少）
        j2 = -(y - self.y0)

        # joint1 控制 Z 轴（关节值增加 → Z增加）
        j1 = z - self.z0

        # joint4 是旋转
        j4 = rotation

        # 检查关节限位（增加小容差避免浮点数精度问题）
        joints = [j1, j2, j3, j4]
        tolerance = 0.001  # 1mm 或 0.001 rad 的容差
        for i, (joint_name, value) in enumerate(zip(self.joint_names, joints)):
            lower, upper = self.joint_limits[joint_name]
            if value < (lower - tolerance) or value > (upper + tolerance):
                self.get_logger().error(
                    f'{joint_name} 超出限位: {value:.4f} (范围: [{lower}, {upper}])'
                )
                # 同时显示笛卡尔空间的限位
                if joint_name == 'joint1':
                    self.get_logger().error(
                        f'  对应Z轴范围: [{self.cartesian_limits["z"][0]:.4f}, {self.cartesian_limits["z"][1]:.4f}]'
                    )
                elif joint_name == 'joint2':
                    self.get_logger().error(
                        f'  对应Y轴范围: [{self.cartesian_limits["y"][0]:.4f}, {self.cartesian_limits["y"][1]:.4f}]'
                    )
                elif joint_name == 'joint3':
                    self.get_logger().error(
                        f'  对应X轴范围: [{self.cartesian_limits["x"][0]:.4f}, {self.cartesian_limits["x"][1]:.4f}]'
                    )
                return None

            # 如果超出但在容差内，钳制到限位
            if value < lower:
                joints[i] = lower
            elif value > upper:
                joints[i] = upper

        return joints

    def check_reachability(self, x, y, z, rotation=0.0):
        """检查目标点是否可达"""
        joints = self.inverse_kinematics(x, y, z, rotation)
        if joints is None:
            return False, None
        return True, joints

    def print_workspace_info(self):
        """打印工作空间信息"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('机械臂工作空间信息')
        self.get_logger().info('='*60)
        self.get_logger().info('\n初始位置（所有关节=0）:')
        self.get_logger().info(f'  X = {self.x0:.5f} m')
        self.get_logger().info(f'  Y = {self.y0:.5f} m')
        self.get_logger().info(f'  Z = {self.z0:.5f} m')

        self.get_logger().info('\n笛卡尔空间工作范围:')
        x_range = self.cartesian_limits['x']
        y_range = self.cartesian_limits['y']
        z_range = self.cartesian_limits['z']
        self.get_logger().info(f'  X: [{x_range[0]:.5f}, {x_range[1]:.5f}] m (范围: {x_range[1]-x_range[0]:.5f} m)')
        self.get_logger().info(f'  Y: [{y_range[0]:.5f}, {y_range[1]:.5f}] m (范围: {y_range[1]-y_range[0]:.5f} m)')
        self.get_logger().info(f'  Z: [{z_range[0]:.5f}, {z_range[1]:.5f}] m (范围: {z_range[1]-z_range[0]:.5f} m)')

        self.get_logger().info('\n关节限位:')
        for joint_name, (lower, upper) in self.joint_limits.items():
            self.get_logger().info(f'  {joint_name}: [{lower:+.5f}, {upper:+.5f}]')
        self.get_logger().info('='*60 + '\n')

    def move_to_joints(self, joint_positions, duration=3.0):
        """移动到指定关节位置"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        goal_msg.trajectory.points = [point]

        self.get_logger().info(f'发送目标: {joint_positions}')
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        return True

    def move_to_cartesian(self, x, y, z, rotation=0.0, duration=3.0):
        """移动到笛卡尔坐标"""
        self.get_logger().info(f'目标位置: X={x:.4f}, Y={y:.4f}, Z={z:.4f}, R={rotation:.4f}')

        joints = self.inverse_kinematics(x, y, z, rotation)
        if joints is None:
            self.get_logger().error('逆运动学求解失败，目标不可达')
            return False

        self.get_logger().info(f'关节值: {[f"{j:.4f}" for j in joints]}')
        return self.move_to_joints(joints, duration)

    def cut_rectangle(self, points, rotation=0.0):
        """
        按照矩形四个点进行切割
        points: 四个点的列表，每个点是 (x, y, z) 元组
        """
        if len(points) != 4:
            self.get_logger().error(f'需要4个点，但提供了{len(points)}个')
            return False

        self.get_logger().info('='*60)
        self.get_logger().info('开始矩形切割任务')
        self.get_logger().info('='*60)

        # 检查所有点是否可达
        self.get_logger().info('\n检查所有点的可达性...')
        for i, (x, y, z) in enumerate(points):
            reachable, joints = self.check_reachability(x, y, z, rotation)
            if not reachable:
                self.get_logger().error(f'点{i+1} ({x}, {y}, {z}) 不可达！')
                return False
            self.get_logger().info(f'点{i+1}: X={x:.4f}, Y={y:.4f}, Z={z:.4f} ✓')

        self.get_logger().info('\n所有点都可达，开始执行...\n')

        # 1. 回到初始位置
        self.get_logger().info('步骤1: 回到初始位置')
        if not self.move_to_joints([0.0, 0.0, 0.0, rotation], duration=3.0):
            return False

        # 2. 依次移动到四个点
        for i, (x, y, z) in enumerate(points):
            self.get_logger().info(f'\n步骤{i+2}: 移动到点{i+1}')
            if not self.move_to_cartesian(x, y, z, rotation, duration=3.0):
                return False

        # 3. 回到第一个点（闭合）
        self.get_logger().info('\n步骤6: 回到点1（闭合矩形）')
        x, y, z = points[0]
        if not self.move_to_cartesian(x, y, z, rotation, duration=3.0):
            return False

        # 4. 回到初始位置
        self.get_logger().info('\n步骤7: 回到初始位置')
        if not self.move_to_joints([0.0, 0.0, 0.0, rotation], duration=3.0):
            return False

        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('矩形切割任务完成！')
        self.get_logger().info('='*60)

        return True


def main(args=None):
    rclpy.init(args=args)

    controller = CartesianCuttingController()

    # 显示工作空间信息
    controller.print_workspace_info()

    # 示例：定义一个矩形的四个点（相对于base_link）
    center_x = controller.x0
    center_y = controller.y0
    center_z = controller.z0

    size = 0.05  # 5cm

    # 定义四个角点（逆时针）
    points = [
        (center_x, center_y - size/2, center_z - size/2),  # 左下
        (center_x, center_y + size/2, center_z - size/2),  # 右下
        (center_x, center_y + size/2, center_z + size/2),  # 右上
        (center_x, center_y - size/2, center_z + size/2),  # 左上
    ]

    controller.get_logger().info('示例：在初始位置附近画5cm×5cm矩形')
    controller.get_logger().info(f'矩形中心: X={center_x:.5f}, Y={center_y:.5f}, Z={center_z:.5f}')

    # 执行切割
    success = controller.cut_rectangle(points, rotation=0.0)

    if success:
        controller.get_logger().info('任务成功完成！')
    else:
        controller.get_logger().error('任务失败！')

    controller.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
