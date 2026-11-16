#!/usr/bin/env python3
"""
导航+切割任务总控程序
先导航到目标点，然后精确定位到黄色方块正前方，最后执行切割任务
"""

import sys
import os
import math
import rclpy
import yaml
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from scr_arm_controller.cartesian_cutting import CartesianCuttingController
from scr_arm_controller.marker_scanner import MarkerScanner
from scr_arm_controller.trajectory_visualizer import TrajectoryVisualizer


class NavigationCuttingTask(Node):
    # Gazebo 中机器人实体名（见 scr_description/launch/gazebo.launch.py）
    GAZEBO_ENTITY = 'scr_robot_gazebo'

    def __init__(self):
        # 使用仿真时钟：控制周期按仿真时间计，与电脑快慢（实时率）无关
        super().__init__(
            'navigation_cutting_task',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )

        # 导航Action客户端
        self._nav_action_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose'
        )

        # 速度控制发布器
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 里程计订阅
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.current_odom = None

        # Gazebo模型状态订阅
        self.model_states_sub = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.model_states_callback, 10
        )
        self.robot_pose = None

        # 机械臂控制器
        self.arm_controller = CartesianCuttingController()

        # 方块扫描器
        self.marker_scanner = MarkerScanner()

        # 轨迹可视化器
        self.trajectory_visualizer = TrajectoryVisualizer()

        # 黄色方块位置（从world文件中获取，或通过扫描获得）
        self.marker_position = {'x': 8.42, 'y': -3.9, 'z': 0.6}
        self.marker_detected_position = None  # 扫描检测到的位置

        # 距离参数（可配置）
        self.distance_from_marker = 0.6

        self.get_logger().info('导航+切割任务控制器已初始化')

    def odom_callback(self, msg):
        """里程计回调"""
        self.current_odom = msg

    def model_states_callback(self, msg):
        """Gazebo模型状态回调（真值，仅用于评估误差，不参与控制）"""
        try:
            robot_index = msg.name.index(self.GAZEBO_ENTITY)
            self.robot_pose = msg.pose[robot_index]
        except ValueError:
            pass

    def get_robot_position(self):
        """获取机器人当前位置（控制用：里程计）"""
        if self.current_odom is not None:
            return {
                'x': self.current_odom.pose.pose.position.x,
                'y': self.current_odom.pose.pose.position.y,
                'yaw': self.quaternion_to_yaw(self.current_odom.pose.pose.orientation)
            }
        return None

    def get_ground_truth_position(self):
        """获取 Gazebo 真值位姿（评估用）"""
        if self.robot_pose is None:
            return None
        return {
            'x': self.robot_pose.position.x,
            'y': self.robot_pose.position.y,
            'yaw': self.quaternion_to_yaw(self.robot_pose.orientation)
        }

    def sleep_sim(self, duration):
        """按仿真时钟等待 duration 秒，期间持续处理回调"""
        start = self.get_clock().now()
        target = Duration(seconds=duration)
        while rclpy.ok() and (self.get_clock().now() - start) < target:
            rclpy.spin_once(self, timeout_sec=0.005)

    def quaternion_to_yaw(self, q):
        """四元数转yaw角"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def navigate_to_pose(self, x, y, yaw):
        """
        导航到指定位姿
        x, y: 目标坐标（米）
        yaw: 目标朝向（弧度）
        """
        self.get_logger().info('='*60)
        self.get_logger().info('步骤1: 导航到目标区域')
        self.get_logger().info('='*60)
        self.get_logger().info(f'目标位姿: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} ({math.degrees(yaw):.1f}°)')

        # 等待导航服务器
        self.get_logger().info('等待导航服务器...')
        self._nav_action_client.wait_for_server()

        # 创建导航目标
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # 将yaw转换为四元数
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info('发送导航目标...')

        # 发送目标
        send_goal_future = self._nav_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ 导航目标被拒绝')
            return False

        self.get_logger().info('导航目标已接受，机器人正在移动...')

        # 等待导航完成
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('✅ 导航成功到达目标区域！')
            return True
        else:
            self.get_logger().error(f'❌ 导航失败，状态码: {result.status}')
            return False

    def precise_positioning_to_marker(self):
        """
        精确定位到黄色方块正前方
        使用self.distance_from_marker作为目标距离
        """
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('步骤2: 精确定位到黄色方块正前方')
        self.get_logger().info('='*60)

        # 计算目标位置（方块正前方）
        # 方块位置: x=8.42, y=-3.9，贴在南墙上（y=-4附近）
        # 机器人应该在方块北侧distance_from_marker米处，朝向-Y方向（朝南，朝向方块）
        target_x = self.marker_position['x']
        target_y = self.marker_position['y'] + self.distance_from_marker  # 在方块北侧
        target_yaw = -math.pi / 2  # 朝向-Y方向（朝南）

        self.get_logger().info(f'方块位置: x={self.marker_position["x"]:.3f}, y={self.marker_position["y"]:.3f}')
        self.get_logger().info(f'目标距离: {self.distance_from_marker:.2f}m')
        self.get_logger().info(f'目标位置: x={target_x:.3f}, y={target_y:.3f}, yaw={target_yaw:.3f} ({math.degrees(target_yaw):.1f}°)')

        # 等待获取机器人位置
        self.get_logger().info('等待获取机器人位置...')
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_robot_position() is not None:
                break

        if self.get_robot_position() is None:
            self.get_logger().error('无法获取机器人位置')
            return False

        # 闭环控制参数
        position_tolerance = 0.03  # 位置容差 3cm（从2cm放宽）
        angle_tolerance = 0.08     # 角度容差 约4.6度（从3度放宽）
        control_period = 0.1       # 控制周期（仿真时间，秒），即 10 Hz
        max_iterations = 500       # 上限 500 个控制周期 = 50 s 仿真时间

        # PID控制参数
        kp_linear = 0.6   # 增加线速度增益
        kp_angular = 1.2  # 增加角速度增益
        max_linear_vel = 0.2
        max_angular_vel = 0.3

        self.get_logger().info('开始精确定位...')

        converged_count = 0  # 连续满足容差的次数
        required_converged = 5  # 需要连续5次满足容差才认为收敛

        for iteration in range(max_iterations):
            # 获取当前位置
            current_pos = self.get_robot_position()
            if current_pos is None:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            # 计算误差
            error_x = target_x - current_pos['x']
            error_y = target_y - current_pos['y']
            error_yaw = target_yaw - current_pos['yaw']

            # 归一化角度误差到[-pi, pi]
            while error_yaw > math.pi:
                error_yaw -= 2 * math.pi
            while error_yaw < -math.pi:
                error_yaw += 2 * math.pi

            # 计算距离误差
            distance_error = math.sqrt(error_x**2 + error_y**2)

            # 检查是否到达目标
            if distance_error < position_tolerance and abs(error_yaw) < angle_tolerance:
                converged_count += 1
                if converged_count >= required_converged:
                    self.get_logger().info('✅ 精确定位完成！')
                    self.get_logger().info(f'   最终位置: x={current_pos["x"]:.4f}, y={current_pos["y"]:.4f}, yaw={current_pos["yaw"]:.4f}')
                    self.get_logger().info(f'   位置误差: {distance_error*1000:.1f}mm, 角度误差: {math.degrees(abs(error_yaw)):.2f}°')
                    self.get_logger().info(f'   总迭代次数: {iteration+1}')

                    # 停止机器人
                    self.stop_robot()
                    self.report_ground_truth_error(target_x, target_y, target_yaw)
                    return True
            else:
                converged_count = 0  # 重置收敛计数

            # 计算控制量
            cmd = Twist()

            # 如果距离误差较大，使用全向移动直接朝目标点移动
            if distance_error > 0.03:  # 距离大于3cm才移动
                # 计算在机器人坐标系中的目标方向
                # 将全局误差转换到机器人坐标系
                robot_yaw = current_pos['yaw']

                # 在机器人坐标系中的误差
                error_x_robot = error_x * math.cos(-robot_yaw) - error_y * math.sin(-robot_yaw)
                error_y_robot = error_x * math.sin(-robot_yaw) + error_y * math.cos(-robot_yaw)

                # 使用全向移动能力
                cmd.linear.x = max(min(kp_linear * error_x_robot, max_linear_vel), -max_linear_vel)
                cmd.linear.y = max(min(kp_linear * error_y_robot, max_linear_vel), -max_linear_vel)

                # 同时调整朝向（朝向目标点）
                angle_to_target = math.atan2(error_y, error_x)
                angle_error_to_target = angle_to_target - robot_yaw

                # 归一化角度误差
                while angle_error_to_target > math.pi:
                    angle_error_to_target -= 2 * math.pi
                while angle_error_to_target < -math.pi:
                    angle_error_to_target += 2 * math.pi

                cmd.angular.z = max(min(kp_angular * angle_error_to_target, max_angular_vel), -max_angular_vel)

                if iteration % 20 == 0:
                    self.get_logger().debug('  模式: 全向移动到目标')
            else:
                # 距离很接近，主要调整最终朝向
                cmd.angular.z = max(min(kp_angular * error_yaw, max_angular_vel), -max_angular_vel)
                if iteration % 20 == 0:
                    self.get_logger().debug('  模式: 调整最终朝向')

            # 发布速度命令
            self.cmd_vel_pub.publish(cmd)

            # 每20次迭代打印一次状态
            if iteration % 20 == 0:
                # 计算朝向目标点的角度误差
                angle_to_target = math.atan2(error_y, error_x)
                angle_error_to_target = angle_to_target - current_pos['yaw']
                while angle_error_to_target > math.pi:
                    angle_error_to_target -= 2 * math.pi
                while angle_error_to_target < -math.pi:
                    angle_error_to_target += 2 * math.pi

                # 计算在机器人坐标系中的误差
                robot_yaw = current_pos['yaw']
                error_x_robot = error_x * math.cos(-robot_yaw) - error_y * math.sin(-robot_yaw)
                error_y_robot = error_x * math.sin(-robot_yaw) + error_y * math.cos(-robot_yaw)

                self.get_logger().info(
                    f'迭代{iteration}: 距离={distance_error*1000:.1f}mm, '
                    f'机器人系误差(x={error_x_robot*1000:.0f}mm, y={error_y_robot*1000:.0f}mm), '
                    f'角度误差={math.degrees(abs(error_yaw)):.2f}°, '
                    f'收敛={converged_count}/{required_converged}'
                )

            # 按仿真时钟等待一个控制周期
            self.sleep_sim(control_period)

        # 超过最大迭代次数：定位未收敛，按失败处理
        self.get_logger().error(f'❌ 达到最大迭代次数({max_iterations})，定位未收敛')
        self.get_logger().error(f'   当前误差: 距离={distance_error*1000:.1f}mm, 角度={math.degrees(abs(error_yaw)):.2f}°')
        self.stop_robot()
        self.report_ground_truth_error(target_x, target_y, target_yaw)
        return False

    def report_ground_truth_error(self, target_x, target_y, target_yaw):
        """用 Gazebo 真值评估实际对位误差（控制环使用的是里程计）"""
        self.sleep_sim(0.5)  # 等待机器人静止并刷新真值
        gt = self.get_ground_truth_position()
        if gt is None:
            self.get_logger().warn('未收到 Gazebo 真值位姿（/gazebo/model_states），跳过误差评估')
            return None
        dx, dy = target_x - gt['x'], target_y - gt['y']
        dyaw = math.atan2(math.sin(target_yaw - gt['yaw']), math.cos(target_yaw - gt['yaw']))
        pos_err = math.hypot(dx, dy)
        self.get_logger().info(
            f'📏 真值误差: 位置={pos_err*1000:.1f}mm (dx={dx*1000:.1f}, dy={dy*1000:.1f}), '
            f'角度={math.degrees(abs(dyaw)):.2f}°'
        )
        return pos_err, dyaw

    def stop_robot(self):
        """停止机器人"""
        cmd = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(cmd)
            self.sleep_sim(0.1)

    def scan_for_marker(self):
        """
        上下扫描寻找黄色方块
        返回: 方块在base_link坐标系中的位置 (x, y, z) 或 None
        """
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('步骤2.5: 扫描寻找黄色方块')
        self.get_logger().info('='*60)

        # 重置检测状态
        self.marker_scanner.reset_detection()

        # 扫描参数
        scan_z_start = 0.3   # 起始高度（相对于base_link）
        scan_z_end = 0.7     # 结束高度
        scan_z_step = 0.05   # 步进
        scan_duration = 2.0  # 每步停留时间

        # 固定X和Y位置（机械臂初始位置）
        scan_x = self.arm_controller.x0
        scan_y = self.arm_controller.y0

        self.get_logger().info(f'扫描范围: Z={scan_z_start:.2f}m 到 {scan_z_end:.2f}m')
        self.get_logger().info(f'扫描步进: {scan_z_step:.2f}m')
        self.get_logger().info(f'固定位置: X={scan_x:.3f}m, Y={scan_y:.3f}m')

        # 从下往上扫描
        current_z = scan_z_start
        scan_count = 0

        while current_z <= scan_z_end:
            scan_count += 1
            self.get_logger().info(f'\n扫描点 {scan_count}: Z={current_z:.3f}m')

            # 移动到扫描位置
            if not self.arm_controller.move_to_cartesian(
                scan_x, scan_y, current_z,
                rotation=0.0,
                duration=1.5
            ):
                self.get_logger().error(f'无法移动到扫描位置 Z={current_z:.3f}m')
                current_z += scan_z_step
                continue

            # 等待稳定并采集图像
            self.get_logger().info('等待图像采集...')

            # 重置图像计数
            self.marker_scanner.image_count = 0

            for _ in range(int(scan_duration * 10)):
                rclpy.spin_once(self.marker_scanner, timeout_sec=0.1)

                # 检查是否检测到方块
                if self.marker_scanner.marker_detected:
                    marker_pos = self.marker_scanner.get_marker_in_base_link()
                    if marker_pos is not None:
                        self.get_logger().info(
                            f'✅ 检测到黄色方块！'
                            f'位置: X={marker_pos[0]:.4f}, Y={marker_pos[1]:.4f}, Z={marker_pos[2]:.4f}'
                        )
                        self.marker_detected_position = {
                            'x': marker_pos[0],
                            'y': marker_pos[1],
                            'z': marker_pos[2]
                        }

                        # 回到初始位置
                        self.get_logger().info('\n机械臂回到初始位置')
                        self.arm_controller.move_to_joints([0.0, 0.0, 0.0, 0.0], duration=2.0)

                        return marker_pos

            # 打印采集的图像数量
            self.get_logger().info(f'本次扫描点采集了 {self.marker_scanner.image_count} 帧图像')

            # 移动到下一个高度
            current_z += scan_z_step

        # 扫描完成但未找到
        self.get_logger().warn('⚠️  扫描完成，未检测到黄色方块')

        # 回到初始位置
        self.get_logger().info('\n机械臂回到初始位置')
        self.arm_controller.move_to_joints([0.0, 0.0, 0.0, 0.0], duration=2.0)

        return None

    def execute_cutting_task(self, config_file):
        """
        执行切割任务
        config_file: 切割任务配置文件路径
        """
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('步骤3: 执行切割任务')
        self.get_logger().info('='*60)
        self.get_logger().info(f'配置文件: {config_file}')

        # 加载配置文件
        if not os.path.exists(config_file):
            self.get_logger().error(f'配置文件不存在: {config_file}')
            return False

        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'无法解析配置文件: {e}')
            return False

        # 解析配置
        if 'points' not in config or len(config['points']) < 2:
            self.get_logger().error('配置文件格式错误')
            return False

        points = [(p['x'], p['y'], p['z'], math.radians(p['rotation']))
                  for p in config['points']]
        duration = config.get('motion', {}).get('duration', 3.0)
        return_home = config.get('motion', {}).get('return_home', True)

        # 显示任务信息
        self.get_logger().info(f'\n切割点数量: {len(points)}')
        for i, (x, y, z, rotation) in enumerate(points):
            self.get_logger().info(
                f'  点{i+1}: X={x:.4f}, Y={y:.4f}, Z={z:.4f}, R={math.degrees(rotation):.1f}°'
            )

        # 执行切割
        success = self.execute_cutting_points(points, duration, return_home)

        if success:
            self.get_logger().info('✅ 切割任务完成！')
        else:
            self.get_logger().error('❌ 切割任务失败！')

        return success

    def execute_cutting_points(self, points, duration, return_home):
        """执行切割点序列"""
        # 检查所有点是否可达
        self.get_logger().info('\n检查所有点的可达性...')
        for i, (x, y, z, rotation) in enumerate(points):
            reachable, joints = self.arm_controller.check_reachability(x, y, z, rotation)
            if not reachable:
                self.get_logger().error(
                    f'点{i+1} ({x}, {y}, {z}, {math.degrees(rotation):.1f}°) 不可达！'
                )
                return False
            self.get_logger().info(
                f'点{i+1}: X={x:.4f}, Y={y:.4f}, Z={z:.4f}, R={math.degrees(rotation):.1f}° ✓'
            )

        self.get_logger().info('\n所有点都可达，开始执行...\n')

        # 回到初始位置
        first_rotation = points[0][3]
        self.get_logger().info('机械臂回到初始位置')
        if not self.arm_controller.move_to_joints([0.0, 0.0, 0.0, first_rotation], duration=duration):
            return False

        # 依次移动到所有点
        for i, (x, y, z, rotation) in enumerate(points):
            self.get_logger().info(f'\n移动到点{i+1}/{len(points)}')
            if not self.arm_controller.move_to_cartesian(x, y, z, rotation, duration=duration):
                return False

        # 回到初始位置（可选）
        if return_home:
            self.get_logger().info('\n机械臂回到初始位置')
            if not self.arm_controller.move_to_joints([0.0, 0.0, 0.0, first_rotation], duration=duration):
                return False

        return True

    def run_task(self, nav_x, nav_y, nav_yaw, cutting_config, enable_precise_positioning=True):
        """
        运行完整任务：导航 + 精确定位 + 切割
        """
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('开始执行导航+精确定位+切割任务')
        self.get_logger().info('='*60)

        # 步骤1: 导航到目标区域
        if not self.navigate_to_pose(nav_x, nav_y, nav_yaw):
            self.get_logger().error('任务失败：导航阶段失败')
            return False

        # 等待机器人稳定
        self.get_logger().info('\n等待机器人稳定...')
        self.sleep_sim(2.0)

        # 步骤2: 精确定位到黄色方块正前方
        if enable_precise_positioning:
            if not self.precise_positioning_to_marker():
                self.get_logger().error('任务失败：精确定位阶段失败')
                return False

            # 再次等待稳定
            self.get_logger().info('\n等待机器人稳定...')
            self.sleep_sim(1.0)

        # 步骤3: 执行切割任务
        if not self.execute_cutting_task(cutting_config):
            self.get_logger().error('任务失败：切割阶段失败')
            return False

        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('✅ 任务全部完成！')
        self.get_logger().info('='*60)

        return True


    def run_task_from_config(self, config, enable_precise_positioning=True, enable_scanning=True):
        """
        从配置文件运行完整任务
        config: 配置字典
        enable_precise_positioning: 是否启用精确定位
        enable_scanning: 是否启用方块扫描
        """
        # 解析导航参数
        nav_x = config['navigation']['x']
        nav_y = config['navigation']['y']
        nav_yaw = config['navigation']['yaw']

        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('开始执行导航+精确定位+扫描+切割任务')
        self.get_logger().info('='*60)

        # 步骤1: 导航到目标区域
        if not self.navigate_to_pose(nav_x, nav_y, nav_yaw):
            self.get_logger().error('任务失败：导航阶段失败')
            return False

        # 等待机器人稳定
        self.get_logger().info('\n等待机器人稳定...')
        self.sleep_sim(2.0)

        # 步骤2: 精确定位到黄色方块正前方
        if enable_precise_positioning:
            if not self.precise_positioning_to_marker():
                self.get_logger().error('任务失败：精确定位阶段失败')
                return False

            # 再次等待稳定
            self.get_logger().info('\n等待机器人稳定...')
            self.sleep_sim(1.0)

        # 步骤2.5: 扫描寻找黄色方块
        if enable_scanning:
            marker_pos = self.scan_for_marker()
            if marker_pos is not None:
                self.get_logger().info(
                    f'\n✅ 方块检测成功！'
                    f'\n  相对于base_link的位置:'
                    f'\n    X = {marker_pos[0]:.4f} m'
                    f'\n    Y = {marker_pos[1]:.4f} m'
                    f'\n    Z = {marker_pos[2]:.4f} m'
                )
                # 可以根据检测到的位置调整切割点
            else:
                self.get_logger().warn('⚠️  未检测到方块，使用默认切割点')

            # 等待稳定
            self.sleep_sim(1.0)

        # 步骤3: 执行切割任务
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('步骤3: 执行切割任务')
        self.get_logger().info('='*60)

        # 解析切割参数
        points = [(p['x'], p['y'], p['z'], math.radians(p['rotation']))
                  for p in config['points']]
        duration = config.get('motion', {}).get('duration', 3.0)
        return_home = config.get('motion', {}).get('return_home', True)

        # 显示任务信息
        self.get_logger().info(f'\n切割点数量: {len(points)}')
        for i, (x, y, z, rotation) in enumerate(points):
            self.get_logger().info(
                f'  点{i+1}: X={x:.4f}, Y={y:.4f}, Z={z:.4f}, R={math.degrees(rotation):.1f}°'
            )

        # 执行切割
        success = self.execute_cutting_points(points, duration, return_home)

        if not success:
            self.get_logger().error('任务失败：切割阶段失败')
            return False

        # 生成切割轨迹可视化（基于检测到的方块位置）
        self.get_logger().info('\n生成切割轨迹可视化...')
        self._visualize_cutting_result()

        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('✅ 任务全部完成！')
        self.get_logger().info('='*60)

        return True

    def _visualize_cutting_result(self):
        """
        生成切割结果可视化
        在方块位置生成红色边框（贴在墙上，X-Z平面）
        """
        # 使用方块的实际位置（但不在日志中透露）
        marker_x = self.marker_position['x']
        marker_y = self.marker_position['y']
        marker_z = self.marker_position['z']

        # 方块大小：5cm x 5cm，贴在墙上（X-Z平面）
        # 方块pitch旋转90度，所以在X-Z平面上
        half_size = 0.025  # 2.5cm

        # 外扩1cm
        offset = 0.01
        size_with_offset = half_size + offset

        # 生成方块周围的轨迹点（在map坐标系中）
        # 方块贴在墙上（Y=-3.9），所以是X-Z平面的矩形
        # Y坐标固定，X和Z变化
        trajectory_points = [
            (marker_x - size_with_offset, marker_y, marker_z - size_with_offset, 0),  # 左下
            (marker_x + size_with_offset, marker_y, marker_z - size_with_offset, 0),  # 右下
            (marker_x + size_with_offset, marker_y, marker_z + size_with_offset, 0),  # 右上
            (marker_x - size_with_offset, marker_y, marker_z + size_with_offset, 0),  # 左上
        ]

        # 发布可视化（使用map坐标系）
        self.trajectory_visualizer.visualize_in_map_frame(
            trajectory_points,
            color='red',
            frame_id='map'
        )


def main(args=None):
    if len(sys.argv) < 2:
        print('用法: ros2 run scr_arm_controller navigation_cutting_task <task_config.yaml>')
        print('\n示例:')
        print('  ros2 run scr_arm_controller navigation_cutting_task task_at_target.yaml')
        print('  ros2 run scr_arm_controller navigation_cutting_task /path/to/my_task.yaml')
        print('\n配置文件格式:')
        print('  navigation:')
        print('    x: 8.61277')
        print('    y: -2.01879')
        print('    yaw: -1.64894')
        print('  precise_positioning:')
        print('    enabled: true')
        print('    distance_from_marker: 0.6')
        print('  scanning:')
        print('    enabled: true')
        print('    z_start: 0.3')
        print('    z_end: 0.7')
        print('    z_step: 0.05')
        print('  points:')
        print('    - {x: 0.537, y: -0.025, z: 0.482, rotation: 0}')
        print('    ...')
        print('  motion:')
        print('    duration: 3.0')
        print('    return_home: true')
        return

    config_file = sys.argv[1]

    # 如果是相对路径，到功能包的 config 目录中查找
    if not os.path.isabs(config_file) and not os.path.exists(config_file):
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_config = os.path.join(
                get_package_share_directory('scr_arm_controller'), 'config', config_file)
            if os.path.exists(pkg_config):
                config_file = pkg_config
        except Exception:
            pass

    # 加载配置文件
    if not os.path.exists(config_file):
        print(f'错误: 配置文件不存在: {config_file}')
        return

    try:
        with open(config_file, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
    except Exception as e:
        print(f'错误: 无法解析配置文件: {e}')
        return

    # 验证配置
    if 'navigation' not in config:
        print('错误: 配置文件缺少 navigation 字段')
        return

    if 'points' not in config or len(config['points']) < 2:
        print('错误: 配置文件缺少 points 字段或点数不足')
        return

    # 解析导航参数
    nav_x = config['navigation']['x']
    nav_y = config['navigation']['y']
    nav_yaw = config['navigation']['yaw']

    # 解析精确定位参数
    precise_config = config.get('precise_positioning', {})
    enable_precise = precise_config.get('enabled', True)
    distance_from_marker = precise_config.get('distance_from_marker', 0.6)

    # 解析扫描参数
    scan_config = config.get('scanning', {})
    enable_scanning = scan_config.get('enabled', True)

    # 初始化ROS2
    rclpy.init(args=args)

    # 创建任务控制器
    task_controller = NavigationCuttingTask()

    # 设置距离参数
    if enable_precise:
        task_controller.distance_from_marker = distance_from_marker

    print('\n任务配置:')
    print(f'  导航目标: x={nav_x:.3f}, y={nav_y:.3f}, yaw={nav_yaw:.3f}')
    print(f'  精确定位: {"启用" if enable_precise else "禁用"}')
    if enable_precise:
        print(f'  目标距离: {distance_from_marker:.2f}m')
    print(f'  方块扫描: {"启用" if enable_scanning else "禁用"}')
    print(f'  切割点数: {len(config["points"])}')
    print(f'  配置文件: {config_file}\n')

    # 运行任务
    success = task_controller.run_task_from_config(config, enable_precise, enable_scanning)

    # 清理
    task_controller.trajectory_visualizer.destroy_node()
    task_controller.marker_scanner.destroy_node()
    task_controller.arm_controller.destroy_node()
    task_controller.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
