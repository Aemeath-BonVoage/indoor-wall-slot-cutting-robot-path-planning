#!/usr/bin/env python3
"""
导航+切割任务总控程序（项目唯一的执行主路径）.

两种任务文件：
1. 单站任务（task_at_target.yaml 格式）：导航到目标点 → 视觉对位到黄色方块正前方
   → 扫描 → 定点保持下切割。
2. 离线开槽规划（scr_planner/scripts/plan_wall_slots.py 的输出，含 wall / stations）：
   导航到标记前 → 视觉对位（确定墙面坐标原点）→ 逐站：横移（远站先用 Nav2）到站
   → 定点保持 → 测残差并补偿机械臂点 → 切割 → 下一站。执行中不重新规划。

  ros2 run scr_arm_controller navigation_cutting_task <task.yaml | plan.yaml>
      [--dry-run-gap=0.01] [--no-compensation] [--max-strafe=1.5]
"""

import sys
import os
import math
import time
import numpy as np
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
from scr_arm_controller.trajectory_visualizer import TrajectoryVisualizer
from scr_arm_controller.base_hold import BaseHold
from scr_arm_controller.visual_servo import pbvs_command
from scr_arm_controller.visual_servo_ros import WallMarkerServo


# ---------------------------------------------------------------------------
# 离线开槽规划文件的解析（纯函数，不依赖 ROS；格式见 scr_planner/python/slot_plan.py
# plan_to_dict）。墙面坐标：u 沿墙向右（面对墙面时），v 离地高度。
# ---------------------------------------------------------------------------
def is_slot_plan(config):
    """任务文件是否为离线开槽规划（而不是单站任务）."""
    return isinstance(config, dict) and 'stations' in config and 'wall' in config


def plan_wall_axes(plan):
    """墙面系 → map：返回 (p0 左端点, t 沿墙向右单位向量, n 指向房间的法向)."""
    p0 = np.array(plan['wall']['left_end'], dtype=float)
    p1 = np.array(plan['wall']['right_end'], dtype=float)
    t = (p1 - p0) / np.linalg.norm(p1 - p0)
    return p0, t, np.array([t[1], -t[0]])


def plan_wall_pose(plan, u, standoff):
    """正对墙面、base_link 中心离墙 standoff、位于墙面 u 处的 map 位姿 (x, y, yaw)."""
    p0, t, n = plan_wall_axes(plan)
    p = p0 + u * t + standoff * n
    return float(p[0]), float(p[1]), math.atan2(-n[1], -n[0])


def station_arm_points(station, plan, dry_run_gap=None):
    """
    一个停靠站的机械臂点 → ([(x, y, z, rotation_rad)], [是否处于切深]).

    dry_run_gap（米）：空切，切深处的点改为刀尖停在规划墙面前 gap 处（Gazebo 的墙是
    刚体，真的下刀会把整台机器人顶开）；退刀点不变。
    """
    r = plan['robot']
    x_cut = r['x_cut']
    x_dry = None if dry_run_gap is None else r['standoff'] - dry_run_gap - r['tool_offset']
    pts, is_cut = [], []
    for p in station['points']:
        cut = abs(p['x'] - x_cut) < 1e-6
        x = x_dry if (cut and x_dry is not None) else p['x']
        pts.append((x, p['y'], p['z'], math.radians(p['rotation'])))
        is_cut.append(cut)
    return pts, is_cut


def compensate_points(points, is_cut, e_l, e_n, psi, tool_offset, x_range, y_range):
    """
    把机械臂点修正到实际停靠位姿下（小角度近似）.

    e_l：底盘横向误差（米，沿墙向右为正）；e_n：离墙偏远量（米）；psi：朝向误差（rad，
    逆时针为正）。刀尖在 base 系的横向位置 fy 对应墙面 u ≈ u_base - fx·psi - fy，令其等于
    名义值得 fy' = fy + e_l - fx·psi（fx = 刀尖在 base 系的前伸）。切深处的点离墙偏远则
    多伸出 e_n；退刀点不动。超出关节限位的分量钳制到限位，返回 (新点列, 被钳制点数)。
    """
    (xlo, xhi), (ylo, yhi) = x_range, y_range
    out, clamped = [], 0
    for (x, y, z, rot), cut in zip(points, is_cut):
        fx = x + tool_offset
        y2 = y + e_l - fx * psi
        x2 = x + e_n if cut else x
        y3, x3 = min(max(y2, ylo), yhi), min(max(x2, xlo), xhi)
        clamped += int(abs(y3 - y2) > 1e-9 or abs(x3 - x2) > 1e-9)
        out.append((x3, y3, z, rot))
    return out, clamped


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

        # 轨迹可视化器
        self.trajectory_visualizer = TrajectoryVisualizer()

        # 黄色方块位置（从world文件中获取，或通过扫描获得）
        self.marker_position = {'x': 8.42, 'y': -3.9245, 'z': 0.6}  # 贴在南墙内表面 y=-3.925
        self.marker_detected_position = None  # 扫描检测到的位置

        # 精确定位参数（可由任务配置 precise_positioning 段覆盖）
        self.distance_from_marker = 0.6
        self.position_tolerance = 0.01             # 1 cm
        self.angle_tolerance = math.radians(1.0)   # 1°
        # 对位模式：'visual' = 相机+激光 PBVS；'odom' = 里程计 + 已知方块坐标（基线）
        self.positioning_mode = 'visual'
        self.observe_joint1 = -0.15                # 视觉伺服时的滑台高度（避开机械臂自遮挡）
        self.odom_bias = (0.0, 0.0, 0.0)           # 实验用：注入里程计漂移 (dx, dy, dyaw)
        # 接近墙面时机械臂完全缩回（joint3 = +0.02 → 刀具前端距底盘中心 0.676 m），
        # 切割时再伸出；对位距离 0.70 m 时留出约 2.4 cm 间隙
        self.approach_joint3 = 0.02
        self.min_clearance = 0.005                 # 防碰撞守护：刀具前端与墙面的最小间隙（米）
        self.guard_horizon = 0.5                   # 防碰撞守护：制动时间常数（秒）
        # 空切间隙（米）：仿真墙为刚体，切割时刀具停在墙前该距离；None 表示按原切割点执行
        self.dry_run_gap = None
        self.guard_enabled = True
        self._guard_active = False
        # 扫描 / 切割期间底盘定点保持（机械臂反作用力会推动底盘）
        self.hold_base = True
        self.base_hold = BaseHold(entity=self.GAZEBO_ENTITY)

        # 多站开槽规划执行（run_slot_plan）
        self.max_strafe = 1.5          # 站间横移超过该距离（米）时先退到离墙 nav_standoff 用 Nav2 过去
        self.nav_standoff = 1.2        # Nav2 目标离墙距离（米）：远离墙面，留出代价地图膨胀余量
        self.strafe_tol = (0.003, 0.005, math.radians(1.0))   # 到站容差：横向 / 离墙 / 朝向
        self.compensate = True         # 到站后按残余误差修正机械臂点
        self.cur_station = -1          # 当前停靠站序号（-1 = 不在站上）；供实验记录读取
        self.phase = 'idle'            # idle / move / cut：相邻两点都在切深时为 cut

        # 视觉伺服测量器（激光拟合墙面 + 相机检测方块）
        self.servo = WallMarkerServo(self)

        self.get_logger().info('导航+切割任务控制器已初始化')

    def destroy_node(self):
        # 先停掉定点保持的后台线程，避免 rclpy.shutdown() 时进程 abort
        self.base_hold.shutdown()
        return super().destroy_node()

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
        """
        获取机器人当前位置（控制用：里程计）.

        self.odom_bias = (dx, dy, dyaw) 用于实验中模拟里程计累积漂移（默认为 0）。
        """
        if self.current_odom is not None:
            dx, dy, dyaw = self.odom_bias
            return {
                'x': self.current_odom.pose.pose.position.x + dx,
                'y': self.current_odom.pose.pose.position.y + dy,
                'yaw': self.quaternion_to_yaw(self.current_odom.pose.pose.orientation) + dyaw
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

    def navigate_to_pose(self, x, y, yaw, nav_retries=10):
        """
        导航到指定位姿

        x, y: 目标坐标（米）
        yaw: 目标朝向（弧度）
        nav_retries: 目标被拒绝时的重试次数
        """
        self.get_logger().info('='*60)
        self.get_logger().info('步骤1: 导航到目标区域')
        self.get_logger().info('='*60)
        self.get_logger().info(f'目标位姿: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} '
                               f'({math.degrees(yaw):.1f}°)')

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

        # 发送目标。action 服务器先于 bt_navigator 激活出现，激活前的目标会被拒绝
        # （启动后立即运行任务时实测如此），故被拒时等待后重试
        for attempt in range(nav_retries + 1):
            send_goal_future = self._nav_action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()
            if goal_handle.accepted:
                break
            if attempt == nav_retries:
                self.get_logger().error('❌ 导航目标被拒绝')
                return False
            self.get_logger().warn(f'导航目标被拒绝（Nav2 可能尚未激活），3 s 后重试 '
                                   f'{attempt + 1}/{nav_retries}')
            self.sleep_sim(3.0)
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

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

        self.get_logger().info(f'方块位置: x={self.marker_position["x"]:.3f}, '
                               f'y={self.marker_position["y"]:.3f}')
        self.get_logger().info(f'目标距离: {self.distance_from_marker:.2f}m')
        self.get_logger().info(f'目标位置: x={target_x:.3f}, y={target_y:.3f}, '
                               f'yaw={target_yaw:.3f} ({math.degrees(target_yaw):.1f}°)')

        if self.positioning_mode == 'visual':
            # 视觉伺服：控制只用传感器测量；上面的目标位姿仅用于真值评估
            return self.precise_positioning_visual(target_x, target_y, target_yaw)

        self.get_logger().info('对位模式: odom（里程计 + 已知方块坐标，基线）')
        self.move_arm_for_approach(joint1=0.0)
        # 等待获取机器人位置
        self.get_logger().info('等待获取机器人位置...')
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_robot_position() is not None:
                break

        if self.get_robot_position() is None:
            self.get_logger().error('无法获取机器人位置')
            return False

        # 闭环控制参数（全向底盘：平移与转向解耦，同时控制）
        position_tolerance = self.position_tolerance   # 位置容差（米）
        angle_tolerance = self.angle_tolerance         # 角度容差（弧度）
        control_period = 0.1       # 控制周期（仿真时间，秒），即 10 Hz
        max_iterations = 500       # 上限 500 个控制周期 = 50 s 仿真时间

        # P 控制参数
        kp_linear = 0.8
        kp_angular = 1.5
        max_linear_vel = 0.2
        max_angular_vel = 0.3
        min_linear_vel = 0.01      # 最小平移速度：避免接近目标时速度过小、迟迟进不了容差
        min_angular_vel = 0.02

        self.get_logger().info(
            f'开始精确定位... 容差: {position_tolerance*1000:.0f}mm / '
            f'{math.degrees(angle_tolerance):.1f}°')

        converged_count = 0  # 连续满足容差的次数
        required_converged = 5  # 需要连续5次满足容差才认为收敛

        for iteration in range(max_iterations):
            # 获取当前位置
            current_pos = self.get_robot_position()
            if current_pos is None:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            # 计算误差（全局系）
            error_x = target_x - current_pos['x']
            error_y = target_y - current_pos['y']
            error_yaw = math.atan2(math.sin(target_yaw - current_pos['yaw']),
                                   math.cos(target_yaw - current_pos['yaw']))
            distance_error = math.hypot(error_x, error_y)

            # 检查是否到达目标（位置与角度同时满足，连续若干周期）
            if distance_error < position_tolerance and abs(error_yaw) < angle_tolerance:
                converged_count += 1
                if converged_count >= required_converged:
                    self.get_logger().info('✅ 精确定位完成！')
                    self.get_logger().info(f'   最终位置: x={current_pos["x"]:.4f}, '
                                           f'y={current_pos["y"]:.4f}, '
                                           f'yaw={current_pos["yaw"]:.4f}')
                    self.get_logger().info(f'   位置误差: {distance_error*1000:.1f}mm, 角度误差: '
                                           f'{math.degrees(abs(error_yaw)):.2f}°')
                    self.get_logger().info(f'   总迭代次数: {iteration+1}')
                    self.stop_robot()
                    self.report_ground_truth_error(target_x, target_y, target_yaw)
                    return True
            else:
                converged_count = 0

            # 全局误差 → 机器人坐标系
            robot_yaw = current_pos['yaw']
            error_x_robot = error_x * math.cos(robot_yaw) + error_y * math.sin(robot_yaw)
            error_y_robot = -error_x * math.sin(robot_yaw) + error_y * math.cos(robot_yaw)

            cmd = Twist()
            # 平移：始终闭环到目标点，直到进入容差的一半（留出余量，避免停在容差边缘）
            if distance_error > position_tolerance * 0.5:
                speed = min(max(kp_linear * distance_error, min_linear_vel), max_linear_vel)
                cmd.linear.x = speed * error_x_robot / distance_error
                cmd.linear.y = speed * error_y_robot / distance_error
            # 转向：始终直接对准目标朝向（全向底盘无需先转向目标点）
            if abs(error_yaw) > angle_tolerance * 0.5:
                w = min(max(kp_angular * abs(error_yaw), min_angular_vel), max_angular_vel)
                cmd.angular.z = math.copysign(w, error_yaw)

            # 发布速度命令（经激光防碰撞守护）
            self.cmd_vel_pub.publish(self.apply_wall_guard(cmd))

            # 每20次迭代打印一次状态
            if iteration % 20 == 0:
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
        self.get_logger().error(f'   当前误差: 距离={distance_error*1000:.1f}mm, '
                                f'角度={math.degrees(abs(error_yaw)):.2f}°')
        self.stop_robot()
        self.report_ground_truth_error(target_x, target_y, target_yaw)
        return False

    def apply_wall_guard(self, cmd):
        """
        激光防碰撞守护：按制动距离限制朝向墙面的速度分量.

        允许的向墙速度 = max(0, (间隙 − min_clearance) / guard_horizon)，越靠近越慢，
        间隙到 min_clearance 时降为 0。若只在间隙已小于阈值时才截断，10 Hz 控制周期内
        机器人会以 0.1–0.2 m/s 冲过阈值 1–2 cm（实测）。
        与对位模式无关（odom / visual 都生效），只依赖激光拟合的墙面与当前机械臂位姿。
        """
        if not self.guard_enabled:
            return cmd
        wall, _ = self.servo.wall_in_base()
        if wall is None:
            return cmd
        try:
            front = self.servo.tool_front_x()
        except Exception:  # noqa: BLE001
            return cmd
        nx, ny = wall['normal']
        # 刀具前端（base_link x 轴上）到墙面的垂直间隙
        clearance = wall['distance'] - front * nx
        toward = cmd.linear.x * nx + cmd.linear.y * ny
        allowed = max(0.0, (clearance - self.min_clearance) / self.guard_horizon)
        if toward > allowed:
            cut = toward - allowed
            cmd.linear.x -= cut * nx
            cmd.linear.y -= cut * ny
            if allowed == 0.0 and not self._guard_active:
                self.get_logger().warn(
                    f'🛡️ 防碰撞守护：刀具前端距墙 {clearance*1000:.1f} mm，已阻止向墙运动')
            self._guard_active = allowed == 0.0
        else:
            self._guard_active = False
        return cmd

    def move_arm_for_approach(self, joint1=0.0):
        """接近墙面前把机械臂移到安全位姿：滑台高度 joint1，伸缩轴完全缩回."""
        self.get_logger().info(
            f'机械臂移至接近位姿 joint1={joint1:.2f} m, joint3={self.approach_joint3:+.3f} m（缩回）')
        self.arm_controller.move_to_joints([joint1, 0.0, self.approach_joint3, 0.0], duration=2.0)
        self.sleep_sim(0.5)

    def precise_positioning_visual(self, target_x, target_y, target_yaw):
        """相机 + 激光融合 PBVS 对位（不使用里程计与已知方块坐标）."""
        D = self.distance_from_marker
        pos_tol, ang_tol = self.position_tolerance, self.angle_tolerance
        control_period = 0.1
        max_iterations = 500
        lost_limit = 20            # 连续 20 个周期（2 s）测量失败则判定失败

        self.get_logger().info(
            f'对位模式: visual（激光拟合墙面 + 相机检测方块），目标离墙 {D:.2f} m，'
            f'容差 {pos_tol*1000:.0f}mm / {math.degrees(ang_tol):.1f}°')

        # 观测位姿：相机与机械臂同在升降滑台上，机械臂会遮挡画面中下部；
        # 降低滑台使相机略低于方块，方块成像在画面上部，避开自遮挡区域
        self.move_arm_for_approach(joint1=self.observe_joint1)
        self.servo.reset_filter()

        converged, lost, required = 0, 0, 5
        m = None
        for it in range(max_iterations):
            m, why = self.servo.measure(D)
            if m is None:
                lost += 1
                self.cmd_vel_pub.publish(Twist())          # 测量丢失时原地停住
                if lost == 1 or lost % 10 == 0:
                    self.get_logger().warn(f'测量失败（{lost}）: {why}')
                if lost >= lost_limit:
                    self.get_logger().error('❌ 视觉测量持续丢失，对位失败')
                    self.stop_robot()
                    return False
                self.sleep_sim(control_period)
                continue
            lost = 0
            e = m['err']
            pos_err = math.hypot(e['normal'], e['lateral'])

            if pos_err < pos_tol and abs(e['yaw']) < ang_tol:
                converged += 1
                if converged >= required:
                    self.stop_robot()
                    self.get_logger().info('✅ 视觉伺服对位完成！')
                    self.get_logger().info(
                        f'   传感器误差: 法向={e["normal"]*1000:.1f}mm, 横向={e["lateral"]*1000:.1f}mm, '
                        f'角度={math.degrees(abs(e["yaw"])):.2f}°, 迭代 {it+1}')
                    self.report_ground_truth_error(target_x, target_y, target_yaw)
                    self.report_perception_error(m)
                    return True
            else:
                converged = 0

            vx, vy, wz = pbvs_command(e, pos_deadband=pos_tol * 0.5, ang_deadband=ang_tol * 0.5)
            cmd = Twist()
            cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz
            self.cmd_vel_pub.publish(self.apply_wall_guard(cmd))

            if it % 20 == 0:
                w = m['wall']
                self.get_logger().info(
                    f'迭代{it}: 离墙={w["distance"]:.3f}m 法向误差={e["normal"]*1000:.0f}mm '
                    f'横向={e["lateral"]*1000:.0f}mm 角度={math.degrees(e["yaw"]):.2f}° '
                    f'(墙面拟合 {w["inliers"]} 点, rms {w["rms"]*1000:.1f}mm, '
                    f'方块像素 {m["pixel"][0]:.0f},{m["pixel"][1]:.0f})')
            self.sleep_sim(control_period)

        self.get_logger().error(f'❌ 达到最大迭代次数({max_iterations})，视觉伺服未收敛')
        self.stop_robot()
        self.report_ground_truth_error(target_x, target_y, target_yaw)
        return False

    def report_perception_error(self, m):
        """用 Gazebo 真值检查感知精度：方块在 base_link 中的估计位置 vs 真值."""
        gt = self.get_ground_truth_position()
        if gt is None:
            return
        rx = self.marker_position['x'] - gt['x']
        ry = self.marker_position['y'] - gt['y']
        c, s = math.cos(gt['yaw']), math.sin(gt['yaw'])
        true_b = (c * rx + s * ry, -s * rx + c * ry)
        est = m['marker_base']
        self.get_logger().info(
            f'🔍 感知误差: 方块估计 base_link=({est[0]:.4f}, {est[1]:.4f}, {est[2]:.4f}) '
            f'真值=({true_b[0]:.4f}, {true_b[1]:.4f}, {self.marker_position["z"] - 0.1024:.4f}) '
            f'→ dx={(est[0]-true_b[0])*1000:.1f}mm dy={(est[1]-true_b[1])*1000:.1f}mm')

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

    def scan_for_marker(self, samples=5):
        """
        检测黄色方块的三维位置（相机射线与激光拟合墙面求交，见 visual_servo_ros）.

        先在当前位姿检测（视觉伺服结束时方块通常已在视野中）；未检测到时再按高度逐步扫描。
        返回: 方块在 base_link 中的位置 (x, y, z)（多帧平均）或 None
        """
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('步骤2.5: 扫描寻找黄色方块')
        self.get_logger().info('='*60)

        def try_measure():
            pts = []
            for _ in range(samples * 4):
                m, _ = self.servo.measure(self.distance_from_marker)
                if m is not None:
                    pts.append(m['marker_base'])
                    if len(pts) >= samples:
                        break
                self.sleep_sim(0.1)
            if len(pts) < samples:
                return None
            mean = [sum(p[k] for p in pts) / len(pts) for k in range(3)]
            return tuple(float(v) for v in mean)

        heights = [None] + [round(0.3 + 0.05 * i, 2) for i in range(9)]   # None = 当前位姿
        marker_pos = None
        for z in heights:
            if z is not None:
                self.get_logger().info(f'扫描高度 Z={z:.2f} m')
                if not self.arm_controller.move_to_cartesian(
                        self.arm_controller.x0 - self.approach_joint3, self.arm_controller.y0, z,
                        rotation=0.0, duration=1.5):
                    continue
                self.sleep_sim(0.3)
            marker_pos = try_measure()
            if marker_pos is not None:
                break

        if marker_pos is None:
            self.get_logger().warn('⚠️  扫描完成，未检测到黄色方块')
        else:
            self.get_logger().info(
                f'✅ 检测到黄色方块！位置: X={marker_pos[0]:.4f}, Y={marker_pos[1]:.4f}, '
                f'Z={marker_pos[2]:.4f}（{samples} 帧平均）')
            self.marker_detected_position = dict(zip('xyz', marker_pos))
            self.report_perception_error({'marker_base': marker_pos})

        # 回到接近位姿（伸缩轴缩回，保持离墙余量）
        self.arm_controller.move_to_joints([0.0, 0.0, self.approach_joint3, 0.0], duration=2.0)
        return marker_pos

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

    def apply_dry_run_gap(self, points):
        """
        空切：把整组切割点沿 x 平移，使最深点的刀具前端停在墙前 dry_run_gap 处.

        Gazebo 的墙是刚体，刀具压到墙上会把整台机器人顶开（实测约 9 mm、3–4°），
        因此仿真中只验证切割轨迹在墙面上的位置，不模拟切入。墙面距离由激光实时测量。
        """
        # 机械臂动作期间主节点回调被阻塞，先处理一会儿回调以刷新激光数据
        self.sleep_sim(0.3)
        wall, why = self.servo.wall_in_base()
        if wall is None:
            self.get_logger().warn(f'空切：无法测量墙面（{why}），按原切割点执行')
            return points
        tool_offset = self.servo.tool_front_x() - self.servo._lookup('base_link', 'link4')[0][0]
        x_max = wall['distance'] / max(wall['normal'][0], 1e-3) - self.dry_run_gap - tool_offset
        deepest = max(p[0] for p in points)
        if deepest <= x_max:
            return points
        shift = x_max - deepest
        x_min = self.arm_controller.cartesian_limits['x'][0]
        shifted = [(max(x + shift, x_min), y, z, r) for x, y, z, r in points]
        self.get_logger().info(
            f'空切：墙面距离 {wall["distance"]:.3f} m，切割点整体后移 {-shift*1000:.1f} mm，'
            f'刀具最深处停在墙前 {self.dry_run_gap*1000:.0f} mm')
        return shifted

    def execute_cutting_points(self, points, duration, return_home):
        """执行切割点序列"""
        if self.dry_run_gap is not None:
            points = self.apply_dry_run_gap(points)
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
        start_joints = [0.0, 0.0, self.approach_joint3, first_rotation]
        if not self.arm_controller.move_to_joints(start_joints, duration=duration):
            return False

        # 依次移动到所有点
        for i, (x, y, z, rotation) in enumerate(points):
            self.get_logger().info(f'\n移动到点{i+1}/{len(points)}')
            if not self.arm_controller.move_to_cartesian(x, y, z, rotation, duration=duration):
                return False

        # 回到初始位置（可选）
        if return_home:
            self.get_logger().info('\n机械臂回到初始位置')
            start_joints = [0.0, 0.0, self.approach_joint3, first_rotation]
            if not self.arm_controller.move_to_joints(start_joints, duration=duration):
                return False

        return True

    def run_task(self, nav_x, nav_y, nav_yaw, cutting_config, enable_precise_positioning=True):
        """运行完整任务：导航 + 精确定位 + 切割"""
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

        # 从这里到切割结束，机械臂动作会推动底盘：开启定点保持（并用真值记录漂移）
        self.base_hold.enabled = self.hold_base
        self.base_hold.hold()
        self.get_logger().info(
            f'底盘定点保持: {"开启" if self.hold_base else "关闭（仅测量漂移）"}')
        try:
            return self._scan_and_cut(config, enable_scanning)
        finally:
            drift = self.base_hold.release()
            if drift is not None:
                self.get_logger().info(
                    f'📏 扫描+切割期间底盘真值最大漂移: {drift[0]*1000:.1f} mm, {drift[1]:.2f}°')

    def _scan_and_cut(self, config, enable_scanning):
        """步骤 2.5 扫描 + 步骤 3 切割（在底盘定点保持下执行）."""
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

    # ------------------------------------------------------------------
    # 多站开槽：执行 plan_wall_slots.py 离线生成的规划文件
    # ------------------------------------------------------------------
    def sim_now(self):
        """仿真时间（秒）：取自定点保持的后台节点（机械臂动作时主节点不处理回调，时钟会停）."""
        return self.base_hold.node.get_clock().now().nanoseconds * 1e-9

    def strafe_to(self, du_target, ref, max_iter=600, alpha=0.3):
        """
        横移到相对参考位姿 du_target（沿墙向右为正，米）处，保持离墙 distance_from_marker.

        横向用里程计（短距离内几乎无漂移、无噪声）；离墙距离与朝向用激光拟合墙面，
        单帧拟合 rms 约 9 mm、朝向抖动约 ±1°，故做指数平滑（alpha）。剩余朝向误差由
        到站后的机械臂点补偿消除。也用于从 Nav2 的离墙位姿（nav_standoff）接近墙面。
        """
        tol_lat, tol_norm, tol_yaw = self.strafe_tol
        D = self.distance_from_marker
        right = np.array([math.sin(ref['yaw']), -math.cos(ref['yaw'])])   # 面墙时的右方
        ok, filt = 0, None
        for _ in range(max_iter):
            pose = self.get_robot_position()
            wall, _ = self.servo.wall_in_base()
            if pose is None or wall is None:
                self.cmd_vel_pub.publish(Twist())
                self.sleep_sim(0.1)
                continue
            raw = np.array([wall['distance'] - D, wall['yaw_error']])
            filt = raw if filt is None else (1 - alpha) * filt + alpha * raw
            du = float((np.array([pose['x'], pose['y']]) - ref['p']) @ right)
            n = np.array(wall['normal'])
            err = {'normal': float(filt[0]), 'lateral': -(du_target - du),
                   'yaw': float(filt[1]), 'n': n, 't': np.array([-n[1], n[0]])}
            if abs(err['lateral']) < tol_lat and abs(err['normal']) < tol_norm and \
                    abs(err['yaw']) < tol_yaw:
                ok += 1
                if ok >= 5:
                    self.stop_robot()
                    return True
            else:
                ok = 0
            vx, vy, wz = pbvs_command(err, max_lin=0.1, pos_deadband=tol_lat * 0.5,
                                      ang_deadband=tol_yaw * 0.5)
            cmd = Twist()
            cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz
            self.cmd_vel_pub.publish(self.apply_wall_guard(cmd))
            self.sleep_sim(0.1)
        self.stop_robot()
        return False

    def measure_residual(self, du_target, ref, n=10):
        """
        到站后的残余误差（只用机器人自身传感器）→ (e_l, e_n, psi).

        e_l：横向（里程计，米，沿墙向右为正）；e_n：离墙（激光拟合墙面，米，>0 表示
        离墙偏远）；psi：朝向（rad，逆时针为正）。
        """
        right = np.array([math.sin(ref['yaw']), -math.cos(ref['yaw'])])
        ls, ns, ys = [], [], []
        for _ in range(n * 3):
            pose = self.get_robot_position()
            wall, _ = self.servo.wall_in_base()
            if pose is not None and wall is not None:
                ls.append(float((np.array([pose['x'], pose['y']]) - ref['p']) @ right)
                          - du_target)
                ns.append(wall['distance'] - self.distance_from_marker)
                ys.append(-wall['yaw_error'])   # yaw_error > 0 需逆时针转 → 机器人偏顺时针
                if len(ls) >= n:
                    break
            self.sleep_sim(0.1)
        if not ls:
            return 0.0, 0.0, 0.0
        return float(np.mean(ls)), float(np.mean(ns)), float(np.mean(ys))

    def run_points(self, points, is_cut, duration):
        """逐点执行机械臂点，并标注阶段（self.phase）：相邻两点都在切深时为 'cut'."""
        prev_cut = False
        for (x, y, z, rot), cut in zip(points, is_cut):
            self.phase = 'cut' if (prev_cut and cut) else 'move'
            if not self.arm_controller.move_to_cartesian(x, y, z, rot, duration=duration):
                self.phase = 'idle'
                return False
            prev_cut = cut
        self.phase = 'idle'
        return True

    def on_station_arrived(self, station):
        """钩子：到站并进入定点保持后调用，返回并入本站记录的字段（实验记录用，默认无）."""
        return {}

    def run_slot_plan(self, plan, dry_run_gap=None):
        """按离线规划逐站执行，返回 (是否全部完成, 每站记录列表)."""
        log = self.get_logger()
        if plan.get('status') != 'success' or not plan.get('stations'):
            log.error(f'规划结果不可执行: {plan.get("status")}')
            return False, []
        r = plan['robot']
        self.distance_from_marker = r['standoff']
        p0, t, _ = plan_wall_axes(plan)
        u_marker = float((np.array([self.marker_position['x'], self.marker_position['y']])
                          - p0) @ t)
        if 'marker_u' in plan and abs(plan['marker_u'] - u_marker) > 1e-3:
            log.warn(f"规划时的标记位置 u={plan['marker_u']:.3f} 与执行器不一致 ({u_marker:.3f})")
        stations = plan['stations']
        log.info(f'离线规划：{len(plan["slots"])} 条槽 {plan["length"]:.2f} m，'
                 f'{len(stations)} 个停靠站（{plan["partition"]} / {plan["order"]}），'
                 f'预测 {plan.get("prediction", {}).get("t_total")} s；定位标记 u = {u_marker:.3f} m')

        # 1. Nav2 导航到标记正前方；2. 视觉伺服对位（确定墙面坐标原点）
        if not self.navigate_to_pose(*plan_wall_pose(plan, u_marker, self.nav_standoff)):
            log.error('任务失败：导航到定位标记失败')
            return False, []
        self.sleep_sim(2.0)
        if not self.precise_positioning_to_marker():
            log.error('任务失败：视觉对位失败')
            return False, []
        pose = self.get_robot_position()
        ref = {'p': np.array([pose['x'], pose['y']]), 'yaw': pose['yaw']}
        limits = self.arm_controller.cartesian_limits

        # 3. 逐站：到站 → 定点保持 → 测残差、补偿 → 切割
        rows, u_prev = [], u_marker
        for st in stations:
            du = st['u_c'] - u_marker
            log.info(f'\n===== 停靠站 S{st["index"]}: u = {st["u_c"]:.3f} m（距标记 '
                     f'{du * 100:+.1f} cm），{len(st["points"])} 个点 =====')
            t0 = self.sim_now()
            nav_leg = abs(st['u_c'] - u_prev) > self.max_strafe
            if nav_leg:
                # 远站：机械臂缩回，退到离墙 nav_standoff 用 Nav2 过去，再横移 / 接近到站
                self.move_arm_for_approach(joint1=0.0)
                if not self.navigate_to_pose(
                        *plan_wall_pose(plan, st['u_c'], self.nav_standoff)):
                    log.error(f'S{st["index"]}: Nav2 导航失败，停止')
                    break
            ok_move = self.strafe_to(du, ref)
            self.base_hold.enabled = self.hold_base
            self.base_hold.hold()
            t_move = self.sim_now() - t0
            self.cur_station = st['index']
            extra = self.on_station_arrived(st) or {}
            pts, is_cut = station_arm_points(st, plan, dry_run_gap)
            e_l, e_n, psi = self.measure_residual(du, ref)
            clamped = 0
            if self.compensate:
                pts, clamped = compensate_points(pts, is_cut, e_l, e_n, psi, r['tool_offset'],
                                                 limits['x'], limits['y'])
            log.info(f'到位 {"✓" if ok_move else "✗（未收敛，按残差补偿后继续）"}，'
                     f'用时 {t_move:.1f} s；残余误差（传感器）: 横向 {e_l * 1000:+.1f} mm, '
                     f'离墙 {e_n * 1000:+.1f} mm, 朝向 {math.degrees(psi):+.2f}°'
                     + ('，已补偿' if self.compensate else '')
                     + (f'；{clamped} 个点被关节限位钳制' if clamped else ''))
            t1 = self.sim_now()
            ok_cut = self.run_points(pts, is_cut, st['motion']['duration'])
            drift = self.base_hold.release()
            self.cur_station = -1
            t_end = self.sim_now()
            row = dict(station=st['index'], u_c=st['u_c'], points=len(st['points']),
                       move_dist_m=abs(st['u_c'] - u_prev), nav_leg=int(nav_leg),
                       station_s=t_end - t0, move_s=t_move, cut_s=t_end - t1,
                       sensed_lateral_mm=e_l * 1000, sensed_standoff_mm=e_n * 1000,
                       sensed_yaw_deg=math.degrees(psi), clamped=clamped,
                       cut_cm=st['cut_length'] * 100, moved=int(ok_move), cut_ok=int(ok_cut))
            if drift is not None:
                row.update(hold_drift_mm=drift[0] * 1000, hold_drift_deg=drift[1])
            row.update(extra)
            rows.append(row)
            u_prev = st['u_c']
            if not ok_cut:
                log.error(f'S{st["index"]} 切割失败，停止')
                break
        self.arm_controller.move_to_joints([0.0, 0.0, self.approach_joint3, 0.0], duration=2.0)
        ok = len(rows) == len(stations) and all(r_['cut_ok'] for r_ in rows)
        log.info(f'{"✅" if ok else "❌"} 多站开槽完成 {len(rows)}/{len(stations)} 站，'
                 f'各站耗时之和 {sum(r_["station_s"] for r_ in rows):.1f} s')
        return ok, rows


def run_plan_main(plan, flags, args=None):
    """离线开槽规划的执行入口（main 识别到规划文件时调用）."""
    rclpy.init(args=args)
    task = NavigationCuttingTask()
    task.compensate = 'no-compensation' not in flags
    if 'max-strafe' in flags:
        task.max_strafe = float(flags['max-strafe'])
    gap = flags.get('dry-run-gap')
    t_wall = time.time()
    ok, rows = task.run_slot_plan(plan, None if gap is None else float(gap))
    for r in rows:
        task.get_logger().info(
            f'  S{r["station"]:>2}: {r["station_s"]:6.1f} s  残差 横向 '
            f'{r["sensed_lateral_mm"]:+5.1f} mm 朝向 {r["sensed_yaw_deg"]:+5.2f}°  '
            f'切割 {"✓" if r["cut_ok"] else "✗"}')
    task.get_logger().info(f'墙钟耗时 {time.time() - t_wall:.0f} s')
    task.trajectory_visualizer.destroy_node()
    task.arm_controller.destroy_node()
    task.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


def main(args=None):
    # 本程序的 --key[=value] 选项（--ros-args 之后的参数留给 rclpy）
    argv = rclpy.utilities.remove_ros_args(sys.argv)
    flags = dict((a[2:].partition('=')[0], a[2:].partition('=')[2] or True)
                 for a in argv[1:] if a.startswith('--'))
    pos = [a for a in argv if not a.startswith('--')]
    if len(pos) < 2:
        print(__doc__)
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

    config_file = pos[1]

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

    # 离线开槽规划文件（plan_wall_slots.py 输出）→ 多站执行
    if is_slot_plan(config):
        run_plan_main(config, flags, args)
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

    # 设置距离与容差参数
    if enable_precise:
        task_controller.distance_from_marker = distance_from_marker
        if 'position_tolerance' in precise_config:
            task_controller.position_tolerance = float(precise_config['position_tolerance'])
        if 'angle_tolerance_deg' in precise_config:
            task_controller.angle_tolerance = math.radians(
                float(precise_config['angle_tolerance_deg']))
        task_controller.positioning_mode = precise_config.get('mode', 'visual')
        if 'observe_joint1' in precise_config:
            task_controller.observe_joint1 = float(precise_config['observe_joint1'])
    task_controller.hold_base = bool(config.get('base_hold', True))
    gap = config.get('motion', {}).get('dry_run_gap')
    task_controller.dry_run_gap = None if gap is None else float(gap)

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
    task_controller.arm_controller.destroy_node()
    task_controller.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
