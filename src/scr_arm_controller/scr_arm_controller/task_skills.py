"""
墙面切割任务的技能库：精对位、墙面坐标横移、残差测量、定点保持下的机械臂点执行.

本模块只提供可复用的“技能”，不包含任务流程。任务流程由行为树编排（scr_task 包，
BehaviorTree.CPP），各技能经 skill_server.py 以 ROS 2 action 的形式提供：

  FineAlign      精对位（AprilTag + 激光墙面 → EKF → 饱和控制律 → 收敛判据），确定墙面原点
  MoveToStation  墙面坐标闭环横移 / 接近（沿墙位置由里程计外推，离墙与朝向由激光更新）
  CutStation     测到站残差 → 补偿 → 执行机械臂点（整站一条连续轨迹，或逐点）
  HoldBase       定点保持
  ScanMarker / RetractArm

精对位模式（positioning_mode）：
  'visual'     AprilTag + 激光墙面 → EKF（里程计预测）→ 平方根整形饱和控制律 → 收敛判据（默认）
  'visual_v1'  第一版：色块 + 激光 → 指数平滑 → 比例 + 最小速度 + 死区（对比基线）
  'odom'       里程计 + 已知标记坐标（对比基线）

技能既可在独立脚本中顺序调用（alignment_benchmark、teach_alignment_offset：节点由调用方
自行 spin），也可由多线程执行器驱动（skill_server：executor_driven = True，等待时不再
spin 本节点）。
"""

import math
import os
import time
from dataclasses import replace

import rclpy
from geometry_msgs.msg import Twist, WrenchStamped
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:            # 纯算法测试环境
    get_package_share_directory = None


from scr_arm_controller import cutting_force
from scr_arm_controller.arm_trajectory import ARM_MOTION_PARAMS, limits_from_params
from scr_arm_controller.base_hold import BaseHold
from scr_arm_controller.cartesian_cutting import CartesianCuttingController
from scr_arm_controller.cut_schedule import CutSchedule, split_plunges, travel_ratio
from scr_arm_controller.docking_control import ConvergenceMonitor, WallAlignController
from scr_arm_controller.slot_execution import cut_phases
from scr_arm_controller.tag_map import TagMap, resolve_path
from scr_arm_controller.trajectory_visualizer import TrajectoryVisualizer
from scr_arm_controller.visual_servo import pbvs_command
from scr_arm_controller.visual_servo_ros import WallMarkerServo
from scr_arm_controller.wall_estimator_ros import WallStateEstimator

# 精对位参数（ROS 参数 alignment.*，可用 --params-file 覆盖；默认值见 config/fine_alignment.yaml）
ALIGNMENT_PARAMS = {
    'detector': 'apriltag',            # 'apriltag' | 'blob'
    'tag_family': 'tag36h11',
    'tag_id': 0,
    'tag_size': 0.10,                  # 黑框边长（米）
    # 零位标定偏置：在标定位姿（真值已知）处“传感器指示值 − 真值”，测量模型内扣除
    'offset_normal': 0.0,              # 米，离墙距离
    'offset_lateral': 0.0,             # 米，沿墙（面向墙面向右为正）
    'offset_yaw_deg': 0.0,             # 度，逆时针为正
    'tolerance_normal': 0.002,         # 收敛容差（米）
    'tolerance_lateral': 0.002,
    'tolerance_yaw_deg': 0.3,
    'settle_cycles': 10,               # 连续满足判据的周期数
    'control_rate': 20.0,              # Hz
    'k_lin': 2.0,                      # 1/s，线性区增益
    'k_ang': 2.0,
    'decel_lin': 0.15,                 # m/s²，制动曲线减速度
    'decel_ang': 0.5,                  # rad/s²
    'max_lin': 0.15,                   # m/s
    'max_ang': 0.3,                    # rad/s
    'timeout': 50.0,                   # 秒（仿真时间）
    'lost_timeout': 10.0,              # 标记丢失超过该时间判定失败（秒）
    'sigma_hold': 0.01,                # σ_u 超过该值（米）时停车等待标记
    'fold_residual': True,             # 多站：以估计的标记位置（而非停车位姿）为墙面坐标原点
    # 多站：站间横移与到站残差测量时，视野内的定位标记继续参与沿墙位置更新
    'strafe_tag_updates': True,
    # 横移前机械臂先回观测位姿（滑台 observe_joint1、伸缩轴缩回），使墙上的标记处在画面内、
    # 不被机械臂遮挡；只在 strafe_tag_updates 时生效。切割位姿下滑台较低，标记落在画面上沿外
    'observe_between_stations': True,
    # 标记地图（tag_map.py 的 YAML；相对路径按本包 config 目录解析）。空串 = 只有 tag_id
    # 一个标记，站间沿墙位置只能由里程计外推
    'tag_map': 'tag_map.yaml',
    # 横移比例修正 s 作为 EKF 的第四个状态在线估计（真实横移 = (1 + s)·里程计横移）
    'estimate_scale': True,
    'scale_init': 0.0,                 # s 的先验（可填离线标定值）
    'scale_sigma': 0.05,               # s 的先验标准差
}


class CutStateReporter:
    """
    切割期间按轨迹时间表发布切削状态（50 Hz，由定点保持节点的里程计节拍驱动）.

    话题 cutting_force/engagement（std_msgs/Float64，切深，米）：切割力插件据此施加切削力；
    话题 cutting_force/expected_wrench（geometry_msgs/WrenchStamped，base_link 系）：按名义
    切割力模型预计的底盘力旋量，定点保持可订阅它作前馈（hold.feedforward_topic）。
    没有切割力插件时这两个话题没有订阅者，不影响执行。
    """

    LEAD = 0.02     # 预计力旋量取一个控制周期之后的值：前馈经定点保持下发要晚一拍

    def __init__(self, hold):
        node = hold.node
        self._depth_pub = node.create_publisher(Float64, 'cutting_force/engagement', 10)
        self._wrench_pub = node.create_publisher(
            WrenchStamped, 'cutting_force/expected_wrench', 10)
        self._schedule = None
        self._t_start = 0.0
        hold.tick_callbacks.append(self._tick)

    def start(self, schedule, t_start):
        self._t_start = t_start
        self._schedule = schedule

    def stop(self):
        if self._schedule is not None:
            self._schedule = None
            self._publish(0.0, cutting_force.Wrench(), None)

    def _tick(self, stamp):
        sch = self._schedule
        if sch is None:
            return
        t = stamp - self._t_start
        if t > sch.duration:
            self.stop()
            return
        self._publish(sch.state(t).depth, sch.wrench(t + self.LEAD), stamp)

    def _publish(self, depth, wrench, stamp):
        self._depth_pub.publish(Float64(data=float(depth)))
        msg = WrenchStamped()
        if stamp is not None:
            ns = int(round(stamp * 1e9))
            msg.header.stamp.sec, msg.header.stamp.nanosec = ns // 1000000000, ns % 1000000000
        msg.header.frame_id = 'base_link'
        msg.wrench.force.x, msg.wrench.force.y, msg.wrench.torque.z = wrench.planar
        self._wrench_pub.publish(msg)


class CuttingSkills(Node):
    """墙面切割技能库（一个节点 + 若干辅助节点：机械臂客户端、定点保持、可视化）."""

    # Gazebo 中机器人实体名（见 scr_description/launch/gazebo.launch.py）
    GAZEBO_ENTITY = 'scr_robot_gazebo'

    def __init__(self, node_name='navigation_cutting_task'):
        # 使用仿真时钟：控制周期按仿真时间计，与电脑快慢（实时率）无关
        super().__init__(
            node_name,
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )

        # 速度控制发布器
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 里程计订阅
        # 局部闭环（精对位预测、横移外推、定点保持）用的里程计话题。融合定位（EKF）时
        # 用轮式里程计 /wheel/odom：robot_localization 输出的 /odom 在 mm/s 级的慢速运动下
        # 明显偏小（定点保持会被它带着漂移），而轮式里程计在短时内与真值一致
        self.declare_parameter('odom_topic', '/odom')
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )
        self.current_odom = None

        # Gazebo模型状态订阅
        self.model_states_sub = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.model_states_callback, 10
        )
        self.robot_pose = None

        # 机械臂控制器
        self.arm_controller = CartesianCuttingController()
        # 机械臂客户端节点只在等待动作时被 spin，自己的时钟停在上次 spin 时；计时取本节点的时钟
        self.arm_controller.time_source = self

        # 轨迹可视化器
        self.trajectory_visualizer = TrajectoryVisualizer()

        # 黄色方块位置（从world文件中获取，或通过扫描获得）
        self.marker_position = {'x': 8.42, 'y': -3.9245, 'z': 0.6}  # 贴在南墙内表面 y=-3.925
        self.marker_detected_position = None  # 扫描检测到的位置

        # 精确定位参数（可由任务配置 precise_positioning 段覆盖）
        self.distance_from_marker = 0.6
        self.position_tolerance = 0.01             # 1 cm
        self.angle_tolerance = math.radians(1.0)   # 1°
        # 对位模式：'visual' = AprilTag + 激光 EKF；'visual_v1' = 第一版色块 PBVS；'odom' = 里程计 + 已知标记坐标
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
        self.base_hold = BaseHold(entity=self.GAZEBO_ENTITY, odom_topic=self.odom_topic)
        self.cut_reporter = CutStateReporter(self.base_hold)

        # 多站开槽规划执行（run_slot_plan）
        self.max_strafe = 1.5          # 站间横移超过该距离（米）时先退到离墙 nav_standoff 用 Nav2 过去
        self.nav_standoff = 1.2        # Nav2 目标离墙距离（米）：远离墙面，留出代价地图膨胀余量
        self.strafe_tol = (0.003, 0.005, math.radians(1.0))   # 到站容差：横向 / 离墙 / 朝向
        self.compensate = True         # 到站后按残余误差修正机械臂点
        self.cur_station = -1          # 当前停靠站序号（-1 = 不在站上）；供实验记录读取
        self.phase = 'idle'            # idle / move / cut：相邻两点都在切深时为 cut

        # 精对位：传感器前端（激光拟合墙面 + 相机检测定位标记）与墙面相对状态估计
        for k, v in ALIGNMENT_PARAMS.items():
            self.declare_parameter('alignment.' + k, v)
        self.align_cfg = {k: self.get_parameter('alignment.' + k).value
                          for k in ALIGNMENT_PARAMS}
        for k, v in ARM_MOTION_PARAMS.items():
            self.declare_parameter('arm_motion.' + k, v)
        self.last_trajectory = None     # 最近一次连续轨迹（arm_trajectory.ArmTrajectory）
        a = self.align_cfg
        self.tag_map = self.load_tag_map()
        self.servo = WallMarkerServo(self, detector=a['detector'], tag_family=self.tag_map.family,
                                     tag_id=self.tag_map.reference_id, tag_size=self.tag_map.size)
        self.servo.tag_ids = self.tag_map.ids
        self.estimator = WallStateEstimator(
            self, self.servo, odom_topic=self.odom_topic, tag_map=self.tag_map,
            estimate_scale=bool(a['estimate_scale']), scale_init=float(a['scale_init']),
            scale_sigma=float(a['scale_sigma']))
        self.apply_alignment_config()
        self.align_trace = []           # 最近一次对位的逐周期记录（实验用）
        self.fold_alignment_residual = bool(a['fold_residual'])
        self.hold_after_align = True    # 精对位 / 横移收敛后立即进入定点保持
        self.align_gt = None            # 宣布到位时刻的真值位姿（评估用）

        # 执行方式：False = 调用方顺序调用、等待时 spin 本节点；True = 节点由多线程执行器
        # 驱动（skill_server），等待时只按仿真时钟轮询
        self.executor_driven = False
        # 中止请求（action 被取消时由 skill_server 置位），各闭环每周期检查
        self.abort_requested = lambda: False
        # 阶段变化回调 (station, phase)：skill_server 据此发布 TaskStatus
        self.on_phase = None
        self.last_align_errors = (math.nan, math.nan, math.nan)
        self.servo_feedback = None      # 墙面闭环每周期的估计回调（action 反馈）

        self.get_logger().info('切割任务技能库已初始化')

    def load_tag_map(self):
        """按 alignment.tag_map 加载标记地图；空串时只有参考标记 alignment.tag_id."""
        a, log = self.align_cfg, self.get_logger()
        config_dir = None
        if get_package_share_directory is not None:
            try:
                config_dir = os.path.join(get_package_share_directory('scr_arm_controller'),
                                          'config')
            except Exception:  # noqa: BLE001
                config_dir = None
        path = resolve_path(str(a['tag_map']), config_dir)
        if path is None:
            m = TagMap.single(int(a['tag_id']), a['tag_family'], float(a['tag_size']))
            log.info(f'标记地图：只有参考标记 {m.reference_id}（站间沿墙位置由里程计外推）')
            return m
        m = TagMap.load(path)
        if m.reference_id != int(a['tag_id']):
            log.warn(f'标记地图的参考标记 {m.reference_id} 与 alignment.tag_id = {a["tag_id"]} '
                     f'不同，以地图为准')
        log.info(f'标记地图 {path}：{len(m)} 个标记，最大间距 {m.spacing():.2f} m，'
                 f'位置误差 σ = {m.sigma(m.ids[-1]) * 1000:.1f} mm')
        return m

    def apply_alignment_config(self, **overrides):
        """应用（并可覆盖）精对位参数：检测器、零位标定偏置、容差."""
        self.align_cfg.update(overrides)
        a = self.align_cfg
        self.servo.detector = a['detector']
        self.servo.detectors = {a['detector']}
        self.estimator.set_offset(float(a['offset_normal']), float(a['offset_lateral']),
                                  math.radians(float(a['offset_yaw_deg'])))

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

    def set_phase(self, phase, station=None, stamp=None, scheduled=False):
        """
        更新当前阶段（实验记录用），并通知 skill_server 发布状态.

        stamp：阶段生效的仿真时刻（秒），不给时为当前时刻。scheduled：按时间表提前发布
        （连续轨迹在开始时就给出之后各段的阶段，stamp 在未来），状态的 detail 标为
        'scheduled'，接收方按 stamp 而不是到达时刻使用它。
        """
        self.phase = phase
        if station is not None:
            self.cur_station = station
        if self.on_phase is not None:
            self.on_phase(self.cur_station, phase, detail='scheduled' if scheduled else '',
                          stamp=stamp)

    def sleep_sim(self, duration):
        """按仿真时钟等待 duration 秒（顺序调用时期间持续处理本节点回调）."""
        start = self.get_clock().now()
        target = Duration(seconds=duration)
        while rclpy.ok() and (self.get_clock().now() - start) < target:
            if self.executor_driven:
                time.sleep(0.002)
            else:
                rclpy.spin_once(self, timeout_sec=0.005)

    def quaternion_to_yaw(self, q):
        """四元数转yaw角"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

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
            # 控制只用传感器测量；上面的目标位姿仅用于真值评估
            return self.precise_positioning_fused(target_x, target_y, target_yaw)
        if self.positioning_mode == 'visual_v1':
            return self.precise_positioning_visual(target_x, target_y, target_yaw)

        self.get_logger().info('对位模式: odom（里程计 + 已知方块坐标，基线）')
        self.move_arm_for_approach(joint1=0.0)
        # 等待获取机器人位置
        self.get_logger().info('等待获取机器人位置...')
        for _ in range(50):
            if self.get_robot_position() is not None:
                break
            self.sleep_sim(0.1)

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
            if self.abort_requested():
                break
            current_pos = self.get_robot_position()
            if current_pos is None:
                self.sleep_sim(0.1)
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
                    self.get_logger().info('精确定位完成！')
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

        # 超过最大迭代次数（或被中止）：定位未收敛，按失败处理
        self.get_logger().error('定位未收敛（达到最大迭代次数或被中止）')
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
                    f'防碰撞守护：刀具前端距墙 {clearance*1000:.1f} mm，已阻止向墙运动')
            self._guard_active = allowed == 0.0
        else:
            self._guard_active = False
        return cmd

    def move_arm_for_approach(self, joint1=0.0, duration=2.0):
        """接近墙面前把机械臂移到安全位姿：滑台高度 joint1，伸缩轴完全缩回."""
        self.get_logger().info(
            f'机械臂移至接近位姿 joint1={joint1:.2f} m, joint3={self.approach_joint3:+.3f} m（缩回）')
        self.arm_controller.move_to_joints([joint1, 0.0, self.approach_joint3, 0.0],
                                           duration=duration)
        self.sleep_sim(0.5)

    def observe_before_strafe(self):
        """站间横移使用标记更新时，先把机械臂移到观测位姿（见 observe_between_stations）."""
        a = self.align_cfg
        if self.estimator.use_tag and bool(a['observe_between_stations']):
            self.move_arm_for_approach(joint1=self.observe_joint1, duration=1.5)

    def make_align_controller(self, dt, max_lin=None):
        a = self.align_cfg
        return WallAlignController(
            k_lin=a['k_lin'], k_ang=a['k_ang'], a_lin=a['decel_lin'], a_ang=a['decel_ang'],
            v_max=a['max_lin'] if max_lin is None else max_lin, w_max=a['max_ang'], dt=dt)

    def run_wall_servo(self, target, monitor, max_lin=None, timeout=None, log_every=40,
                       require_tag=True, ready_cb=None):
        """
        墙面相对状态闭环：估计 → 控制律 → 防碰撞守护 → 收敛判据，直到收敛 / 超时 / 丢失.

        target = (d*, u*, ψ*)。require_tag：开始运动前必须先由标记确定沿墙位置
        （精对位）；False 时沿墙位置来自之前的估计（多站执行的站间移动）。
        ready_cb(X, σ, info)：每周期的估计回调（action 反馈用）。
        返回 (是否收敛, 最后一次估计)。
        """
        a = self.align_cfg
        period = 1.0 / float(a['control_rate'])
        timeout = float(a['timeout']) if timeout is None else timeout
        est, log = self.estimator, self.get_logger()
        ready_cb = ready_cb or self.servo_feedback
        self.base_hold.release()         # 本闭环自己发速度指令，先解除可能仍在运行的定点保持
        self.align_gt = None
        ctrl = self.make_align_controller(period, max_lin)
        ctrl.reset()
        monitor.reset()
        t0 = self.get_clock().now().nanoseconds * 1e-9
        X, it, holding, t_prev = None, 0, False, None
        while rclpy.ok() and not self.abort_requested():
            now = self.get_clock().now().nanoseconds * 1e-9
            # 加速度限幅按实际周期计（仿真加速时 Python 计算耗时按仿真时间放大，周期会变长）
            if t_prev is not None:
                ctrl.dt = min(max(now - t_prev, 0.5 * period), 4 * period)
            t_prev = now
            if now - t0 > timeout:
                log.error(f'对位超时（{timeout:.0f} s）')
                break
            X, info = est.step()
            if X is not None and ready_cb is not None:
                ready_cb(X, est.ekf.sigma(), info)
            cmd = Twist()
            sig = est.ekf.sigma()
            ready = X is not None and (est.u_known or not require_tag)
            last_tag = est.last_tag_time if est.last_tag_time is not None else t0
            if require_tag and now - last_tag > float(a['lost_timeout']):
                log.error(f'定位标记丢失超过 {a["lost_timeout"]:.0f} s：{info.get("tag")}')
                break
            if require_tag and ready and sig[1] > float(a['sigma_hold']):
                ready = False
                if not holding:
                    log.warn(f'沿墙位置不确定度 σ_u = {sig[1] * 1000:.1f} mm，停车等待定位标记')
            holding = not ready
            if ready:
                errs = ctrl.errors(X, target)
                if monitor.update(errs, est.velocity(), sig):
                    # 到位后立即转入定点保持：仿真中轮子摩擦为 0，零速度指令下底盘仍会缓慢
                    # 滑移（实测约 0.6 mm/s、0.08°/s），真机上由电机抱死 / 定点保持承担
                    self.align_gt = self.get_ground_truth_position()
                    self.last_align_errors = tuple(float(e) for e in errs)
                    if self.hold_after_align:
                        self.base_hold.enabled = True
                        self.base_hold.hold()
                    else:
                        self.stop_robot()
                    log.info(f'墙面闭环收敛：误差 法向 {errs[0] * 1000:+.2f} mm、沿墙 '
                             f'{errs[1] * 1000:+.2f} mm、朝向 {math.degrees(errs[2]):+.3f}°，'
                             f'σ = ({sig[0] * 1000:.2f} mm, {sig[1] * 1000:.2f} mm, '
                             f'{math.degrees(sig[2]):.3f}°)，用时 {now - t0:.1f} s')
                    return True, X
                cmd.linear.x, cmd.linear.y, cmd.angular.z = ctrl.command(X, target)
                cmd = self.apply_wall_guard(cmd)
                ctrl.prev[:] = (cmd.linear.x, cmd.linear.y, cmd.angular.z)
            else:
                ctrl.reset()
                monitor.reset()
            self.cmd_vel_pub.publish(cmd)
            self.align_trace.append((
                now - t0, *(X if X is not None else (math.nan,) * 3), *sig,
                cmd.linear.x, cmd.linear.y, cmd.angular.z, info.get('tag') == 'ok',
                info.get('wall') == 'ok'))
            if it % log_every == 0 and X is not None:
                log.info(f'[{now - t0:5.1f}s] d={X[0]:.4f} m u={X[1] * 1000:+7.1f} mm '
                         f'ψ={math.degrees(X[2]):+6.2f}° σ_u={sig[1] * 1000:.2f} mm '
                         f'标记: {info.get("tag")}  墙面: {info.get("wall")}')
            it += 1
            self.sleep_sim(period)
        self.stop_robot()
        return False, X

    def precise_positioning_fused(self, target_x, target_y, target_yaw):
        """
        精对位：AprilTag + 激光墙面 → EKF → 平方根整形饱和控制律 → 收敛判据.

        不使用里程计的绝对位姿与已知标记坐标：里程计只提供短时增量（预测与延迟补偿）。
        """
        a = self.align_cfg
        D = self.distance_from_marker
        self.get_logger().info(
            f'对位模式: visual（{a["detector"]} + 激光墙面 → EKF），目标离墙 {D:.3f} m，'
            f'容差 法向 {a["tolerance_normal"] * 1000:.1f} mm / 沿墙 '
            f'{a["tolerance_lateral"] * 1000:.1f} mm / 朝向 {a["tolerance_yaw_deg"]:.2f}°，'
            f'零位偏置 ({a["offset_normal"] * 1000:+.2f} mm, {a["offset_lateral"] * 1000:+.2f} mm,'
            f' {a["offset_yaw_deg"]:+.3f}°)')
        self.move_arm_for_approach(joint1=self.observe_joint1)
        self.servo.detector = a['detector']
        self.servo.detectors = {a['detector']}
        self.estimator.reset()
        self.estimator.use_tag = True
        self.align_trace = []
        monitor = ConvergenceMonitor(
            tol_d=a['tolerance_normal'], tol_u=a['tolerance_lateral'],
            tol_psi=math.radians(a['tolerance_yaw_deg']), n_required=int(a['settle_cycles']))
        ok, X = self.run_wall_servo((D, 0.0, 0.0), monitor)
        st = self.estimator.ekf.stats
        self.get_logger().info(f'测量更新：墙面 接受 {st["wall"][0]} / 拒收 {st["wall"][1]}，'
                               f'标记 接受 {st["tag"][0]} / 拒收 {st["tag"][1]}')
        self.report_ground_truth_error(target_x, target_y, target_yaw)
        return ok

    def precise_positioning_visual(self, target_x, target_y, target_yaw):
        """第一版：色块 + 激光 PBVS（指数平滑，比例 + 最小速度 + 死区），保留作对比基线."""
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
        self.servo.detectors = {'blob'}
        self.servo.reset_filter()
        self.align_gt = None

        converged, lost, required = 0, 0, 5
        m = None
        for it in range(max_iterations):
            if self.abort_requested():
                break
            m, why = self.servo.measure(D)
            if m is None:
                lost += 1
                self.cmd_vel_pub.publish(Twist())          # 测量丢失时原地停住
                if lost == 1 or lost % 10 == 0:
                    self.get_logger().warn(f'测量失败（{lost}）: {why}')
                if lost >= lost_limit:
                    self.get_logger().error('视觉测量持续丢失，对位失败')
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
                    self.align_gt = self.get_ground_truth_position()   # 评估用
                    self.stop_robot()
                    self.get_logger().info('视觉伺服对位完成！')
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

        self.get_logger().error(f'达到最大迭代次数({max_iterations})，视觉伺服未收敛')
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
            f'感知误差: 方块估计 base_link=({est[0]:.4f}, {est[1]:.4f}, {est[2]:.4f}) '
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
            f'真值误差: 位置={pos_err*1000:.1f}mm (dx={dx*1000:.1f}, dy={dy*1000:.1f}), '
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

        self.servo.detectors.add('blob')     # 扫描沿用色块射线与墙面求交（第一版测量）

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
            self.get_logger().warn('扫描完成，未检测到黄色方块')
        else:
            self.get_logger().info(
                f'检测到黄色方块！位置: X={marker_pos[0]:.4f}, Y={marker_pos[1]:.4f}, '
                f'Z={marker_pos[2]:.4f}（{samples} 帧平均）')
            self.marker_detected_position = dict(zip('xyz', marker_pos))
            self.report_perception_error({'marker_base': marker_pos})

        # 回到接近位姿（伸缩轴缩回，保持离墙余量）
        self.arm_controller.move_to_joints([0.0, 0.0, self.approach_joint3, 0.0], duration=2.0)
        return marker_pos

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
                f'点{i+1}: X={x:.4f}, Y={y:.4f}, Z={z:.4f}, R={math.degrees(rotation):.1f}°'
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
            if self.abort_requested():
                return False
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

    def strafe_to(self, du_target, max_lin=0.1, timeout=60.0, ready_cb=None):
        """
        横移到沿墙 u = du_target（相对定位标记，面向墙面向右为正，米）处，离墙 distance_from_marker.

        与精对位共用墙面相对状态估计（EKF）与控制律：沿墙位置由里程计增量外推（估计横移
        比例修正时按修正后的增量），视野内有地图标记时由标记更新，离墙距离与朝向由激光更新。也用于从 Nav2 的离墙位姿
        （nav_standoff）接近墙面。剩余误差由到站后的机械臂点补偿消除。
        """
        tol_lat, tol_norm, tol_yaw = self.strafe_tol
        monitor = ConvergenceMonitor(tol_d=tol_norm, tol_u=tol_lat, tol_psi=tol_yaw,
                                     v_eps=0.005, w_eps=math.radians(0.5),
                                     sigma_max=(None, None, None), n_required=5)
        ok, _ = self.run_wall_servo((self.distance_from_marker, du_target, 0.0), monitor,
                                    max_lin=max_lin, timeout=timeout, require_tag=False,
                                    ready_cb=ready_cb)
        return ok

    def measure_residual(self, du_target, n=10):
        """
        到站后的残余误差（只用机器人自身传感器，EKF 估计）→ (e_l, e_n, psi).

        e_l：沿墙（米，向右为正）；e_n：离墙（米，>0 表示离墙偏远）；psi：朝向（rad，逆时针为正）。
        底盘处于定点保持中，继续做 n 个周期的预测 + 激光更新后读取估计。
        """
        X = None
        for _ in range(n):
            X, _ = self.estimator.step()
            self.sleep_sim(0.1)
        if X is None:
            return 0.0, 0.0, 0.0
        return (float(X[1] - du_target), float(X[0] - self.distance_from_marker), float(X[2]))

    def arm_motion_config(self):
        """
        读取 arm_motion.* 参数 → (模式, MotionLimits, 轨迹起始提前量).

        feed_basis = 'force_limit' 时进给与下刀速度取切割力模型给出的上限（主轴功率、轮地
        摩擦、定点保持可抵消的偏航力矩），feed_rate / plunge_rate 仍作为上限。
        """
        c = {k: self.get_parameter('arm_motion.' + k).value for k in ARM_MOTION_PARAMS}
        limits = limits_from_params(c)
        self.cut_config = c
        if str(c['feed_basis']) == 'force_limit':
            fl = cutting_force.feed_limit(
                self.cut_blade(c), float(c['slot_depth']),
                power_fraction=float(c['power_fraction']), mu=float(c['friction_mu']),
                friction_margin=float(c['friction_margin']),
                max_yaw_moment=self.base_hold.yaw_authority())
            limits = replace(limits, feed=min(limits.feed, fl.feed),
                             plunge=min(limits.plunge, fl.plunge))
            self.get_logger().info(
                f'进给按切割力模型限制（{c["material"]}）：进给 {limits.feed * 1e3:.2f} mm/s'
                f'（约束：{fl.binding}），下刀 {limits.plunge * 1e3:.2f} mm/s',
                throttle_duration_sec=60.0)
        elif str(c['feed_basis']) != 'fixed':
            self.get_logger().error(f'未知的 arm_motion.feed_basis: {c["feed_basis"]!r}，按 fixed 处理')
        return str(c['mode']), limits, float(c['start_lead'])

    @staticmethod
    def cut_blade(c):
        """arm_motion.material → 刀片与材料参数；未知材料时退回烧结砖."""
        name = str(c['material'])
        if name not in cutting_force.MATERIALS:
            name = 'brick'
        return cutting_force.BladeParams.for_material(name)

    def run_points(self, points, is_cut, duration):
        """
        执行一个停靠站的机械臂点，并标注阶段：相邻两点都在切深时为 'cut'，否则为 'move'.

        arm_motion.mode = 'continuous'（默认）时整站作为一条连续轨迹执行，duration 不起作用；
        'point_to_point' 时逐点执行，每点 duration 秒。
        """
        mode, limits, lead = self.arm_motion_config()
        if mode == 'continuous':
            return self.run_points_continuous(points, is_cut, limits, lead)
        if mode != 'point_to_point':
            self.get_logger().error(f'未知的 arm_motion.mode: {mode!r}')
            return False
        for (x, y, z, rot), phase in zip(points, cut_phases(is_cut)):
            if self.abort_requested():
                self.set_phase('idle')
                return False
            self.set_phase(phase)
            if not self.arm_controller.move_to_cartesian(x, y, z, rot, duration=duration):
                self.set_phase('idle')
                return False
        self.set_phase('idle')
        return True

    def run_points_continuous(self, points, is_cut, limits, start_lead):
        """
        整站机械臂点作为一条连续轨迹执行.

        阶段标注与逐点执行相同（切割段为 'cut'，其余为 'move'），但各段的起止时刻在轨迹
        开始时就已确定，所以一次性按时间表发布，每条状态的时间戳是该阶段的生效时刻。
        """
        end = [None]

        def publish_schedule(t_start, traj):
            prev = None
            for seg in traj.segments:
                phase = 'cut' if seg.kind == 'cut' else 'move'
                if seg.t1 > seg.t0 and phase != prev:
                    self.set_phase(phase, stamp=t_start + seg.t0, scheduled=True)
                    prev = phase
            end[0] = t_start + traj.duration

        c = self.cut_config
        if bool(c['split_plunge']):
            points, is_cut = split_plunges(points, is_cut, float(c['slot_depth']))
        if bool(c['scale_dry_plunge']):
            # 空切时下刀行程短于槽深：下刀速度按行程比例折算，使下刀时间与切深的变化率
            # 和真实下刀一致
            limits = replace(limits, plunge=limits.plunge * travel_ratio(
                points, is_cut, float(c['slot_depth'])))

        def on_start(t_start, traj):
            publish_schedule(t_start, traj)
            self.cut_reporter.start(
                CutSchedule(traj, self.cut_blade(c), float(c['slot_depth']),
                            self.distance_from_marker), t_start)

        self.set_phase('move')
        ok, traj = self.arm_controller.follow_points(
            points, is_cut, limits, start_lead=start_lead, on_start=on_start,
            should_abort=self.abort_requested)
        self.cut_reporter.stop()
        self.last_trajectory = traj
        # 中止时立即回到 idle（时间戳为当前时刻，记录器据此丢弃尚未生效的时间表）
        self.set_phase('idle', stamp=end[0] if ok else None)
        return ok

    def set_wall_origin(self):
        """
        精对位结束后确定墙面坐标原点，返回停车位姿的沿墙残差（米）.

        EKF 的沿墙估计 u 相对定位标记；fold_alignment_residual 时保留停车残差（u ≠ 0），
        后续各站以“标记位置”而不是“停车位姿”为原点。第一版语义（以停车位姿为原点）：
        fold_alignment_residual = False 或非 visual 模式。alignment.strafe_tag_updates 为 true
        （默认）时，之后视野内的地图标记继续参与更新；false 时站间沿墙位置只由里程计外推。
        """
        log, est = self.get_logger(), self.estimator
        if not est.ekf.initialized:
            est.reset()
            for _ in range(20):
                if est.step()[0] is not None:
                    break
                self.sleep_sim(0.1)
        u_res = float(est.ekf.x[1]) if est.ekf.initialized else 0.0
        est.reset_tag_log()
        if self.positioning_mode != 'visual' or not self.fold_alignment_residual:
            est.rebase_u(0.0)
            log.info(f'墙面坐标原点 = 停车位姿（精对位残差 {u_res * 1000:+.2f} mm 未计入）')
        else:
            log.info(f'墙面坐标原点 = 定位标记（停车位姿沿墙残差 {u_res * 1000:+.2f} mm 已计入）')
        est.use_tag = bool(self.align_cfg['strafe_tag_updates'])
        return u_res
