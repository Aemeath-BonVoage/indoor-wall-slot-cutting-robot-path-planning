"""
底盘定点保持 + 漂移测量.

机械臂运动的反作用力会推动底盘：仿真中轮子摩擦为 0、底盘由 planar_move 驱动，
实测机械臂动作时一站内可漂移约 10 cm。切割 / 扫描期间需要把底盘闭环保持在到位位姿。

机械臂控制器在执行动作时会阻塞主节点的回调，因此本模块使用独立节点和独立线程运行：
- 每收到一帧里程计（50 Hz）做一次 P 控制，把底盘保持在 hold() 时的位姿（局部闭环，用
  轮式里程计：融合后的 /odom 在 mm/s 级的慢速运动下明显偏小，见
  docs/experiments/system_modernization）。控制由里程计消息驱动而不是定时器：节点用仿真
  时钟，Gazebo 以 10 Hz 发布 /clock，按仿真时间计时的 50 Hz 定时器实际每 0.1 s 连续触发
  两次，控制周期既不均匀也不是 20 ms（见 docs/experiments/cutting_force 的 Gazebo 验证）；
- 同时订阅 Gazebo 真值（/gazebo/model_states），记录保持期间底盘偏离起点的最大距离与角度。

抗扰保持（可选）：节点参数 hold.mode 选择控制律，默认 'p' 即上述比例控制，行为不变；
'pi' / 'pid' / 'dob' / 'lqg' 使用 disturbance_observer.PlanarHold（切割力作用下比例保持
有稳态偏差，见 docs/experiments/cutting_force）。其余参数：hold.max_lin、hold.max_ang
（指令限幅）、hold.observer_bandwidth、hold.delay_steps（测量到指令生效的整拍延迟）、
hold.velocity_loop_gain、hold.mass、hold.inertia，
以及前馈 hold.feedforward_topic（geometry_msgs/WrenchStamped，base_link 系下预计的外力，
空字符串 = 不用）与 hold.feedforward_gain。
"""
import math
import threading

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from scr_arm_controller.disturbance_observer import PlanarHold


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class BaseHold:
    # 增益按力控底盘（scr_planar_move，速度环带宽约 20 rad/s）整定：外环位置 P 增益取其 1/4–1/3
    def __init__(self, entity='scr_robot_gazebo', rate_hz=50.0,
                 k_lin=5.0, k_ang=6.0, max_lin=0.1, max_ang=0.5, odom_topic='/odom',
                 mode=None):
        sim_time = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        self.node = rclpy.create_node('base_hold', parameter_overrides=[sim_time])
        self.entity = entity

        def param(name, default):
            return self.node.declare_parameter(name, default).value
        mode = mode or param('hold.mode', 'p')
        max_lin = float(param('hold.max_lin', max_lin))
        max_ang = float(param('hold.max_ang', max_ang))
        self.k_lin, self.k_ang, self.max_lin, self.max_ang = k_lin, k_ang, max_lin, max_ang
        # 抗扰控制律；None = 现有的比例保持（默认）
        self._law = None
        self._ff = None                 # 预计的外力 (F_x, F_y, M_z)，base_link 系
        self._ff_gain = float(param('hold.feedforward_gain', 1.0))
        k_in = float(param('hold.velocity_loop_gain', 20.0))
        bandwidth = float(param('hold.observer_bandwidth', 40.0))
        mass, inertia = float(param('hold.mass', 30.5)), float(param('hold.inertia', 0.92))
        delay_steps = int(param('hold.delay_steps', 1))
        ff_topic = param('hold.feedforward_topic', '')
        # 速度环可抵消的偏航力矩：指令限幅对应 I_z·k_in·max_ang；按整定规则留 1.3 倍裕量
        self._yaw_authority = inertia * k_in * max_ang / 1.3
        if mode != 'p':
            self._law = PlanarHold(
                mode, dt=1.0 / rate_hz, kp=(k_lin, k_lin, k_ang),
                limits=(max_lin, max_lin, max_ang), k_in=(k_in, k_in, k_in), mass=mass,
                inertia=inertia, observer_bandwidth=bandwidth, delay_steps=delay_steps)
            self.node.get_logger().info(f'定点保持控制律：{mode}')
            if ff_topic:
                self.node.create_subscription(WrenchStamped, ff_topic, self._on_ff, 10)
        self.enabled = True             # False 时只测量漂移、不发控制（用于对比实验）
        self._odom = None
        self._odom_stamp = None         # 最新里程计的时间戳（秒，仿真时间，分辨率不受 /clock 限制）
        self._period = 1.0 / rate_hz
        self._period_warned = False
        # 每帧里程计调用一次的回调 f(时间戳)：需要 50 Hz、按仿真时间均匀执行的后台任务
        # （例如切削状态的发布）挂在这里，不用仿真时钟的定时器
        self.tick_callbacks = []
        self._gt = None
        self._target = None             # 里程计系中的保持目标 (x, y, yaw)
        self._gt_ref = None             # 真值起点
        self._max_drift = 0.0           # 米
        self._max_yaw = 0.0             # 弧度
        self._lock = threading.Lock()
        self.generation = 0             # 每次 hold() 加 1：调用方只释放自己开始的那次保持

        self.node.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self.node.create_subscription(ModelStates, '/gazebo/model_states', self._on_gt, 10)
        self._pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self.node)
        self._thread = threading.Thread(target=self._exec.spin, daemon=True)
        self._thread.start()

    # ---------- 回调 ----------
    def _on_odom(self, msg):
        p = msg.pose.pose
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._odom_stamp is not None and self._target is not None and not self._period_warned:
            dt = stamp - self._odom_stamp
            if dt > 0.0 and abs(dt - self._period) > 0.5 * self._period:
                self._period_warned = True
                self.node.get_logger().warn(
                    f'里程计周期 {dt * 1e3:.0f} ms 与定点保持的设计周期 '
                    f'{self._period * 1e3:.0f} ms 不符，控制律的增益按设计周期整定')
        self._odom_stamp = stamp
        self._odom = (p.position.x, p.position.y, _yaw(p.orientation))
        for cb in self.tick_callbacks:
            cb(stamp)
        self._step()

    def _on_gt(self, msg):
        try:
            p = msg.pose[msg.name.index(self.entity)]
        except ValueError:
            return
        self._gt = (p.position.x, p.position.y, _yaw(p.orientation))
        with self._lock:
            if self._gt_ref is not None:
                gx, gy, gyaw = self._gt_ref
                d = math.hypot(self._gt[0] - gx, self._gt[1] - gy)
                self._max_drift = max(self._max_drift, d)
                dyaw = math.atan2(math.sin(self._gt[2] - gyaw), math.cos(self._gt[2] - gyaw))
                self._max_yaw = max(self._max_yaw, abs(dyaw))

    def _on_ff(self, msg):
        w = msg.wrench
        self.set_feedforward((w.force.x, w.force.y, w.torque.z))

    def set_feedforward(self, wrench):
        """预计的外力 (F_x, F_y, M_z)（base_link 系）；None = 没有前馈."""
        self._ff = None if wrench is None else tuple(self._ff_gain * v for v in wrench)

    def _step(self):
        if self._target is None or self._odom is None or not self.enabled:
            return
        x, y, yaw = self._odom
        tx, ty, tyaw = self._target
        ex, ey = tx - x, ty - y
        c, s = math.cos(yaw), math.sin(yaw)
        cmd = Twist()
        if self._law is not None:
            # 偏差 = 当前位姿 − 保持点（机体系），与下面比例保持的误差反号
            dyaw = math.atan2(math.sin(yaw - tyaw), math.cos(yaw - tyaw))
            with self._lock:
                vx, vy, wz = self._law.update(
                    (-(c * ex + s * ey), s * ex - c * ey, dyaw), self._ff)
            cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz
            self._pub.publish(cmd)
            return
        cmd.linear.x = max(-self.max_lin, min(self.max_lin, self.k_lin * (c * ex + s * ey)))
        cmd.linear.y = max(-self.max_lin, min(self.max_lin, self.k_lin * (-s * ex + c * ey)))
        eyaw = math.atan2(math.sin(tyaw - yaw), math.cos(tyaw - yaw))
        cmd.angular.z = max(-self.max_ang, min(self.max_ang, self.k_ang * eyaw))
        self._pub.publish(cmd)

    # ---------- 接口 ----------
    def hold(self):
        """从当前位姿开始保持（并开始记录真值漂移）."""
        with self._lock:
            if self._law is not None:
                self._law.reset()
            self._target = self._odom
            self._gt_ref = self._gt
            self.generation += 1
            self._max_drift = 0.0
            self._max_yaw = 0.0

    def yaw_authority(self):
        """定点保持可抵消的偏航力矩（N·m）：I_z·k_in·max_ang / 1.3（指令限幅留 30 % 裕量）."""
        return self._yaw_authority

    @property
    def stamp(self):
        """最新里程计的时间戳（秒）；没有里程计时为 None."""
        return self._odom_stamp

    @property
    def active(self):
        return self._target is not None

    def error(self):
        """当前里程计位姿相对保持目标的 (位置误差 米, 朝向误差 弧度)；未保持时为 None."""
        target, odom = self._target, self._odom
        if target is None or odom is None:
            return None
        dyaw = math.atan2(math.sin(odom[2] - target[2]), math.cos(odom[2] - target[2]))
        return math.hypot(odom[0] - target[0], odom[1] - target[1]), dyaw

    def disturbance_estimate(self):
        """估计的外力 (F_x, F_y, M_z)（base_link 系）；比例保持时为 None."""
        return self._law.disturbance_estimate() if self._law is not None else None

    def release(self):
        """停止保持，返回保持期间的真值最大漂移 (米, 度)；无真值时为 None."""
        self._target = None
        self._pub.publish(Twist())
        with self._lock:
            had_ref = self._gt_ref is not None
            self._gt_ref = None
            return (self._max_drift, math.degrees(self._max_yaw)) if had_ref else None

    def shutdown(self):
        """停止后台线程并销毁节点（须先于 rclpy 关闭调用，否则进程退出时会 abort）."""
        self._target = None
        self._exec.shutdown()
        self._thread.join(timeout=2.0)
        self.node.destroy_node()
