#!/usr/bin/env python3
"""
切割力作用下定点保持的离线仿真模型（不需要 ROS / Gazebo）.

两种被控对象，均按 scr_base_control 中仿真硬件接口的实现建模：

- ForcePlant（plant = force，Gazebo 默认）：指令速度与实际速度之差乘以 M·k_lin 作为驱动力
  （限幅 M·a_max、I_z·α_max），外力直接作用在底盘上；编码器读数由底盘真实运动加误差模型
  得到，因此里程计看得到外力引起的位移。
- ContactPlant（plant = wheels）：轮子跟随指令转速，底盘只受轮地接触力——每个轮在辊子
  轴线方向的滑移速度 e 产生 F = −sat(c·e, μN)。外力使底盘相对轮子滑移，编码器读的是
  轮子转速，看不到这部分位移；只有墙面相对的测量（激光、定位标记）能看到。

坐标：保持点为原点，x 指向墙面，y 向左，ψ 逆时针为正；墙面在 x = D 处。
"""
import math
import os
import random
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'src', 'scr_arm_controller'))

from scr_arm_controller import cutting_force as cf  # noqa: E402
from scr_arm_controller import disturbance_observer as do  # noqa: E402
from scr_arm_controller.wall_ekf import WallEKF  # noqa: E402

T = 0.02                 # 定点保持的控制周期（50 Hz）
K_IN = 20.0              # 底盘速度环增益 k_lin = k_ang（1/s）
KP = (5.0, 5.0, 6.0)     # 现有定点保持的比例增益
PF = cf.Platform()
M, IZ = PF.mass, PF.inertia_z
WHEEL_R = PF.wheel_radius
WHEEL_K = PF.half_length + PF.half_width
# 限幅：现有默认值，以及切割验证用的放宽值（偏航力矩上限取 μ = 0.6 时麦克纳姆轮可传递的
# 0.707·μ·M·g·(l_x + l_y) ≈ 69 N·m，折合 α_max ≈ 75 rad/s²）
LIMITS = {
    'default': dict(hold=(0.1, 0.1, 0.5), a_max=5.0, alpha_max=10.0),
    'raised': dict(hold=(0.25, 0.25, 4.0), a_max=5.0, alpha_max=75.0),
}
SIG_LIDAR = (1.3e-3, math.radians(0.26))     # 单帧激光墙面拟合：距离 / 朝向（10 Hz）
SIG_TAG = 0.3e-3                              # 定位标记的沿墙位置（30 Hz）
GYRO_NOISE, GYRO_BIAS = 0.001, 5e-5           # 模拟陀螺：白噪声（rad/s，100 Hz）与零偏（rad/s）


def clamp(x, lim):
    return max(-lim, min(lim, x))


# 机械臂沿墙轴（joint2）以外侧的连杆（link2–link4）：经关节位置 PID 挂在底盘上，不是刚性
# 连接。质量与到底盘中心的距离取自 URDF（三个连杆的质量和，以及对竖直轴的回转半径），
# 关节 PID 增益取自 scr_description/config/arm_controllers.yaml
ARM_MASS = 1.90             # kg
ARM_RADIUS = 0.414          # m，sqrt(Σ m_i·x_i² / Σ m_i)
ARM_PID = (5000.0, 5000.0, 200.0)   # joint2：N/m、N/(m·s)、N·s/m


class ForcePlant:
    """
    力控底盘（与 ScrMecanumBaseSystem 的 plant = force 相同）与编码器误差模型.

    里程计按控制器的做法积分：每个控制周期读一次轮速、乘以周期累加（odom_period = T）。相对
    连续积分，它使反馈中的位姿超前半个周期。Gazebo 中的阶跃试验表明需要这一项：按连续积分
    的第一版模型把各控制律的峰值高估 3–20 %（见 README 第 10 节）。

    arm_compliance = True（默认）时，机械臂外侧连杆作为一个质点经 joint2 的位置 PID 沿横向
    挂在底盘前方 ARM_RADIUS 处：底盘偏航加速时它滞后，高频下底盘的等效转动惯量小于整机的
    I_z。这一项在不饱和的试验中只改变峰值不到 1 %，在指令长时间饱和的试验中约 6 %。
    """

    def __init__(self, a_max=5.0, alpha_max=10.0, k_lin=K_IN, k_ang=K_IN, seed=1,
                 odom_error=True, arm_compliance=True, odom_period=T):
        # 里程计的积分方式：控制器每个控制周期读一次轮速，乘以周期累加（后向矩形）。
        # odom_period = 0 表示按仿真步长连续积分
        self.odom_period, self._since_odom = odom_period, 0.0
        self.arm = arm_compliance
        self.arm_pos, self.arm_vel, self.arm_int = 0.0, 0.0, 0.0   # joint2 的横向偏差（米）
        self.k_lin, self.k_ang = k_lin, k_ang
        self.f_max, self.t_max = M * a_max, IZ * alpha_max
        self.pose = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]          # 世界系速度
        self.fvel = [0.0, 0.0, 0.0]         # 位姿差分低通（5 ms）后的速度：速度环与编码器共用
        self.odom = [0.0, 0.0, 0.0]
        self.rng = random.Random(seed)
        e = odom_error
        self.scale = (0.004, -0.003, 0.005, -0.006) if e else (0.0,) * 4
        self.slip_sigma = 0.02 if e else 0.0
        self.slip_tau = 0.5
        self.lateral_slip = 0.03 if e else 0.0
        self.yaw_slip = 0.02 if e else 0.0
        self.slip = [0.0] * 4
        self.drive = (0.0, 0.0, 0.0)        # 最近一步的驱动力 / 力矩（机体系近似）

    def step(self, cmd, wrench, dt):
        """推进 dt；cmd 为 (v_x, v_y, ω) 指令（机体系），wrench 为外力 (F_x, F_y, M_z)（机体系）."""
        x, y, psi = self.pose
        c, s = math.cos(psi), math.sin(psi)
        a = dt / (0.005 + dt)
        for i in range(3):
            self.fvel[i] += a * (self.vel[i] - self.fvel[i])
        fx = M * self.k_lin * (cmd[0] * c - cmd[1] * s - self.fvel[0])
        fy = M * self.k_lin * (cmd[0] * s + cmd[1] * c - self.fvel[1])
        norm = math.hypot(fx, fy)
        if norm > self.f_max:
            fx, fy = fx * self.f_max / norm, fy * self.f_max / norm
        tz = clamp(IZ * self.k_ang * (cmd[2] - self.fvel[2]), self.t_max)
        self.drive = (fx, fy, tz)
        if self.arm:
            self._step_with_arm(fx, fy, tz, wrench, c, s, dt)
        else:
            self.vel[0] += dt * (fx + wrench[0] * c - wrench[1] * s) / M
            self.vel[1] += dt * (fy + wrench[0] * s + wrench[1] * c) / M
            self.vel[2] += dt * (tz + wrench[2]) / IZ
        for i in range(3):
            self.pose[i] += dt * self.vel[i]
        if self.odom_period <= 0.0:
            self._encoders(c, s, dt)
            return
        self._since_odom += dt
        if self._since_odom >= self.odom_period - 1e-9:
            c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
            self._encoders(c, s, self._since_odom)
            self._since_odom = 0.0

    def _step_with_arm(self, fx, fy, tz, wrench, c, s, dt):
        """底盘（去掉机械臂外侧连杆）+ 横向弹性连接的质点；小角度下横向与偏航耦合."""
        kp, ki, kd = ARM_PID
        fj = -(kp * self.arm_pos + ki * self.arm_int + kd * self.arm_vel)   # 关节对质点的力
        m_b, i_b = M - ARM_MASS, IZ - ARM_MASS * ARM_RADIUS ** 2
        # 机体系下的合力：x 方向质点与底盘刚性连接
        bx = c * fx + s * fy + wrench[0]
        by = -s * fx + c * fy + wrench[1] - fj
        ax, ay = bx / M, by / m_b
        alpha = (tz + wrench[2] - ARM_RADIUS * fj) / i_b
        # 质点的横向加速度 = 底盘横向 + 偏航引起的切向 + 关节相对运动
        self.arm_vel += dt * (fj / ARM_MASS - ay - ARM_RADIUS * alpha)
        self.arm_pos += dt * self.arm_vel
        self.arm_int += dt * self.arm_pos
        self.vel[0] += dt * (c * ax - s * ay)
        self.vel[1] += dt * (s * ax + c * ay)
        self.vel[2] += dt * alpha

    def _encoders(self, c, s, dt):
        vx = c * self.fvel[0] + s * self.fvel[1]
        vy = (1.0 + self.lateral_slip) * (-s * self.fvel[0] + c * self.fvel[1])
        wz = (1.0 + self.yaw_slip) * self.fvel[2]
        w = [vx - vy - WHEEL_K * wz, vx + vy + WHEEL_K * wz,
             vx + vy - WHEEL_K * wz, vx - vy + WHEEL_K * wz]
        if self.slip_sigma > 0.0:
            a = math.exp(-dt / self.slip_tau)
            b = self.slip_sigma * math.sqrt(1.0 - a * a)
            for i in range(4):
                self.slip[i] = a * self.slip[i] + b * self.rng.gauss(0.0, 1.0)
        w = [wi * (1.0 + sc + sl) for wi, sc, sl in zip(w, self.scale, self.slip)]
        integrate_odom(self.odom, w, dt)


class ContactPlant:
    """轮地接触底盘（与 plant = wheels 的接触力模型相同）：里程计只反映轮子的转动."""

    POS = ((PF.half_length, PF.half_width), (PF.half_length, -PF.half_width),
           (-PF.half_length, PF.half_width), (-PF.half_length, -PF.half_width))
    H = math.sqrt(0.5)
    ROLLER = ((H, -H), (H, H), (H, H), (H, -H))

    def __init__(self, mu=0.6, damping=3000.0, tau_wheel=0.02):
        self.mu, self.c, self.tau = mu, damping, tau_wheel
        self.pose = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]          # 机体系速度
        self.wheel = [0.0] * 4              # 轮缘线速度 r·ω（“前进为正”）
        self.odom = [0.0, 0.0, 0.0]
        self.loads = [M * cf.G / 4.0] * 4
        self.saturated = 0

    def step(self, cmd, wrench, dt):
        vx, vy, wz = self.vel
        target = (cmd[0] - cmd[1] - WHEEL_K * cmd[2], cmd[0] + cmd[1] + WHEEL_K * cmd[2],
                  cmd[0] + cmd[1] - WHEEL_K * cmd[2], cmd[0] - cmd[1] + WHEEL_K * cmd[2])
        fx, fy, tz = wrench
        for i in range(4):
            self.wheel[i] += dt / self.tau * (target[i] - self.wheel[i])
            px, py = self.POS[i]
            rx, ry = self.ROLLER[i]
            e = (vx - wz * py - self.wheel[i]) * rx + (vy + wz * px) * ry
            lim = self.mu * max(self.loads[i], 0.0)
            f = -clamp(self.c * e, lim)
            self.saturated += abs(self.c * e) > lim
            fx += f * rx
            fy += f * ry
            tz += f * (px * ry - py * rx)
        self.vel = [vx + dt * fx / M, vy + dt * fy / M, wz + dt * tz / IZ]
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        self.pose[0] += dt * (c * self.vel[0] - s * self.vel[1])
        self.pose[1] += dt * (s * self.vel[0] + c * self.vel[1])
        self.pose[2] += dt * self.vel[2]
        integrate_odom(self.odom, self.wheel, dt)


def integrate_odom(odom, w, dt):
    """轮缘线速度 [lf, rf, lb, rb] → 麦克纳姆正运动学 → 里程计积分（中点法）."""
    vx = 0.25 * (w[0] + w[1] + w[2] + w[3])
    vy = 0.25 * (-w[0] + w[1] + w[2] - w[3])
    wz = 0.25 / WHEEL_K * (-w[0] + w[1] - w[2] + w[3])
    mid = odom[2] + 0.5 * wz * dt
    c, s = math.cos(mid), math.sin(mid)
    odom[0] += (c * vx - s * vy) * dt
    odom[1] += (s * vx + c * vy) * dt
    odom[2] += wz * dt


def body_deviation(pose):
    """位姿相对保持点（原点）的偏差，表示在机体系."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (c * pose[0] + s * pose[1], -s * pose[0] + c * pose[1], pose[2])


def tip_deviation(pose, tip):
    """刀尖（机体系 tip = (x_t, y_t)）相对名义位置的偏移 (Δx 进深方向, Δy 沿墙向左)."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (pose[0] + c * tip[0] - s * tip[1] - tip[0],
            pose[1] + s * tip[0] + c * tip[1] - tip[1])


class ConstantWrench:
    """试验用的外力：t_on 到 t_off 之间为常值 (F_x, F_y, M_z)（接口与 CuttingProcess 相同）."""

    def __init__(self, wrench, t_on=1.0, t_off=7.0, tip=None):
        self.w = cf.Wrench(fx=wrench[0], fy=wrench[1], mz=wrench[2])
        self.profile = cf.CutProfile(kind='step', t_start=t_on, feed_rate=1.0,
                                     length=t_off - t_on)
        self.platform = cf.Platform(tool_y=tip[1] if tip else 0.0)

    def wrench(self, t):
        on = self.profile.t_start <= t < self.profile.t_end
        return self.w if on else cf.Wrench()

    nominal = wrench


def make_hold(name, limits='raised', **kw):
    """按名称构造三轴保持控制器；返回 (控制器, 前馈比例)."""
    lim = LIMITS[limits]['hold']
    ff = 0.0
    base = name
    if name.endswith('+FF'):
        base, ff = name[:-3], 0.7       # 前馈按“模型低估 30 %”计，不假设模型准确
    mode = {'P': 'p', 'PI': 'pi', 'PID': 'pid', 'DOB': 'dob', 'LQG': 'lqg'}[base]
    return do.PlanarHold(mode, dt=T, kp=KP, limits=lim, **kw), ff


def simulate(hold, process, t_end, plant=None, ff=0.0, dt=1e-3, deviation=None):
    """
    闭环仿真.

    hold：PlanarHold；process：cf.CuttingProcess；ff：前馈比例（0 = 不用）；
    deviation：可选的偏差测量函数 f(k, plant) → (e_x, e_y, e_ψ)（默认取上一拍的里程计）。
    返回每个控制周期的记录（numpy 数组的字典）。
    """
    plant = plant or ForcePlant()
    n_sub = int(round(T / dt))
    tip = PF.tool_tip(process.profile.depth)
    tip = (tip[0], process.platform.tool_y)
    prev_odom = tuple(plant.odom)
    rec = {k: [] for k in ('t', 'x', 'y', 'psi', 'ox', 'oy', 'opsi', 'ux', 'uy', 'uw',
                           'fx', 'fy', 'mz', 'tipx', 'tipy')}
    for k in range(int(round(t_end / T))):
        t = k * T
        w = process.wrench(t)
        if isinstance(plant, ContactPlant):
            plant.loads = cf.wheel_loads(w, PF)
        dev = deviation(k, plant) if deviation else body_deviation(prev_odom)
        prev_odom = tuple(plant.odom)
        ffw = tuple(ff * v for v in process.nominal(t).planar) if ff else None
        cmd = hold.update(dev, ffw) if not isinstance(dev, dict) else \
            hold.update(dev['dev'], ffw, dev.get('var'))
        tx, ty = tip_deviation(plant.pose, tip)
        for key, val in zip(rec, (t, *plant.pose, *plant.odom, *cmd, *w.planar, tx, ty)):
            rec[key].append(val)
        for _ in range(n_sub):
            plant.step(cmd, w.planar, dt)
    return {k: np.array(v) for k, v in rec.items()}


def metrics(rec, profile, limits=None, settle_mm=0.5):
    """一次仿真的指标（毫米 / 度）：统计窗口为刀具在墙内的时段（下刀开始到退刀）."""
    t = rec['t']
    win = (t >= profile.t_start) & (t <= profile.t_end)
    tip = np.hypot(rec['tipx'], rec['tipy'])[win] * 1e3
    cross = np.abs(rec['tipy'][win]) * 1e3
    depth = np.abs(rec['tipx'][win]) * 1e3
    base = np.hypot(rec['x'], rec['y'])[win] * 1e3
    yaw = np.degrees(np.abs(rec['psi'][win]))
    out = dict(tip_peak_mm=tip.max(), tip_rms_mm=math.sqrt(np.mean(tip ** 2)),
               along_wall_peak_mm=cross.max(), depth_peak_mm=depth.max(),
               base_peak_mm=base.max(), yaw_peak_deg=yaw.max(),
               cmd_lin_rms=math.sqrt(np.mean(rec['ux'][win] ** 2 + rec['uy'][win] ** 2)),
               cmd_ang_rms=math.sqrt(np.mean(rec['uw'][win] ** 2)))
    # 退刀（切削力突然消失）后 2 s 内的刀尖偏移峰值：刀具已离开墙面，不影响槽线
    rel = (t > profile.t_end) & (t <= profile.t_end + 2.0)
    out['release_peak_mm'] = float((np.hypot(rec['tipx'], rec['tipy'])[rel] * 1e3).max())
    # 恢复时间：切削期间刀尖偏差最后一次超过 settle_mm 的时刻（相对开始进给）
    eng = (t >= profile.t_feed) & (t <= profile.t_end)
    te, tipe = t[eng], np.hypot(rec['tipx'], rec['tipy'])[eng] * 1e3
    over = np.where(tipe > settle_mm)[0]
    if len(over) == 0:
        out['recovery_s'] = 0.0
    elif over[-1] >= len(te) - 2:
        out['recovery_s'] = float('nan')        # 切削结束时仍未回到容差内
    else:
        out['recovery_s'] = te[over[-1]] - profile.t_feed
    if limits is not None:
        lim = LIMITS[limits]['hold']
        sat = (np.abs(rec['ux'][win]) >= 0.999 * lim[0]) | \
            (np.abs(rec['uy'][win]) >= 0.999 * lim[1]) | \
            (np.abs(rec['uw'][win]) >= 0.999 * lim[2])
        out['saturated_frac'] = float(np.mean(sat))
    return out


# ---------------------------------------------------------------------------
# 里程计看不到打滑时的测量与估计
# ---------------------------------------------------------------------------
def relative(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    ex, ey = b[0] - a[0], b[1] - a[1]
    return c * ex + s * ey, -s * ex + c * ey, b[2] - a[2]


class WallSensors:
    """
    激光（10 Hz：离墙距离、朝向）、定位标记（30 Hz：沿墙位置）与陀螺（每拍的转角增量）.

    激光与标记测量的是上一拍的状态。陀螺测的是底盘真实角速度，与轮子是否打滑无关。
    """

    def __init__(self, seed, use_tag=False, use_gyro=False):
        self.rng = np.random.default_rng(seed)
        self.use_tag, self.use_gyro = use_tag, use_gyro
        self.prev = (0.0, 0.0, 0.0)
        self.heading = 0.0          # 陀螺积分 + 激光慢校正的朝向

    def sample(self, k, pose):
        x, y, psi = self.prev
        out = {}
        if k % 5 == 0:
            out['lidar'] = (PF.standoff - x + self.rng.normal(0, SIG_LIDAR[0]),
                            psi + self.rng.normal(0, SIG_LIDAR[1]))
        if self.use_tag and int(k * 0.6) != int((k - 1) * 0.6):
            out['tag'] = -y + self.rng.normal(0, SIG_TAG)
        if self.use_gyro:
            # 一拍内两个 100 Hz 采样的平均
            rate = (pose[2] - psi) / T + GYRO_BIAS + self.rng.normal(0, GYRO_NOISE / math.sqrt(2))
            self.heading += rate * T
            if 'lidar' in out:      # 互补滤波：激光以约 5 s 的时间常数校正陀螺积分的漂移
                self.heading += 0.02 * (out['lidar'][1] - self.heading)
            out['heading'] = self.heading
        self.prev = tuple(pose)
        return out


def kalman_dob_feedback(sensors):
    """卡尔曼形式的扰动观测器直接使用带噪声、低速率的墙面相对测量；无标记时沿墙方向用里程计."""
    state = {'odom': (0.0, 0.0, 0.0)}

    def deviation(k, plant):
        z = sensors.sample(k, plant.pose)
        odom_dev = body_deviation(state['odom'])
        state['odom'] = tuple(plant.odom)
        dev = [None, odom_dev[1] if not sensors.use_tag else None, None]
        var = [SIG_LIDAR[0] ** 2, 1e-8 if not sensors.use_tag else SIG_TAG ** 2,
               SIG_LIDAR[1] ** 2]
        if 'lidar' in z:
            dev[0], dev[2] = -(z['lidar'][0] - PF.standoff), z['lidar'][1]
        if 'tag' in z:
            dev[1] = -z['tag']
        if 'heading' in z:
            dev[2], var[2] = z['heading'], (2e-4) ** 2
        return {'dev': dev, 'var': var}
    return deviation


def wall_ekf_feedback(sensors):
    """现有的墙面相对 EKF（里程计增量预测 + 激光 / 标记更新）给出偏差，供比例保持使用."""
    ekf = WallEKF()
    ekf.reset(PF.standoff, 0.0, 0.0, sigma=(1e-3, 1e-3, math.radians(0.1)))
    state = {'odom': (0.0, 0.0, 0.0)}
    r_wall = np.diag(np.square(SIG_LIDAR))
    r_tag = np.diag([3e-3 ** 2, SIG_TAG ** 2])

    def deviation(k, plant):
        odom = tuple(plant.odom)
        ekf.predict(*relative(state['odom'], odom))
        state['odom'] = odom
        z = sensors.sample(k, plant.pose)
        if 'lidar' in z:
            ekf.update_wall(z['lidar'][0], -z['lidar'][1], r_wall)
        if 'tag' in z:
            d, u, psi = ekf.x[0], z['tag'], ekf.x[2]
            c, s = math.cos(psi), math.sin(psi)
            ekf.update_tag(np.array([d * c + u * s, -d * s + u * c]), r_tag)
        return (-(ekf.x[0] - PF.standoff), -ekf.x[1], ekf.x[2])
    return deviation
