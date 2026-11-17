"""
全向底盘的墙面精对位控制律与收敛判据——纯算法部分（不依赖 ROS）.

结构参照 Nav2 opennav_docking：检测 → 滤波位姿 → 控制器 → 收敛判断，各环节独立：
  - 检测与滤波见 fiducial.py / wall_ekf.py；
  - 本模块给出控制律 WallAlignController 与收敛判据 ConvergenceMonitor。

控制律（推导与稳定性分析见 docs/experiments/fine_alignment/README.md）：
  全向底盘在墙面系中的运动学可精确反馈线性化：先在 (n, l)（指向墙面 / 左方）中给出期望
  平移速度，再旋转到 base_link，平移与转向解耦，每个通道都是一个积分器 ė = v。
  每个通道用“平方根”整形的饱和比例律（ArduPilot sqrt_controller）::

      v(e) = k·e                                  |e| ≤ e₀ = a/k²
      v(e) = sign(e)·√(2a(|e| − e₀/2))            |e| > e₀

  远处为恒减速度 a 的制动曲线（不会因速度上限内的大误差冲过头），近处为线性，
  斜率 k 与线性区连续可导；再经速度上限与加速度（变化率）限幅。v(e)/e ∈ (0, k]，
  离散采样下只要 k·T 满足线性区的稳定条件，全局都满足（扇区条件）。
  与第一版的“比例 + 最小速度 + 死区”相比：没有最小速度（不会在零点附近形成继电器式
  极限环），没有死区（稳态误差不再由死区宽度决定），停止由收敛判据决定而不是由控制律截断。
"""
import math

import numpy as np


def sqrt_controller(error, k, accel):
    """平方根整形的比例律：返回速度（与 error 同号），斜率在零点为 k，远处恒减速度 accel."""
    if k <= 0.0:
        return math.copysign(math.sqrt(2.0 * accel * abs(error)), error)
    if accel <= 0.0:
        return k * error
    e0 = accel / (k * k)
    if abs(error) <= e0:
        return k * error
    return math.copysign(math.sqrt(2.0 * accel * (abs(error) - 0.5 * e0)), error)


def rate_limit(value, previous, max_step):
    """每周期变化量不超过 max_step（加速度限幅）."""
    return previous + max(-max_step, min(max_step, value - previous))


class WallAlignController:
    """
    墙面精对位控制器：输入 (d, u, ψ) 估计与目标，输出 base_link 速度 (vx, vy, wz).

    k_lin / k_ang：线性区增益（1/s）；a_lin / a_ang：制动曲线的减速度；
    v_max / w_max：速度上限；acc_lin / acc_ang：指令变化率上限（应大于制动减速度，
    使减速段不被限幅，否则限幅器相当于额外的延迟，会产生超调）；dt：控制周期（秒）。
    """

    def __init__(self, k_lin=1.5, k_ang=1.5, a_lin=0.15, a_ang=0.3, v_max=0.15,
                 w_max=0.3, acc_lin=0.4, acc_ang=0.8, dt=0.05):
        self.k_lin, self.k_ang = k_lin, k_ang
        self.a_lin, self.a_ang = a_lin, a_ang
        self.v_max, self.w_max = v_max, w_max
        self.acc_lin, self.acc_ang = acc_lin, acc_ang
        self.dt = dt
        self.prev = np.zeros(3)

    def reset(self, current=(0.0, 0.0, 0.0)):
        self.prev = np.asarray(current, dtype=float)

    def errors(self, state, target):
        """(e_d, e_u, e_ψ)：离墙偏远、沿墙偏右、逆时针偏转为正."""
        d, u, psi = state
        d_t, u_t, psi_t = target
        return d - d_t, u - u_t, math.atan2(math.sin(psi - psi_t), math.cos(psi - psi_t))

    def command(self, state, target, max_toward_wall=None):
        """
        计算一次速度指令.

        max_toward_wall：向墙速度上限（米/秒，来自防碰撞守护；None 表示不限）。
        """
        e_d, e_u, e_psi = self.errors(state, target)
        psi = state[2]
        # 平移：在 (n, l) 中沿误差向量方向，速度大小按平方根律整形 → 直线逼近目标
        e = math.hypot(e_d, e_u)
        speed = min(abs(sqrt_controller(e, self.k_lin, self.a_lin)), self.v_max)
        v_n = speed * e_d / e if e > 1e-12 else 0.0     # e_d > 0（偏远）→ 向墙
        v_l = speed * e_u / e if e > 1e-12 else 0.0     # e_u > 0（偏右）→ 向左
        if max_toward_wall is not None and v_n > max_toward_wall:
            v_n = max(max_toward_wall, 0.0)
        # (n, l) → base_link：R(−ψ)
        c, s = math.cos(psi), math.sin(psi)
        vx = c * v_n + s * v_l
        vy = -s * v_n + c * v_l
        wz = -max(-self.w_max, min(self.w_max, sqrt_controller(e_psi, self.k_ang, self.a_ang)))
        cmd = np.array([vx, vy, wz])
        # 加速度限幅：平移按矢量限幅（保持方向），转向单独限幅
        dv = cmd[:2] - self.prev[:2]
        n = np.linalg.norm(dv)
        step = self.acc_lin * self.dt
        if n > step:
            cmd[:2] = self.prev[:2] + dv * step / n
        cmd[2] = rate_limit(cmd[2], self.prev[2], self.acc_ang * self.dt)
        self.prev = cmd
        return float(cmd[0]), float(cmd[1]), float(cmd[2])


class ConvergenceMonitor:
    """
    收敛判据：误差在容差内 且 底盘近似静止 且 估计足够确定，连续 n_required 个周期.

    只看误差会在机器人仍以几 mm/s 滑行时就宣布到位（停车后再滑出容差）；
    要求实测速度接近 0 才能保证停下来的位置就是判定时的位置。σ 条件防止在标记长时间
    丢失、仅靠里程计外推时宣布到位。
    """

    def __init__(self, tol_d=0.002, tol_u=0.002, tol_psi=math.radians(0.3),
                 v_eps=0.003, w_eps=math.radians(0.3), sigma_max=(0.002, 0.002, None),
                 n_required=10):
        self.tol = (tol_d, tol_u, tol_psi)
        self.v_eps, self.w_eps = v_eps, w_eps
        self.sigma_max = sigma_max
        self.n_required = n_required
        self.count = 0

    def reset(self):
        self.count = 0

    def within(self, errors, velocity=(0.0, 0.0, 0.0), sigma=None):
        ok = all(abs(e) < t for e, t in zip(errors, self.tol))
        ok = ok and math.hypot(velocity[0], velocity[1]) < self.v_eps
        ok = ok and abs(velocity[2]) < self.w_eps
        if sigma is not None:
            ok = ok and all(m is None or s < m for s, m in zip(sigma, self.sigma_max))
        return ok

    def update(self, errors, velocity=(0.0, 0.0, 0.0), sigma=None):
        """返回是否已连续 n_required 个周期满足条件."""
        self.count = self.count + 1 if self.within(errors, velocity, sigma) else 0
        return self.count >= self.n_required
