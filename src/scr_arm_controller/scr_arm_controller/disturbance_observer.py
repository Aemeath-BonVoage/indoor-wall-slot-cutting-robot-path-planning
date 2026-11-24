"""
定点保持的抗扰控制律：P / PID / 扰动观测器（扩张状态观测器）与设计工具（不依赖 ROS）.

单轴模型（x、y、ψ 三个轴在保持点附近解耦）。底盘速度环是对速度的比例控制，外力
（切割力）以加速度的形式进入::

    ė = v
    v̇ = k_in·(u + d − v),      d = F / (M·k_in)

e 为偏离保持点的位移（或转角），v 为速度，u 为速度指令，k_in 为速度环增益（仿真中的
力控底盘为 20 /s），F 为外力（力矩），M 为质量（转动惯量）。d 是“折算到输入端的扰动”：
它与 u 从同一个通道进入（匹配扰动），因此只要估计出 d，在指令中减去它就能抵消。

- 只用比例反馈 u = −k_p·e 时，常值外力留下稳态偏差 e_ss = F / (M·k_in·k_p)；
- PI / PID 用积分消除稳态偏差，需要抗积分饱和；
- 扰动观测器把 d 作为第三个状态（ḋ = 0）与 (e, v) 一起估计，u = −k_p·ê − k_d·v̂ − d̂。
  观测器用实际下发（限幅后）的指令推进，限幅期间估计仍然正确，不会像积分器那样累积；
  由规划可预知的扰动 d_ff 直接进入观测器模型与指令，观测器只需估计剩余部分。

离散化：指令零阶保持，周期 T，a = exp(−k_in·T)::

    e⁺ = e + (1 − a)/k_in·v + (T − (1 − a)/k_in)·(u + d)
    v⁺ = a·v + (1 − a)·(u + d)

测量到指令的延迟按整拍计（delay_steps）：观测器用延迟后的指令推进，并把估计向前预测
delay_steps 拍后再用于反馈。
"""
import math

import numpy as np


def clamp(x, lim):
    return max(-lim, min(lim, x))


# ---------------------------------------------------------------------------
# 模型与设计工具
# ---------------------------------------------------------------------------
class AxisModel:
    """单轴标称模型的零阶保持离散化."""

    def __init__(self, k_in, dt):
        self.k_in, self.dt = float(k_in), float(dt)
        a = math.exp(-k_in * dt)
        g = (1.0 - a) / k_in
        self.A = np.array([[1.0, g], [0.0, a]])
        self.B = np.array([dt - g, 1.0 - a])
        # 增广状态 (e, v, d)：d 为常值扰动
        self.Aa = np.array([[1.0, g, dt - g], [0.0, a, 1.0 - a], [0.0, 0.0, 1.0]])
        self.Ba = np.array([dt - g, 1.0 - a, 0.0])
        self.C = np.array([1.0, 0.0, 0.0])


def observer_gain(model, poles):
    """
    “当前估计器”的观测增益 L：使 (I − L·C)·A_a 的特征值为给定的 poles（Ackermann 公式）.

    poles 为三个 z 平面极点（实数，或一对共轭复数加一个实数）。
    """
    A, C = model.Aa, model.C
    coef = np.real(np.poly(poles))                  # z³ + c₁z² + c₂z + c₃
    alpha = sum(c * np.linalg.matrix_power(A, 3 - i) for i, c in enumerate(coef))
    obs = np.vstack([C @ np.linalg.matrix_power(A, i) for i in (1, 2, 3)])
    return alpha @ np.linalg.solve(obs, np.array([0.0, 0.0, 1.0]))


def bandwidth_gain(model, omega_o):
    """带宽参数化：三个观测器极点都放在 z = exp(−ω_o·T)."""
    return observer_gain(model, [math.exp(-omega_o * model.dt)] * 3)


def dare(A, B, Q, R, iters=20000, tol=1e-13):
    """离散代数 Riccati 方程的迭代解（值迭代）."""
    B = B.reshape(len(A), -1)
    R = np.atleast_2d(R)
    P = Q.copy()
    for _ in range(iters):
        K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
        Pn = Q + A.T @ P @ (A - B @ K)
        Pn = 0.5 * (Pn + Pn.T)
        if np.max(np.abs(Pn - P)) < tol * max(1.0, np.max(np.abs(Pn))):
            P = Pn
            break
        P = Pn
    return P, np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)


def lqr_gains(model, q_e, r_u, q_v=0.0):
    """状态反馈 u = −k_p·e − k_d·v：最小化 Σ(q_e·e² + q_v·v² + r_u·u²)，返回 (k_p, k_d)."""
    _, K = dare(model.A, model.B, np.diag([q_e, q_v]), r_u)
    return float(K[0, 0]), float(K[0, 1])


def lqi_gains(model, q_i, q_e, r_u, q_v=0.0):
    """带积分状态的 LQR（LQI）：z⁺ = z + T·e，返回 PID 形式的 (k_p, k_i, k_d)."""
    A = np.zeros((3, 3))
    A[:2, :2] = model.A
    A[2, 0], A[2, 2] = model.dt, 1.0
    B = np.array([model.B[0], model.B[1], 0.0])
    _, K = dare(A, B, np.diag([q_e, q_v, q_i]), r_u)
    return float(K[0, 0]), float(K[0, 2]), float(K[0, 1])


def kalman_gain(model, sigma_d, sigma_y, sigma_v=0.0, sigma_e=0.0):
    """
    稳态卡尔曼增益（当前估计器形式）.

    sigma_d：扰动 d 每拍的随机游走标准差；sigma_v / sigma_e：速度 / 位置的过程噪声；
    sigma_y：测量噪声标准差。只有比值 sigma_d / sigma_y 等决定增益。
    """
    A, C = model.Aa, model.C.reshape(1, 3)
    Q = np.diag([sigma_e ** 2, sigma_v ** 2, sigma_d ** 2])
    P, _ = dare(A.T, C.T, Q, np.array([[sigma_y ** 2]]))
    return (P @ C.T / (C @ P @ C.T + sigma_y ** 2)).ravel()


def pi_triple_pole(k_in):
    """PI 整定：闭环 s³ + k·s² + k·k_p·s + k·k_i 的三个极点重合于 −k/3，返回 (k_p, k_i)."""
    return k_in / 3.0, k_in ** 2 / 27.0


def pid_pole_placement(k_in, omega, zeta=1.0):
    """
    PID 极点配置：闭环 s³ + k(1 + k_d)s² + k·k_p·s + k·k_i = (s + ω)(s² + 2ζω·s + ω²).

    返回 (k_p, k_i, k_d)；k_d 作用在速度（测量的微分）上。
    """
    c2 = (1.0 + 2.0 * zeta) * omega
    c1 = (1.0 + 2.0 * zeta) * omega ** 2
    c0 = omega ** 3
    return c1 / k_in, c0 / k_in, c2 / k_in - 1.0


# ---------------------------------------------------------------------------
# 单轴控制律
# ---------------------------------------------------------------------------
class AxisP:
    """比例保持 u = −k_p·y − d_ff（现有的定点保持）."""

    def __init__(self, kp, u_max=math.inf):
        self.kp, self.u_max = kp, u_max

    def reset(self):
        pass

    def update(self, y, d_ff=0.0):
        return clamp(-self.kp * y - d_ff, self.u_max) if y is not None else 0.0

    def get_state(self):
        return np.zeros(0)

    def set_state(self, x):
        pass

    @property
    def disturbance(self):
        return 0.0


class AxisPID:
    """
    离散 PID：积分用后向欧拉，微分作用在测量上并经一阶低通；反算式抗积分饱和.

    u = −(k_p·y + I + k_d·v̂) − d_ff，I⁺ = I + k_i·T·y − (T/T_t)·(u_sat − u)。
    """

    def __init__(self, kp, ki, kd=0.0, dt=0.02, u_max=math.inf, tau_d=0.02, anti_windup=True):
        self.kp, self.ki, self.kd, self.dt, self.u_max = kp, ki, kd, dt, u_max
        self.alpha = tau_d / (tau_d + dt) if kd != 0.0 else 0.0
        self.k_aw = min(1.0, dt * ki / kp) if (anti_windup and kp > 0.0) else 0.0
        self.reset()

    def reset(self):
        self.i = 0.0
        self.v = 0.0
        self.y_prev = None

    def update(self, y, d_ff=0.0):
        if y is None:
            y = self.y_prev if self.y_prev is not None else 0.0
        if self.y_prev is None:
            self.y_prev = y
        self.v = self.alpha * self.v + (1.0 - self.alpha) * (y - self.y_prev) / self.dt
        self.y_prev = y
        i_new = self.i + self.ki * self.dt * y
        u_raw = -(self.kp * y + i_new + self.kd * self.v) - d_ff
        u = clamp(u_raw, self.u_max)
        self.i = i_new - self.k_aw * (u - u_raw)
        return u

    def get_state(self):
        return np.array([self.i, self.v, self.y_prev or 0.0])

    def set_state(self, x):
        self.i, self.v, self.y_prev = float(x[0]), float(x[1]), float(x[2])

    @property
    def disturbance(self):
        """积分项即折算到输入端的扰动估计."""
        return self.i


class DisturbanceObserver:
    """
    扩张状态观测器：估计 (e, v, d)，d 为折算到输入端的剩余扰动.

    gain 为固定增益 L（3 维，带宽参数化或稳态卡尔曼增益）；kalman 给出 (q, r0[, p0]) 时
    改用协方差递推（q 为每拍过程噪声方差的对角线，r0 为默认测量方差），这时允许某一拍
    没有测量（只预测）或测量方差逐次不同，用于低速率、带噪声的墙面相对测量。给出 p0 时
    表示保持开始时的偏差已知为 0（方差 p0），不用第一帧带噪声的测量做初值。
    """

    def __init__(self, model, gain=None, kalman=None):
        self.m = model
        self.L = None if gain is None else np.asarray(gain, dtype=float)
        self.kalman = kalman
        if (gain is None) == (kalman is None):
            raise ValueError('give either a fixed gain or Kalman covariances')
        self.reset()

    def reset(self, e=0.0, variance=None):
        """以偏差 e 初始化；卡尔曼形式下 variance 为 e 的方差（默认取测量方差 r0）."""
        self.x = np.array([e, 0.0, 0.0])
        if self.kalman is not None:
            r = self.kalman[1] if variance is None else variance
            self.P = np.diag([r, 1e-4, 1e-2])

    def predict(self, u):
        """用实际作用在对象上的输入 u（含已知扰动）推进一拍."""
        self.x = self.m.Aa @ self.x + self.m.Ba * u
        if self.kalman is not None:
            self.P = self.m.Aa @ self.P @ self.m.Aa.T + np.diag(self.kalman[0])

    def correct(self, y, r=None):
        if self.kalman is None:
            self.x = self.x + self.L * (y - self.x[0])
            return
        r = self.kalman[1] if r is None else r
        k = self.P[:, 0] / (self.P[0, 0] + r)
        self.x = self.x + k * (y - self.x[0])
        ikc = np.eye(3) - np.outer(k, self.m.C)
        self.P = ikc @ self.P @ ikc.T + r * np.outer(k, k)       # Joseph 形式

    def ahead(self, inputs):
        """按给定的未来输入序列向前预测，返回预测状态（不改变滤波器状态）."""
        x = self.x
        for u in inputs:
            x = self.m.Aa @ x + self.m.Ba * u
        return x


class AxisDOB:
    """
    基于扰动观测器的保持律 u = −k_p·ê − k_d·v̂ − d̂ − d_ff.

    delay_steps：测量到指令生效的整拍延迟；predict_ahead 为真时用延迟期间已下发的指令把
    估计预测到指令生效的时刻。观测器始终用限幅后的指令推进。
    """

    def __init__(self, model, kp, kd=0.0, gain=None, kalman=None, u_max=math.inf,
                 delay_steps=1, predict_ahead=True):
        self.m, self.kp, self.kd, self.u_max = model, kp, kd, u_max
        self.n = int(delay_steps)
        self.predict_ahead = predict_ahead
        self.obs = DisturbanceObserver(model, gain, kalman)
        self.reset()

    def reset(self):
        self.obs.reset()
        self.buf = [0.0] * (self.n + 1)     # 已下发的 (u + d_ff)：buf[0] 最旧
        self.started = False

    def update(self, y, d_ff=0.0, r=None):
        """一个控制周期：y 为本拍的偏差测量（None 表示本拍没有测量），返回速度指令."""
        if not self.started:
            if y is None:
                return clamp(-d_ff, self.u_max)
            kal = self.obs.kalman
            if kal is not None and len(kal) > 2:
                self.obs.reset(0.0, kal[2])
                self.obs.correct(y, r)
            else:
                self.obs.reset(y, r)
            self.started = True
        else:
            self.obs.predict(self.buf[0])
            if y is not None:
                self.obs.correct(y, r)
        x = self.obs.ahead(self.buf[1:]) if self.predict_ahead else self.obs.x
        u = clamp(-self.kp * x[0] - self.kd * x[1] - x[2] - d_ff, self.u_max)
        self.buf = self.buf[1:] + [u + d_ff]
        return u

    def get_state(self):
        return np.concatenate([self.obs.x, self.buf])

    def set_state(self, x):
        self.obs.x = np.array(x[:3], dtype=float)
        self.buf = [float(v) for v in x[3:]]
        self.started = True

    @property
    def disturbance(self):
        return float(self.obs.x[2])


def state_space(ctrl):
    """
    控制律在线性区（不限幅、每拍都有测量、d_ff = 0）的状态空间 (A, B, C, D)：y → u.

    直接对实现逐个状态分量求响应得到，分析用的模型与运行的代码因此是同一个。
    """
    u_max, ctrl.u_max = ctrl.u_max, math.inf
    n = len(ctrl.get_state())
    A, B, C = np.zeros((n, n)), np.zeros(n), np.zeros(n)
    try:
        for j in range(n + 1):
            x0, y = np.zeros(n), 0.0
            if j < n:
                x0[j] = 1.0
            else:
                y = 1.0
            ctrl.set_state(x0)
            u = ctrl.update(y)
            if j < n:
                A[:, j], C[j] = ctrl.get_state(), u
            else:
                B[:], D = ctrl.get_state(), u
    finally:
        ctrl.u_max = u_max
        ctrl.reset()
    return A, B, C, D


# ---------------------------------------------------------------------------
# 三轴保持控制器
# ---------------------------------------------------------------------------
MODES = ('p', 'pi', 'pid', 'dob', 'lqg')


class PeriodGuard:
    """
    测量周期监视：控制律按固定周期 period 设计，实际周期偏离过多时不应继续使用.

    update(stamp) 传入每帧测量的时间戳（秒），返回当前是否可用：最近 window 个间隔的中位数
    与 period 的相对偏差不超过 tolerance 时为 True；间隔不足 3 个时按可用处理。用中位数而
    不是单个间隔，个别丢帧或抖动不会触发切换。
    """

    def __init__(self, period, tolerance=0.5, window=10):
        self.period = float(period)
        self.tolerance = float(tolerance)
        self.window = int(window)
        self._last = None
        self._intervals = []
        self.ok = True

    @property
    def median(self):
        """最近间隔的中位数（秒）；还没有间隔时为 None."""
        if not self._intervals:
            return None
        s = sorted(self._intervals)
        n = len(s)
        return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])

    def update(self, stamp):
        if self._last is not None:
            dt = stamp - self._last
            if dt > 0.0:
                self._intervals.append(dt)
                del self._intervals[:-self.window]
        self._last = stamp
        if len(self._intervals) >= 3:
            self.ok = abs(self.median - self.period) <= self.tolerance * self.period
        return self.ok


class PlanarHold:
    """
    平面三轴 (x, y, ψ) 的定点保持控制器.

    update() 的输入是机体系下的偏差（当前位姿 − 保持点），输出速度指令 (v_x, v_y, ω)。
    mode：
      'p'    比例（k_p），与现有定点保持相同；
      'pi'   比例积分，默认按三重极点整定；
      'pid'  极点配置（pid_omega 默认 0.4·k_in，pid_zeta）；
      'dob'  扰动观测器，观测器带宽 observer_bandwidth（rad/s），反馈增益 k_p、k_d；
      'lqg'  扰动观测器取稳态卡尔曼增益（noise_ratio = σ_d / σ_y），反馈取 LQR（lqr_q、lqr_r）。
    """

    def __init__(self, mode='p', dt=0.02, kp=(5.0, 5.0, 6.0), kd=(0.0, 0.0, 0.0),
                 limits=(0.1, 0.1, 0.5), k_in=(20.0, 20.0, 20.0), mass=30.5, inertia=0.92,
                 ki=None, pid_omega=None, pid_zeta=1.0, observer_bandwidth=40.0,
                 noise_ratio=0.5, lqr_q=None, lqr_r=1.0, delay_steps=1, predict_ahead=True,
                 anti_windup=True, kalman=None):
        if mode not in MODES:
            raise ValueError(f'unknown hold mode: {mode}')
        self.mode, self.dt = mode, dt
        self.k_in = tuple(k_in)
        self.inertia = (mass, mass, inertia)
        self.axes = []
        for i in range(3):
            model = AxisModel(k_in[i], dt)
            if mode == 'p':
                ax = AxisP(kp[i], limits[i])
            elif mode == 'pi':
                p, k = pi_triple_pole(k_in[i]) if ki is None else (kp[i], ki[i])
                ax = AxisPID(p, k, 0.0, dt, limits[i], anti_windup=anti_windup)
            elif mode == 'pid':
                w = pid_omega[i] if pid_omega is not None else 0.4 * k_in[i]
                p, k, d = pid_pole_placement(k_in[i], w, pid_zeta)
                ax = AxisPID(p, k, d, dt, limits[i], anti_windup=anti_windup)
            elif mode == 'dob':
                kw = dict(kalman=kalman[i]) if kalman is not None else \
                    dict(gain=bandwidth_gain(model, observer_bandwidth))
                ax = AxisDOB(model, kp[i], kd[i], u_max=limits[i], delay_steps=delay_steps,
                             predict_ahead=predict_ahead, **kw)
            else:
                q = lqr_q[i] if lqr_q is not None else kp[i] ** 2
                p, d = lqr_gains(model, q, lqr_r)
                ax = AxisDOB(model, p, d, gain=kalman_gain(model, noise_ratio, 1.0),
                             u_max=limits[i], delay_steps=delay_steps,
                             predict_ahead=predict_ahead)
            self.axes.append(ax)

    def reset(self):
        for ax in self.axes:
            ax.reset()

    def feedforward(self, wrench):
        """已知外力 (F_x, F_y, M_z)（机体系）→ 折算到输入端的扰动 d_ff（每轴）."""
        if wrench is None:
            return (0.0, 0.0, 0.0)
        return tuple(w / (m * k) for w, m, k in zip(wrench, self.inertia, self.k_in))

    def update(self, dev, ff_wrench=None, variance=None):
        """
        一个控制周期.

        dev：(e_x, e_y, e_ψ)，某一分量为 None 表示该轴本拍没有测量（仅 dob / lqg 的卡尔曼
        形式利用这一点，其余控制律沿用上一拍的值）；ff_wrench：由规划预知的外力；
        variance：各轴本拍测量的方差（仅卡尔曼形式）。
        """
        d_ff = self.feedforward(ff_wrench)
        out = []
        for i, ax in enumerate(self.axes):
            if isinstance(ax, AxisDOB) and variance is not None:
                out.append(ax.update(dev[i], d_ff[i], variance[i]))
            else:
                out.append(ax.update(dev[i], d_ff[i]))
        return tuple(out)

    def disturbance_estimate(self):
        """估计的外力 (F_x, F_y, M_z)（不含前馈部分）."""
        return tuple(ax.disturbance * m * k
                     for ax, m, k in zip(self.axes, self.inertia, self.k_in))
