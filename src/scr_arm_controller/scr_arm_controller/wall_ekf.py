"""
墙面相对状态的扩展卡尔曼滤波（EKF）——纯算法部分（不依赖 ROS）.

状态 X = (d, u, ψ[, s])，前三项相对墙面坐标原点（参考定位标记）所在的墙面（与项目的
墙面坐标 (u, v) 一致）：
  d  base_link 原点到墙面的垂直距离（米，> 0）
  u  base_link 原点沿墙的位置，以参考标记为 0，面向墙面时向右为正（米）
  ψ  底盘朝向相对“正对墙面”的偏角，逆时针为正（rad）
  s  可选：里程计横移（base_link y 方向）的比例修正，真实横移 = (1 + s)·里程计横移

记 (n, l) 为“指向墙面 / 面向墙面时的左方”两个水平方向。底盘 x 轴在 (n, l) 中为
(cos ψ, sin ψ)，y 轴为 (−sin ψ, cos ψ)。

过程模型（里程计增量 (Δx, Δy, Δψ) = 两次里程计位姿之间的 SE(2) 相对变换，表示在
前一时刻的 base_link 中，因此用前一时刻的朝向即为精确合成，与增量大小无关）::

    Δy' = (1 + s)·Δy                       （不估计 s 时 s ≡ 0）
    d⁺ = d − (Δx cos ψ − Δy' sin ψ)
    u⁺ = u − (Δx sin ψ + Δy' cos ψ)
    ψ⁺ = ψ + Δψ
    s⁺ = s + w_s,   Var(w_s) = q_s²·|Δy|   （随横移距离缓慢变化）

过程噪声按里程计运动模型（Thrun《概率机器人》5.4）随位移与转角增长，另加一项很小的
随机游走（允许墙面 / 标记参照缓慢修正，也防止长时间静止后协方差过小而拒收测量）。

为什么要估计 s：激光只观测 d 与 ψ，站间的沿墙位置只能由里程计外推。麦克纳姆轮横移时
辊子打滑，编码器系统性地高估横向位移（本项目的编码器模型为 3 %），这一误差与横移距离
成正比、方向固定，白噪声式的过程噪声描述不了它。把它作为状态估计后，只要在一段横移中
持续看到标记（同一个标记在视野内移动，或两个已知间距的标记），s 就可观测，看不到标记
的区段用修正后的里程计外推。s 只在有净位移的两次位置观测之间可观测：往返同一位置
（去程、回程的误差相互抵消）不提供 s 的信息。

测量模型（安装误差 / 零位标定偏置 c = (c_d, c_u, c_ψ) 在模型内部扣除：传感器“看到”的是
X + c，滤波器估计的是 X）::

    激光墙面   z_L = (d, φ),  φ = −ψ（墙面法向在 base_link 中的方位角）          线性
    标记 i     z_T = R(−ψ)·(d, u − u_i)                                           非线性
               = (d cos ψ + (u − u_i) sin ψ, −d sin ψ + (u − u_i) cos ψ)

u_i 为标记 i 在墙面坐标中的已知位置（标记地图，参考标记 u = 0）。手工粘贴的标记位置有
毫米级误差 σ_map：把它作为测量噪声的一部分（R + σ_map²·h_u h_uᵀ，h_u = ∂z_T/∂u），
从一个标记过渡到另一个标记时的不一致不会被门限当作离群值拒收。

每次更新先做马氏距离门限（χ² 检验，2 自由度 99 % 分位 9.21），超出则拒收（离群值：
激光拟合到障碍物、误检、遮挡时的残缺角点等）。协方差更新用 Joseph 形式保证数值对称正定。
"""
import math

import numpy as np

CHI2_2DOF_99 = 9.21


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class WallEKF:
    """(d, u, ψ) 三状态 EKF，可选第四个状态：横移比例修正 s."""

    def __init__(self, alpha_trans=0.02, alpha_rot=0.02, alpha_cross=0.002,
                 q_walk=(1e-4, 1e-4, math.radians(0.01)), gate=CHI2_2DOF_99,
                 estimate_scale=False, scale_init=0.0, scale_sigma=0.05, scale_walk=0.002):
        """
        过程噪声与门限参数.

        alpha_trans：平移噪声 σ = alpha_trans·|Δp|（2 % 的里程计尺度误差）；
        alpha_rot：转角噪声 σ = alpha_rot·|Δψ|；alpha_cross：平移引起的转角噪声（rad/m）；
        q_walk：每次预测附加的随机游走标准差 (d, u, ψ)；
        estimate_scale：是否估计横移比例修正 s；scale_init / scale_sigma：s 的先验均值与标准差
        （离线标定值可作为先验）；scale_walk：s 的随机游走（1/√m，每米横移的增长）。
        """
        self.alpha_trans, self.alpha_rot, self.alpha_cross = alpha_trans, alpha_rot, alpha_cross
        self.q_walk = np.asarray(q_walk, dtype=float)
        self.gate = gate
        self.estimate_scale = bool(estimate_scale)
        self.scale_walk = scale_walk
        # s 的估计跨越多次对位保留（它是底盘与地面的性质，不随墙面原点重置）
        self._scale = (float(scale_init), float(scale_sigma) ** 2)
        self.offset = np.zeros(3)          # 零位标定偏置 c（传感器看到 X + c）
        self.x = None
        self.P = None
        self.stats = {'wall': [0, 0], 'tag': [0, 0]}   # [接受, 拒收]
        self.last_nis = {'wall': float('nan'), 'tag': float('nan')}

    # ------------------------------------------------------------------
    @property
    def n(self):
        return 4 if self.estimate_scale else 3

    @property
    def initialized(self):
        return self.x is not None

    @property
    def scale(self):
        """当前的横移比例修正 s 与其标准差（不估计时为 (0, 0)）."""
        if not self.estimate_scale:
            return 0.0, 0.0
        if self.initialized:
            return float(self.x[3]), math.sqrt(max(self.P[3, 3], 0.0))
        return self._scale[0], math.sqrt(self._scale[1])

    def _save_scale(self):
        if self.estimate_scale and self.initialized:
            self._scale = (float(self.x[3]), float(self.P[3, 3]))

    def reset(self, d, u, psi, sigma=(0.01, 0.5, math.radians(2.0))):
        """以给定状态与标准差初始化（未见到标记时 u 取 0、σ_u 取大值）；s 沿用之前的估计."""
        self._save_scale()
        var = list(np.asarray(sigma, dtype=float) ** 2)
        x = [d, u, psi]
        if self.estimate_scale:
            x.append(self._scale[0])
            var.append(self._scale[1])
        self.x = np.array(x, dtype=float)
        self.P = np.diag(var)

    def clear(self):
        """清空 (d, u, ψ)（开始新一次对位），保留 s 的估计."""
        self._save_scale()
        self.x = None
        self.P = None

    def sigma(self):
        """(d, u, ψ) 的标准差."""
        return np.sqrt(np.diag(self.P)[:3]) if self.P is not None else np.full(3, np.inf)

    # ------------------------------------------------------------------
    def predict(self, dx, dy, dpsi):
        """用里程计相对位姿 (Δx, Δy, Δψ)（前一时刻 base_link 中）预测."""
        n = self.n
        d, u, psi = self.x[:3]
        k = 1.0 + self.x[3] if self.estimate_scale else 1.0
        dyc = k * dy
        c, s = math.cos(psi), math.sin(psi)
        self.x[:3] = (d - (dx * c - dyc * s), u - (dx * s + dyc * c), wrap(psi + dpsi))
        # 对状态的雅可比（ψ 列与 s 列非对角）
        F = np.eye(n)
        F[0, 2] = dx * s + dyc * c
        F[1, 2] = -(dx * c - dyc * s)
        if self.estimate_scale:
            F[0, 3] = dy * s
            F[1, 3] = -dy * c
        # 对增量的雅可比，增量噪声 Σ_u = diag(σ_x², σ_y², σ_ψ²)
        G = np.zeros((n, 3))
        G[:3, :] = [[-c, k * s, 0.0],
                    [-s, -k * c, 0.0],
                    [0.0, 0.0, 1.0]]
        dp = math.hypot(dx, dy)
        st = self.alpha_trans * dp
        sr = self.alpha_rot * abs(dpsi) + self.alpha_cross * dp
        walk = list(self.q_walk ** 2)
        if self.estimate_scale:
            walk.append(self.scale_walk ** 2 * abs(dy))
        Q = G @ np.diag([st ** 2, st ** 2, sr ** 2]) @ G.T + np.diag(walk)
        self.P = F @ self.P @ F.T + Q

    def correct_increment(self, dy):
        """按当前 s 修正一段里程计横移（延迟补偿等只需要修正量、不需要协方差的场合）."""
        return (1.0 + self.x[3]) * dy if (self.estimate_scale and self.initialized) else dy

    # ------------------------------------------------------------------
    @staticmethod
    def h_wall(X):
        return np.array([X[0], -X[2]])

    @staticmethod
    def H_wall(X):
        return np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])

    @staticmethod
    def h_tag(X, u_tag=0.0):
        d, u, psi = X[:3]
        u = u - u_tag
        c, s = math.cos(psi), math.sin(psi)
        return np.array([d * c + u * s, -d * s + u * c])

    @staticmethod
    def H_tag(X, u_tag=0.0):
        d, u, psi = X[:3]
        u = u - u_tag
        c, s = math.cos(psi), math.sin(psi)
        return np.array([[c, s, -d * s + u * c],
                         [-s, c, -d * c - u * s]])

    def _update(self, kind, z, R, h, H, angle_rows=()):
        n = self.n
        Xs = self.x[:3] + self.offset
        y = np.asarray(z, dtype=float) - h(Xs)
        for i in angle_rows:
            y[i] = wrap(y[i])
        Hm = np.zeros((len(y), n))
        Hm[:, :3] = H(Xs)
        S = Hm @ self.P @ Hm.T + R
        nis = float(y @ np.linalg.solve(S, y))
        self.last_nis[kind] = nis
        if nis > self.gate:
            self.stats[kind][1] += 1
            return False, nis
        K = self.P @ Hm.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.x[2] = wrap(self.x[2])
        IKH = np.eye(n) - K @ Hm
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
        self.stats[kind][0] += 1
        return True, nis

    def update_wall(self, distance, phi, R):
        """激光墙面：distance（米）、φ = 墙面法向在 base_link 中的方位角（rad）."""
        return self._update('wall', (distance, phi), np.asarray(R, dtype=float),
                            self.h_wall, self.H_wall, angle_rows=(1,))

    def update_tag(self, p_base, R, u_tag=0.0, sigma_map=0.0):
        """
        标记中心在 base_link 水平面内的位置 (x, y)（米）及其 2×2 协方差.

        u_tag：该标记在墙面坐标中的已知位置（参考标记为 0）；sigma_map：其位置误差（米）。
        """
        R = np.asarray(R, dtype=float)
        if sigma_map > 0.0:
            hu = self.H_tag(self.x[:3] + self.offset, u_tag)[:, 1]
            R = R + sigma_map ** 2 * np.outer(hu, hu)
        return self._update('tag', p_base, R, lambda X: self.h_tag(X, u_tag),
                            lambda X: self.H_tag(X, u_tag))

    # ------------------------------------------------------------------
    @staticmethod
    def init_from(wall, p_tag=None, offset=np.zeros(3), u_tag=0.0):
        """
        由一次激光（与可选的标记）测量反解状态，用于初始化.

        wall = (distance, φ)；p_tag = 标记中心 (x, y)（base_link）；u_tag = 该标记在墙面坐标中
        的位置。返回 (d, u, ψ, 是否有 u)。
        """
        d, psi = wall[0], -wall[1]
        u = 0.0
        if p_tag is not None:
            c, s = math.cos(psi), math.sin(psi)
            # (d, u − u_tag) = R(ψ)·p_tag
            u = s * p_tag[0] + c * p_tag[1] + u_tag
        X = np.array([d, u, psi]) - offset
        return X[0], X[1], X[2], p_tag is not None
