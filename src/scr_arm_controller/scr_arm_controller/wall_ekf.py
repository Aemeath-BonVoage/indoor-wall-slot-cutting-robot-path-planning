"""
墙面相对状态的扩展卡尔曼滤波（EKF）——纯算法部分（不依赖 ROS）.

状态 X = (d, u, ψ)，均相对定位标记所在的墙面（与项目的墙面坐标 (u, v) 一致）：
  d  base_link 原点到墙面的垂直距离（米，> 0）
  u  base_link 原点沿墙的位置，以定位标记为 0，面向墙面时向右为正（米）
  ψ  底盘朝向相对“正对墙面”的偏角，逆时针为正（rad）

记 (n, l) 为“指向墙面 / 面向墙面时的左方”两个水平方向。底盘 x 轴在 (n, l) 中为
(cos ψ, sin ψ)，y 轴为 (−sin ψ, cos ψ)。

过程模型（里程计增量 (Δx, Δy, Δψ) = 两次里程计位姿之间的 SE(2) 相对变换，表示在
前一时刻的 base_link 中，因此用前一时刻的朝向即为精确合成，与增量大小无关）::

    d⁺ = d − (Δx cos ψ − Δy sin ψ)
    u⁺ = u − (Δx sin ψ + Δy cos ψ)
    ψ⁺ = ψ + Δψ

过程噪声按里程计运动模型（Thrun《概率机器人》5.4）随位移与转角增长，另加一项很小的
随机游走（允许墙面 / 标记参照缓慢修正，也防止长时间静止后协方差过小而拒收测量）。

测量模型（安装误差 / 零位标定偏置 c = (c_d, c_u, c_ψ) 在模型内部扣除：传感器“看到”的是
X + c，滤波器估计的是 X）::

    激光墙面   z_L = (d, φ),  φ = −ψ（墙面法向在 base_link 中的方位角）          线性
    标记中心   z_T = R(−ψ)·(d, u) = (d cos ψ + u sin ψ, −d sin ψ + u cos ψ)     非线性

每次更新先做马氏距离门限（χ² 检验，2 自由度 99 % 分位 9.21），超出则拒收（离群值：
激光拟合到障碍物、误检、遮挡时的残缺角点等）。协方差更新用 Joseph 形式保证数值对称正定。
"""
import math

import numpy as np

CHI2_2DOF_99 = 9.21


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class WallEKF:
    """(d, u, ψ) 三状态 EKF."""

    def __init__(self, alpha_trans=0.02, alpha_rot=0.02, alpha_cross=0.002,
                 q_walk=(1e-4, 1e-4, math.radians(0.01)), gate=CHI2_2DOF_99):
        """
        过程噪声与门限参数.

        alpha_trans：平移噪声 σ = alpha_trans·|Δp|（2 % 的里程计尺度误差）；
        alpha_rot：转角噪声 σ = alpha_rot·|Δψ|；alpha_cross：平移引起的转角噪声（rad/m）；
        q_walk：每次预测附加的随机游走标准差 (d, u, ψ)。
        """
        self.alpha_trans, self.alpha_rot, self.alpha_cross = alpha_trans, alpha_rot, alpha_cross
        self.q_walk = np.asarray(q_walk, dtype=float)
        self.gate = gate
        self.offset = np.zeros(3)          # 零位标定偏置 c（传感器看到 X + c）
        self.x = None
        self.P = None
        self.stats = {'wall': [0, 0], 'tag': [0, 0]}   # [接受, 拒收]
        self.last_nis = {'wall': float('nan'), 'tag': float('nan')}

    # ------------------------------------------------------------------
    @property
    def initialized(self):
        return self.x is not None

    def reset(self, d, u, psi, sigma=(0.01, 0.5, math.radians(2.0))):
        """以给定状态与标准差初始化（未见到标记时 u 取 0、σ_u 取大值）."""
        self.x = np.array([d, u, psi], dtype=float)
        self.P = np.diag(np.asarray(sigma, dtype=float) ** 2)

    def sigma(self):
        return np.sqrt(np.diag(self.P)) if self.P is not None else np.full(3, np.inf)

    # ------------------------------------------------------------------
    def predict(self, dx, dy, dpsi):
        """用里程计相对位姿 (Δx, Δy, Δψ)（前一时刻 base_link 中）预测."""
        d, u, psi = self.x
        c, s = math.cos(psi), math.sin(psi)
        self.x = np.array([d - (dx * c - dy * s), u - (dx * s + dy * c), wrap(psi + dpsi)])
        # 对状态的雅可比（只有 ψ 列非对角）
        F = np.eye(3)
        F[0, 2] = dx * s + dy * c
        F[1, 2] = -(dx * c - dy * s)
        # 对增量的雅可比，增量噪声 Σ_u = diag(σ_x², σ_y², σ_ψ²)
        G = np.array([[-c, s, 0.0],
                      [-s, -c, 0.0],
                      [0.0, 0.0, 1.0]])
        dp = math.hypot(dx, dy)
        st = self.alpha_trans * dp
        sr = self.alpha_rot * abs(dpsi) + self.alpha_cross * dp
        Q = G @ np.diag([st ** 2, st ** 2, sr ** 2]) @ G.T + np.diag(self.q_walk ** 2)
        self.P = F @ self.P @ F.T + Q

    # ------------------------------------------------------------------
    @staticmethod
    def h_wall(X):
        return np.array([X[0], -X[2]])

    @staticmethod
    def H_wall(X):
        return np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])

    @staticmethod
    def h_tag(X):
        d, u, psi = X
        c, s = math.cos(psi), math.sin(psi)
        return np.array([d * c + u * s, -d * s + u * c])

    @staticmethod
    def H_tag(X):
        d, u, psi = X
        c, s = math.cos(psi), math.sin(psi)
        return np.array([[c, s, -d * s + u * c],
                         [-s, c, -d * c - u * s]])

    def _update(self, kind, z, R, h, H, angle_rows=()):
        Xs = self.x + self.offset
        y = np.asarray(z, dtype=float) - h(Xs)
        for i in angle_rows:
            y[i] = wrap(y[i])
        Hm = H(Xs)
        S = Hm @ self.P @ Hm.T + R
        nis = float(y @ np.linalg.solve(S, y))
        self.last_nis[kind] = nis
        if nis > self.gate:
            self.stats[kind][1] += 1
            return False, nis
        K = self.P @ Hm.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.x[2] = wrap(self.x[2])
        IKH = np.eye(3) - K @ Hm
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
        self.stats[kind][0] += 1
        return True, nis

    def update_wall(self, distance, phi, R):
        """激光墙面：distance（米）、φ = 墙面法向在 base_link 中的方位角（rad）."""
        return self._update('wall', (distance, phi), np.asarray(R, dtype=float),
                            self.h_wall, self.H_wall, angle_rows=(1,))

    def update_tag(self, p_base, R):
        """标记中心在 base_link 水平面内的位置 (x, y)（米）及其 2×2 协方差."""
        return self._update('tag', p_base, np.asarray(R, dtype=float), self.h_tag, self.H_tag)

    # ------------------------------------------------------------------
    @staticmethod
    def init_from(wall, p_tag=None, offset=np.zeros(3)):
        """
        由一次激光（与可选的标记）测量反解状态，用于初始化.

        wall = (distance, φ)；p_tag = 标记中心 (x, y)（base_link）。返回 (d, u, ψ, 是否有 u)。
        """
        d, psi = wall[0], -wall[1]
        u = 0.0
        if p_tag is not None:
            c, s = math.cos(psi), math.sin(psi)
            # (d, u) = R(ψ)·p_tag
            u = s * p_tag[0] + c * p_tag[1]
        X = np.array([d, u, psi]) - offset
        return X[0], X[1], X[2], p_tag is not None
