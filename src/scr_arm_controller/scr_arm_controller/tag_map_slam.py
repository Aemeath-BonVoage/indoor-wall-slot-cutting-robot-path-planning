"""
标记地图在线估计：把非参考标记的沿墙位置并入墙面 EKF 的状态（纯算法部分，不依赖 ROS）.

状态 X = (d, u, ψ[, s], m_1 … m_K)：前四项与 WallEKF 相同，m_j 为第 j 个非参考标记在墙面
坐标中的沿墙位置。标记 i 的测量模型与 WallEKF 相同，只是标记位置由已知常数变为状态::

    z_i = R(−ψ)·(d, u − m_i)              ∂z_i/∂m_i = −∂z_i/∂u

可观测性（详见 docs/experiments/tag_map_estimation/README.md 第 2 节）：
  - 所有测量（标记、里程计增量、激光的 d 与 ψ）对“全体 u 与 m_i 同时平移”不变，这是唯一的
    规范自由度（gauge）。参考标记 m_0 ≡ 0 把它固定，所以参考标记不进入状态；
  - 两个标记同时在画面内时，z_i − z_j = R(−ψ)·(0, m_j − m_i) 直接给出间距，与里程计无关；
  - 只在不同时刻看到两个标记时，间距由“标记测量 + 里程计横移”给出，与横移比例 s 相关；
    s 本身由同一个标记在画面内的移动确定（WallEKF 文档），二者合起来可分离；
  - 相机的尺度误差（标记打印尺寸、焦距）使全部标记测量同比例缩放，与 s、m_i 一起构成近似的
    第二个规范自由度（只由深度方向的弱信息约束）。一段已知的标记间距可以把它固定。

为什么是 EKF-SLAM 而不是因子图：每面墙只有几个标记，状态维数 ≤ 10；墙面相对状态近似线性
（|ψ| < 5°，标记位置只有 1 维），线性高斯问题中滤波器在最后时刻给出的地图边缘分布与批量
最小二乘（平滑）相同，因子图只在重新线性化与离群值处理上占优。batch_wall_pass() 给出同一
问题的批量高斯–牛顿解，作为离线对照（单元测试中两者一致）。

跨对位保留：横移比例 s 与地图 m 是墙面与底盘的性质，WallEKF.clear() / reset() 只重置
(d, u, ψ)，(s, m) 的均值与协方差（含二者的互协方差）保留。第二遍作业可把第一遍的估计
（均值 + 协方差，TagMapEstimate.to_dict()）作为先验载入。
"""
import math

import numpy as np

from scr_arm_controller.tag_map import TagMap
from scr_arm_controller.wall_ekf import WallEKF, wrap


class TagMapEKF(WallEKF):
    """WallEKF + 非参考标记的沿墙位置（EKF-SLAM，1 维路标）."""

    def __init__(self, tag_map, prior_sigma=None, prior_cov=None, map_walk=0.0, map_floor=0.0,
                 reinit_first_sighting=True, **kw):
        """
        tag_map：TagMap，提供标记 ID、名义位置（先验均值）与参考标记.

        prior_sigma：先验标准差（米，标量或 {id: σ}）；默认取地图中每个标记的 sigma_position。
        prior_cov：完整的先验协方差（K×K，按 self.map_ids 的顺序；载入上一遍的估计时使用），
        给出时覆盖 prior_sigma。map_walk：每次预测附加给 m 的随机游走标准差（米；标记被碰动
        时允许缓慢修正，默认 0 = 标记固定）。map_floor：导出地图时加到每个标记方差上的系统误差
        标准差（米）。相机的系统误差（与方位角有关的横向偏差、深度偏差）对同一标记每帧相同，
        不随帧数平均掉；滤波器内部不建模它，只在 map_estimate() 中计入，使导出的协方差
        （下一遍的先验）与实际误差相符。reinit_first_sighting：一个标记首次被看到、而测量
        与先验不一致（被 χ² 门限拒收，例如实际粘贴误差远超先验 σ）时，按这次测量与当前底盘
        估计重新初始化该标记的位置（EKF-SLAM 中新路标的常规做法），而不是一直拒收；只用于
        尚未被接受过的非参考标记，载入上一遍的估计（prior_cov）时视为已确认。其余参数同 WallEKF。
        """
        super().__init__(**kw)
        self.reference_id = tag_map.reference_id
        self.map_ids = [i for i in tag_map.ids if i != tag_map.reference_id]
        self._col = {i: j for j, i in enumerate(self.map_ids)}
        K = len(self.map_ids)
        m0 = np.array([tag_map.u[i] for i in self.map_ids], dtype=float)
        if prior_cov is not None:
            Pm = np.array(prior_cov, dtype=float).reshape(K, K)
        else:
            if prior_sigma is None:
                sig = [tag_map.sigma(i) for i in self.map_ids]
            elif isinstance(prior_sigma, dict):
                sig = [float(prior_sigma.get(i, 0.0)) for i in self.map_ids]
            else:
                sig = [float(prior_sigma)] * K
            # σ = 0 的标记视为已测量：给一个极小的方差保持协方差正定
            Pm = np.diag([max(s, 1e-6) ** 2 for s in sig])
        self.map_walk = float(map_walk)
        self.map_floor = float(map_floor)
        self.template = tag_map
        # 跨对位保留的部分 (s?, m) 的均值与协方差
        if self.estimate_scale:
            mean = np.concatenate([[self._scale[0]], m0])
            cov = np.zeros((K + 1, K + 1))
            cov[0, 0] = self._scale[1]
            cov[1:, 1:] = Pm
        else:
            mean, cov = m0, Pm
        self._persist = (mean, cov)
        self.stats['tag_pair'] = 0      # 同一帧中两个以上标记同时更新的次数
        self.stats['tag_init'] = 0      # 首次看到时按测量重新初始化的次数
        self.reinit_first_sighting = bool(reinit_first_sighting)
        self._confirmed = set(self.map_ids) if prior_cov is not None else set()

    # ------------------------------------------------------------------
    @property
    def n_robot(self):
        return 4 if self.estimate_scale else 3

    @property
    def n(self):
        return self.n_robot + len(self.map_ids)

    def _m0(self):
        return self.n_robot

    def _save_persist(self):
        if self.initialized:
            k = 3
            self._persist = (self.x[k:].copy(), self.P[k:, k:].copy())
            if self.estimate_scale:
                self._scale = (float(self.x[3]), float(self.P[3, 3]))

    def _save_scale(self):
        self._save_persist()

    def reset(self, d, u, psi, sigma=(0.01, 0.5, math.radians(2.0))):
        """以给定 (d, u, ψ) 与标准差初始化；(s, m) 沿用之前的估计（含互协方差）."""
        self._save_persist()
        mean, cov = self._persist
        n = 3 + len(mean)
        self.x = np.concatenate([[d, u, psi], mean]).astype(float)
        self.P = np.zeros((n, n))
        self.P[:3, :3] = np.diag(np.asarray(sigma, dtype=float) ** 2)
        self.P[3:, 3:] = cov

    def clear(self):
        self._save_persist()
        self.x = None
        self.P = None

    # ------------------------------------------------------------------
    def tag_u(self, tag_id):
        """标记 tag_id 当前的沿墙位置估计（参考标记为 0；不在地图中返回 None）."""
        if tag_id == self.reference_id:
            return 0.0
        j = self._col.get(int(tag_id))
        if j is None:
            return None
        src = self.x if self.initialized else np.concatenate([np.zeros(3), self._persist[0]])
        return float(src[self._m0() + j])

    def tag_sigma(self, tag_id):
        if tag_id == self.reference_id:
            return 0.0
        j = self._col[int(tag_id)]
        if self.initialized:
            return math.sqrt(max(self.P[self._m0() + j, self._m0() + j], 0.0))
        k = j + (1 if self.estimate_scale else 0)
        return math.sqrt(max(self._persist[1][k, k], 0.0))

    def map_estimate(self):
        """当前的地图估计：TagMapEstimate（ID、均值、协方差）."""
        if self.initialized:
            k = self._m0()
            mean, cov = self.x[k:].copy(), self.P[k:, k:].copy()
        else:
            k = 1 if self.estimate_scale else 0
            mean, cov = self._persist[0][k:].copy(), self._persist[1][k:, k:].copy()
        cov = cov + np.eye(len(mean)) * self.map_floor ** 2
        return TagMapEstimate(self.template, self.map_ids, mean, cov)

    # ------------------------------------------------------------------
    def predict(self, dx, dy, dpsi):
        n, nr = self.n, self.n_robot
        d, u, psi = self.x[:3]
        k = 1.0 + self.x[3] if self.estimate_scale else 1.0
        dyc = k * dy
        c, s = math.cos(psi), math.sin(psi)
        self.x[:3] = (d - (dx * c - dyc * s), u - (dx * s + dyc * c), wrap(psi + dpsi))
        # 只有机器人块与 s 列非单位阵：按块传播，地图块不动
        F = np.eye(nr)
        F[0, 2] = dx * s + dyc * c
        F[1, 2] = -(dx * c - dyc * s)
        if self.estimate_scale:
            F[0, 3] = dy * s
            F[1, 3] = -dy * c
        G = np.zeros((nr, 3))
        G[:3, :] = [[-c, k * s, 0.0], [-s, -k * c, 0.0], [0.0, 0.0, 1.0]]
        dp = math.hypot(dx, dy)
        st = self.alpha_trans * dp
        sr = self.alpha_rot * abs(dpsi) + self.alpha_cross * dp
        walk = list(self.q_walk ** 2)
        if self.estimate_scale:
            walk.append(self.scale_walk ** 2 * abs(dy))
        Q = G @ np.diag([st ** 2, st ** 2, sr ** 2]) @ G.T + np.diag(walk)
        P = self.P
        Prr = F @ P[:nr, :nr] @ F.T + Q
        Prm = F @ P[:nr, nr:]
        P[:nr, :nr] = Prr
        P[:nr, nr:] = Prm
        P[nr:, :nr] = Prm.T
        if self.map_walk > 0.0:
            P[nr:, nr:] += np.eye(n - nr) * self.map_walk ** 2

    def update_tag_id(self, p_base, R, tag_id):
        """
        标记 tag_id 中心在 base_link 中的位置测量（与 WallEKF.update_tag 相同的 z、R）.

        参考标记：m = 0，不改变地图列；其他地图内标记：m 为状态。返回 (是否接受, NIS)。
        不在地图中的 ID 返回 (False, nan)。
        """
        tag_id = int(tag_id)
        if tag_id != self.reference_id and tag_id not in self._col:
            return False, math.nan
        n = self.n
        Xs = self.x[:3] + self.offset
        u_tag = self.tag_u(tag_id)
        z_hat = self.h_tag(Xs, u_tag)
        Hr = self.H_tag(Xs, u_tag)
        H = np.zeros((2, n))
        H[:, :3] = Hr
        if tag_id != self.reference_id:
            H[:, self._m0() + self._col[tag_id]] = -Hr[:, 1]
        p = np.asarray(p_base, dtype=float)
        R = np.asarray(R, dtype=float)
        ok, nis = self._update_full('tag', p - z_hat, R, H)
        if tag_id == self.reference_id:
            return ok, nis
        if ok:
            self._confirmed.add(tag_id)
        elif self.reinit_first_sighting and tag_id not in self._confirmed:
            self._reinit_tag(tag_id, p, R)
            self.stats['tag'][1] -= 1          # 这一帧不计为拒收
            ok, nis = self.update_tag_id(p, R, tag_id)
        return ok, nis

    def _reinit_tag(self, tag_id, p, R):
        """
        由一次测量与当前底盘估计重新初始化标记位置：m = (u + c_u) − (sin ψ'·p_x + cos ψ'·p_y).

        协方差按 m 对 (u, ψ) 与测量的一阶传播构造，与机器人状态、其他标记的相关性随之建立。
        """
        j = self._m0() + self._col[tag_id]
        psi = self.x[2] + self.offset[2]
        c, s = math.cos(psi), math.sin(psi)
        self.x[j] = self.x[1] + self.offset[1] - (s * p[0] + c * p[1])
        J = np.zeros(self.n)
        J[1] = 1.0
        J[2] = -(c * p[0] - s * p[1])
        g = np.array([s, c])
        P = self.P
        J[j] = 0.0
        row = J @ P
        P[j, :] = row
        P[:, j] = row
        P[j, j] = float(J @ P @ J) + float(g @ R @ g)
        self._confirmed.add(tag_id)
        self.stats['tag_init'] += 1

    def update_tags(self, obs):
        """
        同一帧的多个标记 [(tag_id, p_base, R), ...] 作为一个联合测量更新.

        逐个顺序更新与联合更新在线性情形下等价；联合更新让门限检验整帧一致性。
        返回被接受的 ID 列表（整帧接受或拒收；只有一个标记时等价于 update_tag_id）。
        """
        obs = [o for o in obs if int(o[0]) == self.reference_id or int(o[0]) in self._col]
        if not obs:
            return []
        if len(obs) == 1:
            ok, _ = self.update_tag_id(obs[0][1], obs[0][2], obs[0][0])
            return [int(obs[0][0])] if ok else []
        n = self.n
        Xs = self.x[:3] + self.offset
        ys, Hs, Rs = [], [], []
        for tag_id, p, R in obs:
            tag_id = int(tag_id)
            u_tag = self.tag_u(tag_id)
            Hr = self.H_tag(Xs, u_tag)
            H = np.zeros((2, n))
            H[:, :3] = Hr
            if tag_id != self.reference_id:
                H[:, self._m0() + self._col[tag_id]] = -Hr[:, 1]
            ys.append(np.asarray(p, dtype=float) - self.h_tag(Xs, u_tag))
            Hs.append(H)
            Rs.append(np.asarray(R, dtype=float))
        y = np.concatenate(ys)
        H = np.vstack(Hs)
        Rb = np.zeros((len(y), len(y)))
        for k, R in enumerate(Rs):
            Rb[2 * k:2 * k + 2, 2 * k:2 * k + 2] = R
        gate = _chi2_99(len(y))
        ok, _ = self._update_full('tag', y, Rb, H, gate=gate)
        if ok:
            self.stats['tag_pair'] += 1
        return [int(o[0]) for o in obs] if ok else []

    def _update_full(self, kind, y, R, H, gate=None):
        S = H @ self.P @ H.T + R
        nis = float(y @ np.linalg.solve(S, y))
        self.last_nis[kind] = nis
        if nis > (self.gate if gate is None else gate):
            self.stats[kind][1] += 1
            return False, nis
        K = np.linalg.solve(S, H @ self.P).T
        self.x = self.x + K @ y
        self.x[2] = wrap(self.x[2])
        IKH = np.eye(self.n) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
        self.stats[kind][0] += 1
        return True, nis

    def update_wall(self, distance, phi, R):
        Hr = self.H_wall(None)
        H = np.zeros((2, self.n))
        H[:, :3] = Hr
        y = np.array([distance, phi], dtype=float) - self.h_wall(self.x[:3] + self.offset)
        y[1] = wrap(y[1])
        return self._update_full('wall', y, np.asarray(R, dtype=float), H)

    def update_tag(self, p_base, R, u_tag=0.0, sigma_map=0.0):
        """兼容 WallEKF 接口：按 u_tag 找到对应的地图标记（只用于单元测试与对照）."""
        for i in [self.reference_id] + self.map_ids:
            if abs(self.template.u[i] - u_tag) < 1e-9:
                return self.update_tag_id(p_base, R, i)
        return super().update_tag(p_base, R, u_tag, sigma_map)


def _chi2_99(dof):
    """χ² 分布 99 % 分位（Wilson–Hilferty 近似，dof ≥ 2 时误差 < 1 %）."""
    if dof == 2:
        return 9.21
    z = 2.3263
    a = 2.0 / (9.0 * dof)
    return dof * (1.0 - a + z * math.sqrt(a)) ** 3


class TagMapEstimate:
    """估计出的标记地图：均值 + 协方差，可写成带协方差的地图 YAML，供下一遍作为先验."""

    def __init__(self, template, ids, mean, cov):
        self.template = template
        self.ids = list(ids)
        self.mean = np.asarray(mean, dtype=float)
        self.cov = np.asarray(cov, dtype=float)

    def sigma(self):
        return np.sqrt(np.maximum(np.diag(self.cov), 0.0))

    def tag_map(self):
        """TagMap（per-tag σ 取边缘标准差；互协方差丢失，只给不估计地图的 WallEKF 用）."""
        t = self.template
        tags = {t.reference_id: (0.0, t.v[t.reference_id])}
        sig = {t.reference_id: 0.0}
        for i, m, s in zip(self.ids, self.mean, self.sigma()):
            tags[i] = (float(m), t.v[i])
            sig[i] = float(s)
        return TagMap(tags, reference_id=t.reference_id, family=t.family, size=t.size,
                      sigma_position=sig)

    def to_dict(self):
        d = self.tag_map().to_dict()
        d['tag_map']['covariance'] = {
            'ids': [int(i) for i in self.ids],
            'matrix': [[float(f'{v:.6e}') for v in row] for row in self.cov]}
        return d

    @staticmethod
    def prior_from_dict(data):
        """从 to_dict() 的结果恢复 (TagMap, prior_cov 或 None)."""
        m = TagMap.from_dict(data)
        c = data.get('tag_map', data).get('covariance')
        if not c:
            return m, None
        ids = [int(i) for i in c['ids']]
        mids = [i for i in m.ids if i != m.reference_id]
        cov = np.array(c['matrix'], dtype=float)
        if sorted(ids) != sorted(mids) or cov.shape != (len(ids), len(ids)):
            raise ValueError('地图协方差与标记 ID 不一致')
        perm = [ids.index(i) for i in mids]
        return m, cov[np.ix_(perm, perm)]


# ----------------------------------------------------------------------
# 批量最小二乘（离线对照）
# ----------------------------------------------------------------------
def batch_wall_pass(epochs, tag_map, prior_sigma, x0, P0, estimate_scale=False,
                    scale_init=0.0, scale_sigma=0.05, ekf_kw=None, iterations=5):
    """
    一遍作业的批量高斯–牛顿解（稠密，适合几百个历元的离线对照）.

    epochs：[{'odom': (dx, dy, dψ) 或 None（第一个历元）, 'wall': (z(2), R) 或 None,
              'tags': [(id, p(2), R), ...]}, ...]；x0、P0：第一个历元 (d, u, ψ) 的先验。
    过程噪声与 WallEKF.predict 相同（由 ekf_kw 给出同样的参数）。
    未知量：每个历元的 (d, u, ψ)、可选的 s、非参考标记的 m。返回 dict(X, s, m, cov_m, ids)。
    """
    ekf_kw = dict(ekf_kw or {})
    proto = WallEKF(estimate_scale=False, **ekf_kw)
    ids = [i for i in tag_map.ids if i != tag_map.reference_id]
    col = {i: j for j, i in enumerate(ids)}
    Kn = len(epochs)
    ns = 1 if estimate_scale else 0
    N = 3 * Kn + ns + len(ids)
    i_s = 3 * Kn
    i_m = 3 * Kn + ns
    sig = (prior_sigma if isinstance(prior_sigma, dict)
           else {i: prior_sigma for i in ids})

    # 初值：按里程计积分 + 名义地图
    theta = np.zeros(N)
    X = np.array(x0, dtype=float)
    for k, e in enumerate(epochs):
        if k > 0:
            f = WallEKF()
            f.reset(*X)
            f.predict(*e['odom'])
            X = f.x[:3]
        theta[3 * k:3 * k + 3] = X
    if estimate_scale:
        theta[i_s] = scale_init
    theta[i_m:] = [tag_map.u[i] for i in ids]
    m_prior = theta[i_m:].copy()

    def residuals(th):
        rows, Js = [], []

        def add(r, J, Rinv_sqrt):
            rows.append(Rinv_sqrt @ r)
            Js.append(Rinv_sqrt @ J)

        # 先验
        L0 = np.linalg.cholesky(np.linalg.inv(P0))
        J = np.zeros((3, N))
        J[:, 0:3] = np.eye(3)
        add(th[0:3] - x0, J, L0.T)
        if estimate_scale:
            J = np.zeros((1, N))
            J[0, i_s] = 1.0
            add(np.array([th[i_s] - scale_init]), J, np.array([[1.0 / scale_sigma]]))
        for i in ids:
            J = np.zeros((1, N))
            J[0, i_m + col[i]] = 1.0
            add(np.array([th[i_m + col[i]] - m_prior[col[i]]]), J,
                np.array([[1.0 / max(sig.get(i, 0.0), 1e-6)]]))
        for k, e in enumerate(epochs):
            Xk = th[3 * k:3 * k + 3]
            if k > 0:
                dx, dy, dpsi = e['odom']
                Xp = th[3 * (k - 1):3 * k]
                sc = th[i_s] if estimate_scale else 0.0
                f = WallEKF(estimate_scale=estimate_scale, scale_init=sc, scale_sigma=1.0,
                            **ekf_kw)
                f.reset(*Xp, sigma=(1.0, 1.0, 1.0))
                f.P = np.zeros((f.n, f.n))
                f.predict(dx, dy, dpsi)
                Q = f.P[:3, :3] + 1e-14 * np.eye(3)
                d_, u_, psi_ = Xp
                kk = 1.0 + sc
                c, s_ = math.cos(psi_), math.sin(psi_)
                Fx = np.eye(3)
                Fx[0, 2] = dx * s_ + kk * dy * c
                Fx[1, 2] = -(dx * c - kk * dy * s_)
                r = Xk - f.x[:3]
                r[2] = wrap(r[2])
                J = np.zeros((3, N))
                J[:, 3 * k:3 * k + 3] = np.eye(3)
                J[:, 3 * (k - 1):3 * k] = -Fx
                if estimate_scale:
                    J[:, i_s] = -np.array([dy * s_, -dy * c, 0.0])
                add(r, J, np.linalg.cholesky(np.linalg.inv(Q)).T)
            if e.get('wall') is not None:
                z, R = e['wall']
                r = proto.h_wall(Xk) - np.asarray(z)
                r[1] = wrap(r[1])
                J = np.zeros((2, N))
                J[:, 3 * k:3 * k + 3] = proto.H_wall(Xk)
                add(r, J, np.linalg.cholesky(np.linalg.inv(R)).T)
            for tag_id, p, R in e.get('tags', ()):
                tag_id = int(tag_id)
                if tag_id != tag_map.reference_id and tag_id not in col:
                    continue
                ut = 0.0 if tag_id == tag_map.reference_id else th[i_m + col[tag_id]]
                r = proto.h_tag(Xk, ut) - np.asarray(p)
                Hr = proto.H_tag(Xk, ut)
                J = np.zeros((2, N))
                J[:, 3 * k:3 * k + 3] = Hr
                if tag_id != tag_map.reference_id:
                    J[:, i_m + col[tag_id]] = -Hr[:, 1]
                add(r, J, np.linalg.cholesky(np.linalg.inv(R)).T)
        return np.concatenate(rows), np.vstack(Js)

    for _ in range(iterations):
        r, J = residuals(theta)
        A = J.T @ J
        step = np.linalg.solve(A, -J.T @ r)
        theta = theta + step
        if np.max(np.abs(step)) < 1e-9:
            break
    r, J = residuals(theta)
    cov = np.linalg.inv(J.T @ J)
    return {'X': theta[:3 * Kn].reshape(Kn, 3),
            's': float(theta[i_s]) if estimate_scale else 0.0,
            'm': theta[i_m:].copy(), 'cov_m': cov[i_m:, i_m:].copy(), 'ids': ids}
