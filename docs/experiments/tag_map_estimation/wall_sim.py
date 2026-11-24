"""
离线测量仿真器：沿录制的 11 站真值轨迹合成里程计、激光与标记测量，驱动墙面估计器.

轨迹与时间：取 default_s1 / default_s2 两次 Gazebo 运行中估计器每个周期的时刻（约 10 Hz，
只在精对位、站间横移与到站残差测量阶段运行；切割期间不运行，下一周期的里程计增量跨过
整个切割）与该时刻的底盘真值。闭环的影响（底盘停在估计认为的位置）只有毫米级，对测量
合成可以忽略，所以同一条真值轨迹可以被不同的估计器复用。

噪声模型取自 noise_model.yaml（由 noise_model.py 从录制数据拟合；假设项单独注明）：
  里程计横移：dy_odom = dy_true / (1 + s_run + δ_seg)，s_run 每次试验抽一次，δ_seg 每段横移
            抽一次；朝向：dψ_odom = dψ_true + κ·dy_true + ε_seg（按横移量分摊）；
            切割期间的定点保持另加沿墙 / 朝向白噪声；精对位阶段的转向按编码器模型的
            yaw_slip = 2 % 放大（假设，取自 URDF 参数）。
  激光：d、ψ 各为白噪声 + 每个停靠站重新抽取的偏差。
  标记（base_link 中的标记中心）：横向 = a + b·方位角 + 噪声（静止：拟合的高斯；横移中：
        按门限接受率标定的高斯，见 calibrate_tag_noise.py），深度 = 偏差 + 高斯；可见范围为
        拟合的 u − u_i 窗口按离墙距离缩放。
估计器使用的测量协方差与 ROS 节点相同：激光取 fit_wall 协方差的中位数，标记按
fiducial.tag_position_covariance 的下限（横向 0.3 mm、深度 5 mm）沿视线方向构造。

站点误差：每站到站残差测量阶段最后一个周期的 û − u（估计 − 真值），对应 Gazebo 报告中
“补偿后刀尖沿墙误差”（到站真值误差减去补偿所用的传感器残差，二者都由估计误差决定）。
"""
import math
import os

import numpy as np
import yaml

from common import HERE, load_bag, relative_pose, to_wall, wrap
from scr_arm_controller.tag_map import TagMap
from scr_arm_controller.tag_map_slam import TagMapEKF
from scr_arm_controller.wall_ekf import WallEKF

R_TAG_LAT = 3e-4         # fiducial.tag_position_covariance 的横向下限（米）
R_TAG_DEP = 5e-3         # 深度下限（米）
YAW_SLIP = 0.02          # 编码器模型的转向打滑（URDF 参数；假设误差全部体现在里程计中）
_SCN_CACHE = {}


def load_model(path=os.path.join(HERE, 'noise_model.yaml'), calibrated=True):
    """
    噪声模型参数.

    calibrated：横移中的标记横向噪声用 calibrate_tag_noise.py 标定的高斯标准差（不用拟合的
    混合分布，时间对齐项 τ = 0）；False 时用拟合值（只在标定脚本中使用）。
    """
    with open(path) as f:
        m = yaml.safe_load(f)
    o, l, t = m['odometry'], m['laser'], m['tag']
    out = {
        's_mean': o['lateral_scale_mean']['value'],
        's_run': o['lateral_scale_run_sigma']['value'],
        's_seg': o['lateral_scale_segment_sigma']['value'],
        'kappa': o['heading_per_strafe_odom']['value'],
        'h_seg': o['heading_per_strafe_odom']['residual_sigma_rad'],
        'hold_u': o['hold_drift']['lateral_sigma_m'],
        'hold_psi': o['hold_drift']['heading_sigma_rad'],
        'l_d': l['distance_sigma_m']['value'],
        'l_psi': l['yaw_within_station_sigma_rad']['value'],
        'l_d_bias': l['distance_station_bias_sigma_m']['value'],
        'l_psi_bias': l['yaw_station_bias_sigma_rad']['value'],
        'R_wall': np.diag([l['covariance_used_by_estimator']['var_d_median'],
                           l['covariance_used_by_estimator']['var_phi_median']]),
        't_a': t['lateral_bias_m']['value'],
        't_b': t['lateral_bias_per_rad_bearing']['value'],
        't_static': t['lateral_sigma_static_m']['value'],
        't_tau': t['time_offset_s']['value'],
        't_core': t['lateral_sigma_moving_m']['value'],
        't_tail_w': t['moving_outlier_fraction']['value'],
        't_tail': t['moving_outlier_fraction']['sigma_m'],
        't_dep_bias': t['depth_bias_m']['value'],
        't_dep': t['depth_sigma_m']['value'],
        'vis_lo': m['visibility']['rel_u_min_m'],
        'vis_hi': m['visibility']['rel_u_max_m'],
        'vis_d': m['visibility']['at_distance_m'],
    }
    cal = t.get('lateral_sigma_moving_calibrated_m')
    if calibrated and cal is not None:
        out.update(t_core=cal['value'], t_tail_w=0.0, t_tau=0.0)
    return out


def load_scenario(name):
    """估计器周期的时刻、真值 (d, u, ψ)、阶段，以及相邻周期之间的真值增量."""
    if name in _SCN_CACHE:
        return _SCN_CACHE[name]
    b = load_bag(name)
    t = np.array([float(r['t']) for r in b['estimate']])
    pose = [b['truth'].at(tk) for tk in t]
    d, u, psi = to_wall(*np.array(pose).T)
    st_ph = []
    for tk in t:
        st, ph = -1, 'idle'
        for t0, t1, s_, p_ in b['phases']:
            if t0 <= tk < t1 + 1e-6:
                st, ph = s_, p_
        st_ph.append((st, ph))
    inc = np.zeros((len(t), 3))
    seg = np.full(len(t), -1, dtype=int)          # 增量所属的站间横移编号（−1 = 非横移）
    gap = np.zeros(len(t), dtype=bool)            # 增量跨过切割（定点保持）
    moves = [(t0, t1) for t0, t1, s_, p_ in b['phases'] if p_ == 'move' and s_ == -1]
    for k in range(1, len(t)):
        inc[k] = relative_pose(pose[k - 1], pose[k])
        mid = 0.5 * (t[k - 1] + t[k])
        for j, (t0, t1) in enumerate(moves):
            if t0 - 0.05 <= mid <= t1 + 0.05:
                seg[k] = j
        gap[k] = t[k] - t[k - 1] > 0.5
    # 每站到站残差测量的最后一个周期
    station_end = {}
    for k, (st, ph) in enumerate(st_ph):
        if ph == 'arrived' and st >= 0:
            station_end[st] = k
    align = np.array([ph == 'align' for _, ph in st_ph])
    _, u_a, _ = to_wall(*b['truth'].at(t + 0.05))
    _, u_b, _ = to_wall(*b['truth'].at(t - 0.05))
    v = (u_a - u_b) / 0.1                         # 沿墙真值速度（估计器时间戳只有 0.1 s 分辨率）
    scn = dict(name=name, t=t, d=d, u=u, psi=psi, inc=inc, seg=seg, gap=gap, v=v,
               n_seg=len(moves), align=align,
               stations=[station_end[k] for k in sorted(station_end)],
               station_ids=sorted(station_end))
    _SCN_CACHE[name] = scn
    return scn


class Trial:
    """一次试验的全部随机量（同一试验的不同估计方法共用，配对比较）."""

    def __init__(self, scn, model, rng, true_tags, overrides=None):
        m = dict(model)
        m.update(overrides or {})
        self.m = m
        n = len(scn['t'])
        self.s_true = m['s_mean'] + m['s_run'] * rng.standard_normal()
        seg_s = m['s_seg'] * rng.standard_normal(scn['n_seg'])
        seg_h = m['h_seg'] * rng.standard_normal(scn['n_seg'])
        # 每段横移的朝向误差按横移量分摊
        seg_len = np.zeros(scn['n_seg'])
        for k in range(n):
            if scn['seg'][k] >= 0:
                seg_len[scn['seg'][k]] += abs(scn['inc'][k, 1])
        odom = scn['inc'].copy()
        for k in range(1, n):
            dx, dy, dpsi = scn['inc'][k]
            j = scn['seg'][k]
            sk = self.s_true + (seg_s[j] if j >= 0 else 0.0)
            odom[k, 1] = dy / (1.0 + sk)
            odom[k, 2] = dpsi * (1.0 + (YAW_SLIP if scn['align'][k] else 0.0)) \
                + m['kappa'] * dy
            if j >= 0 and seg_len[j] > 0:
                odom[k, 2] += seg_h[j] * abs(dy) / seg_len[j]
            if scn['gap'][k]:
                odom[k, 1] += m['hold_u'] * rng.standard_normal()
                odom[k, 2] += m['hold_psi'] * rng.standard_normal()
        self.odom = odom
        # 激光：白噪声 + 每段（两次横移之间）重新抽取的偏差
        block = np.zeros(n, dtype=int)
        b = 0
        for k in range(n):
            if k > 0 and scn['seg'][k] >= 0 and scn['seg'][k - 1] < 0:
                b += 1
            block[k] = b
        nb = block.max() + 1
        bd = m['l_d_bias'] * rng.standard_normal(nb)
        bp = m['l_psi_bias'] * rng.standard_normal(nb)
        self.wall = np.column_stack([
            scn['d'] + bd[block] + m['l_d'] * rng.standard_normal(n),
            -scn['psi'] - bp[block] + m['l_psi'] * rng.standard_normal(n)])
        # 标记：每个周期、每个标记的测量（不可见时为 nan）
        self.tags = {}
        moving = np.abs(scn['v']) > 0.01
        for i, ut in true_tags.items():
            z = np.full((n, 2), np.nan)
            rel = scn['u'] - ut
            scale = scn['d'] / m['vis_d']
            vis = (rel >= m['vis_lo'] * scale) & (rel <= m['vis_hi'] * scale)
            tail = rng.random(n) < m['t_tail_w']
            lat_noise = np.where(moving, np.where(tail, m['t_tail'], m['t_core']),
                                 m['t_static']) * rng.standard_normal(n)
            dep_noise = m['t_dep'] * rng.standard_normal(n)
            for k in np.where(vis)[0]:
                h = WallEKF.h_tag((scn['d'][k], scn['u'][k], scn['psi'][k]), ut)
                brg = math.atan2(h[1], h[0])
                z[k, 0] = h[0] + m['t_dep_bias'] + dep_noise[k]
                z[k, 1] = (h[1] + m['t_a'] + m['t_b'] * brg + m['t_tau'] * scn['v'][k]
                           + lat_noise[k])
            self.tags[i] = z


def tag_R(p):
    """与 ROS 节点相同的标记测量协方差（横向 / 深度下限，沿视线方向）."""
    los = np.asarray(p) / np.linalg.norm(p)
    return R_TAG_LAT ** 2 * (np.eye(2) - np.outer(los, los)) + R_TAG_DEP ** 2 * np.outer(los, los)


def run_pass(scn, trial, est_map, method, estimate_scale, sigma_map=0.002,
             prior_sigma=0.005, prior=None, ekf_kw=None, record=False, map_floor=0.0):
    """
    一遍作业.

    est_map：估计器使用的 TagMap（名义 / 实测 / 先验）；method：'known'（WallEKF，地图位置
    为常数，σ_map 计入测量噪声）或 'slam'（TagMapEKF）；prior：(TagMap, cov) 用上一遍的估计
    作先验（method = 'slam'）。返回 dict：站点误差、σ_u、地图估计等。
    """
    ekf_kw = dict(ekf_kw or {})
    if method == 'slam':
        if prior is not None:
            f = TagMapEKF(prior[0], prior_cov=prior[1], estimate_scale=estimate_scale,
                          map_floor=map_floor, **ekf_kw)
        else:
            f = TagMapEKF(est_map, prior_sigma=prior_sigma, estimate_scale=estimate_scale,
                          map_floor=map_floor, **ekf_kw)
        u_of = f.tag_u
    else:
        f = WallEKF(estimate_scale=estimate_scale, **ekf_kw)
        u_of = est_map.u.get

    def sig_of(i):
        return 0.0 if i == est_map.reference_id else sigma_map

    n = len(scn['t'])
    u_known = False
    m = trial.m
    R_wall = m['R_wall']
    st_set = {k: j for j, k in enumerate(scn['stations'])}
    moving = np.abs(scn['v']) > 0.01
    n_mov = [0, 0]                               # 横移中的标记更新 [接受, 总数]
    reject_streak = 0
    err = np.full(len(scn['stations']), np.nan)
    sig = np.full(len(scn['stations']), np.nan)
    trace = [] if record else None
    for k in range(n):
        if f.initialized and k > 0:
            f.predict(*trial.odom[k])
        zw = trial.wall[k]
        if not f.initialized:
            f.reset(zw[0], 0.0, -zw[1])
        else:
            f.update_wall(zw[0], zw[1], R_wall)
        obs = [(i, z[k]) for i, z in trial.tags.items()
               if not np.isnan(z[k, 0]) and (i == est_map.reference_id or i in est_map.u)]
        if obs:
            if not u_known:
                i0, p0 = min(obs, key=lambda o: abs(o[1][1]))
                _, u0, _, _ = WallEKF.init_from((f.x[0], -f.x[2]), p0, u_tag=u_of(i0))
                f.x[1] = u0
                f.P[1, :] = 0.0
                f.P[:, 1] = 0.0
                f.P[1, 1] = 0.01 ** 2
                u_known = True
            n_acc = 0
            for i, p in obs:
                R = tag_R(p)
                if method == 'slam':
                    ok, _ = f.update_tag_id(p, R, i)
                else:
                    ok, _ = f.update_tag(p, R, u_tag=est_map.u[i], sigma_map=sig_of(i))
                n_acc += int(ok)
                if moving[k]:
                    n_mov[0] += int(ok)
                    n_mov[1] += 1
            # 与 WallStateEstimator 相同：连续 15 帧整帧拒收时由标记重新初始化 u
            reject_streak = 0 if n_acc else reject_streak + 1
            if reject_streak >= 15:
                u_known, reject_streak = False, 0
        if k in st_set:
            j = st_set[k]
            err[j] = f.x[1] - scn['u'][k]
            sig[j] = math.sqrt(f.P[1, 1])
        if record:
            row = [scn['t'][k], f.x[1] - scn['u'][k], math.sqrt(f.P[1, 1]),
                   f.x[2] - scn['psi'][k], math.sqrt(f.P[2, 2]), f.scale[0], f.scale[1]]
            if method == 'slam':
                row += [v for i in f.map_ids for v in (f.tag_u(i), f.tag_sigma(i))]
            trace.append(row)
    out = {'err': err, 'sigma': sig, 'scale': f.scale, 'stats': f.stats,
           'acc_moving': n_mov[0] / max(n_mov[1], 1)}
    if method == 'slam':
        est = f.map_estimate()
        out['map'] = est
    if record:
        out['trace'] = np.array(trace)
    return out


def layouts():
    """标记布局（设计位置，相对参考标记）：1 / 2 / 3 / 6 个标记."""
    return {
        1: {0: 0.0},
        2: {0: 0.0, 4: 1.2},
        3: {0: 0.0, 2: 0.6, 4: 1.2},
        6: {5: -0.3, 0: 0.0, 1: 0.3, 2: 0.6, 3: 0.9, 4: 1.2},
    }


def make_map(tags, sigma=0.002):
    return TagMap(dict(tags), reference_id=0, sigma_position=sigma)
