#!/usr/bin/env python3
"""
从已录制的数据拟合离线测量仿真器的噪声模型.

  python3 docs/experiments/tag_map_estimation/noise_model.py

数据：bag_data/（4 个数据包：default_s1/s2、single_off_s1/s2，extract_bags.py 的输出）与
along_wall_accuracy/runs/*/ekf_trace.csv（31 次运行，估计器每周期的标记测量与真值）。

输出：
  noise_model.yaml        仿真器使用的参数（每项注明“拟合 / 假设”与样本量）
  noise_segments.csv      每段站间横移：里程计与真值的横移、朝向变化、陀螺积分
  noise_tag_residuals.csv 标记测量残差按方位角与横移速度分组的统计
"""
import csv
import glob
import math
import os

import numpy as np
import yaml

from common import (AW_RUNS, BAG_DATA, HERE, load_bag, parse_tags, parse_trace_tags,
                    phase_at, read_csv, relative_pose, to_wall, WORLD_TAG_U, wrap)

BAGS = ('default_s1', 'default_s2', 'single_off_s1', 'single_off_s2')


def strafe_segments(b, name):
    """站间横移（phase = move 且 station = −1）的里程计与真值增量."""
    out = []
    gyro = b['gyro']
    for t0, t1, st, ph in b['phases']:
        if not (ph == 'move' and st == -1):
            continue
        po = b['odom'].at(t0), b['odom'].at(t1)
        pt = b['truth'].at(t0), b['truth'].at(t1)
        dxo, dyo, dpo = relative_pose(*po)
        dxt, dyt, dpt = relative_pose(*pt)
        sel = (gyro[:, 0] >= t0) & (gyro[:, 0] < t1)
        dpg = float(np.sum(gyro[sel, 1]) * 0.02)      # 50 Hz
        out.append(dict(bag=name, t0=t0, t1=t1, dx_odom=dxo, dy_odom=dyo, dpsi_odom=dpo,
                        dx_true=dxt, dy_true=dyt, dpsi_true=dpt, dpsi_gyro=dpg))
    return out


def hold_drift(b):
    """两次横移之间（到站 → 切割 → 回观测位姿）里程计与真值的沿墙位移之差."""
    out = []
    moves = [(t0, t1) for t0, t1, st, ph in b['phases'] if ph == 'move' and st == -1]
    for (a0, a1), (c0, c1) in zip(moves[:-1], moves[1:]):
        _, dyo, dpo = relative_pose(b['odom'].at(a1), b['odom'].at(c0))
        _, dyt, dpt = relative_pose(b['truth'].at(a1), b['truth'].at(c0))
        out.append((dyo - dyt, dpo - dpt))
    return np.array(out)


def laser_residuals(b):
    """激光墙面拟合（base_link）相对真值的残差，按阶段分组."""
    w = b['wall']
    x, y, yaw = b['truth'].at(w[:, 0])
    d, u, psi = to_wall(x, y, yaw)
    ed = w[:, 1] - d
    epsi = np.array([wrap(v) for v in (-w[:, 2] - psi)])
    phase = [phase_at(b['phases'], t) for t in w[:, 0]]
    return w[:, 0], ed, epsi, d, phase, w[:, 3], w[:, 5]


def tag_residuals_bag(b):
    """估计器每周期的标记测量（已做延迟补偿）相对真值的残差，附真值横移速度与方位角."""
    rows = []
    tr = b['truth']
    for r in b['estimate']:
        t = float(r['t'])
        d, u, psi = to_wall(*tr.at(t))
        d2, u2, _ = to_wall(*tr.at(t + 0.05))
        d1, u1, _ = to_wall(*tr.at(t - 0.05))
        v = (u2 - u1) / 0.1
        for i, px, py, acc in parse_tags(r['tags']):
            c, s = math.cos(psi), math.sin(psi)
            du = u - WORLD_TAG_U[i]
            h = np.array([d * c + du * s, -d * s + du * c])
            rows.append((t, i, px - h[0], py - h[1], math.atan2(py, px), v, d, u, acc))
    return rows


def tag_residuals_trace():
    """along_wall 的 31 次运行：同样的残差（真值取记录回调时刻，时间对齐较粗）."""
    rows = []
    for path in sorted(glob.glob(os.path.join(AW_RUNS, '*', 'ekf_trace.csv'))):
        run = os.path.basename(os.path.dirname(path))
        tr = read_csv(path)
        tu = np.array([float(r['true_u']) for r in tr])
        tt = np.array([float(r['t']) for r in tr])
        for k, r in enumerate(tr):
            if not r.get('tag_xy'):
                continue
            d, u, psi = float(r['true_d']), float(r['true_u']), float(r['true_psi'])
            k0, k1 = max(k - 1, 0), min(k + 1, len(tr) - 1)
            v = (tu[k1] - tu[k0]) / max(tt[k1] - tt[k0], 1e-3) if tt[k1] - tt[k0] < 0.25 else 0
            for i, px, py in parse_trace_tags(r['tag_xy']):
                c, s = math.cos(psi), math.sin(psi)
                du = u - WORLD_TAG_U[i]
                h = np.array([d * c + du * s, -d * s + du * c])
                rows.append((run, i, px - h[0], py - h[1], math.atan2(py, px), v, d, u))
    return rows


def gauss_mixture(x, iters=200):
    """两分量零均值高斯混合的 EM 估计：核心（小 σ）+ 尾部（大 σ）."""
    x = np.asarray(x, dtype=float)
    s1 = 1.4826 * np.median(np.abs(x))
    s2, w2 = 5 * s1, 0.1
    for _ in range(iters):
        p1 = (1 - w2) * np.exp(-0.5 * (x / s1) ** 2) / s1
        p2 = w2 * np.exp(-0.5 * (x / s2) ** 2) / s2
        r1 = p1 / (p1 + p2)
        w2 = 1 - r1.mean()
        s1 = math.sqrt((r1 * x ** 2).sum() / r1.sum())
        s2 = math.sqrt(((1 - r1) * x ** 2).sum() / (1 - r1).sum())
    return {'sigma_core': s1, 'sigma_tail': s2, 'weight_tail': w2, 'resp_core': r1}


def robust(x):
    x = np.asarray(x, dtype=float)
    med = np.median(x)
    mad = 1.4826 * np.median(np.abs(x - med))
    return x[np.abs(x - med) < 5 * mad + 1e-12]


def main():
    segs, holds = [], []
    lasers = []
    tag_rows = []
    per_run = {}
    for name in BAGS:
        b = load_bag(name)
        s = strafe_segments(b, name)
        segs += s
        holds.append(hold_drift(b))
        lasers.append((name, laser_residuals(b)))
        if name.startswith('default'):
            tag_rows += tag_residuals_bag(b)
        long = [g for g in s if abs(g['dy_true']) > 0.03]
        dyo = np.array([g['dy_odom'] for g in long])
        dyt = np.array([g['dy_true'] for g in long])
        # 每次运行的横移比例（最小二乘）：dy_true = (1 + s)·dy_odom
        per_run[name] = float(dyt @ dyo / (dyo @ dyo) - 1.0)

    with open(os.path.join(HERE, 'noise_segments.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(segs[0].keys()))
        w.writeheader()
        for g in segs:
            w.writerow({k: (f'{v:.7g}' if isinstance(v, float) else v) for k, v in g.items()})

    # ---------------- 里程计 ----------------
    long = [g for g in segs if abs(g['dy_true']) > 0.03]
    s_seg = np.array([g['dy_true'] / g['dy_odom'] - 1.0 for g in long])
    run_of = [g['bag'] for g in long]
    s_dev = np.array([sv - per_run[r] for sv, r in zip(s_seg, run_of)])
    dyt = np.array([g['dy_true'] for g in long])
    # 朝向：里程计 / 陀螺增量误差与横移距离的关系 dψ_err = κ·Δy_true + e
    epo = np.array([wrap(g['dpsi_odom'] - g['dpsi_true']) for g in long])
    epg = np.array([g['dpsi_gyro'] - g['dpsi_true'] for g in long])
    kappa = float(dyt @ epo / (dyt @ dyt))
    kappa_g = float(dyt @ epg / (dyt @ dyt))
    res_o = epo - kappa * dyt
    res_g = epg - kappa_g * dyt
    seg_len = np.abs(dyt)
    seg_dur = np.array([g['t1'] - g['t0'] for g in long])
    hold = np.vstack(holds)

    # ---------------- 激光 ----------------
    ed_all, ep_all, ed_static_means, ep_static_means, ep_static_within, ed_static_within = \
        [], [], [], [], [], []
    ep_lag1 = []
    for name, (t, ed, ep, d, phase, var_d, var_phi) in lasers:
        ed_all.append(ed)
        ep_all.append(ep)
        # 静止段：到站（arrived）与切割期间的定点保持（cut）
        ph = np.array([p[1] for p in phase])
        st = np.array([p[0] for p in phase])
        for k in sorted(set(st[st >= 0])):
            m = (st == k) & np.isin(ph, ('arrived', 'cut'))
            if m.sum() < 15:
                continue
            ed_static_means.append(np.mean(ed[m]))
            ep_static_means.append(np.mean(ep[m]))
            ep_static_within.append(np.std(ep[m] - np.mean(ep[m])))
            ed_static_within.append(np.std(ed[m] - np.mean(ed[m])))
            e = ep[m] - np.mean(ep[m])
            ep_lag1.append(float(np.corrcoef(e[:-1], e[1:])[0, 1]))
    ed_all = robust(np.concatenate(ed_all))
    ep_all = robust(np.concatenate(ep_all))
    laser_var_phi = float(np.median(np.concatenate([lz[1][6] for lz in lasers])))
    laser_var_d = float(np.median(np.concatenate([lz[1][5] for lz in lasers])))

    # ---------------- 标记 ----------------
    T = np.array([r[1:8] for r in tag_rows], dtype=float)   # id, rx, ry, bearing, v, d, u
    static = np.abs(T[:, 4]) < 0.002
    moving = np.abs(T[:, 4]) > 0.01
    # 静止：ry = a + b·bearing
    A = np.column_stack([np.ones(static.sum()), T[static, 3]])
    coef, *_ = np.linalg.lstsq(A, T[static, 2], rcond=None)
    ry_static_res = T[static, 2] - A @ coef
    # 运动：在静止模型之外加一项 τ·v（时间对齐误差），残差用两分量零均值高斯混合（EM）描述
    r0 = T[moving, 2] - coef[0] - coef[1] * T[moving, 3]
    v_m = T[moving, 4]
    tau = 0.0
    for _ in range(5):
        ry_mov_all = r0 - tau * v_m
        mix = gauss_mixture(ry_mov_all)
        w_in = mix['resp_core']
        tau = float((w_in * v_m) @ r0 / ((w_in * v_m) @ v_m))
    ry_mov_all = r0 - tau * v_m
    mix = gauss_mixture(ry_mov_all)
    coef_m = (coef[0], coef[1], tau)
    s_rob = mix['sigma_core']
    ry_mov_res = ry_mov_all[mix['resp_core'] > 0.5]
    out_m = mix['resp_core'] <= 0.5
    rx_static = robust(T[static, 1])

    tr_rows = tag_residuals_trace()
    TT = np.array([r[1:8] for r in tr_rows], dtype=float)
    st_tr = np.abs(TT[:, 4]) < 0.002
    mv_tr = np.abs(TT[:, 4]) > 0.01
    ry_tr_static = robust(TT[st_tr, 2] - coef[0] - coef[1] * TT[st_tr, 3])
    ry_tr_mov = robust(TT[mv_tr, 2] - coef[0] - coef[1] * TT[mv_tr, 3])

    with open(os.path.join(HERE, 'noise_tag_residuals.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(('source', 'speed', 'bearing_deg', 'n', 'ry_mean_mm', 'ry_std_mm',
                    'rx_mean_mm', 'rx_std_mm'))
        for src, M in (('bags', T), ('along_wall_trace', TT)):
            for sp_name, sel in (('static', np.abs(M[:, 4]) < 0.002),
                                 ('moving', np.abs(M[:, 4]) > 0.01)):
                for lo, hi in ((-40, -20), (-20, -5), (-5, 5), (5, 20), (20, 40)):
                    bdeg = np.degrees(M[:, 3])
                    m = sel & (bdeg >= lo) & (bdeg < hi)
                    if m.sum() < 5:
                        continue
                    w.writerow((src, sp_name, f'{lo}…{hi}', int(m.sum()),
                                f'{np.mean(M[m, 2]) * 1e3:.3f}', f'{np.std(M[m, 2]) * 1e3:.3f}',
                                f'{np.mean(M[m, 1]) * 1e3:.3f}', f'{np.std(M[m, 1]) * 1e3:.3f}'))

    s_runs = np.array(list(per_run.values()))
    model = {
        'odometry': {
            'lateral_scale_mean': {
                'value': round(float(np.mean(s_runs)), 5), 'source': 'fitted',
                'n': f'{len(s_runs)} runs ({len(long)} strafes)',
                'note': 'true strafe = (1 + s) * odom strafe; per-run least squares',
                'per_run': {k: round(v, 5) for k, v in per_run.items()}},
            'lateral_scale_run_sigma': {
                'value': round(float(np.std(s_runs, ddof=1)), 5), 'source': 'fitted',
                'n': f'{len(s_runs)} runs'},
            'lateral_scale_segment_sigma': {
                'value': round(float(np.std(robust(s_dev), ddof=1)), 5), 'source': 'fitted',
                'n': f'{len(long)} strafes (|dy| > 3 cm), deviation from the run mean',
                'note': 'random slip, redrawn per strafe; segment length '
                        f'{np.median(seg_len) * 1000:.0f} mm median, '
                        f'{np.median(seg_dur):.1f} s median'},
            'heading_per_strafe_odom': {
                'value': round(kappa, 6), 'unit': 'rad per m of true strafe',
                'source': 'fitted', 'n': f'{len(long)} strafes',
                'residual_sigma_rad': round(float(np.std(res_o, ddof=1)), 6)},
            'heading_per_strafe_gyro': {
                'value': round(kappa_g, 6), 'unit': 'rad per m of true strafe',
                'source': 'fitted', 'n': f'{len(long)} strafes',
                'residual_sigma_rad': round(float(np.std(res_g, ddof=1)), 6),
                'note': 'the wall estimator uses /wheel/odom, not the gyro'},
            'hold_drift': {
                'lateral_sigma_m': round(float(np.std(hold[:, 0], ddof=1)), 7),
                'heading_sigma_rad': round(float(np.std(hold[:, 1], ddof=1)), 7),
                'source': 'fitted', 'n': f'{len(hold)} station holds (arrival to next strafe)',
                'note': 'odometry minus truth while the base holds position during cutting'},
        },
        'laser': {
            'distance_bias_m': {'value': round(float(np.mean(ed_all)), 6), 'source': 'fitted',
                                'n': f'{len(ed_all)} scans'},
            'distance_sigma_m': {'value': round(float(np.std(ed_all, ddof=1)), 6),
                                 'source': 'fitted', 'n': f'{len(ed_all)} scans'},
            'yaw_bias_rad': {'value': round(float(np.mean(ep_all)), 7), 'source': 'fitted',
                             'n': f'{len(ep_all)} scans'},
            'yaw_sigma_rad': {'value': round(float(np.std(ep_all, ddof=1)), 7),
                              'source': 'fitted', 'n': f'{len(ep_all)} scans',
                              'note': 'all phases, after 5-MAD outlier removal'},
            'yaw_station_bias_sigma_rad': {
                'value': round(float(np.std(ep_static_means, ddof=1)), 7), 'source': 'fitted',
                'n': f'{len(ep_static_means)} station holds',
                'note': 'mean yaw error per static station hold; pose-dependent '
                        '(range quantisation), redrawn per station in the simulator'},
            'yaw_within_station_sigma_rad': {
                'value': round(float(np.mean(ep_static_within)), 7), 'source': 'fitted',
                'lag1_autocorrelation': round(float(np.mean(ep_lag1)), 3)},
            'distance_station_bias_sigma_m': {
                'value': round(float(np.std(ed_static_means, ddof=1)), 7), 'source': 'fitted'},
            'covariance_used_by_estimator': {
                'var_d_median': laser_var_d, 'var_phi_median': laser_var_phi,
                'source': 'fit_wall covariance, median over all scans'},
        },
        'tag': {
            'lateral_bias_m': {'value': round(float(coef[0]), 7), 'source': 'fitted',
                               'n': f'{int(static.sum())} static detections (2 runs)'},
            'lateral_bias_per_rad_bearing': {
                'value': round(float(coef[1]), 7), 'source': 'fitted',
                'note': 'ry = a + b * bearing (bearing from base_link x axis)'},
            'lateral_sigma_static_m': {
                'value': round(float(np.std(robust(ry_static_res), ddof=1)), 7),
                'source': 'fitted', 'n': f'{int(static.sum())} detections'},
            'time_offset_s': {
                'value': round(float(coef_m[2]), 5), 'source': 'fitted',
                'n': f'{int(moving.sum())} detections while strafing (|v| > 1 cm/s)',
                'note': 'ry error = tau * along-wall speed (residual time misalignment)'},
            'lateral_sigma_moving_m': {
                'value': round(float(s_rob), 7), 'source': 'fitted',
                'n': f'{len(ry_mov_res)} detections'},
            'moving_outlier_fraction': {
                'value': round(float(mix['weight_tail']), 4), 'source': 'fitted',
                'sigma_m': round(float(mix['sigma_tail']), 6),
                'note': 'two-component zero-mean Gaussian mixture (EM): core sigma = '
                        'lateral_sigma_moving_m, tail weight / sigma here; consecutive '
                        'residuals are uncorrelated at 10 Hz'},
            'depth_bias_m': {'value': round(float(np.mean(rx_static)), 6), 'source': 'fitted'},
            'depth_sigma_m': {'value': round(float(np.std(rx_static, ddof=1)), 6),
                              'source': 'fitted'},
            'cross_check_along_wall_trace': {
                'static_lateral_sigma_m': round(float(np.std(ry_tr_static, ddof=1)), 7),
                'moving_lateral_sigma_m': round(float(np.std(ry_tr_mov, ddof=1)), 7),
                'n_static': int(len(ry_tr_static)), 'n_moving': int(len(ry_tr_mov)),
                'note': '31 runs; truth sampled at the recorder callback (coarser timing)'},
        },
        'visibility': {
            'window_m': None, 'source': 'fitted', 'note': 'filled below'},
        'assumed': {
            'survey_sigma_m': {'value': 0.0005, 'source': 'assumed',
                               'note': 'surveyed map: tape / total-station measurement after '
                                       'placement'},
            'placement_sigma_m': {'value': [0.0, 0.001, 0.003, 0.005], 'source': 'assumed',
                                  'note': 'Monte Carlo variable, Gaussian 1-sigma per '
                                          'non-reference marker'},
        },
    }
    # 可见范围：被检测到时“底盘 u − 标记 u”（离墙 0.65–0.75 m 的周期）
    vis = T[(T[:, 5] > 0.65) & (T[:, 5] < 0.75)]
    rel = vis[:, 6] - np.array([WORLD_TAG_U[int(i)] for i in vis[:, 0]])
    model['visibility'] = {
        'rel_u_min_m': round(float(np.min(rel)), 3), 'rel_u_max_m': round(float(np.max(rel)), 3),
        'at_distance_m': 0.70, 'source': 'fitted',
        'n': f'{len(rel)} detections (2 runs), robot u minus tag u when detected',
        'note': 'scaled with distance to the wall in the simulator'}
    with open(os.path.join(HERE, 'noise_model.yaml'), 'w') as f:
        f.write('# 离线测量仿真器的噪声模型（noise_model.py 生成；source: fitted = 由录制数据拟合，\n'
                '# assumed = 假设）。数据：bag_data/ 的 4 次运行与 along_wall_accuracy 的 31 次运行\n')
        yaml.safe_dump(model, f, sort_keys=False, allow_unicode=True)
    print(yaml.safe_dump(model, sort_keys=False, allow_unicode=True))
    print('segment s:', np.round(s_seg, 4))


if __name__ == '__main__':
    os.chdir(BAG_DATA)
    main()
