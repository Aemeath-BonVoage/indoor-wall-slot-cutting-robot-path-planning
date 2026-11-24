#!/usr/bin/env python3
"""
不估计横移比例时剩余 1–2 mm 沿墙偏差的离线检验.

  OPENBLAS_NUM_THREADS=1 nice -n 19 python3 bias_analysis.py      # 约 2 min

假设：横移中里程计的比例误差（编码器高估 ~3 %）持续产生同号的标记新息；标记的横向测量
y ≈ (u − m) − d·ψ 不能区分 u 与 ψ，EKF 按先验协方差把新息的一部分分给 ψ。ψ 被推向正值，
激光（每帧 σ 0.22°）拉回得慢；到站后标记仍约束 u − d·ψ，于是 û − u ≈ d·(ψ̂ − ψ) > 0。

检验：
  1. Gazebo 记录（along_wall_accuracy 的 ekf_trace）：每站到站残差测量结束时的 û − u 与
     ψ̂ − ψ，配置 tags_0.6 / scale_tags_0.6 / tags_0.3 / default，验证 û − u ≈ d·(ψ̂ − ψ)；
  2. 仿真器（tags_0.6 配置：3 个标记、地图正确、不估计比例），逐项改变里程计误差：
     完整模型 / 横移比例误差置 0 / 只保留比例误差 / 关闭全部编码器误差（对应 Gazebo 的
     odom_error:=false）/ 估计比例；
  3. 比例误差扫描 ε = 0 … 5 %：偏差应随 ε 线性增长、同号；
  4. 激光朝向权重扫描（估计器使用的 σ_φ 放大 / 缩小）：偏差应随激光变弱而增大。
输出 bias_gazebo.csv、bias_sim.csv。
"""
import csv
import math
import os

import numpy as np

from common import AW_RUNS, HERE, read_csv
from wall_sim import layouts, load_model, load_scenario, make_map, run_pass, Trial

N = 40
GZ_CONFIGS = ('tags_0.6', 'scale_tags_0.6', 'tags_0.3', 'default', 'tags_0.6_err3')


def gazebo():
    rows = []
    for cfg in GZ_CONFIGS:
        for seed in (1, 2):
            path = os.path.join(AW_RUNS, f'{cfg}_s{seed}', 'ekf_trace.csv')
            if not os.path.exists(path):
                continue
            tr = read_csv(path)
            last = {}
            for r in tr:
                if r['phase'] == 'arrived' and int(r['station']) >= 0:
                    last[int(r['station'])] = r
            for st, r in sorted(last.items()):
                eu = float(r['u']) - float(r['true_u'])
                ep = float(r['psi']) - float(r['true_psi'])
                rows.append(dict(config=cfg, seed=seed, station=st, u_err_mm=eu * 1e3,
                                 psi_err_deg=math.degrees(ep),
                                 d_times_psi_err_mm=float(r['true_d']) * ep * 1e3))
    with open(os.path.join(HERE, 'bias_gazebo.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        for r in rows:
            w.writerow({k: (f'{v:.4f}' if isinstance(v, float) else v) for k, v in r.items()})
    for cfg in GZ_CONFIGS:
        rs = [r for r in rows if r['config'] == cfg]
        if not rs:
            continue
        eu = np.array([r['u_err_mm'] for r in rs])
        ep = np.array([r['psi_err_deg'] for r in rs])
        dp = np.array([r['d_times_psi_err_mm'] for r in rs])
        c = np.corrcoef(eu, dp)[0, 1]
        print(f'Gazebo {cfg:16s} n={len(rs)} û−u {eu.mean():+.2f} ± {eu.std():.2f} mm, '
              f'ψ̂−ψ {ep.mean():+.3f}°, d·δψ {dp.mean():+.2f} mm, corr {c:.2f}')
    return rows


def sim_case(overrides, scale=False, ekf_kw=None, laser_factor=1.0, n=N):
    model = load_model()
    if laser_factor != 1.0:
        model = dict(model)
        model['R_wall'] = model['R_wall'] * np.array([[1.0, 1.0], [1.0, laser_factor ** 2]])
    tags = layouts()[3]
    eu, ep, dp = [], [], []
    for k in range(n):
        scn = load_scenario(('default_s1', 'default_s2')[k % 2])
        tr = Trial(scn, model, np.random.default_rng(4000 + k), tags, overrides=overrides)
        r = run_pass(scn, tr, make_map(tags), 'known', scale, ekf_kw=ekf_kw, record=True)
        trc = r['trace']
        idx = scn['stations']
        eu += list(r['err'])
        ep += list(trc[idx, 3])
        dp += list(scn['d'][idx] * trc[idx, 3])
    eu, ep, dp = np.array(eu), np.array(ep), np.array(dp)
    return dict(u_err_mean_mm=eu.mean() * 1e3, u_err_std_mm=eu.std() * 1e3,
                abs_max_mean_mm=np.mean(np.abs(eu).reshape(n, -1).max(axis=1)) * 1e3,
                psi_err_mean_deg=math.degrees(ep.mean()),
                d_psi_mean_mm=dp.mean() * 1e3, corr=float(np.corrcoef(eu, dp)[0, 1]))


def main():
    gazebo()
    off_all = {'s_mean': 0.0, 's_run': 0.0, 's_seg': 0.0, 'h_seg': 0.0, 'kappa': 0.0,
               'hold_u': 0.0, 'hold_psi': 0.0}
    cases = [
        ('full odometry model', {}, False),
        ('lateral scale error = 0 (random slip kept)', {'s_mean': 0.0, 's_run': 0.0}, False),
        ('only lateral scale error (no random slip, no heading noise)',
         {'s_seg': 0.0, 'h_seg': 0.0, 'kappa': 0.0, 'hold_u': 0.0, 'hold_psi': 0.0}, False),
        ('strafe heading noise = 0', {'h_seg': 0.0}, False),
        ('all encoder errors off (odom_error:=false)', off_all, False),
        ('full odometry model, scale estimated', {}, True),
    ]
    rows = []
    for name, ov, sc in cases:
        r = sim_case(ov, sc)
        rows.append(dict(sweep='case', case=name, value='', **r))
        print(f'{name:60s}', {k: round(v, 3) for k, v in r.items()})
    for eps in (0.0, 0.01, 0.02, 0.03, 0.04, 0.05):
        s = 1.0 / (1.0 + eps) - 1.0
        r = sim_case({'s_mean': s, 's_run': 0.0}, False, n=20)
        rows.append(dict(sweep='lateral scale error', case=f'odometry over-reads strafe by '
                         f'{eps * 100:.0f} %', value=eps, **r))
        print('eps', eps, {k: round(v, 3) for k, v in r.items()})
    for fac in (0.5, 1.0, 2.0, 4.0):
        r = sim_case({}, False, laser_factor=fac, n=20)
        rows.append(dict(sweep='laser yaw sigma factor', case=f'estimator laser σ_φ × {fac}',
                         value=fac, **r))
        print('laser', fac, {k: round(v, 3) for k, v in r.items()})
    with open(os.path.join(HERE, 'bias_sim.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        for r in rows:
            w.writerow({k: (f'{v:.4f}' if isinstance(v, float) else v) for k, v in r.items()})


if __name__ == '__main__':
    main()
