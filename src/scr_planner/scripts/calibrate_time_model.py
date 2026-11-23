#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用 Gazebo 执行记录标定时间模型.

每站耗时 = t_settle + 横移距离 / v_strafe + 点数 × t_point + 切割长度 / v_feed
（逐点执行时没有最后一项，见 slot_plan.TimeModel）。

  python3 src/scr_planner/scripts/calibrate_time_model.py <run_dir> [<run_dir> ...]
      [--method=joint|phases|phases-reg|traj] [--feed=0.05]
      [--compare=align_v1,align_v2,align_v3,traj_v1] [--loo]

每个 run_dir 含 execute_slot_plan.py 输出的 stations.csv（每站一行：move_dist_m、points、
station_s = 到站 move_s + 切割 cut_s + 其余固定开销）。

--method=joint（默认）：对 station_s 做整体最小二乘，三个参数同时拟合。
--method=phases：按执行阶段分别估计，
    横移阶段  move_s = a + d / v_strafe（最小二乘）；
    切割阶段  t_point = Σ cut_s / Σ points（比值估计，偶发停顿按平均摊入）；
    固定开销  c = mean(station_s − move_s − cut_s)；t_settle = a + c。
    切割阶段偶有约 2–3 s 的停顿，落在少数站上。整体最小二乘会把它归到点数或横移距离上
    （横移距离只有 0.05–0.23 m 三档，杠杆点很少），而分阶段估计只让它影响 t_point 的均值。
--method=phases-reg：同 phases，但切割阶段也用回归 cut_s = c_cut + n × t_point，截距 c_cut
    计入每站固定开销（t_settle = a + c + c_cut）。横移前机械臂回到观测位姿后（精对位默认
    observe_between_stations），每站切割阶段多出一段与点数无关的“观测位姿 → 第一个切割点”
    运动；比值估计会把它摊进 t_point，使“每少一个点省多少时间”偏大。
--method=traj：机械臂连续轨迹（arm_motion.mode = continuous）。同 phases-reg，但切割阶段的
    回归多一项切割长度：cut_s = c_cut + n × t_point + L / v_feed。给出 --feed 时 v_feed 取该值
    （进给是指令值，不需要估计），只拟合 c_cut 与 t_point；不给时 v_feed 也由回归得到，
    用来核对实测的切割速度与指令进给是否一致。
--compare：在同一批数据上评估已有预设（slot_plan.TIME_MODELS）的每站 RMSE、平均偏差与
    每次运行各站耗时之和的误差。
--loo：逐次运行留一交叉验证（用其余运行拟合，预测留出运行），给出样本外每站 RMSE。
"""
import csv
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for d in (os.path.join(HERE, '..', 'python'),
          os.path.join(HERE, '..', '..', 'share', 'scr_planner', 'python')):
    if os.path.isdir(d):
        sys.path.insert(0, d)


def load(dirs):
    rows = []
    for d in dirs:
        for r in csv.DictReader(open(os.path.join(d, 'stations.csv'))):
            r['run'] = os.path.basename(os.path.normpath(d))
            rows.append(r)
    return rows


def arrays(rows):
    f = lambda k: np.array([float(r[k]) for r in rows])  # noqa: E731
    return dict(d=f('move_dist_m'), n=f('points'), y=f('station_s'),
                move=f('move_s'), cut=f('cut_s'), L=f('cut_cm') / 100.0)


INF = float('inf')


def fit_joint(a):
    A = np.c_[np.ones_like(a['d']), a['d'], a['n']]
    (t_settle, k, t_point), *_ = np.linalg.lstsq(A, a['y'], rcond=None)
    return t_settle, (1.0 / k if k > 1e-9 else INF), t_point, INF


def fit_phases(a):
    (a0, k), *_ = np.linalg.lstsq(np.c_[np.ones_like(a['d']), a['d']], a['move'], rcond=None)
    overhead = float(np.mean(a['y'] - a['move'] - a['cut']))
    t_point = float(a['cut'].sum() / a['n'].sum())
    return a0 + overhead, (1.0 / k if k > 1e-9 else INF), t_point, INF


def fit_phases_reg(a):
    (a0, k), *_ = np.linalg.lstsq(np.c_[np.ones_like(a['d']), a['d']], a['move'], rcond=None)
    overhead = float(np.mean(a['y'] - a['move'] - a['cut']))
    (c_cut, t_point), *_ = np.linalg.lstsq(np.c_[np.ones_like(a['n']), a['n']], a['cut'],
                                           rcond=None)
    return a0 + overhead + c_cut, (1.0 / k if k > 1e-9 else INF), float(t_point), INF


def fit_traj(a, feed=None):
    """连续轨迹：切割阶段 cut_s = c_cut + n × t_point + L / v_feed（feed 给定时不拟合 v_feed）."""
    (a0, k), *_ = np.linalg.lstsq(np.c_[np.ones_like(a['d']), a['d']], a['move'], rcond=None)
    overhead = float(np.mean(a['y'] - a['move'] - a['cut']))
    one = np.ones_like(a['n'])
    if feed:
        (c_cut, t_point), *_ = np.linalg.lstsq(np.c_[one, a['n']], a['cut'] - a['L'] / feed,
                                               rcond=None)
    else:
        (c_cut, t_point, kl), *_ = np.linalg.lstsq(np.c_[one, a['n'], a['L']], a['cut'],
                                                   rcond=None)
        feed = 1.0 / kl if kl > 1e-9 else INF
    return (a0 + overhead + c_cut, (1.0 / k if k > 1e-9 else INF), float(t_point),
            float(feed))


FIT = {'joint': fit_joint, 'phases': fit_phases, 'phases-reg': fit_phases_reg,
       'traj': fit_traj}


def preset_params(tm):
    return tm.t_settle, tm.v_strafe, tm.t_point, tm.v_feed


def predict(p, a):
    t_settle, v, t_point = p[:3]
    v_feed = p[3] if len(p) > 3 else INF
    return t_settle + a['d'] / v + a['n'] * t_point + a['L'] / v_feed


def stats(p, rows):
    a = arrays(rows)
    e = predict(p, a) - a['y']
    runs = sorted({r['run'] for r in rows})
    tot = [float(np.sum(e[[r['run'] == k for r in rows]])) for k in runs]
    return dict(rmse=float(np.sqrt(np.mean(e ** 2))), bias=float(np.mean(e)),
                r2=1 - float(np.sum(e ** 2) / np.sum((a['y'] - a['y'].mean()) ** 2)),
                total_err=tot)


def fmt(p):
    s = f't_settle = {p[0]:.2f} s, v_strafe = {p[1]:.4f} m/s, t_point = {p[2]:.3f} s'
    if len(p) > 3 and np.isfinite(p[3]):
        s += f', v_feed = {p[3]:.4f} m/s'
    return s


def main():
    dirs = [a for a in sys.argv[1:] if not a.startswith('--')]
    if not dirs:
        print(__doc__)
        sys.exit(1)
    kw = dict((a[2:].split('=', 1) + [''])[:2] for a in sys.argv[1:] if a.startswith('--'))
    method = kw.get('method', 'joint')
    rows = load(dirs)
    runs = sorted({r['run'] for r in rows})
    fit = FIT[method]
    if method == 'traj' and kw.get('feed'):
        fit = lambda a: fit_traj(a, float(kw['feed']))  # noqa: E731
    p = fit(arrays(rows))
    s = stats(p, rows)
    print(f'stations: {len(rows)}（{len(runs)} 次运行），method = {method}')
    print(f'{fmt(p)}；每站 RMSE = {s["rmse"]:.2f} s，R2 = {s["r2"]:.3f}，'
          f'各站耗时之和误差 = {", ".join(f"{t:+.1f}" for t in s["total_err"])} s')
    if 'compare' in kw:
        import slot_plan as sp
        for name in kw['compare'].split(','):
            q = preset_params(sp.TimeModel.preset(name))
            c = stats(q, rows)
            print(f'  预设 {name:9s}（{fmt(q)}）：每站 RMSE = {c["rmse"]:.2f} s，平均偏差 = '
                  f'{c["bias"]:+.2f} s，各站耗时之和误差 = '
                  f'{", ".join(f"{t:+.1f}" for t in c["total_err"])} s')
    if 'loo' in kw and len(runs) > 1:
        errs = []
        for k in runs:
            train = [r for r in rows if r['run'] != k]
            test = [r for r in rows if r['run'] == k]
            q = fit(arrays(train))
            c = stats(q, test)
            errs.append(c['rmse'] ** 2 * len(test))
            print(f'  留出 {k}：{fmt(q)}；留出运行每站 RMSE = {c["rmse"]:.2f} s，'
                  f'各站耗时之和误差 = {c["total_err"][0]:+.1f} s')
        print(f'  留一交叉验证每站 RMSE = {np.sqrt(sum(errs) / len(rows)):.2f} s')


if __name__ == '__main__':
    main()
