#!/usr/bin/env python3
"""
带切割力的整墙切割试验的指标（真值）：刀尖偏移、底盘漂移、切削力核对.

  python3 station_metrics.py runs/<运行名> [...]            # 打印每次运行的指标
  python3 station_metrics.py --csv station_runs.csv runs/*_s?   # 汇总成一张表

数据来自 execute_slot_plan.py 的输出：tool_trace.csv（20 Hz：刀尖墙面坐标、阶段、底盘真值
位置与朝向）、wrench.csv（50 Hz：插件施加的力旋量与执行器预计的力旋量）、summary.yaml
（槽线误差、覆盖率）、stations.csv。

刀尖偏移的定义与离线仿真相同：底盘相对“本站机械臂轨迹开始时刻”的位姿变化折算到刀尖
（底盘前方 0.735 m，即真实切到槽深时刀尖的位置）处的位移，沿墙与进深两个方向合成。
它只反映底盘的运动；机械臂自身的变形体现在刀尖槽线误差（真实刀尖位置到规划槽线的距离）里。
"""
import argparse
import csv
import gzip
import math
import os
import sys

import numpy as np
import yaml

TIP_X = 0.735       # 刀尖在底盘前方的距离（离墙 0.70 m + 槽深 35 mm）


def read_csv(path):
    if not os.path.exists(path) and os.path.exists(path + '.gz'):
        path += '.gz'
    if not os.path.exists(path):
        return None
    op = gzip.open if path.endswith('.gz') else open
    with op(path, 'rt') as f:
        return list(csv.DictReader(f))


def cut_runs(trace):
    """刀尖轨迹中连续的切割段 → [(停靠站, 起止下标, 槽向)]."""
    out, start = [], None
    for k, r in enumerate(trace + [None]):
        cutting = r is not None and r['phase'] == 'cut' and int(r['station']) >= 0
        if cutting and start is None:
            start = k
        elif start is not None and (not cutting or r['station'] != trace[start]['station']):
            a, b = trace[start], trace[k - 1]
            du, dv = float(b['u']) - float(a['u']), float(b['v']) - float(a['v'])
            out.append((int(a['station']), start, k, 'h' if abs(du) > abs(dv) else 'v'))
            start = k if cutting else None
    return out


def tip_deviation(trace):
    """
    每个样本的刀尖偏移 (沿墙 mm, 进深 mm, 朝向 deg)；不在停靠站上的样本为 nan.

    参考位姿取每站机械臂轨迹开始时（该站第一个 move / cut 阶段的样本）的底盘真值。
    """
    n = len(trace)
    dev = np.full((n, 3), np.nan)
    ref, cur = None, None
    for k, r in enumerate(trace):
        st = int(r['station'])
        if st < 0 or r['phase'] not in ('move', 'cut'):
            continue
        pose = (float(r['base_u']), float(r['base_d']), float(r['base_yaw']))
        if st != cur:
            cur, ref = st, pose
        dyaw = math.atan2(math.sin(pose[2] - ref[2]), math.cos(pose[2] - ref[2]))
        # 面向墙面时逆时针转动使刀尖向左（−u）移动
        dev[k] = ((pose[0] - ref[0] - TIP_X * dyaw) * 1e3, (pose[1] - ref[1]) * 1e3,
                  math.degrees(dyaw))
    return dev


def run_metrics(run):
    trace = read_csv(os.path.join(run, 'tool_trace.csv'))
    with open(os.path.join(run, 'summary.yaml'), encoding='utf-8') as f:
        summary = yaml.safe_load(f)
    out = dict(run=os.path.basename(os.path.normpath(run)), all_ok=int(bool(summary['all_ok'])),
               stations_done=summary['stations_done'], sim_time_s=summary['sim_time_s'],
               stations_total_s=summary['stations_total_s'],
               path_err_mean_mm=summary.get('path_err_mean_mm'),
               path_err_max_mm=summary.get('path_err_max_mm'),
               coverage=summary.get('coverage_path'),
               base_drift_max_mm=summary.get('station_base_drift_mm_max_abs'))
    if trace is None or 'base_yaw' not in trace[0]:
        return out
    dev = tip_deviation(trace)
    mag = np.hypot(dev[:, 0], dev[:, 1])
    on = ~np.isnan(mag)
    out.update(tip_peak_all_mm=float(mag[on].max()), yaw_peak_deg=float(np.abs(dev[on, 2]).max()))
    runs = cut_runs(trace)
    for key, name in (('h', 'horizontal'), ('v', 'vertical')):
        idx = np.concatenate([np.arange(a, b) for _, a, b, o in runs if o == key] or [[]])
        idx = idx.astype(int)
        if len(idx) == 0:
            continue
        m = mag[idx]
        out[f'tip_peak_{name}_mm'] = float(np.nanmax(m))
        out[f'tip_rms_{name}_mm'] = float(np.sqrt(np.nanmean(m ** 2)))
        out[f'along_peak_{name}_mm'] = float(np.nanmax(np.abs(dev[idx, 0])))
        out[f'depth_peak_{name}_mm'] = float(np.nanmax(np.abs(dev[idx, 1])))
    cut_idx = np.concatenate([np.arange(a, b) for _, a, b, _ in runs]).astype(int)
    out['tip_peak_cut_mm'] = float(np.nanmax(mag[cut_idx]))
    out['tip_rms_cut_mm'] = float(np.sqrt(np.nanmean(mag[cut_idx] ** 2)))
    wrench = read_csv(os.path.join(run, 'wrench.csv'))
    if wrench:
        w = np.array([[float(r[k]) for k in ('t', 'fx', 'fy', 'mz', 'expected_fx',
                                             'expected_fy', 'expected_mz')] for r in wrench])
        t = np.array([float(r['t']) for r in trace])
        # 横槽进给期间的力（取每段中间 60 %，避开加减速）
        sel = np.zeros(len(w), bool)
        for _, a, b, o in runs:
            if o == 'h' and b - a > 10:
                t0, t1 = t[a], t[b - 1]
                sel |= (w[:, 0] > t0 + 0.2 * (t1 - t0)) & (w[:, 0] < t1 - 0.2 * (t1 - t0))
        if sel.any():
            out.update(fy_abs_mean_N=float(np.abs(w[sel, 2]).mean()),
                       mz_abs_mean_Nm=float(np.abs(w[sel, 3]).mean()),
                       fx_mean_N=float(w[sel, 1].mean()),
                       fy_cov=float(np.abs(w[sel, 2]).std() / np.abs(w[sel, 2]).mean()),
                       expected_fy_abs_mean_N=float(np.abs(w[sel, 5]).mean()),
                       expected_mz_abs_mean_Nm=float(np.abs(w[sel, 6]).mean()))
        out['force_peak_N'] = float(np.hypot(w[:, 1], w[:, 2]).max())
        out['mz_peak_Nm'] = float(np.abs(w[:, 3]).max())
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--csv', help='汇总表输出路径')
    args = ap.parse_args()
    rows = []
    for run in args.runs:
        if not os.path.exists(os.path.join(run, 'summary.yaml')):
            print(f'{run}: no summary.yaml (run did not finish)', file=sys.stderr)
            continue
        rows.append(run_metrics(run))
        print(', '.join(f'{k} {v:.3g}' if isinstance(v, float) else f'{k} {v}'
                        for k, v in rows[-1].items()))
    if args.csv and rows:
        fields = []
        for r in rows:
            fields += [k for k in r if k not in fields]
        with open(args.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            for r in rows:
                w.writerow({k: (f'{v:.4g}' if isinstance(v, float) else v)
                            for k, v in r.items()})


if __name__ == '__main__':
    main()
