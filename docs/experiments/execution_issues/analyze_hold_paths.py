#!/usr/bin/env python3
"""
第 6.4 节：定点保持控制律 p / dob 在单站任务、独立精对位与示教零位上的配对对照（汇总与图）.

  python3 analyze_hold_paths.py

读取 runs/hold_task_<p|dob>_s<k>/ 与 runs/hold_align_<p|dob>_s<k>/（hold_paths.py 的输出），
输出 hold_paths_runs.csv（每次运行一行）、hold_paths_summary.csv（每条路径、每个控制律的
范围与配对差）、fig_hold_paths.png/pdf。

单站任务：任务总时长（第一条到最后一条阶段消息，仿真秒）、精对位后的真值误差（日志）、
定点保持期间的真值漂移（日志中技能服务器给出的最大值，以及由真值记录按“定点保持开始时的
位姿”计算的最大位移与偏航）。
独立精对位：每个控制律 4 个起点，到位后保持 8 s 内的真值最大位移 / 偏航、到位用时与到位误差。
示教零位：示教得到的偏置（法向 / 沿墙 / 朝向）、示教期间真值的最大位移与偏航。
runs/hold_odom10_<p|dob>_s<k>/：两个工具按节点默认的 /odom（EKF，仿真中 10 Hz）运行，精对位得不到
测量，只比较示教期间的定点保持（path = teach_odom10）。
"""
import csv
import glob
import math
import os
import re
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..'))
import figstyle as fs  # noqa: E402

RUNS = os.path.join(HERE, 'runs')
HOLD_PHASES = ('hold', 'scan', 'cut', 'arm')


def read_csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def drift(truth, t0, t1):
    """真值在 [t0, t1] 内相对 t0 时刻位姿的最大位移（mm）与偏航（°）."""
    seg = [r for r in truth if t0 <= r[0] <= t1]
    if len(seg) < 2:
        return math.nan, math.nan
    x0, y0, a0 = seg[0][1:]
    d = max(math.hypot(r[1] - x0, r[2] - y0) for r in seg) * 1e3
    a = max(abs(wrap(r[3] - a0)) for r in seg)
    return d, math.degrees(a)


def task_run(d):
    with open(os.path.join(d, 'summary.yaml')) as f:
        sm = yaml.safe_load(f)
    row = dict(path='task', hold=sm['hold'], seed=sm['seed'], client_rc=sm.get('client_rc'))
    log = open(os.path.join(d, 'task_layer.log'), errors='replace').read()
    m = re.findall(r'真值误差: 位置=([\d.]+)mm \(dx=([-\d.]+), dy=([-\d.]+)\), 角度=([\d.]+)°', log)
    if m:
        row.update(align_pos_err_mm=float(m[-1][0]), align_yaw_err_deg=float(m[-1][3]))
    m = re.findall(r'定点保持期间底盘真值最大漂移: ([\d.]+) mm, ([-\d.]+)°', log)
    if m:
        row.update(hold_drift_log_mm=float(m[-1][0]), hold_yaw_log_deg=abs(float(m[-1][1])))
    row['recoveries'] = len(re.findall(r'CountRecovery|重新导航', log))
    truth = [tuple(float(x) for x in r.values()) for r in read_csv(os.path.join(d, 'truth.csv'))]
    st = read_csv(os.path.join(d, 'status.csv'))
    st = [(float(r['stamp']), r['phase']) for r in st]
    st.sort()
    if st:
        t_first = st[0][0]
        hold = [t for t, p in st if p in HOLD_PHASES]
        t_end = max(t for t, _ in st)
        row['task_s'] = t_end - t_first
        if hold:
            row['hold_s'] = t_end - hold[0]
            row['hold_drift_mm'], row['hold_yaw_deg'] = drift(truth, hold[0], t_end)
    return row


def align_run(d):
    with open(os.path.join(d, 'summary.yaml')) as f:
        sm = yaml.safe_load(f)
    rows = []
    path = os.path.join(d, 'align_trials.csv')
    if os.path.exists(path):
        for r in read_csv(path):
            rows.append(dict(path='align', hold=sm['hold'], seed=sm['seed'],
                             trial=int(r['trial']), success=r['success'] == 'True',
                             align_s=float(r['align_s']),
                             align_pos_err_mm=math.hypot(float(r['normal_err_mm']),
                                                         float(r['lateral_err_mm'])),
                             hold_drift_mm=float(r['hold_drift_max_mm']),
                             hold_yaw_deg=float(r['hold_yaw_max_deg'])))
    teach = dict(path='teach', hold=sm['hold'], seed=sm['seed'], teach_rc=sm.get('teach_rc'))
    if sm.get('taught'):
        t = sm['taught']
        teach.update(offset_normal_mm=t['offset_normal'] * 1e3,
                     offset_lateral_mm=t['offset_lateral'] * 1e3,
                     offset_yaw_deg=t['offset_yaw_deg'])
    tp = os.path.join(d, 'teach_truth.csv')
    if os.path.exists(tp):
        truth = [tuple(float(x) for x in r.values()) for r in read_csv(tp)]
        if len(truth) > 10:
            # 示教采样为最后 settle 3 s + 30 s；进程在采样结束后立即退出，取记录的最后 33 s
            t1 = truth[-1][0]
            teach['teach_drift_mm'], teach['teach_yaw_deg'] = drift(truth, t1 - 33.0, t1)
            teach['teach_s'] = 33.0
    rows.append(teach)
    return rows


def rng(v):
    v = [x for x in v if isinstance(x, (int, float)) and not math.isnan(x)]
    if not v:
        return ''
    return f'{min(v):.3f}–{max(v):.3f}' if len(v) > 1 else f'{v[0]:.3f}'


def paired(rows, key, group=('seed',)):
    """dob − p 的配对差（同路径、同种子，有 trial 时同试验）：平均与范围."""
    p = {tuple(r.get(g) for g in group): r.get(key) for r in rows if r['hold'] == 'p'}
    d = {tuple(r.get(g) for g in group): r.get(key) for r in rows if r['hold'] == 'dob'}
    ok = [k for k in p if k in d and isinstance(p[k], float) and isinstance(d[k], float)]
    diff = [d[k] - p[k] for k in ok if not (math.isnan(p[k]) or math.isnan(d[k]))]
    if not diff:
        return ''
    return f'{np.mean(diff):+.3f} ({min(diff):+.3f} … {max(diff):+.3f}, n = {len(diff)})'


def main():
    rows = []
    for d in sorted(glob.glob(os.path.join(RUNS, 'hold_task_*'))):
        if os.path.exists(os.path.join(d, 'summary.yaml')):
            rows.append(task_run(d))
    for d in sorted(glob.glob(os.path.join(RUNS, 'hold_align_*'))):
        if os.path.exists(os.path.join(d, 'summary.yaml')):
            rows += align_run(d)
    # 两个工具按文档默认的 /odom（EKF，仿真中 10 Hz）运行：精对位得不到测量，只比较示教中的保持
    for d in sorted(glob.glob(os.path.join(RUNS, 'hold_odom10_*'))):
        if os.path.exists(os.path.join(d, 'summary.yaml')):
            rows += [dict(r, path='teach_odom10') for r in align_run(d) if r['path'] == 'teach']
    fields = list(dict.fromkeys(k for r in rows for k in r))
    with open(os.path.join(HERE, 'hold_paths_runs.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow({k: (f'{v:.4f}' if isinstance(v, float) else v) for k, v in r.items()})
    metrics = {
        'task': ['task_s', 'hold_s', 'align_pos_err_mm', 'align_yaw_err_deg', 'hold_drift_mm',
                 'hold_yaw_deg', 'hold_drift_log_mm', 'hold_yaw_log_deg'],
        'align': ['align_s', 'align_pos_err_mm', 'hold_drift_mm', 'hold_yaw_deg'],
        'teach': ['offset_normal_mm', 'offset_lateral_mm', 'offset_yaw_deg', 'teach_drift_mm',
                  'teach_yaw_deg'],
        'teach_odom10': ['offset_normal_mm', 'offset_lateral_mm', 'offset_yaw_deg',
                         'teach_drift_mm', 'teach_yaw_deg'],
    }
    summary = []
    for path, keys in metrics.items():
        rs = [r for r in rows if r['path'] == path]
        group = ('seed', 'trial') if path == 'align' else ('seed',)
        for k in keys:
            summary.append(dict(
                path=path, metric=k,
                p=rng([r.get(k) for r in rs if r['hold'] == 'p']),
                dob=rng([r.get(k) for r in rs if r['hold'] == 'dob']),
                n_p=sum(1 for r in rs if r['hold'] == 'p' and k in r),
                n_dob=sum(1 for r in rs if r['hold'] == 'dob' and k in r),
                paired_dob_minus_p=paired(rs, k, group)))
    with open(os.path.join(HERE, 'hold_paths_summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0]))
        w.writeheader()
        w.writerows(summary)
    for s in summary:
        print(s)
    figure(rows)


def figure(rows):
    panels = [('task', 'hold_drift_mm', 'Single-station task'),
              ('align', 'hold_drift_mm', 'Fine alignment, 8 s hold'),
              ('teach', 'teach_drift_mm', 'Teaching, /wheel/odom'),
              ('teach_odom10', 'teach_drift_mm', 'Teaching, /odom 10 Hz')]
    fig, axes = fs.subplots(1, 4, width='double', height=2.3)
    for ax, (path, key, title), letter in zip(axes, panels, 'abcd'):
        rs = [r for r in rows if r['path'] == path and isinstance(r.get(key), float)]
        keyf = (lambda r: (r['seed'], r.get('trial')))
        keys = sorted({keyf(r) for r in rs},
                      key=lambda k: (k[0], k[1] if k[1] is not None else -1))
        off = {k: (-0.15 + 0.3 * n / max(len(keys) - 1, 1)) for n, k in enumerate(keys)}
        pts = {}
        for x, (mode, role) in enumerate((('p', 'baseline'), ('dob', 'proposed'))):
            for r in rs:
                if r['hold'] == mode:
                    xy = (x + off[keyf(r)], r[key])
                    pts[(mode, keyf(r))] = xy
                    ax.plot(*xy, **fs.marker(role))
        for k in keys:
            if ('p', k) in pts and ('dob', k) in pts:
                (x0, y0), (x1, y1) = pts[('p', k)], pts[('dob', k)]
                ax.plot([x0, x1], [y0, y1], color=fs.color('neutral'), lw=0.4, zorder=0)
        ax.set_xticks([0, 1])
        ax.set_xticklabels(['P', 'DOB'])
        ax.set_xlim(-0.5, 1.5)
        if path == 'teach_odom10':
            ax.set_yscale('log')
            ax.set_yticks([0.01, 0.1, 1, 10])
            ax.set_yticklabels(['0.01', '0.1', '1', '10'])
        else:
            ax.set_ylim(bottom=0)
        fs.panel_label(ax, letter, title)
    axes[0].set_ylabel(fs.label('Max. base drift (ground truth)', 'mm'))
    fs.save(fig, os.path.join(HERE, 'fig_hold_paths'))


if __name__ == '__main__':
    main()
