#!/usr/bin/env python3
"""
汇总精对位对比实验：合并各次运行的 trials.csv，输出 summary.csv、robustness.png、
traces.png 以及 README 用的 Markdown 表格.

  python3 docs/experiments/alignment_robustness/make_report.py
"""
import csv
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = ['run_nominal_launch', 'run_camera_yaw']
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'src', 'scr_arm_controller'))

from scr_arm_controller.alignment_benchmark import COND_ORDER, PIPELINES, plot  # noqa: E402


def load():
    rows = []
    for r in RUNS:
        p = os.path.join(HERE, r, 'trials.csv')
        if os.path.exists(p):
            with open(p) as f:
                for row in csv.DictReader(f):
                    row['run'] = r
                    rows.append(row)
    return rows


def f(r, k):
    try:
        return float(r[k])
    except (KeyError, ValueError):
        return math.nan


def summarize(rows):
    keys = sorted({(r['condition'], r['pipeline']) for r in rows},
                  key=lambda k: (COND_ORDER.index(k[0]), list(PIPELINES).index(k[1])))
    out = []
    for cond, pipe in keys:
        rs = [r for r in rows if r['condition'] == cond and r['pipeline'] == pipe]
        pe = np.array([f(r, 'pos_err_mm') for r in rs])
        lat = np.array([f(r, 'lateral_err_mm') for r in rs])
        nor = np.array([f(r, 'normal_err_mm') for r in rs])
        yaw = np.array([f(r, 'yaw_err_deg') for r in rs])
        est = np.array([f(r, 'est_lateral_err_mm') for r in rs])
        out.append({
            'condition': cond, 'pipeline': pipe, 'n': len(rs),
            'success': sum(r['success'] == 'True' for r in rs),
            'collisions': sum(r['collision'] == 'True' for r in rs),
            'pos_mean_mm': np.mean(pe), 'pos_std_mm': np.std(pe), 'pos_max_mm': np.max(pe),
            'normal_mean_mm': np.mean(nor), 'lateral_mean_mm': np.mean(lat),
            'lateral_absmax_mm': np.max(np.abs(lat)), 'yaw_abs_mean_deg': np.mean(np.abs(yaw)),
            'est_lateral_mean_mm': np.nanmean(est) if np.isfinite(est).any() else math.nan,
            'min_clearance_mm': np.min([f(r, 'clearance_mm') for r in rs]),
            'time_mean_s': np.mean([f(r, 'time_s') for r in rs]),
        })
    return out


def markdown(summary):
    names = {'v1': '第一版', 'v2': '本版', 'v2_blob': '本版 + 色块', 'v2_taught': '本版 + 示教零位'}
    lines = ['| 条件 | 流水线 | 成功 | 位置误差 mean ± std (max) | 法向均值 | 沿墙均值 | 朝向误差绝对值均值 '
             '| 最小刀尖间隙 | 耗时（仿真） |', '|---|---|---|---|---|---|---|---|---|']
    for s in summary:
        lines.append(
            f"| {s['condition']} | {names[s['pipeline']]} | {s['success']}/{s['n']} | "
            f"{s['pos_mean_mm']:.2f} ± {s['pos_std_mm']:.2f} ({s['pos_max_mm']:.2f}) mm | "
            f"{s['normal_mean_mm']:+.2f} mm | {s['lateral_mean_mm']:+.2f} mm | "
            f"{s['yaw_abs_mean_deg']:.2f}° | {s['min_clearance_mm']:.1f} mm | "
            f"{s['time_mean_s']:.1f} s |")
    return '\n'.join(lines)


def plot_traces(path):
    """本版名义条件与间歇遮挡条件的估计时间历程（沿墙估计、σ_u、标记更新）."""
    p = os.path.join(HERE, 'run_nominal_launch', 'traces.csv')
    if not os.path.exists(p):
        return
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['font.family'] = ['Noto Sans CJK JP', 'DejaVu Sans']
    with open(p) as fh:
        tr = list(csv.DictReader(fh))
    fig, ax = plt.subplots(1, 3, figsize=(11, 3.3))
    for cond, col in [('nominal', '#2F6FDE'), ('occlusion_intermittent', '#D9480F')]:
        for k in range(3):
            rs = [r for r in tr if r['condition'] == cond and r['pipeline'] == 'v2'
                  and int(r['trial']) == k and r['u'] != 'nan']
            if not rs:
                continue
            t = np.array([float(r['t']) for r in rs])
            u = np.array([float(r['u']) for r in rs]) * 1000
            d = (np.array([float(r['d']) for r in rs]) - 0.70) * 1000
            su = np.array([float(r['sigma_u']) for r in rs]) * 1000
            ok = np.array([r['tag_ok'] == 'True' for r in rs])
            lab = {'nominal': '名义', 'occlusion_intermittent': '间歇遮挡'}[cond] if k == 0 else None
            ax[0].plot(t, np.abs(d), color=col, lw=1, label=lab)
            ax[1].plot(t, np.abs(u), color=col, lw=1, label=lab)
            ax[2].plot(t, su, color=col, lw=1, label=lab)
            if cond == 'occlusion_intermittent':
                ax[2].plot(t[~ok], su[~ok], '.', ms=2, color=col)
    ax[0].set_ylabel('|d − d*| 估计（mm）')
    ax[1].set_ylabel('|u| 估计（mm）')
    ax[2].set_ylabel('σ_u（mm），点 = 该周期无标记更新')
    for a in ax:
        a.set_xlabel('时间（s，仿真）')
        a.set_yscale('log')
        a.spines[['top', 'right']].set_visible(False)
    ax[0].legend(frameon=False, fontsize=8)
    fig.suptitle('本版精对位的估计时间历程（Gazebo，每个条件前 3 次试验）', fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=150)


def main():
    rows = load()
    summary = summarize(rows)
    with open(os.path.join(HERE, 'summary.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(summary[0].keys()))
        w.writeheader()
        w.writerows({k: (round(v, 4) if isinstance(v, float) else v) for k, v in s.items()}
                    for s in summary)
    plot(summary, os.path.join(HERE, 'robustness.png'))
    plot_traces(os.path.join(HERE, 'traces.png'))
    print(markdown(summary))
    print(f'\n{len(rows)} 次试验')


if __name__ == '__main__':
    main()
