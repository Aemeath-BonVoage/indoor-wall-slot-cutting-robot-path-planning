#!/usr/bin/env python3
"""
汇总精对位对比实验：合并各次运行的 trials.csv，输出 summary.csv、robustness.png、
traces.png 以及 README 用的 Markdown 表格；各次运行目录与 v1_baseline/ 下的 robustness.png
由各自的 summary.csv 重绘。图使用 docs/figstyle 的样式（PDF + PNG）.

  python3 docs/experiments/alignment_robustness/make_report.py
"""
import csv
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = ['run_nominal_launch', 'run_occlusion_static_v2', 'run_nominal_teach30s', 'run_camera_yaw']
# 被后续运行取代的行：(运行目录, 条件, 流水线)
#   occlusion_static / v2：运行中机器重启，只完成 3 次，整组在 run_occlusion_static_v2 重跑；
#   nominal / v2_taught：所用偏置来自 8 s 示教（偏置本身噪声过大），改用 30 s 示教重跑，
#   旧结果作为消融单独报告（teach8s_summary）
SUPERSEDED = {('run_nominal_launch', 'occlusion_static', 'v2'),
              ('run_nominal_launch', 'nominal', 'v2_taught')}
AIM_D = 0.70       # 瞄准点：base_link x 轴与墙面的交点（离墙 0.70 m）
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'src', 'scr_arm_controller'))
sys.path.insert(0, os.path.join(HERE, '..', '..'))

import figstyle as fs  # noqa: E402
from scr_arm_controller.alignment_benchmark import COND_ORDER, PIPELINES  # noqa: E402

# 图中的名称
COND_EN = {
    'nominal': 'Nominal', 'odom_drift_3cm': 'Odom. drift\n3 cm, 1°',
    'odom_drift_10cm': 'Odom. drift\n10 cm, 3°', 'drift_10cm_noguard': 'Drift 10 cm,\nno guard',
    'sensor_noise_low': 'Noise\n2 px, 2 cm', 'sensor_noise_high': 'Noise\n5 px, 5 cm',
    'light_strong': 'Strong\nlight', 'light_cool': 'Cool\nlight',
    'occlusion_intermittent': 'Intermittent\nocclusion', 'occlusion_static': 'Static\nocclusion',
    'camera_yaw_1deg': 'Camera\nyaw 1°'}
PIPE_EN = {'v1': 'v1 (colour blob + PBVS)', 'v2': 'v2 (AprilTag + LiDAR EKF)',
           'v2_blob': 'v2 estimator + colour blob', 'v2_taught': 'v2 + taught zero offset'}
PIPE_ROLE = {'v1': 'baseline', 'v2': 'proposed', 'v2_blob': 'alt', 'v2_taught': 'alt2'}


def load():
    rows = []
    for r in RUNS:
        p = os.path.join(HERE, r, 'trials.csv')
        if os.path.exists(p):
            with open(p) as f:
                for row in csv.DictReader(f):
                    row['run'] = r
                    if (r, row['condition'], row['pipeline']) not in SUPERSEDED:
                        rows.append(row)
    return rows


def aim_error(r):
    """瞄准点沿墙误差（mm）：base_link x 轴与墙面交点的沿墙位置 u − d·tan ψ."""
    return f(r, 'lateral_err_mm') - AIM_D * 1000 * math.tan(math.radians(f(r, 'yaw_err_deg')))


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
        aim = np.array([aim_error(r) for r in rs])
        out.append({
            'condition': cond, 'pipeline': pipe, 'n': len(rs),
            'success': sum(r['success'] == 'True' for r in rs),
            'collisions': sum(r['collision'] == 'True' for r in rs),
            'pos_mean_mm': np.mean(pe), 'pos_std_mm': np.std(pe), 'pos_max_mm': np.max(pe),
            'normal_mean_mm': np.mean(nor), 'lateral_mean_mm': np.mean(lat),
            'lateral_absmax_mm': np.max(np.abs(lat)), 'yaw_abs_mean_deg': np.mean(np.abs(yaw)),
            'aim_abs_mean_mm': np.mean(np.abs(aim)), 'aim_absmax_mm': np.max(np.abs(aim)),
            'est_lateral_mean_mm': np.nanmean(est) if np.isfinite(est).any() else math.nan,
            'min_clearance_mm': np.min([f(r, 'clearance_mm') for r in rs]),
            'time_mean_s': np.mean([f(r, 'time_s') for r in rs]),
        })
    return out


def markdown(summary):
    names = {'v1': '第一版', 'v2': '本版', 'v2_blob': '本版 + 色块', 'v2_taught': '本版 + 示教零位'}
    lines = ['| 条件 | 流水线 | 成功 | 位置误差 mean ± std (max) | 法向均值 | 沿墙均值 | 朝向绝对值均值 '
             '| 瞄准点绝对值均值 (max) | 最小刀尖间隙 | 耗时（仿真） |',
             '|---|---|---|---|---|---|---|---|---|---|']
    for s in summary:
        lines.append(
            f"| {s['condition']} | {names[s['pipeline']]} | {s['success']}/{s['n']} | "
            f"{s['pos_mean_mm']:.2f} ± {s['pos_std_mm']:.2f} ({s['pos_max_mm']:.2f}) mm | "
            f"{s['normal_mean_mm']:+.2f} mm | {s['lateral_mean_mm']:+.2f} mm | "
            f"{s['yaw_abs_mean_deg']:.2f}° | "
            f"{s['aim_abs_mean_mm']:.2f} ({s['aim_absmax_mm']:.2f}) mm | "
            f"{s['min_clearance_mm']:.1f} mm | "
            f"{s['time_mean_s']:.1f} s |")
    return '\n'.join(lines)


def plot_robustness(summary, path):
    """各条件、各流水线的真值位置误差（均值 ± 标准差）；全部失败的格只标注."""
    conds = [c for c in COND_ORDER if any(s['condition'] == c for s in summary)]
    pipes = [p for p in PIPELINES if any(s['pipeline'] == p for s in summary)]
    top = 16.0
    wide = len(conds) > 3
    fig, ax = fs.subplots(width='double' if wide else 'single', height=2.5 if wide else 2.2)
    width = min(0.8 / len(pipes), 0.3)
    for j, c in enumerate(conds):
        present = [p for p in pipes
                   if any(s['condition'] == c and s['pipeline'] == p for s in summary)]
        for i, p in enumerate(present):
            s = next(s for s in summary if s['condition'] == c and s['pipeline'] == p)
            x = j + (i - (len(present) - 1) / 2) * width
            n, ok = int(s['n']), int(s['success'])
            if ok == 0:
                ax.text(x, 0.3, f'all {n} failed', rotation=90, ha='center', va='bottom',
                        fontsize=7, color=fs.color(PIPE_ROLE[p]))
                continue
            ax.bar(x, float(s['pos_mean_mm']), width, yerr=float(s['pos_std_mm']),
                   error_kw=dict(lw=0.6, capsize=1.5), **fs.bar(PIPE_ROLE[p]))
            if ok < n:
                ax.text(x, min(float(s['pos_mean_mm']) + float(s['pos_std_mm']) + 0.3, top - 1),
                        f'{ok}/{n}', ha='center', va='bottom', fontsize=7)
    for p in pipes:
        ax.bar(0, 0, 0, label=PIPE_EN[p], **fs.bar(PIPE_ROLE[p]))
    ax.set_xticks(range(len(conds)))
    ax.set_xticklabels([COND_EN.get(c, c) for c in conds])
    ax.set_xlim(-0.6, len(conds) - 0.4)
    ax.set_ylabel(fs.label('Final position error', 'mm'))
    ax.set_ylim(0, top)
    if wide:
        ax.legend(ncol=2, loc='upper left')
    else:
        ax.legend(ncol=1, loc='lower left', bbox_to_anchor=(0, 1.0))
    fs.save(fig, path)


def plot_v1_archive(path):
    """v1_baseline/：第一版的里程计对位与视觉 PBVS（存档实验，对数纵轴）."""
    p = os.path.join(HERE, 'v1_baseline', 'summary.csv')
    with open(p) as fh:
        rows = list(csv.DictReader(fh))
    conds = list(dict.fromkeys(r['condition'] for r in rows))
    modes = [('odom', 'Odometry only (baseline)', 'baseline'),
             ('visual', 'Visual PBVS (LiDAR + colour blob)', 'proposed')]
    fig, ax = fs.subplots(width='single', height=2.4)
    w = 0.38
    for i, (m, lab, role) in enumerate(modes):
        for j, c in enumerate(conds):
            r = next((r for r in rows if r['condition'] == c and r['mode'] == m), None)
            if r is None:
                continue
            present = [mm for mm, _, _ in modes
                       if any(q['condition'] == c and q['mode'] == mm for q in rows)]
            x = j + (present.index(m) - (len(present) - 1) / 2) * w
            mean, std = float(r['pos_mean_mm']), float(r['pos_std_mm'])
            # 对数纵轴：柱从坐标下限 1 mm 画起，下误差棒截在下限以上
            ax.bar(x, mean - 1.0, w, bottom=1.0, **fs.bar(role))
            ax.errorbar(x, mean, yerr=[[min(std, mean - 1.05)], [std]], fmt='none',
                        ecolor=fs.BLACK, elinewidth=0.6, capsize=1.5)
            notes = []
            if int(r['success']) < int(r['n']):
                notes.append(f"{r['success']}/{r['n']}")
            if int(r['collisions']):
                notes.append(f"{r['collisions']} hit")
            if notes:
                ax.text(x, (mean + std) * 1.15, '\n'.join(notes), ha='center', va='bottom',
                        fontsize=7)
        ax.bar(0, 0, 0, label=lab, **fs.bar(role))
    ax.set_yscale('log')
    fs.plain_log(ax)
    ax.set_ylim(1, 300)
    ax.set_xticks(range(len(conds)))
    ax.set_xticklabels([COND_EN.get(c, c) for c in conds], fontsize=7)
    ax.set_xlim(-0.6, len(conds) - 0.4)
    ax.set_ylabel(fs.label('Final position error', 'mm'))
    fs.light_grid(ax, axis='y')
    ax.legend(loc='lower left', bbox_to_anchor=(0, 1.0))
    fs.save(fig, path)


def plot_traces(path):
    """本版名义条件与间歇遮挡条件的估计时间历程（沿墙估计、σ_u、标记更新）."""
    p = os.path.join(HERE, 'traces.csv')
    if not os.path.exists(p):
        return
    with open(p) as fh:
        tr = list(csv.DictReader(fh))
    fig, ax = fs.subplots(1, 3, width='double', height=2.3)
    for cond, role, name in [('nominal', 'proposed', 'Nominal'),
                             ('occlusion_intermittent', 'alt', 'Intermittent occlusion')]:
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
            lab = name if k == 0 else None
            kw = fs.line(role, lw=0.8)
            ax[0].plot(t, np.abs(d), label=lab, **kw)
            ax[1].plot(t, np.abs(u), label=lab, **kw)
            ax[2].plot(t, su, label=lab, **kw)
            if cond == 'occlusion_intermittent':
                ax[2].plot(t[~ok], su[~ok], 'o', ms=1.6, color=fs.color(role), mew=0,
                           label='No marker update' if k == 0 else None)
    ax[0].set_ylabel(fs.label('Distance estimate |d − d*|', 'mm'))
    ax[1].set_ylabel(fs.label('Along-wall estimate |u|', 'mm'))
    ax[2].set_ylabel(fs.label('Std. dev. of u estimate', 'mm'))
    for a in ax:
        a.set_xlabel(fs.label('Simulation time', 's'))
        a.set_yscale('log')
        fs.plain_log(a)
    fs.panel_labels(ax)
    h0, l0 = ax[0].get_legend_handles_labels()
    h2, l2 = ax[2].get_legend_handles_labels()
    fig.legend(h0 + h2[-1:], l0 + l2[-1:], loc='upper center', ncol=3,
               bbox_to_anchor=(0.5, 1.0))
    fs.fixed_layout(fig, left=0.42, right=0.06, bottom=0.46, top=0.36, wspace=0.62)
    fs.save(fig, path)


def main():
    rows = load()
    summary = summarize(rows)
    with open(os.path.join(HERE, 'summary.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(summary[0].keys()))
        w.writeheader()
        w.writerows({k: (round(v, 4) if isinstance(v, float) else v) for k, v in s.items()}
                    for s in summary)
    plot_robustness(summary, os.path.join(HERE, 'robustness.png'))
    plot_traces(os.path.join(HERE, 'traces.png'))
    # 各次运行目录的单独汇总图（由该目录的 summary.csv 重绘）
    for r in RUNS:
        p = os.path.join(HERE, r, 'summary.csv')
        if os.path.exists(p) and os.path.exists(os.path.join(HERE, r, 'robustness.png')):
            with open(p) as fh:
                plot_robustness(list(csv.DictReader(fh)), os.path.join(HERE, r, 'robustness.png'))
    plot_v1_archive(os.path.join(HERE, 'v1_baseline', 'robustness.png'))
    print(markdown(summary))
    print(f'\n{len(rows)} 次试验')
    # 消融：8 s 示教偏置下的 nominal / v2_taught（被 30 s 示教取代）
    with open(os.path.join(HERE, 'run_nominal_launch', 'trials.csv')) as fh:
        old = [r for r in csv.DictReader(fh)
               if r['condition'] == 'nominal' and r['pipeline'] == 'v2_taught']
    if old:
        t8 = summarize(old)
        print('\n8 s 示教消融：\n' + markdown(t8))


if __name__ == '__main__':
    main()
