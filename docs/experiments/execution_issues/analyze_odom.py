#!/usr/bin/env python3
"""
慢速运动下 EKF 偏差的汇总与作图（第 5 节，不需要 Gazebo）.

  python3 analyze_odom.py

读 runs/odom_slow/phases.csv（odom_slow.py 的配对实验）与 odom_bag/*.csv（odom_bag_check.py
从此前运行的记录包提取的窗口），输出 odom_summary.csv 与 fig_odom_slow.png/.pdf。
"""
import csv
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
PHASES = [('open_vx_0.7', 'vx −0.7 mm/s'), ('open_vx_3', 'vx −3 mm/s'),
          ('open_vy_0.7', 'vy −0.7 mm/s'), ('zero', 'Zero cmd'), ('hold', 'Hold'),
          ('hold_arm', 'Hold + arm')]


def main():
    rows = list(csv.DictReader(open(os.path.join(HERE, 'runs', 'odom_slow', 'phases.csv'))))
    summ = []
    for avg in ('false', 'true'):
        for ph, _ in PHASES:
            sel = [r for r in rows if r['average_twist'] == avg and r['phase'] == ph]
            # EKF 与轮式里程计位姿的位移之差（mm）：沿指令方向的分量（有指令时）或位移的模
            key = 'along_mm' if ph.startswith('open') else 'mm'
            err = [float(r[f'ekf_{key}']) - float(r[f'wheel_{key}']) for r in sel]
            truth = [float(r[f'truth_{key}']) for r in sel]
            wheel_err = [float(r[f'wheel_{key}']) - float(r[f'truth_{key}']) for r in sel]
            summ.append(dict(average_twist=avg, phase=ph, n=len(sel),
                             truth_mm=round(float(np.mean(truth)), 3),
                             ekf_minus_wheel_mm=' / '.join(f'{e:+.2f}' for e in err),
                             wheel_minus_truth_mm=' / '.join(f'{e:+.2f}' for e in wheel_err)))
    with open(os.path.join(HERE, 'odom_summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(summ[0]))
        w.writeheader()
        w.writerows(summ)
    for s in summ:
        print(s)

    sys.path.insert(0, os.path.join(REPO, 'docs'))
    import figstyle as fs
    fig, axes = fs.subplots(1, 2, width='double', height=2.5,
                            gridspec_kw=dict(width_ratios=[1.6, 1.0]))
    ax = axes[0]
    for k, (avg, name, role) in enumerate((('false', 'Instantaneous twist (before)', 'baseline'),
                                           ('true', 'Averaged twist (after)', 'proposed'))):
        for i, (ph, _) in enumerate(PHASES):
            sel = [r for r in rows if r['average_twist'] == avg and r['phase'] == ph]
            key = 'along_mm' if ph.startswith('open') else 'mm'
            err = [float(r[f'ekf_{key}']) - float(r[f'wheel_{key}']) for r in sel]
            x = i + (k - 0.5) * 0.3
            ax.bar(x, np.mean(err), 0.28, **fs.bar(role), label=name if i == 0 else None)
            ax.plot([x] * len(err), err, **fs.marker('reference', ms=2.5))
    ax.axhline(0, color=fs.color('reference'), lw=0.5)
    ax.set_xticks(range(len(PHASES)))
    ax.set_xticklabels([p[1] for p in PHASES], fontsize=7)
    ax.set_ylabel('EKF − wheel odometry (mm)')
    ax.legend(loc='upper left', fontsize=7)
    fs.light_grid(ax, axis='y')
    ax.set_title('(a) Paired test, 8 s / 16 s phases, n = 2 each')

    ax = axes[1]
    for k, (name, role) in enumerate((('cont_joint_s1', 'proposed'), ('p_f5.5_joint_s1', 'alt'))):
        b = list(csv.DictReader(open(os.path.join(HERE, 'odom_bag', f'{name}.csv'))))
        b = [r for r in b if float(r['truth_mm']) < 20.0]
        bx = [float(r['twist_bias_x_mm_s']) for r in b]
        by = [float(r['twist_bias_y_mm_s']) for r in b]
        ax.plot(bx, by, **fs.marker(role, ms=2.5), label=f'{name} (n = {len(b)})')
    ax.axhline(0, color=fs.color('reference'), lw=0.5)
    ax.axvline(0, color=fs.color('reference'), lw=0.5)
    ax.set_xlabel('Twist bias, body x (mm/s)')
    ax.set_ylabel('Twist bias, body y (mm/s)')
    ax.legend(loc='upper left', fontsize=7)
    fs.light_grid(ax)
    ax.set_title('(b) Earlier runs, 8 s windows')
    fs.save(fig, os.path.join(HERE, 'fig_odom_slow.png'))


if __name__ == '__main__':
    main()
