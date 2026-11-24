#!/usr/bin/env python3
"""
扰动观测器定点保持在默认空切任务上的对照（第 6 节，不需要 Gazebo）.

  python3 analyze_dob.py

用 arm_trajectory/make_report.py 的同一套评估函数（刀尖轨迹误差、切割段内横向偏差、机械臂
运动期间的底盘漂移、到站误差、覆盖率、各站耗时）汇总：
  runs/dry_dob_<规划>_s<种子>   hold.mode = dob（本节）
  runs/dry_p_<规划>_s<种子>     hold.mode = p（本节，同一版本代码的对照）
  ../arm_trajectory/runs/cont_<规划>_s<种子>   hold.mode = p（此前的运行）
输出 dob_runs.csv、dob_summary.csv、fig_dob_dryrun.png/.pdf。
"""
import csv
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(HERE, '..', 'arm_trajectory'))
import make_report as mr  # noqa: E402

CONFIGS = [('p_earlier', 'P (earlier runs)', 'neutral',
            os.path.join(HERE, '..', 'arm_trajectory', 'runs', 'cont_{plan}_s{seed}')),
           ('p', 'P', 'baseline', os.path.join(HERE, 'runs', 'dry_p_{plan}_s{seed}')),
           ('dob', 'DOB', 'proposed', os.path.join(HERE, 'runs', 'dry_dob_{plan}_s{seed}'))]
KEYS = ['stations_total_s', 'base_move_s', 'cut_action_s', 'path_err_mean_mm', 'path_err_p95_mm',
        'path_err_max_mm', 'cross_rms_mm', 'cross_max_mm', 'arm_drift_mm_mean',
        'arm_drift_mm_max', 'hold_drift_mm_max', 'lateral_mm_mean', 'lateral_mm_max',
        'coverage_path', 'hold_yaw_max_deg']


def hold_yaw_max(d):
    """技能服务器日志中每站“定点保持期间底盘真值最大漂移: x mm, y°”的偏航最大值."""
    import re
    with open(os.path.join(d, 'task_layer.log'), errors='replace') as f:
        pat = r'定点保持期间底盘真值最大漂移: [\d.]+ mm, ([\d.]+)°'
        v = [float(m) for m in re.findall(pat, f.read())]
    return max(v) if v else float('nan')


def main():
    runs = []
    for cfg, _, _, pattern in CONFIGS:
        for plan in ('joint', 'rule'):
            for seed in (1, 2):
                d = pattern.format(plan=plan, seed=seed)
                if not os.path.exists(os.path.join(d, 'summary.yaml')):
                    print(f'missing {d}')
                    continue
                res, _ = mr.summarize(d)
                res['hold_yaw_max_deg'] = hold_yaw_max(d)
                runs.append(dict(config=cfg, plan=plan, seed=seed,
                                 **{k: res.get(k, float('nan')) for k in KEYS},
                                 all_ok=res['all_ok'], recoveries=res['recoveries']))
    with open(os.path.join(HERE, 'dob_runs.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(runs[0]))
        w.writeheader()
        for r in runs:
            w.writerow({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()})
    summ = []
    for cfg, _, _, _ in CONFIGS:
        for plan in ('joint', 'rule', 'all'):
            sel = [r for r in runs if r['config'] == cfg and plan in ('all', r['plan'])]
            if not sel:
                continue
            row = dict(config=cfg, plan=plan, n=len(sel))
            for k in KEYS:
                v = np.array([r[k] for r in sel], dtype=float)
                row[f'{k}_mean'] = round(float(np.mean(v)), 4)
                row[f'{k}_max'] = round(float(np.max(v)), 4)
            summ.append(row)
    with open(os.path.join(HERE, 'dob_summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(summ[0]))
        w.writeheader()
        w.writerows(summ)
    # 配对差（同规划、同种子）：DOB − P（本节）
    pairs = {}
    for r in runs:
        pairs.setdefault((r['plan'], r['seed']), {})[r['config']] = r
    for k in KEYS:
        d = [p['dob'][k] - p['p'][k] for p in pairs.values() if 'dob' in p and 'p' in p]
        if d:
            print(f'{k:22s} DOB - P: mean {np.mean(d):+.4f}, min {np.min(d):+.4f}, '
                  f'max {np.max(d):+.4f} (n = {len(d)})')
    plot(runs)


def plot(runs):
    sys.path.insert(0, os.path.join(REPO, 'docs'))
    import figstyle as fs
    panels = [('stations_total_s', 'Sum of station times (s)'),
              ('path_err_mean_mm', 'Tool-tip path error, mean (mm)'),
              ('path_err_max_mm', 'Tool-tip path error, max (mm)'),
              ('arm_drift_mm_max', 'Base drift during arm motion, max (mm)'),
              ('hold_yaw_max_deg', 'Base yaw during hold, max (deg)')]
    fig, axes = fs.subplots(1, len(panels), width='double', height=2.2)
    for letter, ax, (k, label) in zip('abcde', axes, panels):
        for i, (cfg, name, role, _) in enumerate(CONFIGS):
            for j, plan in enumerate(('joint', 'rule')):
                v = [r[k] for r in runs if r['config'] == cfg and r['plan'] == plan]
                x = j + (i - 1) * 0.25
                ax.plot([x] * len(v), v, **fs.marker(role, ms=3),
                        label=name if (j == 0 and ax is axes[0]) else None)
        ax.set_xticks([0, 1])
        ax.set_xticklabels(['Joint plan', 'Rule plan'])
        ax.set_xlim(-0.6, 1.6)
        ax.set_ylabel(label)
        fs.light_grid(ax, axis='y')
        fs.panel_label(ax, letter)
    h, lab = axes[0].get_legend_handles_labels()
    axes[4].legend(h, lab, loc='center', bbox_to_anchor=(0.5, 0.36), fontsize=7)
    fs.save(fig, os.path.join(HERE, 'fig_dob_dryrun.png'))


if __name__ == '__main__':
    main()
