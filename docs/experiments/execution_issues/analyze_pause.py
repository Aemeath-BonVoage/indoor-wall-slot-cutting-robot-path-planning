#!/usr/bin/env python3
"""
第 3 节图 2：每个机械臂轨迹目标的响应延迟（不需要 Gazebo）.

  python3 analyze_pause.py

读 goal_timing_p2p.csv 与 goal_timing_dry.csv（pause_timing.py 的输出）。每个目标的响应延迟取
“发送 → 已接受”（墙钟）与“结果晚于名义结束的时间”两者中较大的一个。后者对逐点执行（2 s 的
目标）取墙钟；对连续轨迹（5–20 s 的目标，墙钟受实时因子影响）取仿真时间（0.1 s 分辨率）。
"""
import csv
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
GROUPS = [('Point-to-point,\nheartbeat 3 s', 'goal_timing_p2p.csv',
           lambda r: r.startswith('p2p_rule_s'), 'baseline'),
          ('Point-to-point,\nheartbeat 0.1 s', 'goal_timing_p2p.csv',
           lambda r: r.startswith('p2p_rule_hb100'), 'proposed'),
          ('Continuous,\nheartbeat 3 s', 'goal_timing_dry.csv', lambda r: True, 'alt')]


def delay(r, sim):
    ack = float(r['ack_wall_ms']) / 1e3
    res = float(r['extra_sim_s']) if sim else float(r['result_wall_s']) - float(r['duration_s'])
    return max(ack, res, 0.012)          # < 12 ms 画在 12 ms


def main():
    sys.path.insert(0, os.path.join(REPO, 'docs'))
    import figstyle as fs
    fig, ax = fs.subplots(width='single', height=2.6)
    for k, (name, f, sel, role) in enumerate(GROUPS):
        rows = [r for r in csv.DictReader(open(os.path.join(HERE, f))) if sel(r['run'])]
        d = np.array([delay(r, f == 'goal_timing_dry.csv') for r in rows])
        jitter = (np.arange(len(d)) % 9 - 4) * 0.035
        ax.plot(k + jitter, d, **fs.marker(role, ms=2))
        ax.text(k, 5.5, f'n = {len(d)}', ha='center', fontsize=7)
        print(name.replace('\n', ' '), len(d), 'max %.3f' % d.max(),
              '> 0.3 s:', int((d > 0.3).sum()))
    ax.axhline(3.0, **fs.line('reference'), lw=0.6)
    ax.text(1.0, 2.6, 'Default heartbeat period (3 s)', ha='center', va='top', fontsize=7)
    ax.set_yscale('log')
    ax.set_ylim(0.01, 9)
    fs.plain_log(ax)
    ax.set_xticks(range(len(GROUPS)))
    ax.set_xticklabels([g[0] for g in GROUPS], fontsize=7)
    ax.set_xlim(-0.5, len(GROUPS) - 0.5)
    ax.set_ylabel('Response delay per goal (s)')
    fs.light_grid(ax, axis='y')
    fs.save(fig, os.path.join(HERE, 'fig_pause.png'))


if __name__ == '__main__':
    main()
