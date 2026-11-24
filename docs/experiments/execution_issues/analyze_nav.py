#!/usr/bin/env python3
"""
导航段到达过程的汇总与作图（不需要 Gazebo，读 runs/nav_* 的记录）.

  python3 analyze_nav.py

输出：
  nav_legs_all.csv     全部导航段（每段一行，含配置与种子）
  nav_summary.csv      按配置 × 方向汇总
  fig_nav_arrival.png/.pdf   修正前后每段耗时的分布 + 修正前一段停滞的时间线
"""
import csv
import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
# (结果目录前缀, 图例, 配色角色)；stress = 导航期间另有 3 个忙循环进程
CONFIGS = [('nav_default', '20 ms', 'baseline'),
           ('nav_timeout', '1000 ms', 'proposed'),
           ('nav_stress_default', '20 ms, loaded', 'alt2'),
           ('nav_stress_timeout', '1000 ms, loaded', 'alt')]


def load_legs():
    rows = []
    for cfg, _, _ in CONFIGS:
        for d in sorted(glob.glob(os.path.join(HERE, 'runs', f'{cfg}_s*'))):
            path = os.path.join(d, 'legs.csv')
            if not os.path.exists(path):
                continue
            seed = int(d.rsplit('_s', 1)[1])
            with open(path) as f:
                for r in csv.DictReader(f):
                    r = dict(config=cfg, seed=seed, **r)
                    rows.append(r)
    return rows


NAV2_EVENTS = ('acknowledge goal request', 'Running spin', 'Running wait', 'Running backup',
               'Exceeded time allowance', 'Failed to make progress', 'Goal succeeded',
               'Begin navigating', 'Reached the goal', 'failed to generate')


def nav2_events(d):
    """
    Nav2 的关键日志行（墙钟时间戳相对启动日志第一行）写入 <运行>/nav2_events.txt 并返回.

    完整启动日志不提交；有 launch.log 时从中提取，否则读已提交的 nav2_events.txt。
    """
    path = os.path.join(d, 'nav2_events.txt')
    log = os.path.join(d, 'launch.log')
    if os.path.exists(log):
        import re
        t0, keep = None, []
        with open(log, errors='replace') as f:
            for line in f:
                m = re.search(r'\[(\d{10}\.\d+)\] \[([\w.]+)\]: (.*)', line)
                if not m:
                    continue
                t0 = float(m[1]) if t0 is None else t0
                if any(k in m[3] for k in NAV2_EVENTS):
                    keep.append(f'{float(m[1]) - t0:9.3f} [{m[2]}] {m[3].strip()}')
        with open(path, 'w') as f:
            f.write('\n'.join(keep) + '\n')
    with open(path) as f:
        return f.read().splitlines()


def timed_out_counts(d):
    """
    每段中 bt_navigator 报告“等待服务器确认目标超时”的次数（按 Begin navigating 分段）.
    """
    out, leg = {}, -1
    for line in nav2_events(d):
        if 'Begin navigating' in line:
            leg += 1
        elif 'acknowledge goal request' in line and leg >= 0:
            out[leg] = out.get(leg, 0) + 1
    return out


def summarize(rows):
    out = []
    for cfg, _, _ in CONFIGS:
        for goal in ('marker', 'start', 'all'):
            sel = [r for r in rows if r['config'] == cfg and (goal == 'all' or r['goal'] == goal)]
            if not sel:
                continue
            t = np.array([float(r['time_s']) for r in sel])
            ok = sum(r['status'] == 'succeeded' for r in sel)
            out.append(dict(
                config=cfg, goal=goal, legs=len(sel), succeeded=ok,
                time_mean_s=round(float(t.mean()), 2), time_median_s=round(float(np.median(t)), 2),
                time_std_s=round(float(t.std(ddof=1)) if len(t) > 1 else 0.0, 2),
                time_max_s=round(float(t.max()), 2),
                legs_with_recovery=sum(int(r['recoveries']) > 0 for r in sel),
                recoveries=sum(int(r['recoveries']) for r in sel),
                spins=sum(int(r['spins']) for r in sel), waits=sum(int(r['waits']) for r in sel),
                ack_timeouts=sum(int(r['ack_timeouts']) for r in sel),
                legs_with_ack_timeout=sum(int(r['ack_timeouts']) > 0 for r in sel),
                tail_still_mean_s=round(float(np.mean([float(r['tail_still_s']) for r in sel])),
                                        2),
                final_true_xy_max_m=round(max(float(r['final_true_xy_m']) for r in sel), 3),
                final_loc_err_max_m=round(max(float(r['final_loc_err_m']) for r in sel), 3)))
    return out


def plot(rows):
    sys.path.insert(0, os.path.join(REPO, 'docs'))
    import figstyle as fs
    fig, axes = fs.subplots(1, 3, width='double', height=2.6,
                            gridspec_kw=dict(width_ratios=[0.55, 1.0, 1.3]))
    # (a) 历史日志：起点 → 墙前的 MPPI 导航段，有 / 无“确认目标超时”
    ax = axes[0]
    hist = [r for r in csv.DictReader(open(os.path.join(HERE, 'nav_history.csv')))
            if r['controller'] == 'MPPI' and r['result'] == 'succeeded' and is_marker_leg(r)]
    groups = [('No timeout', [r for r in hist if ack_total(r) == 0], 'proposed'),
              ('follow_path\ntimeout', [r for r in hist if int(r['ack_follow_path']) > 0],
               'baseline')]
    for k, (name, sel, role) in enumerate(groups):
        t = np.array([float(r['time_s']) for r in sel])
        jitter = (np.arange(len(t)) % 7 - 3) * 0.05
        ax.plot(k + jitter, t, **fs.marker(role, ms=2.5))
    ax.set_xticks([0, 1])
    ax.set_xticklabels([f'{g[0]}\nn = {len(g[1])}' for g in groups], fontsize=7)
    ax.set_xlim(-0.5, 1.5)
    ax.set_ylabel('Navigation time, start → wall (s)')
    fs.light_grid(ax, axis='y')
    ax.set_title('(a) Earlier runs (logs)')

    # (b) 本节的对照：每种配置 24 段（2 个种子 × 12 段）
    ax = axes[1]
    width = 0.8 / len(CONFIGS)
    for g, goal in enumerate(('marker', 'start')):
        for c, (cfg, name, role) in enumerate(CONFIGS):
            t = [float(r['time_s']) for r in rows if r['config'] == cfg and r['goal'] == goal]
            if not t:
                continue
            x = g + (c - (len(CONFIGS) - 1) / 2) * width
            ax.boxplot([t], positions=[x], widths=width * 0.8, showfliers=False,
                       medianprops=dict(color=fs.color(role)), whiskerprops=dict(lw=0.6),
                       capprops=dict(lw=0.6), boxprops=dict(lw=0.6))
            jitter = (np.arange(len(t)) % 5 - 2) * width * 0.08
            ax.plot(x + jitter, t, **fs.marker(role, ms=2),
                    label=name if g == 0 else None)
    ax.set_xticks([0, 1])
    ax.set_xticklabels(['Start → wall', 'Wall → start'])
    ax.set_ylabel('Navigation time (s)')
    ax.set_ylim(21.5, 29.5)
    ax.legend(loc='upper right', ncol=2, fontsize=7, handletextpad=0.2, columnspacing=0.6,
              title='Acknowledgement timeout', title_fontsize=7)
    fs.light_grid(ax, axis='y')
    ax.set_title('(b) This study, n = 12 legs per box')

    # (c) 历史运行中的一次停滞（plan_validation joint_s1，记录包提取，extract_stall_example.py）
    ax = axes[2]
    ex = list(csv.DictReader(open(os.path.join(HERE, 'stall_example_joint_s1.csv'))))
    ctrl = [(float(r['t']), float(r['wz'])) for r in ex if r['source'] == 'controller']
    spin = [(float(r['t']), float(r['wz'])) for r in ex if r['source'] == 'spin']
    truth = [(float(r['t']), float(r['dist'])) for r in ex if r['kind'] == 'truth']
    ax.plot(*zip(*ctrl), **fs.marker('alt', ms=1.5),
            label='/cmd_vel from controller')
    ax.plot(*zip(*spin), **fs.marker('baseline', ms=2),
            label='/cmd_vel from Spin')
    ax.axvspan(spin[0][0], spin[-1][0], color=fs.color('neutral'), alpha=0.15, lw=0)
    ax.set_xlabel('Time since recording start (s)')
    ax.set_ylabel('Angular velocity command (rad/s)')
    ax.set_xlim(15, 65)
    ax2 = ax.twinx()
    ax2.plot(*zip(*truth), **fs.line('proposed'), label='Distance to goal (truth)')
    ax2.set_ylabel('Distance to goal (m)')
    ax2.set_ylim(0, 3)
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc='upper right', fontsize=7, markerscale=2)
    ax.set_title('(c) Stall in plan_validation joint_s1')
    fs.save(fig, os.path.join(HERE, 'fig_nav_arrival.png'))


def is_marker_leg(r):
    """历史日志中从起点出发、到 room3 南墙标记前的导航段."""
    try:
        sx, sy = (float(v) for v in r['start'].split())
        gx, gy = (float(v) for v in r['goal'].split())
    except ValueError:
        return False
    return abs(sx) < 0.2 and abs(sy) < 0.2 and abs(gx - 8.42) < 0.02 and abs(gy + 2.72) < 0.02


def ack_total(r):
    return int(r['ack_follow_path']) + int(r['ack_compute_path']) + int(r['ack_behavior'])


def main():
    rows = load_legs()
    for cfg, _, _ in CONFIGS:
        for d in sorted(glob.glob(os.path.join(HERE, 'runs', f'{cfg}_s*'))):
            seed = int(d.rsplit('_s', 1)[1])
            counts = timed_out_counts(d)
            for r in rows:
                if r['config'] == cfg and r['seed'] == seed:
                    r['ack_timeouts'] = counts.get(int(r['leg']), 0)
    fields = list(rows[0])
    with open(os.path.join(HERE, 'nav_legs_all.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)
    summ = summarize(rows)
    with open(os.path.join(HERE, 'nav_summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(summ[0]))
        w.writeheader()
        w.writerows(summ)
    for s in summ:
        print(s)
    plot(rows)


if __name__ == '__main__':
    main()
