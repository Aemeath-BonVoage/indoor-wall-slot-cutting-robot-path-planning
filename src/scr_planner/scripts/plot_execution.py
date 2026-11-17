#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""画出一次多站开槽执行：规划槽线 vs Gazebo 真值刀尖轨迹.

  python3 src/scr_planner/scripts/plot_execution.py <task.yaml> <run_dir> <out.png>
"""
import csv
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'python'))
import slot_plan as sp  # noqa: E402


def main():
    task, run, out = sys.argv[1:4]
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    matplotlib.rcParams['font.sans-serif'] = ['Noto Sans CJK SC', 'Noto Sans CJK JP',
                                              'Noto Serif CJK JP', 'DejaVu Sans']
    matplotlib.rcParams['axes.unicode_minus'] = False

    cfg = yaml.safe_load(open(task, encoding='utf-8'))
    scene, _ = sp.scene_from_config(cfg)
    plan = yaml.safe_load(open(os.path.join(run, 'slot_plan.yaml'), encoding='utf-8'))
    trace = list(csv.DictReader(open(os.path.join(run, 'tool_trace.csv'))))
    stations = list(csv.DictReader(open(os.path.join(run, 'stations.csv'))))
    depth = plan['slot_spec']['depth']
    robot = sp.RobotModel()

    fig = plt.figure(figsize=(13, 6.2))
    gs = fig.add_gridspec(2, 3, height_ratios=[1.6, 1], width_ratios=[2.2, 1, 1])
    ax = fig.add_subplot(gs[0, :])
    paths = [np.array(s_['cut_path']) for s_ in plan['slots'] if s_['cut_path']]
    centers = [np.array(s_['conduit_display']) for s_ in plan['slots'] if s_['conduit_display']]
    conduit = np.vstack(paths)
    u0, u1 = conduit[:, 0].min() - 0.25, conduit[:, 0].max() + 0.25
    lo, hi = robot.reach_band
    ax.add_patch(Rectangle((u0, lo), u1 - u0, hi - lo, color='#D9EAD3', alpha=0.6,
                           label=f'机械臂可达高度 {lo:.2f}–{hi:.2f} m'))
    for o in scene.obstacles:
        g = o.grown(scene.margin_of(o))
        ax.add_patch(Rectangle((g.x / 100, g.y / 100), g.w / 100, g.h / 100, color='#DDDDDD'))
        ax.add_patch(Rectangle((o.x / 100, o.y / 100), o.w / 100, o.h / 100, color='#E06666'))
        ax.text(o.x / 100 + o.w / 200, o.y / 100 + o.h / 100 + 0.01, o.label, ha='center',
                fontsize=8)
    for s in plan['stations']:
        a, b = s['window']
        ax.axvspan(a, b, ymin=0, ymax=0.04, color='#1F4E9C' if s['index'] % 2 == 0 else '#E69138')
        ax.text((a + b) / 2, 0.02, f"{s['index']}", ha='center', fontsize=7, color='w')
    for k, (pth, cen) in enumerate(zip(paths, centers)):
        ax.plot(pth[:, 0], pth[:, 1], '-', color='#C00000', lw=7, alpha=0.35,
                label=f'规划槽线（直角折线，槽宽 {plan["slot_spec"]["width"] * 1000:.0f} mm）'
                if k == 0 else None)
        ax.plot(cen[:, 0], cen[:, 1], '--', color='#7F0000', lw=0.8,
                label='线管中心线（R12 cm 弯，仅示意）' if k == 0 else None)
        ax.plot(*pth[0], 's', color='k', ms=7)
        ax.plot(*pth[-1], 'o', color='#C00000', ms=8)
    deep = [(float(r['u']), float(r['v'])) for r in trace
            if int(r['station']) >= 0 and r['phase'] == 'cut']
    if deep:
        d = np.array(deep)
        ax.plot(d[:, 0], d[:, 1], '.', color='#1F4E9C', ms=1.5,
                label='Gazebo 真值刀尖（切割阶段）')
    ax.set_xlim(u0, u1)
    ax.set_ylim(0, 0.95)
    ax.set_aspect('equal')
    ax.set_xlabel('沿墙 u (m)')
    ax.set_ylabel('离地 v (m)')
    ax.legend(loc='upper right', fontsize=8, ncol=3)
    ax.set_title(f"room3 南墙：规划槽线与 Gazebo 中实际刀尖轨迹（{plan['partition']} / "
                 f"{plan['order']}，{len(plan['stations'])} 站）", fontsize=10)

    # 拐弯处放大
    ax2 = fig.add_subplot(gs[1, 0])
    c = paths[0][2] if len(paths[0]) > 2 else paths[0][0]
    for k, (pth, cen) in enumerate(zip(paths, centers)):
        ax2.plot(pth[:, 0], pth[:, 1], '-', color='#C00000', lw=1.2,
                 label='规划槽线' if k == 0 else None)
        ax2.plot(cen[:, 0], cen[:, 1], '--', color='#7F0000', lw=0.8,
                 label='线管弯（示意）' if k == 0 else None)
    if deep:
        ax2.plot(d[:, 0], d[:, 1], '.', color='#1F4E9C', ms=2, label='真值刀尖')
    ax2.set_xlim(c[0] - 0.16, c[0] + 0.04)
    ax2.set_ylim(c[1] - 0.16, c[1] + 0.04)
    ax2.set_aspect('equal')
    ax2.set_title('拐角处放大：直角切割，管子在槽内按 R12 cm 弯过', fontsize=9)
    ax2.legend(fontsize=7)

    idx = [int(s['station']) for s in stations]
    ax3 = fig.add_subplot(gs[1, 1])
    for k, col in (('lateral_mm', '#1F4E9C'), ('standoff_mm', '#E69138')):
        if k in stations[0]:
            ax3.plot(idx, [float(s[k]) for s in stations], 'o-', color=col, ms=4,
                     label={'lateral_mm': '横向', 'standoff_mm': '离墙'}[k])
    ax3.axhline(0, color='k', lw=0.5)
    ax3.set_xlabel('停靠站')
    ax3.set_ylabel('真值停靠误差 (mm)')
    ax3.legend(fontsize=7)
    ax3.set_title('停靠误差', fontsize=9)

    ax4 = fig.add_subplot(gs[1, 2])
    ax4.bar(idx, [float(s['move_s']) for s in stations], color='#999999', label='横移+到位')
    has_cut = any(float(s['cut_s']) > 0 for s in stations)
    if has_cut:
        ax4.bar(idx, [float(s['cut_s']) for s in stations],
                bottom=[float(s['move_s']) for s in stations], color='#1F4E9C', label='切割')
    ax4.set_xlabel('停靠站')
    ax4.set_ylabel('仿真时间 (s)')
    ax4.legend(fontsize=7)
    ax4.set_title('每站耗时' if has_cut else '每站横移+到位耗时', fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print('saved', out)


if __name__ == '__main__':
    main()
