#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""画出一次多站开槽执行：规划槽线 vs Gazebo 真值刀尖轨迹（docs/figstyle 统一样式，英文）.

  python3 src/scr_planner/scripts/plot_execution.py <task.yaml> <run_dir> <out.png>

同时写出与 <out.png> 同名的 PDF。
"""
import csv
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'python'))
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'docs'))
import slot_plan as sp  # noqa: E402


def main():
    task, run, out = sys.argv[1:4]
    import figstyle as fs
    from matplotlib.patches import Rectangle

    cfg = yaml.safe_load(open(task, encoding='utf-8'))
    scene, _ = sp.scene_from_config(cfg)
    plan = yaml.safe_load(open(os.path.join(run, 'slot_plan.yaml'), encoding='utf-8'))
    trace = list(csv.DictReader(open(os.path.join(run, 'tool_trace.csv'))))
    stations = list(csv.DictReader(open(os.path.join(run, 'stations.csv'))))
    depth = plan['slot_spec']['depth']
    robot = sp.RobotModel()

    fig = fs.figure(width='double', height=4.6)
    gs = fig.add_gridspec(2, 3, height_ratios=[1.5, 1])
    ax = fig.add_subplot(gs[0, :])
    paths = [np.array(s_['cut_path']) for s_ in plan['slots'] if s_['cut_path']]
    centers = [np.array(s_['conduit_display']) for s_ in plan['slots'] if s_['conduit_display']]
    conduit = np.vstack(paths)
    u0, u1 = conduit[:, 0].min() - 0.25, conduit[:, 0].max() + 0.25
    lo, hi = robot.reach_band
    ax.add_patch(Rectangle((u0, lo), u1 - u0, hi - lo, color='#E8F1F8', lw=0,
                           label=f'Arm reach band {lo:.2f}–{hi:.2f} m'))
    for o in scene.obstacles:
        g = o.grown(scene.margin_of(o))
        ax.add_patch(Rectangle((g.x / 100, g.y / 100), g.w / 100, g.h / 100, color=fs.LIGHT,
                               lw=0))
        ax.add_patch(Rectangle((o.x / 100, o.y / 100), o.w / 100, o.h / 100, color=fs.GREY,
                               lw=0))
    for s in plan['stations']:
        a, b = s['window']
        ax.axvspan(a, b, ymin=0, ymax=0.05, color=fs.BLUE if s['index'] % 2 == 0 else fs.SKY,
                   lw=0)
        ax.text((a + b) / 2, 0.012, f"{s['index']}", ha='center', va='bottom', fontsize=7,
                color='white' if s['index'] % 2 == 0 else fs.BLACK)
    for k, (pth, cen) in enumerate(zip(paths, centers)):
        ax.plot(pth[:, 0], pth[:, 1], '-', color=fs.VERMILLION, lw=4, alpha=0.35,
                solid_capstyle='butt',
                label=(f'Planned slot (right-angle polyline, width '
                       f'{plan["slot_spec"]["width"] * 1000:.0f} mm)') if k == 0 else None)
        ax.plot(cen[:, 0], cen[:, 1], color=fs.VERMILLION, lw=0.6, ls=(0, (3, 1.5)),
                label='Conduit centre line (R 12 cm bends, schematic)' if k == 0 else None)
        ax.plot(*pth[0], 's', color=fs.BLACK, ms=4)
        ax.plot(*pth[-1], 'o', color=fs.VERMILLION, ms=4)
    deep = [(float(r['u']), float(r['v'])) for r in trace
            if int(r['station']) >= 0 and r['phase'] == 'cut']
    if deep:
        d = np.array(deep)
        ax.plot(d[:, 0], d[:, 1], '.', color=fs.BLUE, ms=0.8, rasterized=True,
                label='Tool tip, Gazebo ground truth (cutting phase)')
    ax.set_xlim(u0, u1)
    ax.set_ylim(0, 0.95)
    ax.set_aspect('equal')
    ax.set_xlabel(fs.label('Along-wall position u', 'm'))
    ax.set_ylabel(fs.label('Height v', 'm'))
    ax.legend(loc='upper right', ncol=2)
    fs.panel_label(ax, 'a', f"room3 south wall: planned slots and executed tool-tip path "
                   f"({plan['partition']} / {plan['order']}, {len(plan['stations'])} stations)")

    # 拐弯处放大
    ax2 = fig.add_subplot(gs[1, 0])
    c = paths[0][2] if len(paths[0]) > 2 else paths[0][0]
    for k, (pth, cen) in enumerate(zip(paths, centers)):
        ax2.plot(pth[:, 0], pth[:, 1], '-', color=fs.VERMILLION, lw=1.0,
                 label='Planned slot' if k == 0 else None)
        ax2.plot(cen[:, 0], cen[:, 1], color=fs.VERMILLION, lw=0.6, ls=(0, (3, 1.5)),
                 label='Conduit bend' if k == 0 else None)
    if deep:
        ax2.plot(d[:, 0], d[:, 1], '.', color=fs.BLUE, ms=1.2, rasterized=True,
                 label='Tool tip')
    ax2.set_xlim(c[0] - 0.16, c[0] + 0.04)
    ax2.set_ylim(c[1] - 0.16, c[1] + 0.04)
    ax2.set_aspect('equal')
    ax2.set_xlabel(fs.label('u', 'm'))
    ax2.set_ylabel(fs.label('v', 'm'))
    ax2.legend(loc='upper left')
    fs.panel_label(ax2, 'b', 'Corner (zoom)')

    idx = [int(s['station']) for s in stations]
    ax3 = fig.add_subplot(gs[1, 1])
    for k, role in (('lateral_mm', 'proposed'), ('standoff_mm', 'alt2')):
        if k in stations[0]:
            ax3.plot(idx, [float(s[k]) for s in stations],
                     label={'lateral_mm': 'Along-wall', 'standoff_mm': 'Stand-off'}[k],
                     **fs.line(role, marker=True))
    ax3.axhline(0, color=fs.GREY, lw=0.5)
    ax3.set_xlabel('Station')
    ax3.set_ylabel(fs.label('Arrival error (ground truth)', 'mm'))
    ax3.legend(loc='best')
    fs.panel_label(ax3, 'c', 'Arrival error')

    ax4 = fig.add_subplot(gs[1, 2])
    ax4.bar(idx, [float(s['move_s']) for s in stations], label='Strafe + settle',
            **fs.bar('neutral'))
    has_cut = any(float(s['cut_s']) > 0 for s in stations)
    if has_cut:
        ax4.bar(idx, [float(s['cut_s']) for s in stations],
                bottom=[float(s['move_s']) for s in stations], label='Cutting',
                **fs.bar('proposed'))
    ax4.set_xlabel('Station')
    ax4.set_ylabel(fs.label('Simulation time', 's'))
    ax4.legend(loc='upper left')
    fs.panel_label(ax4, 'd', 'Time per station' if has_cut else 'Strafe + settle time')
    print('saved', fs.save(fig, out))


if __name__ == '__main__':
    main()
