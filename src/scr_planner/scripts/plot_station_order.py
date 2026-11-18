#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""停靠站划分 / 顺序优化的对比图（docs/figstyle 统一样式，英文）.

  python3 src/scr_planner/scripts/plot_station_order.py <naive_run> <opt_run> <results.csv> \
      <out.png> [--time-model=NAME]

--time-model 是图中“模型预测”所用的时间模型预设，默认 align_v1：
docs/experiments/slot_execution 的运行与 station_order_results.csv 都在该标定下得到。
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

SLOT_NAMES = ('Slot A: outlet A to new outlet A', 'Slot B: outlet B to desk outlet B',
              'Slot C', 'Slot D')


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    opts = dict(a[2:].split('=', 1) for a in sys.argv[1:] if a.startswith('--') and '=' in a)
    naive_run, opt_run, results_csv, out = args[:4]
    time_model = opts.get('time-model', 'align_v1')
    import figstyle as fs

    fig = fs.figure(width='double', height=3.9)
    gs = fig.add_gridspec(2, 2, width_ratios=[1.7, 1])
    titles = {naive_run: 'Baseline: per-slot partition, slots executed one by one',
              opt_run: 'Optimised: joint partition of both slots (DP) + sweep order'}
    for row, run in enumerate((naive_run, opt_run)):
        ax = fig.add_subplot(gs[row, 0])
        with open(os.path.join(run, 'slot_plan.yaml'), encoding='utf-8') as f:
            plan = yaml.safe_load(f)
        with open(os.path.join(run, 'summary.yaml'), encoding='utf-8') as f:
            summ = yaml.safe_load(f)
        for k, s in enumerate(plan['slots']):
            p = np.array(s['cut_path'])
            ax.plot(p[:, 0], p[:, 1], '-', color=(fs.VERMILLION, fs.PURPLE)[k % 2], lw=3,
                    alpha=0.5, solid_capstyle='butt', label=SLOT_NAMES[k])
        lo, hi = plan['robot']['reach_band']
        for s in plan['stations']:
            a, b = s['window']
            ax.add_patch(fs.plt.Rectangle((a, lo), b - a, hi - lo, fill=False, lw=0.5,
                                          ec=fs.BLUE, alpha=0.7))
        us = [plan['marker_u']] + [s['u_c'] for s in plan['stations']]
        ys = np.linspace(0.12, 0.04, len(us))
        ax.plot(us, ys, '-o', color=fs.BLUE, ms=2.2, lw=0.7, label='Station sequence')
        ax.plot(us[0], ys[0], '*', color=fs.ORANGE, ms=5, label='Reference marker (start)')
        # 站序号：每一趟扫掠（u 递增的一段）交替标在点的上方 / 下方，避免两趟的序号重叠
        sweep = 0
        for k, (u, y) in enumerate(zip(us[1:], ys[1:])):
            if k > 0 and u < us[k]:
                sweep += 1
            ty, va = (0.165, 'bottom') if sweep % 2 == 0 else (0.01, 'top')
            ax.text(u, ty, str(k), ha='center', va=va, fontsize=7, color=fs.BLUE)
        ax.set_xlim(min(us) - 0.15, max(us) + 0.2)
        ax.set_ylim(-0.1, 1.08)
        ax.set_aspect('equal')
        ax.set_ylabel(fs.label('Height v', 'm'))
        fs.panel_label(ax, 'ab'[row], titles[run])
        fs.note(ax, f"{len(plan['stations'])} stations, strafe "
                    f"{plan['prediction']['travel_m']:.2f} m\n"
                    f"Gazebo {summ['stations_total_s']:.0f} s "
                    f"(model {sum_pred(plan, time_model):.0f} s)",
                x=0.99, y=1.0)
        if row == 1:
            ax.set_xlabel(fs.label('Along-wall position u', 'm'))
            ax.legend(loc='lower left', bbox_to_anchor=(0, -0.62), ncol=2)

    ax = fig.add_subplot(gs[:, 1])
    with open(results_csv) as f:
        rows = list(csv.DictReader(f))
    groups = [('0', '2'), ('0', '3'), ('0', '4'), ('1', '2'), ('1', '3'), ('1', '4')]
    labels = ['Part.\n2', 'Part.\n3', 'Part.\n4', 'Load-b.\n2', 'Load-b.\n3', 'Load-b.\n4']
    x = np.arange(len(groups))
    for k, (m, role, lab) in enumerate((('sweep', 'neutral', 'Order only'),
                                        ('greedy', 'alt', '+ shared stations (greedy)'),
                                        ('dp', 'proposed', '+ DP partition'))):
        vals, ns = [], []
        for lb, n in groups:
            sel = [r for r in rows if r['load_bearing'] == lb and r['slots'] == n]
            base = {r['wall']: float(r['t_total']) for r in sel if r['method'] == 'naive'}
            sv = [(base[r['wall']] - float(r['t_total'])) / base[r['wall']] * 100
                  for r in sel if r['method'] == m]
            vals.append(np.mean(sv) if sv else 0)
            ns.append(len(sv))
        ax.bar(x + (k - 1) * 0.27, vals, 0.26, label=lab, **fs.bar(role))
    for i, n in enumerate(ns):
        ax.text(x[i], -0.5, f'n={n}', ha='center', va='top', fontsize=7)
    ax.axhline(0, color=fs.BLACK, lw=0.6)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=7)
    ax.set_xlabel('Wall type (partition / load-bearing) and number of slots')
    ax.set_ylabel(fs.label('Total-time saving vs. baseline', '%'))
    ax.set_ylim(-2.5, max(15, ax.get_ylim()[1]))
    ax.spines['bottom'].set_visible(False)
    ax.tick_params(axis='x', length=0)
    ax.legend(loc='upper left')
    fs.panel_label(ax, 'c', '100 random walls (calibrated time model)')
    print('saved', fs.save(fig, out))


def sum_pred(plan, time_model='align_v1'):
    import slot_plan as sp
    p, _, _ = sp.plan_from_dict(plan)
    return sp.predict_time(p.stations, p.start_u, sp.TimeModel.preset(time_model))['t_total']


if __name__ == '__main__':
    main()
