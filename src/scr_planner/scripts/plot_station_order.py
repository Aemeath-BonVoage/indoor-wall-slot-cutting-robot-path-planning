#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""停靠站划分 / 顺序优化的对比图.

  python3 src/scr_planner/scripts/plot_station_order.py <naive_run> <opt_run> <results.csv> <out.png>
"""
import csv
import os
import sys

import numpy as np
import yaml


def main():
    naive_run, opt_run, results_csv, out = sys.argv[1:5]
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    matplotlib.rcParams['font.sans-serif'] = ['Noto Sans CJK SC', 'Noto Sans CJK JP',
                                              'Noto Serif CJK JP', 'DejaVu Sans']
    matplotlib.rcParams['axes.unicode_minus'] = False

    fig = plt.figure(figsize=(14, 6.4))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.6, 1])
    titles = {naive_run: '基线：每条槽各自划分，逐条槽执行',
              opt_run: '优化：两条槽合并划分（DP）+ 最优扫掠顺序'}
    for row, run in enumerate((naive_run, opt_run)):
        ax = fig.add_subplot(gs[row, 0])
        plan = yaml.safe_load(open(os.path.join(run, 'slot_plan.yaml'), encoding='utf-8'))
        st = list(csv.DictReader(open(os.path.join(run, 'stations.csv'))))
        summ = yaml.safe_load(open(os.path.join(run, 'summary.yaml'), encoding='utf-8'))
        for k, s in enumerate(plan['slots']):
            p = np.array(s['cut_path'])
            ax.plot(p[:, 0], p[:, 1], '-', color=('#C00000', '#7030A0')[k], lw=5, alpha=0.45,
                    label=s['label'])
        lo, hi = plan['robot']['reach_band']
        for s in plan['stations']:
            a, b = s['window']
            ax.add_patch(plt.Rectangle((a, lo), b - a, hi - lo, fill=False, lw=0.8,
                                       ec='#1F4E9C', alpha=0.6))
        us = [plan['marker_u']] + [s['u_c'] for s in plan['stations']]
        ys = np.linspace(0.18, 0.08, len(us))
        ax.plot(us, ys, '-o', color='#1F4E9C', ms=3, lw=1)
        ax.plot(us[0], ys[0], '*', color='#E69138', ms=12, label='定位标记（起点）')
        for k, (u, y) in enumerate(zip(us[1:], ys[1:])):
            ax.text(u, y - 0.03, str(k), ha='center', fontsize=6, color='#1F4E9C')
        ax.set_xlim(min(us) - 0.15, max(us) + 0.2)
        ax.set_ylim(0.0, 0.9)
        ax.set_aspect('equal')
        ax.set_ylabel('离地 v (m)')
        ax.set_title(f"{titles[run]}\n{len(plan['stations'])} 站，横移 {plan['prediction']['travel_m']:.2f} m，"
                     f"Gazebo 实测 {summ['stations_total_s']:.0f} s（模型 {sum_pred(plan):.0f} s）",
                     fontsize=9)
        if row == 1:
            ax.set_xlabel('沿墙 u (m)')
        ax.legend(fontsize=7, loc='upper right', ncol=3)

    ax = fig.add_subplot(gs[:, 1])
    rows = list(csv.DictReader(open(results_csv)))
    groups = [('0', '2'), ('0', '3'), ('0', '4'), ('1', '2'), ('1', '3'), ('1', '4')]
    labels = ['非承重\n2 槽', '非承重\n3 槽', '非承重\n4 槽', '承重\n2 槽', '承重\n3 槽', '承重\n4 槽']
    x = np.arange(len(groups))
    for k, (m, col, lab) in enumerate((('sweep', '#999999', '只优化顺序'),
                                       ('greedy', '#6FA8DC', '+ 共享停靠站（贪心）'),
                                       ('dp', '#1F4E9C', '+ DP 划分'))):
        vals, ns = [], []
        for lb, n in groups:
            sel = [r for r in rows if r['load_bearing'] == lb and r['slots'] == n]
            base = {r['wall']: float(r['t_total']) for r in sel if r['method'] == 'naive'}
            sv = [(base[r['wall']] - float(r['t_total'])) / base[r['wall']] * 100
                  for r in sel if r['method'] == m]
            vals.append(np.mean(sv) if sv else 0)
            ns.append(len(sv))
        ax.bar(x + (k - 1) * 0.27, vals, 0.26, color=col, label=lab)
    for i, n in enumerate(ns):
        ax.text(x[i], -1.2, f'n={n}', ha='center', fontsize=7)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylabel('总时间相对基线的节省 (%)')
    ax.set_ylim(-2, max(15, ax.get_ylim()[1]))
    ax.set_title('100 面随机墙（标定后的时间模型）', fontsize=9)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print('saved', out)


def sum_pred(plan):
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'python'))
    import slot_plan as sp
    p, _, _ = sp.plan_from_dict(plan)
    return sp.predict_time(p.stations, p.start_u, sp.TimeModel())['t_total']


if __name__ == '__main__':
    main()
