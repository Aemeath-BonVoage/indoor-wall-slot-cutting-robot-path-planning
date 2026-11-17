#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多停靠站划分与访问顺序优化的离线对比实验.

随机生成 N 面墙（任务线盒都在机械臂可切高度内，每面墙 2–4 条槽），对同一组槽线比较：

  naive     每条槽各自贪心划分停靠站（站不共享），逐条槽执行、每条从左到右（基线）
  sweep     每条槽各自划分，但全部站按一维最优扫掠顺序执行（只优化顺序）
  greedy    全部槽合并后贪心划分（最少站数），扫掠顺序（共享停靠站）
  dp        全部槽合并后动态规划划分（最小化 站数×每站开销 + 点数×每点时间），扫掠顺序

用标定后的时间模型（slot_plan.TimeModel）预测总时间：Σ(t_settle + 横移距离/v) + 点数×t_point。

  PYTHONUNBUFFERED=1 python3 src/scr_planner/scripts/station_order_benchmark.py \
      --t-settle 12 --v-strafe 0.05 --t-point 2.0
"""
import argparse
import csv
import os
import random
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(HERE, '..', 'python'))

import slot_plan as sp  # noqa: E402
import wall_rules as wr  # noqa: E402
from scr_planner_py import OrientedAStar  # noqa: E402

CONFIGS = (('naive', 'per_slot', 'slot_by_slot'), ('sweep', 'per_slot', 'sweep'),
           ('greedy', 'greedy', 'sweep'), ('dp', 'dp', 'sweep'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--walls', type=int, default=100)
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--t-settle', type=float, default=sp.TimeModel.t_settle)
    ap.add_argument('--v-strafe', type=float, default=sp.TimeModel.v_strafe)
    ap.add_argument('--t-point', type=float, default=sp.TimeModel.t_point)
    ap.add_argument('--out', default=os.path.join(REPO, 'docs', 'experiments', 'slot_execution'))
    args = ap.parse_args()

    tm = sp.TimeModel(args.t_settle, args.v_strafe, args.t_point)
    robot = sp.RobotModel()
    band = tuple(b * 100 for b in robot.reach_band)
    planner = OrientedAStar()
    rng = random.Random(args.seed)
    wall = sp.WallFrame((10.0, 0.0), (5.0, 0.0))
    rows, done, skipped = [], 0, 0
    while done < args.walls:
        n_tasks = rng.choice((2, 3, 4))
        lb = rng.random() < 0.25
        scene = wr.random_scene(rng, load_bearing=lb, band=band, n_tasks=n_tasks)
        if scene is None:
            continue
        start_u = rng.uniform(0.3, scene.width / 100 - 0.3)    # 定位标记位置随机
        plans = {name: sp.plan_task(scene, wall, robot, planner, partition=part, order=order,
                                    start_u=start_u, tm=tm)
                 for name, part, order in CONFIGS}
        if any(p.status != 'success' for p in plans.values()):
            skipped += 1                                         # 任一条槽无合规路径：跳过整面墙
            continue
        for name, p in plans.items():
            rows.append(dict(wall=done, load_bearing=int(lb), slots=n_tasks,
                             slot_length_m=round(p.length, 3), method=name, **p.prediction))
        d = {n: plans[n].prediction for n in plans}
        print(f"[{done:3d}] {'承重' if lb else '非承重'} {n_tasks} 槽 "
              + ' '.join(f"{n}:{d[n]['stations']}站/{d[n]['t_total']:.0f}s" for n in d))
        done += 1

    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, 'station_order_results.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    print(f'\n墙数 {done}（跳过有槽无合规路径的 {skipped} 面），时间模型 {tm}')
    by = {n: [r for r in rows if r['method'] == n] for n, _, _ in CONFIGS}
    base = np.array([r['t_total'] for r in by['naive']])
    summary = []
    for n, part, order in CONFIGS:
        rs = by[n]
        t = np.array([r['t_total'] for r in rs])
        save = (base - t) / base * 100
        s = dict(method=n, partition=part, order=order,
                 stations_mean=round(np.mean([r['stations'] for r in rs]), 2),
                 points_mean=round(np.mean([r['points'] for r in rs]), 1),
                 travel_m_mean=round(np.mean([r['travel_m'] for r in rs]), 3),
                 t_total_mean=round(float(t.mean()), 1),
                 saving_vs_naive_mean_pct=round(float(save.mean()), 1),
                 saving_vs_naive_median_pct=round(float(np.median(save)), 1),
                 saving_vs_naive_max_pct=round(float(save.max()), 1))
        summary.append(s)
        print(s)
    g = np.array([r['t_total'] for r in by['greedy']])
    dpt = np.array([r['t_total'] for r in by['dp']])
    print(f"dp 相对 greedy：更好 {int((dpt < g - 1e-6).sum())} 面，相同 "
          f"{int((abs(dpt - g) <= 1e-6).sum())} 面，更差 {int((dpt > g + 1e-6).sum())} 面；"
          f"平均节省 {float(((g - dpt) / g * 100).mean()):.1f}%")
    with open(os.path.join(args.out, 'station_order_summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0].keys()))
        w.writeheader()
        w.writerows(summary)


if __name__ == '__main__':
    main()
