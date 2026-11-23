#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""无合规方案墙面的诊断与替代方案评估（离线，不需要 ROS / Gazebo）.

取 task_planning 基准（种子 1000–1399）中联合规划无解的墙面，对每面墙：
  1. 诊断（plan_alternatives.diagnose）：最少拿掉几条槽、冲突的回路组合、放松哪条规则才可行；
  2. 逐级枚举替代方案（换引线点 / 走地 / 走顶 / 移动线盒），每一级取排序第一的方案，
     用联合规划求预测时间，并用独立检查器统计违规。

  python3 src/scr_planner/scripts/infeasible_walls_benchmark.py [--jobs 2] [--out <dir>]
      [--results docs/experiments/task_planning/data/results.csv] [--time-model traj_v1]

输出（默认 docs/experiments/room_planning/data/）：
  infeasible_walls.csv         每面墙一行：诊断结果与排序第一的替代方案
  infeasible_alternatives.csv  每面墙 × 每一级一行
"""
import argparse
import csv
import multiprocessing as mp
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
for d in (os.path.join(HERE, '..', 'python'),
          os.path.join(HERE, '..', '..', 'share', 'scr_planner', 'python')):
    if os.path.isdir(d):
        sys.path.insert(0, d)

import plan_alternatives as pa  # noqa: E402
import slot_plan as sp  # noqa: E402
import task_plan as tp  # noqa: E402
import wall_gen as wg  # noqa: E402
from scr_planner_py import OrientedAStar  # noqa: E402

WALL = sp.WallFrame((0.0, 0.0), (6.0, 0.0))
_G = {}


def _init(tm_name):
    _G.update(planner=OrientedAStar(), robot=sp.RobotModel(), tm=sp.TimeModel.preset(tm_name))


def run(seed):
    pl, robot, tm = _G['planner'], _G['robot'], _G['tm']
    scene, meta = wg.residential_wall(seed, routable=tp.standalone_routable(pl, robot))
    t0 = time.perf_counter()
    diag = pa.diagnose(scene, robot, pl)
    t_diag = time.perf_counter() - t0
    wall = dict(seed=seed, slots=meta['n_slots'], load_bearing=int(meta['load_bearing']),
                weak=sum(c == 'weak' for c in scene.circuits),
                unused_feeds=len(pa.unused_feeds(scene)),
                rule_any_order=int(diag['feasible']), diag_s=round(t_diag, 2))
    if not diag['feasible']:
        wall.update(min_remove=diag['min_remove'],
                    blockers=' '.join(map(str, diag['blockers'])),
                    core=' '.join(map(str, diag['core'])), conflict=diag['conflict'],
                    relax=' '.join(diag['relax']), cause=diag['cause'])
    t0 = time.perf_counter()
    st = {}
    alts = pa.alternatives(scene, robot, pl, stats=st)
    t_alt = time.perf_counter() - t0
    rows = []
    for tier in pa.TIERS:
        mine = [a for a in alts if pa.TIERS[a.tier] == tier]
        r = dict(seed=seed, slots=meta['n_slots'], tier=tier, feasible=int(bool(mine)),
                 n_feasible=len(mine))
        if mine:
            a = mine[0]
            t1 = time.perf_counter()
            plan = pa.finalize(a, WALL, robot, pl, tm)
            s = a.summary()
            v = pa.check(a, scene, robot)
            r.update(changed=len(s['changed_slots']), moved_cm=s['moved_cm'],
                     conduit_m=s['conduit_m'], manual_m=s['manual_m'],
                     robot_stub_m=s['robot_stub_m'], installer_cost=s['installer_cost'],
                     robot_slots=len(plan.slots), stations=plan.prediction['stations'],
                     t_pred=plan.prediction['t_total'], length_m=round(plan.length, 3),
                     violations=sum(v.values()), status=plan.status,
                     plan_s=round(time.perf_counter() - t1, 2),
                     what=' | '.join(c.label for c in a.changes))
        rows.append(r)
    best = next((r for t in pa.TIERS for r in rows if r['tier'] == t and r['feasible']), None)
    wall.update(alternatives=len(alts), searches=st.get('searches', 0), alt_s=round(t_alt, 2),
                best_tier=best['tier'] if best else '',
                best_cost=best['installer_cost'] if best else '',
                best_t_pred=best['t_pred'] if best else '',
                best_what=best['what'] if best else '')
    return wall, rows


def write(path, rows):
    keys = []
    for r in rows:
        keys += [k for k in r if k not in keys]
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--results', default=os.path.join(REPO, 'docs', 'experiments', 'task_planning',
                                                      'data', 'results.csv'))
    ap.add_argument('--out', default=os.path.join(REPO, 'docs', 'experiments', 'room_planning',
                                                  'data'))
    ap.add_argument('--jobs', type=int, default=2)
    ap.add_argument('--time-model', default='traj_v1', choices=sorted(sp.TIME_MODELS))
    ap.add_argument('--limit', type=int, default=0)
    args = ap.parse_args()
    with open(args.results) as f:
        seeds = sorted(int(r['seed']) for r in csv.DictReader(f)
                       if r['method'] == 'joint' and r['feasible'] == '0')
    if args.limit:
        seeds = seeds[:args.limit]
    os.makedirs(args.out, exist_ok=True)
    t0 = time.time()
    walls, alts = [], []
    with mp.Pool(min(args.jobs, 2), initializer=_init, initargs=(args.time_model,)) as pool:
        for i, (w, rows) in enumerate(pool.imap(run, seeds)):
            walls.append(w)
            alts += rows
            if i % 10 == 0:
                print(f'[{i:3d}] {time.time() - t0:5.0f} s  seed {w["seed"]} '
                      f'{w.get("cause", "")} -> {w["best_tier"]}', flush=True)
    write(os.path.join(args.out, 'infeasible_walls.csv'), walls)
    write(os.path.join(args.out, 'infeasible_alternatives.csv'), alts)
    print(f'{len(seeds)} 面墙，{time.time() - t0:.0f} s')


if __name__ == '__main__':
    main()
