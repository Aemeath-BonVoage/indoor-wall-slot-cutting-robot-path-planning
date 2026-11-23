#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""房间级规划的离线对比实验（不需要 ROS / Gazebo）.

对 room_gen.residential_room(seed) 生成的每个房间：落实跨墙角回路 → 每面墙联合规划（无解时
取替代方案）→ 用五种方法排墙的访问顺序与扫掠方向（room_plan.METHODS）：

  fixed                 按墙的列出顺序（顺时针），每面墙一律 + 方向（基线）
  nearest               最近邻：每步去转场时间最短的 (墙, 方向)（基线）
  fixed_order_best_dir  固定顺序 + 最优方向（消融）
  best_order_fixed_dir  最优顺序 + 一律 + 方向（消融）
  optimal               子集动态规划的精确最优（本文方法）

  python3 src/scr_planner/scripts/room_planning_benchmark.py --rooms 200 --seed0 0 --jobs 2

输出（默认 docs/experiments/room_planning/data/）：
  rooms.csv   每个房间一行：各方法的预测总时间、最优方案的时间构成、规划耗时、合规检查
  walls.csv   每个房间 × 每面有任务的墙一行

可中断续跑：每个房间算完后写入 <out>/cache/room_<seed>.json，再次运行时已有缓存的房间直接
读取（--fresh 忽略缓存重算）。缓存不提交；规划耗时按墙钟计，机器忙或进程被挂起时不可用。
"""
import argparse
import csv
import json
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

import room_gen as rg  # noqa: E402
import room_plan as rp  # noqa: E402
import slot_plan as sp  # noqa: E402
import task_plan as tp  # noqa: E402
from scr_planner_py import OrientedAStar  # noqa: E402

_G = {}


def _init(tm_name):
    _G.update(planner=OrientedAStar(), robot=sp.RobotModel(), tm=sp.TimeModel.preset(tm_name),
              transit=rp.TransitModel())


def run_cached(args):
    seed, cache, fresh = args
    path = os.path.join(cache, f'room_{seed}.json')
    if not fresh and os.path.exists(path):
        with open(path, encoding='utf-8') as f:
            return json.load(f)
    out = run(seed)
    with open(path + '.tmp', 'w', encoding='utf-8') as f:
        json.dump(out, f, ensure_ascii=False)
    os.replace(path + '.tmp', path)
    return out


def run(seed):
    pl, robot, tm, transit = _G['planner'], _G['robot'], _G['tm'], _G['transit']
    t0 = time.perf_counter()
    room = rg.residential_room(seed, routable=tp.standalone_routable(pl, robot))
    t_gen = time.perf_counter() - t0
    floor = rp.FloorMap(room.boundary, room.floor_obstacles)
    t0 = time.perf_counter()
    walls, fixed, protected, notes = rp.apply_links(room, robot, pl)
    t_link = time.perf_counter() - t0
    t0 = time.perf_counter()
    results = [rp.plan_wall(i, w, floor, robot, pl, tm, fixed[i], protected[i])
               for i, w in enumerate(walls)]
    t_walls = time.perf_counter() - t0
    t0 = time.perf_counter()
    active, inst = rp.build_instance(room, results, floor, transit)
    t_inst = time.perf_counter() - t0
    row = dict(room.meta, link_method=notes[0]['method'] if notes else '',
               link_manual_m=notes[0]['manual_m'] if notes else '',
               link_conduit_m=notes[0]['conduit_m'] if notes else '',
               active_walls=len(active),
               walls_success=sum(r.status == 'success' for r in results),
               walls_fallback=sum(r.status == 'fallback' for r in results),
               walls_infeasible=sum(r.status == 'infeasible' for r in results),
               walls_offwall_only=sum(r.status == 'no_task' and
                                      (walls[r.index].has_tasks or bool(fixed[r.index]))
                                      for r in results))
    for m in rp.METHODS:
        t1 = time.perf_counter()
        t, seq = rp.solve(inst, m)
        row[f't_{m}'] = round(t, 3)
        row[f'ms_{m}'] = round((time.perf_counter() - t1) * 1000, 3)
        if m == 'optimal':
            best = seq
    if len(active) <= 5:
        row['t_brute'] = round(rp.solve_brute(inst)[0], 3)
    plan = rp.sequence_room(room, results, floor, transit, 'optimal', notes)
    by = {r.index: r for r in results}
    viol = blocked = 0
    wall_rows = []
    for r in results:
        if r.status == 'no_task' and not walls[r.index].has_tasks:
            continue
        w = dict(seed=seed, wall=r.name, status=r.status, slots=len(walls[r.index].scene.tasks),
                 load_bearing=int(walls[r.index].scene.load_bearing),
                 length_m=round(walls[r.index].frame.length, 2), plan_s=round(r.plan_s, 2),
                 alternatives=r.n_alternatives)
        if r.alternative is not None and r.alternative.changes:
            s = r.alternative.summary()
            w.update(tier=s['tier'], installer_cost=s['installer_cost'], manual_m=s['manual_m'],
                     conduit_m=s['conduit_m'], moved_cm=s['moved_cm'])
        for d, o in r.options.items():
            v = r.violations(robot, d)
            w[f'service{d}'] = round(o.service, 2)
            w[f'viol{d}'] = sum(v.values())
            w['stations'] = len(o.plan.stations)
            viol += sum(v.values())
            blocked += o.blocked
        wall_rows.append(w)
    tot = plan.totals
    row.update(t_service=round(tot['t_service'], 2), t_nav=round(tot['t_nav'], 2),
               t_align=round(tot['t_align'], 2), nav_m=round(tot['nav_m'], 2),
               stations=tot['stations'], violations=viol, blocked_stations=blocked,
               sequence=' '.join(f'{by[i].name}{d}' for i, d in plan.sequence),
               direct_legs=sum(floor.line_free(a, b) for a, b in _legs(room, plan, by)),
               gen_s=round(t_gen, 2), link_s=round(t_link, 2), walls_s=round(t_walls, 2),
               instance_s=round(t_inst, 3))
    assert best == plan.methods['optimal']['seq']
    return row, wall_rows


def _legs(room, plan, by):
    prev = room.start_pose[:2]
    for i, d in plan.sequence:
        o = by[i].options[d]
        yield prev, o.entry
        prev = o.exit


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
    ap.add_argument('--rooms', type=int, default=200)
    ap.add_argument('--seed0', type=int, default=0)
    ap.add_argument('--jobs', type=int, default=2)
    ap.add_argument('--time-model', default='traj_v1', choices=sorted(sp.TIME_MODELS))
    ap.add_argument('--out', default=os.path.join(REPO, 'docs', 'experiments', 'room_planning',
                                                  'data'))
    ap.add_argument('--fresh', action='store_true')
    args = ap.parse_args()
    cache = os.path.join(args.out, 'cache')
    os.makedirs(cache, exist_ok=True)
    seeds = list(range(args.seed0, args.seed0 + args.rooms))
    jobs = [(s, cache, args.fresh) for s in seeds]
    t0 = time.time()
    rooms, walls = [], []
    with mp.Pool(min(args.jobs, 2), initializer=_init, initargs=(args.time_model,)) as pool:
        for i, (r, ws) in enumerate(pool.imap(run_cached, jobs)):
            rooms.append(r)
            walls += ws
            if i % 20 == 0:
                print(f'[{i:4d}] {time.time() - t0:5.0f} s  seed {r["seed"]}: fixed '
                      f'{r["t_fixed"]:.1f} nearest {r["t_nearest"]:.1f} optimal '
                      f'{r["t_optimal"]:.1f}', flush=True)
    write(os.path.join(args.out, 'rooms.csv'), rooms)
    write(os.path.join(args.out, 'walls.csv'), walls)
    print(f'{len(seeds)} 个房间，{time.time() - t0:.0f} s')


if __name__ == '__main__':
    main()
