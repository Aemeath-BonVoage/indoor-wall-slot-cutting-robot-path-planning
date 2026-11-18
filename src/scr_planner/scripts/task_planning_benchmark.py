#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务级联合规划的离线对比实验（不需要 ROS / Gazebo）.

对 wall_gen.residential_wall(seed) 生成的每面墙（任务各自单独可行）比较：

  per_slot      规则路线（逐条槽规范代价最优）+ 每条槽各自贪心划分 + 逐条槽执行（基线）
  current       规则路线 + 全部槽合并 DP 划分 + 一维最优扫掠顺序（现有方法）
  current_v0    同 current，但用修正前的评估：不剔除窗口截断残片（slot_plan.SLIVER = 0）、
                DP 不计横移、站位固定在窗口左端（与 slot_execution 报告中的流程相同）
  lattice_indep 站感知选线：各槽独立、取最优相位，不共享窗口、不做坐标下降（消融）
  lattice_seq   站感知选线 + 共享已打开窗口，逐条规划、不做坐标下降（消融）
  lattice_cd    站感知选线 + 共享窗口 + 两轮坐标下降，不含规则路线候选（消融）
  joint         lattice_cd 的全部候选 + 规则路线，取预测时间最短者（本文方法）
  lower_bound   预测时间的下界（task_plan.lower_bound）

另外逐条槽记录经典 Lee 布线（逐格 Dijkstra + 转弯代价）与 OrientedAStar 的对照。

  python3 src/scr_planner/scripts/task_planning_benchmark.py --scenes 400 --jobs 2
  python3 src/scr_planner/scripts/task_planning_benchmark.py --sensitivity --scenes 120
  python3 src/scr_planner/scripts/task_planning_benchmark.py --time-model align_v1 --out <dir>

--time-model：时间模型预设（slot_plan.TIME_MODELS），默认 align_v3（当前默认执行路径的标定）；
  data/ 下的原始结果用 align_v1（第一版精对位下的标定）生成，align_v2/data/ 用 align_v2，
  align_v3/data/ 用 align_v3。

输出（默认 docs/experiments/task_planning/<预设>/data/；align_v1 为 data/）：
  results.csv（每面墙 × 每种方法一行）、lee.csv（每条槽一行）、sensitivity.csv
"""
import argparse
import csv
import functools
import multiprocessing as mp
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
for d in (os.path.join(HERE, '..', 'python'),
          os.path.join(HERE, '..', '..', 'share', 'scr_planner', 'python')):
    if os.path.isdir(d):
        sys.path.insert(0, d)

import slot_plan as sp  # noqa: E402
import task_plan as tp  # noqa: E402
import wall_gen as wg  # noqa: E402
import wall_rules as wr  # noqa: E402
from scr_planner_py import OrientedAStar  # noqa: E402

WALL = sp.WallFrame((0.0, 0.0), (6.0, 0.0))
_G = {}


def _init():
    _G['planner'] = OrientedAStar()
    _G['robot'] = sp.RobotModel()


def make_scene(seed):
    pl, robot = _G['planner'], _G['robot']
    scene, meta = wg.residential_wall(seed, routable=tp.standalone_routable(pl, robot))
    # 定位标记（停靠站横移的起点）在墙上的位置随机
    start_u = random.Random(seed * 7919 + 1).uniform(0.3, scene.width / 100 - 0.3)
    return scene, meta, start_u


def legacy_plan(slots, robot, start_u, tm):
    """修正前的“规则路线 + DP 划分 + 扫掠”流程（只用于量化评估修正本身的影响）."""
    sliver, sp.SLIVER = sp.SLIVER, 0.0
    try:
        band = robot.reach_band
        plan = sp.SlotPlan(status='success', slots=slots, partition='dp', order='sweep',
                           start_u=start_u)
        if any(s.status != 'success' for s in slots):
            plan.status = 'no path'
            return plan
        pieces = [q for s in slots for q in sp.clip_polyline(s.cut_path, band[0], band[1], 1)]
        wins = sp.partition_dp(pieces, robot, tm)
        st = sp.build_stations(wins, pieces, WALL, robot, plan.slot_depth)
        plan.stations = sp.order_stations(st, start_u, 'sweep')
        plan.prediction = sp.predict_time(plan.stations, start_u, tm)
        return plan
    finally:
        sp.SLIVER = sliver


def exact_time(plan, tm):
    """与 slot_plan.predict_time 相同的模型，但不取整（用于和下界比较）."""
    u, t = plan.start_u, 0.0
    for st in plan.stations:
        t += tm.move(st.u_c - u) + tm.t_point * len(st.points)
        u = st.u_c
    return t


def _row(meta, name, plan, slots, ms, scene, robot, extra=None, tm=None):
    ok = plan is not None and plan.status == 'success'
    r = dict(seed=meta['seed'], load_bearing=int(meta['load_bearing']), slots=meta['n_slots'],
             weak=sum(c == 'weak' for c in scene.circuits), method=name, feasible=int(ok),
             plan_ms=round(ms, 1))
    if ok:
        v = tp.plan_violations(scene, slots, robot)
        r.update({k: plan.prediction[k] for k in ('stations', 'points', 'travel_m', 't_move',
                                                  't_cut', 't_total')})
        r.update(t_exact=round(exact_time(plan, tm or sp.TimeModel()), 4),
                 length_m=round(plan.length, 3),
                 turns=sum(len(s.cut_path) - 2 for s in slots),
                 violations=sum(v.values()))
    r.update(extra or {})
    return r


def run_scene(seed, tm=None, methods=None):
    pl, robot = _G['planner'], _G['robot']
    tm = tm or sp.TimeModel()
    scene, meta, su = make_scene(seed)
    rows = []

    def timed(fn):
        t0 = time.perf_counter()
        out = fn()
        return out, (time.perf_counter() - t0) * 1000

    want = (lambda m: methods is None or m in methods)
    (rule, ms_rule) = timed(lambda: sp.plan_routes(scene, robot, pl))
    for name, part, order in (('per_slot', 'per_slot', 'slot_by_slot'),
                              ('current', 'dp', 'sweep')):
        if want(name):
            plan, ms = timed(lambda: sp.assemble_plan(rule, WALL, robot, part, order, su, tm))
            rows.append(_row(meta, name, plan, rule, ms + ms_rule, scene, robot, tm=tm))
    if want('current_v0'):
        plan, ms = timed(lambda: legacy_plan(rule, robot, su, tm))
        rows.append(_row(meta, 'current_v0', plan, rule, ms + ms_rule, scene, robot, tm=tm))
    variants = (('lattice_indep', dict(share=False, passes=0, include_rule=False)),
                ('lattice_seq', dict(share=True, passes=0, include_rule=False)),
                ('lattice_cd', dict(share=True, passes=2, include_rule=False)),
                ('joint', dict(share=True, passes=2, include_rule=True)))
    for name, kw in variants:
        if not want(name):
            continue
        st = {}
        (slots, plan), ms = timed(lambda: tp.joint_routes(scene, WALL, robot, pl, tm, su,
                                                          stats=st, **kw))
        rows.append(_row(meta, name, plan, slots, ms, scene, robot,
                         dict(searches=st.get('searches'), evaluations=st.get('evaluations'),
                              source=st.get('source', '')), tm=tm))
    if want('lower_bound'):
        lb, ms = timed(lambda: tp.lower_bound(scene, robot, pl, tm, su))
        r = dict(seed=seed, load_bearing=int(meta['load_bearing']), slots=meta['n_slots'],
                 weak=sum(c == 'weak' for c in scene.circuits), method='lower_bound',
                 feasible=int(lb is not None), plan_ms=round(ms, 1))
        if lb:
            r.update(stations=lb['stations'], points=lb['points'],
                     travel_m=round(lb['travel_m'], 4), t_total=round(lb['t_total'], 1),
                     t_exact=round(lb['t_total'], 4))
        rows.append(r)
    return rows


def run_lee(seed):
    """每条槽单独（他槽线盒为障碍）：Lee 布线与 OrientedAStar 的代价、弯数、违规."""
    pl, robot = _G['planner'], _G['robot']
    scene, meta, _ = make_scene(seed)
    rows = []
    for k, (s, g, _, _) in enumerate(sp.scene_tasks(scene)):
        grid = sp.route_grid(scene, k, [], robot)
        rules = scene.params()
        relaxed = scene.params(rules=False)
        relaxed.update({kk: rules[kk] for kk in ('weight_x', 'cost_weight', 'allow_x')
                        if kk in rules})
        a_rel = pl.plan(grid, s, g, **relaxed)
        a_rule = pl.plan(grid, s, g, **rules)
        lee = tp.lee_route(grid, s, g, **relaxed)
        minbend = dict(relaxed, turn_penalty=1000.0)
        lee_mb = tp.lee_route(grid, s, g, **minbend)
        circ = scene.circuit_of(k)

        def viol(corners):
            return sum(scene.violations(wr.densify(corners), circ).values())

        rows.append(dict(
            seed=seed, slot=k, load_bearing=int(meta['load_bearing']),
            astar_relaxed_cost=round(a_rel['cost'], 6), lee_cost=round(lee['cost'], 6),
            astar_relaxed_ms=round(a_rel['time_ms'], 2), lee_ms=round(lee['time_ms'], 1),
            astar_rule_ok=int(a_rule['ok']), astar_rule_cost=round(a_rule['cost'], 6),
            astar_rule_turns=a_rule['turns'], astar_rule_viol=viol(a_rule['corners']),
            lee_turns=lee['turns'], lee_viol=viol(lee['corners']),
            lee_minbend_turns=lee_mb['turns'], lee_minbend_viol=viol(lee_mb['corners']),
            astar_rule_len=sum(abs(b[0] - a[0]) + abs(b[1] - a[1]) for a, b in
                               zip(a_rule['corners'][:-1], a_rule['corners'][1:])),
            lee_len=sum(abs(b[0] - a[0]) + abs(b[1] - a[1]) for a, b in
                        zip(lee['corners'][:-1], lee['corners'][1:]))))
    return rows


SENS = (('calibrated', 1.0, 1.0, 1.0), ('settle_x2', 2.0, 1.0, 1.0),
        ('settle_x0.5', 0.5, 1.0, 1.0), ('point_x0.5', 1.0, 1.0, 0.5),
        ('strafe_x2', 1.0, 2.0, 1.0))


def run_sens(seed, tm=None):
    base = tm or sp.TimeModel()
    out = []
    for name, ks, kv, kp in SENS:
        tm = sp.TimeModel(base.t_settle * ks, base.v_strafe * kv, base.t_point * kp)
        for r in run_scene(seed, tm, methods=('current', 'joint', 'lower_bound')):
            r['model'] = name
            out.append(r)
    return out


def write(path, rows):
    keys = []
    for r in rows:
        keys += [k for k in r if k not in keys]
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenes', type=int, default=400)
    ap.add_argument('--seed0', type=int, default=1000)
    ap.add_argument('--jobs', type=int, default=2)
    ap.add_argument('--sensitivity', action='store_true')
    ap.add_argument('--time-model', default='align_v3', choices=sorted(sp.TIME_MODELS))
    ap.add_argument('--out', default=None)
    args = ap.parse_args()
    tm = sp.TimeModel.preset(args.time_model)
    args.out = args.out or os.path.join(
        REPO, 'docs', 'experiments', 'task_planning',
        *(('data',) if args.time_model == 'align_v1' else (args.time_model, 'data')))
    print(f'时间模型 {args.time_model}：{tm}', flush=True)
    os.makedirs(args.out, exist_ok=True)
    seeds = list(range(args.seed0, args.seed0 + args.scenes))
    t0 = time.time()
    with mp.Pool(min(args.jobs, 2), initializer=_init) as pool:
        if args.sensitivity:
            rows = [r for rs in pool.imap(functools.partial(run_sens, tm=tm), seeds) for r in rs]
            write(os.path.join(args.out, 'sensitivity.csv'), rows)
        else:
            rows = []
            for i, rs in enumerate(pool.imap(functools.partial(run_scene, tm=tm), seeds)):
                rows += rs
                if i % 20 == 0:
                    d = {r['method']: r.get('t_total') for r in rs
                         if r['method'] in ('current', 'joint', 'lower_bound')}
                    print(f'[{i:4d}] {time.time() - t0:6.0f} s  {d}', flush=True)
            write(os.path.join(args.out, 'results.csv'), rows)
            lee = [r for rs in pool.imap(run_lee, seeds) for r in rs]
            write(os.path.join(args.out, 'lee.csv'), lee)
    print(f'{len(seeds)} 面墙，{time.time() - t0:.0f} s')


if __name__ == '__main__':
    main()
