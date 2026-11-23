#!/usr/bin/env python3
"""
离线检查：可加的时间模型能否代替轨迹生成器的逐段时长（不需要 ROS / Gazebo）.

  python3 docs/experiments/arm_trajectory/model_check.py [--walls 60]

规划器的目标函数、停靠站 DP 与下界都要求机械臂时间对“点数”和“切割长度”可加：
    T_arm ≈ 点数 × t_point + 切割长度 / v_feed
而真实的轨迹时长是各段 S 曲线时长之和，其中站内换位的时长随距离变化。这里对
wall_gen.residential_wall 生成的墙（规则路线 + DP 划分，与 task_planning_benchmark 相同的
种子）逐站用 arm_trajectory 生成名义轨迹，比较两者，并给出只对点数回归得到的 t_point。
输出 model_check.csv（逐站）并打印统计量。
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'src', 'scr_planner', 'python'))
sys.path.insert(0, os.path.join(REPO, 'src', 'scr_planner', 'scripts'))
sys.path.insert(0, os.path.join(REPO, 'src', 'scr_arm_controller'))

import slot_plan as sp  # noqa: E402
import task_planning_benchmark as bm  # noqa: E402
from scr_arm_controller.arm_trajectory import MotionLimits, plan_trajectory  # noqa: E402

OBSERVE = (-0.15, 0.0, 0.02, 0.0)       # 横移前机械臂所在的观测位姿（关节值）
DRY_RUN_GAP = 0.01


def station_rows(plan, robot, lim):
    xc = robot.x_cut(plan.slot_depth)
    x_dry = robot.standoff - DRY_RUN_GAP - robot.tool_offset
    out = []
    for st in plan.stations:
        pts, cut = [], []
        for p in st.points:
            c = abs(p['x'] - xc) < 1e-6
            x = x_dry if c else p['x']
            pts.append((p['z'] - robot.z0, -(p['y'] - robot.y0), -(x - robot.x0),
                        math.radians(p['rotation'])))
            cut.append(c)
        traj = plan_trajectory(OBSERVE, pts, cut, lim)
        rapid = traj.summary().get('rapid', (0, 0.0, 0.0))
        out.append(dict(points=len(pts), cut_m=st.cut_length, pieces=len(st.pieces),
                        traj_s=traj.duration, rapid_s=rapid[1]))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('--walls', type=int, default=60)
    ap.add_argument('--seed0', type=int, default=1000)
    args = ap.parse_args()
    bm._init()
    robot, planner = bm._G['robot'], bm._G['planner']
    tm, lim = sp.TimeModel(), MotionLimits()
    rows = []
    for seed in range(args.seed0, args.seed0 + args.walls):
        scene, _, start_u = bm.make_scene(seed)
        plan = sp.assemble_plan(sp.plan_routes(scene, robot, planner), bm.WALL, robot, 'dp',
                                'sweep', start_u, tm)
        if plan.status == 'success':
            rows += [dict(seed=seed, **r) for r in station_rows(plan, robot, lim)]
    with open(os.path.join(HERE, 'model_check.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows({k: (f'{v:.4f}' if isinstance(v, float) else v) for k, v in r.items()}
                    for r in rows)
    n = np.array([r['points'] for r in rows], float)
    y = np.array([r['traj_s'] for r in rows]) - np.array([r['cut_m'] for r in rows]) / lim.feed
    t_point = float(n @ y / (n @ n))                    # 过原点的最小二乘
    e = n * t_point - y
    print(f'{len(rows)} stations on {len({r["seed"] for r in rows})} walls')
    print(f'additive fit: t_point = {t_point:.3f} s; per-station RMSE '
          f'{np.sqrt(np.mean(e**2)):.2f} s, max {np.abs(e).max():.2f} s')
    if math.isfinite(tm.v_feed):
        e = n * tm.t_point - y
        print(f'preset (t_point = {tm.t_point:.3f} s): per-station RMSE '
              f'{np.sqrt(np.mean(e**2)):.2f} s, mean bias {e.mean():+.2f} s')


if __name__ == '__main__':
    main()
