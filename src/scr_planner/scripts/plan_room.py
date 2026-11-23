#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线房间规划（不需要 ROS）：房间任务 YAML → 房间级方案 + 每面墙的规划文件.

  python3 src/scr_planner/scripts/plan_room.py <room_task.yaml> <out_dir>
      [--method=optimal|nearest|fixed] [--time-model=traj_v1|...]
  python3 src/scr_planner/scripts/plan_room.py --generate=<seed> <room_task.yaml>

输出目录：
  room_plan.yaml       墙的访问顺序、每面墙的扫掠方向与入口标记、转场与作业的预测时间、
                       各基线的总时间、对施工的改动（替代方案）与跨墙角回路的处理方式
  wall_<name>.yaml     每面墙的规划文件，格式与 plan_wall_slots.py 的输出相同，执行器可逐面墙
                       直接执行（先把任务执行器的定位标记参数设为该墙的入口标记）

--generate：按种子生成一个随机房间任务（room_gen.residential_room）并写成 YAML。
方法说明见 python/room_plan.py 与 docs/experiments/room_planning。
"""
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
for d in (os.path.join(HERE, '..', 'python'),
          os.path.join(HERE, '..', '..', 'share', 'scr_planner', 'python')):
    if os.path.isdir(d):
        sys.path.insert(0, d)

import room_plan as rp  # noqa: E402
import slot_plan as sp  # noqa: E402
import task_plan as tp  # noqa: E402
from scr_planner_py import OrientedAStar  # noqa: E402


def plan_file(task_file, method=None, time_model=None):
    with open(task_file, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    opt = cfg.get('options', {})
    room = rp.room_from_config(cfg)
    robot = sp.RobotModel()
    tm = sp.TimeModel.preset(time_model or opt.get('time_model', 'traj_v1'))
    plan = rp.plan_room(room, robot, OrientedAStar(), tm, method=method or 'optimal')
    return plan, rp.plan_to_dicts(plan, robot, opt.get('duration', 2.0))


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    kw = dict(a[2:].split('=', 1) for a in sys.argv[1:] if a.startswith('--') and '=' in a)
    dump = lambda d, f: yaml.safe_dump(d, f, allow_unicode=True, sort_keys=False)  # noqa: E731
    if 'generate' in kw and args:
        import room_gen
        pl, robot = OrientedAStar(), sp.RobotModel()
        room = room_gen.residential_room(int(kw['generate']),
                                         routable=tp.standalone_routable(pl, robot))
        with open(args[0], 'w', encoding='utf-8') as f:
            dump(rp.room_to_config(room), f)
        print(f'{args[0]}: {room.meta}')
        return
    if len(args) < 2:
        print(__doc__)
        sys.exit(1)
    plan, (summary, files) = plan_file(args[0], kw.get('method'), kw.get('time-model'))
    os.makedirs(args[1], exist_ok=True)
    with open(os.path.join(args[1], 'room_plan.yaml'), 'w', encoding='utf-8') as f:
        dump(summary, f)
    for name, d in files.items():
        with open(os.path.join(args[1], f'wall_{name}.yaml'), 'w', encoding='utf-8') as f:
            dump(d, f)
    p = summary['prediction']
    print(f"{summary['status']}: {p['walls']} 面墙 {p['stations']} 个停靠站，预测 {p['t_total']} s"
          f"（作业 {p['t_service']} + 导航 {p['t_nav']} + 精对位 {p['t_align']}）")
    for s in summary['sequence']:
        print(f"  {s['step']}. {s['wall']}{s['direction']}  标记 u = {s['marker_u']} m，"
              f"转场 {s['nav_distance_m']} m，墙上 {s['t_wall']} s（{s['status']}）")
    for m, v in summary['methods'].items():
        print(f"  {m:22s} {v['t_total']:8.1f} s  {' '.join(v['sequence'])}")
    for c in summary['installer_changes']:
        print(f"  改动 {c['wall']}: {[x['label'] for x in c['changes']]}")
    for ln in summary['links']:
        print(f"  跨墙角回路 {ln['walls']}: {ln['method']}")
    sys.exit(0 if summary['status'] == 'success' else 2)


if __name__ == '__main__':
    main()
