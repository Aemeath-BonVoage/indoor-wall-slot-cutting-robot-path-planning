#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线开槽规划（不需要 ROS）：墙面任务 YAML → 规划结果 YAML（执行器只读这个文件）.

  python3 src/scr_planner/scripts/plan_wall_slots.py <task.yaml> <plan.yaml>
      [--routes=rule|joint] [--partition=dp|greedy|per_slot] [--order=sweep|slot_by_slot]

--routes=rule（默认）：逐条槽按开槽规范代价选线，再划分停靠站；
--routes=joint：槽线与停靠站联合规划，最小化预测执行时间（task_plan.py，
docs/experiments/task_planning）。两者输出格式相同，执行器无需区分。

规划结果包含：每条槽的正交折线、停靠站（按执行顺序）、每站底盘位姿与机械臂切割点、
预测执行时间。参考原点是 Gazebo 中的黄色定位标记（执行时先对位到它）。
"""
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
for d in (os.path.join(HERE, '..', 'python'),                       # 源码树
          os.path.join(HERE, '..', '..', 'share', 'scr_planner', 'python')):  # 安装树
    if os.path.isdir(d):
        sys.path.insert(0, d)

import slot_plan as sp  # noqa: E402
from scr_planner_py import OrientedAStar  # noqa: E402

# Gazebo room.world 中黄色定位标记（与 NavigationCuttingTask.marker_position 一致）
MARKER_XY = (8.42, -3.9245)


def plan_file(task_file, partition=None, order=None, routes=None):
    with open(task_file, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    scene, wall = sp.scene_from_config(cfg)
    robot = sp.RobotModel()
    opt = cfg.get('options', {})
    import numpy as np
    marker_u = float((np.array(MARKER_XY) - wall.p0) @ wall.t)
    plan = sp.plan_task(scene, wall, robot, OrientedAStar(),
                        restrict_to_reach=opt.get('restrict_to_reach', True),
                        retract_between_turns=opt.get('retract_between_turns', True),
                        partition=partition or opt.get('partition', 'dp'),
                        order=order or opt.get('order', 'sweep'), start_u=marker_u,
                        routes=routes or opt.get('routes', 'rule'))
    return sp.plan_to_dict(plan, wall, robot, opt.get('duration', 2.0), scene=scene,
                           marker_u=marker_u), plan


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    if len(args) < 2:
        print(__doc__)
        sys.exit(1)
    kw = dict(a[2:].split('=', 1) for a in sys.argv[1:] if a.startswith('--'))
    d, plan = plan_file(args[0], kw.get('partition'), kw.get('order'), kw.get('routes'))
    with open(args[1], 'w', encoding='utf-8') as f:
        yaml.safe_dump(d, f, allow_unicode=True, sort_keys=False)
    print(f"{d['status']}: {len(d['slots'])} 条槽 {d['length']:.3f} m，{len(d['stations'])} 个停靠站 "
          f"({d['routes']} / {d['partition']} / {d['order']})，预测 {d['prediction']}")
    sys.exit(0 if d['status'] == 'success' else 2)


if __name__ == '__main__':
    main()
