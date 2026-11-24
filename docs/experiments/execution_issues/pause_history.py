#!/usr/bin/env python3
"""
此前提交的运行中，切割阶段（CutStation）每站的额外耗时（第 3 节，不需要 Gazebo）.

  python3 pause_history.py

- 逐点执行（plan_validation、slot_execution、along_wall_accuracy 的运行）：
  额外耗时 = cut_s − (1.02 + 2.029 × 点数)，系数为 plan_validation 对 55 站的回归；
- 连续轨迹（arm_trajectory/stations_all.csv 中 mode = continuous 的站）：
  额外耗时 = cut_s − traj_s − 全部站的中位数（traj_s 为轨迹生成器给出的名义时长，中位数约 1.1 s
  是 CutStation 开始时的残差测量等固定开销）。
输出 pause_history.csv（每站一行）并打印统计。
"""
import csv
import glob
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.join(HERE, '..')


def main():
    rows = []
    seen = set()
    for f in sorted(glob.glob(os.path.join(EXP, 'plan_validation', 'runs', '*', 'stations.csv'))
                    + glob.glob(os.path.join(EXP, 'slot_execution', 'runs', '*', 'stations.csv'))
                    + glob.glob(os.path.join(EXP, 'along_wall_accuracy', 'runs', '*',
                                             'stations.csv'))):
        exp, run = f.split(os.sep)[-4], f.split(os.sep)[-2]
        if run.startswith(('v1_', 'oldsim_')):
            continue                                  # 仿真配置 v1 / 更早的运行不计
        for r in csv.DictReader(open(f)):
            try:
                ex = float(r['cut_s']) - (1.02 + 2.029 * int(r['points']))
            except (KeyError, ValueError):
                continue
            rows.append(dict(experiment=exp, run=run, station=r['station'],
                             mode='point_to_point', points=r['points'], excess_s=round(ex, 3)))
            seen.add((exp, run))
    st = list(csv.DictReader(open(os.path.join(EXP, 'arm_trajectory', 'stations_all.csv'))))
    cont = [r for r in st if r['mode'] == 'continuous']
    med = float(np.median([float(r['cut_s']) - float(r['traj_s']) for r in cont]))
    for r in cont:
        rows.append(dict(experiment='arm_trajectory', run=r['run'], station=r['station'],
                         mode='continuous', points=r['points'],
                         excess_s=round(float(r['cut_s']) - float(r['traj_s']) - med, 3)))
    with open(os.path.join(HERE, 'pause_history.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    for mode in ('point_to_point', 'continuous'):
        sel = [r for r in rows if r['mode'] == mode]
        ex = np.array([r['excess_s'] for r in sel])
        runs = {(r['experiment'], r['run']) for r in sel}
        pts = sum(int(r['points']) for r in sel)
        print(f'{mode}: {len(runs)} runs, {len(sel)} stations, {pts} arm points; excess median '
              f'{np.median(ex):+.3f} s, > 0.5 s: {int((ex > 0.5).sum())}, > 1.5 s: '
              f'{int((ex > 1.5).sum())}, max {ex.max():+.2f} s')
        print('   largest:', sorted(np.round(ex, 2))[-8:])


if __name__ == '__main__':
    main()
