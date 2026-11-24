#!/usr/bin/env python3
"""
Gazebo 验证方案（报告第 7 节）中每个配置的预期值：用离线仿真器按同样的配置运行 40 次.

  OPENBLAS_NUM_THREADS=1 nice -n 19 python3 gazebo_predictions.py     # 约 1 min

配置与 Gazebo 方案一一对应（世界中的标记在 0、0.6、1.2 m，地图记录值按配置偏移）：
  known_err5      已知地图（不在线估计），记录值 ID 2 +5 mm、ID 4 −5 mm，估计比例（对照）
  slam_ok_p1      在线估计，记录值正确（0.6 / 1.2），先验 σ 5 mm（检查开启后不变差）
  slam_err3_p1/p2 在线估计，记录值 +3 / −3 mm（与 tags_0.6_err3 相同）；第一遍 / 第二遍
  slam_err5_p1/p2 在线估计，记录值 +5 / −5 mm
  slam_err5_noscale_p1  同上但不估计比例
  tags_0.6_noodomerr    已知正确地图、不估计比例、关闭全部编码器误差（第 4 部分的确认实验）
输出 gazebo_predictions.csv。
"""
import csv
import os

import numpy as np

from common import HERE
from wall_sim import layouts, load_model, load_scenario, make_map, run_pass, Trial

N = 40
FLOOR = 0.0005
ERR3 = {2: 0.003, 4: -0.003}
ERR5 = {2: 0.005, 4: -0.005}
OFF_ALL = {'s_mean': 0.0, 's_run': 0.0, 's_seg': 0.0, 'h_seg': 0.0, 'kappa': 0.0,
           'hold_u': 0.0, 'hold_psi': 0.0}


def main():
    model = load_model()
    tags = layouts()[3]
    cfgs = [
        ('known_err5', ERR5, 'known', True, {}),
        ('slam_ok', {}, 'slam', True, {}),
        ('slam_err3', ERR3, 'slam', True, {}),
        ('slam_err5', ERR5, 'slam', True, {}),
        ('slam_err5_noscale', ERR5, 'slam', False, {}),
        ('tags_0.6_noodomerr', {}, 'known', False, OFF_ALL),
        ('tags_0.6', {}, 'known', False, {}),
    ]
    rows = []
    for name, err, method, scale, ov in cfgs:
        rec = {'p1': [], 'p2': []}
        maps = {'p1': [], 'p2': []}
        for k in range(N):
            rng = np.random.default_rng(6000 + k)
            scn1 = load_scenario(('default_s1', 'default_s2')[k % 2])
            scn2 = load_scenario(('default_s1', 'default_s2')[1 - k % 2])
            tr1 = Trial(scn1, model, rng, tags, overrides=ov)
            est_map = make_map({i: u + err.get(i, 0.0) for i, u in tags.items()})
            r1 = run_pass(scn1, tr1, est_map, method, scale, prior_sigma=0.005,
                          map_floor=FLOOR)
            rec['p1'].append(r1['err'])
            if method == 'slam':
                est = r1['map']
                maps['p1'].append(est.mean - np.array([tags[i] for i in est.ids]))
                tr2 = Trial(scn2, model, rng, tags, overrides=ov)
                r2 = run_pass(scn2, tr2, est_map, 'slam', scale, prior=(est.tag_map(), est.cov),
                              map_floor=FLOOR)
                rec['p2'].append(r2['err'])
                e2 = r2['map']
                maps['p2'].append(e2.mean - np.array([tags[i] for i in e2.ids]))
        for p in ('p1', 'p2'):
            if not rec[p]:
                continue
            e = np.array(rec[p]) * 1e3
            mx = np.abs(e).max(axis=1)
            row = dict(config=f'{name}_{p}', n=N, run_max_mean_mm=mx.mean(),
                       run_max_p5_mm=np.percentile(mx, 5), run_max_p95_mm=np.percentile(mx, 95),
                       station_p95_mm=np.percentile(np.abs(e), 95),
                       signed_mean_mm=e.mean())
            if maps[p]:
                me = np.array(maps[p]) * 1e3
                row.update(map_err_id2_mm=f'{me[:, 0].mean():+.2f} ± {me[:, 0].std():.2f}',
                           map_err_id4_mm=f'{me[:, 1].mean():+.2f} ± {me[:, 1].std():.2f}')
            else:
                row.update(map_err_id2_mm='', map_err_id4_mm='')
            rows.append(row)
            print({k: (round(v, 2) if isinstance(v, float) else v) for k, v in row.items()})
    with open(os.path.join(HERE, 'gazebo_predictions.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        for r in rows:
            w.writerow({k: (f'{v:.2f}' if isinstance(v, float) else v) for k, v in r.items()})


if __name__ == '__main__':
    main()
