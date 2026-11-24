#!/usr/bin/env python3
"""
离线仿真器与 Gazebo 的对照：用 along_wall_accuracy 中做过的配置驱动仿真器，比较站点误差.

  python3 docs/experiments/tag_map_estimation/validate_sim.py   # 约 1 min

Gazebo 一侧的站点误差取“补偿后刀尖沿墙误差” = 到站真值误差 − 补偿所用的传感器残差
（stations_all.csv 的 lateral_mm − sensed_lateral_mm），它等于 −(û − u)。仿真一侧取每站
到站残差测量最后一个周期的 û − u。每个配置仿真 40 次（两条录制轨迹交替）。

输出 validation.csv：配置、Gazebo 每次运行的最大误差与带符号的平均误差、仿真的均值与
5 / 95 分位。
"""
import csv
import os

import numpy as np

from common import HERE, REPO
from wall_sim import layouts, load_model, load_scenario, make_map, run_pass, Trial

AW = os.path.join(REPO, 'docs', 'experiments', 'along_wall_accuracy', 'stations_all.csv')
# 配置 → (布局, 估计器使用的地图相对实际位置的偏差, 是否估计比例)
CONFIGS = {
    'single_on': (1, {}, False),
    'scale_single': (1, {}, True),
    'tags_1.2': (2, {}, False),
    'scale_tags_1.2': (2, {}, True),
    'tags_0.6': (3, {}, False),
    'scale_tags_0.6': (3, {}, True),
    'default': (3, {}, True),
    'tags_0.6_err3': (3, {2: 0.003, 4: -0.003}, False),
    'scale_tags_0.6_err3': (3, {2: 0.003, 4: -0.003}, True),
    'tags_0.3': (6, {}, False),
}
N = 40


def gazebo_stats():
    runs = {}
    with open(AW) as f:
        for r in csv.DictReader(f):
            e = -(float(r['lateral_mm']) - float(r['sensed_lateral_mm']))   # = û − u（mm）
            runs.setdefault((r['config'], r['seed']), []).append(e)
    out = {}
    for (cfg, seed), es in runs.items():
        out.setdefault(cfg, []).append((max(abs(e) for e in es), float(np.mean(es))))
    return out


def main():
    model = load_model()
    gz = gazebo_stats()
    rows = []
    for name, (lay, map_err, scale) in CONFIGS.items():
        tags = layouts()[lay]
        est_map = make_map({i: u + map_err.get(i, 0.0) for i, u in tags.items()})
        mx, mean = [], []
        for k in range(N):
            scn = load_scenario(('default_s1', 'default_s2')[k % 2])
            tr = Trial(scn, model, np.random.default_rng(2000 + k), tags)
            r = run_pass(scn, tr, est_map, 'known', scale)
            mx.append(np.nanmax(np.abs(r['err'])) * 1e3)
            mean.append(np.nanmean(r['err']) * 1e3)
        g = gz.get(name, [])
        row = dict(config=name, markers=lay, estimate_scale=int(scale),
                   gazebo_runs=len(g),
                   gazebo_max_mm=' / '.join(f'{a:.1f}' for a, _ in g),
                   gazebo_signed_mean_mm=' / '.join(f'{b:+.1f}' for _, b in g),
                   sim_n=N, sim_max_mean_mm=f'{np.mean(mx):.2f}',
                   sim_max_p5_mm=f'{np.percentile(mx, 5):.2f}',
                   sim_max_p95_mm=f'{np.percentile(mx, 95):.2f}',
                   sim_signed_mean_mm=f'{np.mean(mean):+.2f}')
        rows.append(row)
        print(row)
    with open(os.path.join(HERE, 'validation.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


if __name__ == '__main__':
    main()
