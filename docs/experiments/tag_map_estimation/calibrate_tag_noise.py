#!/usr/bin/env python3
"""
标定横移中标记横向测量噪声：使仿真中横移时标记更新的门限接受率等于 Gazebo 中的实测值.

  python3 docs/experiments/tag_map_estimation/calibrate_tag_noise.py

为什么不直接拟合：估计器发布的时间戳与数据包的接收时间都只有 0.1 s 分辨率（/clock 10 Hz），
横移速度约 5 cm/s 时真值的取样时刻误差就有 ±2.5 mm，掩盖了真实的测量误差（noise_model.py
得到的“横移中”残差是这一误差的上界）。门限接受率不受时间戳影响：默认配置的两次 Gazebo
运行中，横移时标记更新的接受率为 575 / 593 = 97.0 %（default_s1 271 / 289，default_s2
304 / 304）。本脚本在同样的估计器配置（3 个标记 + 横移比例估计）下扫描高斯噪声标准差，
取接受率最接近 97.0 % 的值，写回 noise_model.yaml 的 tag.lateral_sigma_moving_calibrated_m；
同时把时间对齐项 τ 置 0（拟合值同样受时间戳分辨率影响）。
"""
import os

import numpy as np
import yaml

from common import HERE
from wall_sim import layouts, load_model, load_scenario, make_map, run_pass, Trial

TARGET = 575 / 593
GRID_MM = (0.3, 0.4, 0.5, 0.55, 0.6, 0.65, 0.7, 0.8)


def main(trials=16):
    model = load_model(calibrated=False)
    tags = layouts()[3]
    rows = []
    for sm in GRID_MM:
        acc = []
        for k in range(trials):
            scn = load_scenario(('default_s1', 'default_s2')[k % 2])
            tr = Trial(scn, model, np.random.default_rng(1000 + k), tags,
                       overrides={'t_core': sm * 1e-3, 't_tail_w': 0.0, 't_tau': 0.0})
            acc.append(run_pass(scn, tr, make_map(tags), 'known', True)['acc_moving'])
        rows.append((sm, float(np.mean(acc))))
        print(f'σ = {sm:.2f} mm: 接受率 {np.mean(acc):.4f}')
    best = min(rows, key=lambda r: abs(r[1] - TARGET))
    path = os.path.join(HERE, 'noise_model.yaml')
    with open(path) as f:
        head = [ln for ln in f.read().splitlines() if ln.startswith('#')]
    with open(path) as f:
        m = yaml.safe_load(f)
    m['tag']['lateral_sigma_moving_calibrated_m'] = {
        'value': round(best[0] * 1e-3, 6), 'source': 'calibrated',
        'n': f'gate acceptance while strafing 575/593 = {TARGET:.3f} (2 Gazebo runs); '
             f'{trials} simulated runs per grid value',
        'grid': [[g, round(a, 4)] for g, a in rows],
        'note': 'Gaussian; replaces the fitted moving-residual mixture, which is dominated '
                'by the 0.1 s timestamp resolution; time_offset set to 0 for the same reason'}
    with open(path, 'w') as f:
        f.write('\n'.join(head) + '\n')
        yaml.safe_dump(m, f, sort_keys=False, allow_unicode=True)
    print('选用', best)


if __name__ == '__main__':
    main()
