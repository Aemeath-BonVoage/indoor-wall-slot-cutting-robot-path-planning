#!/usr/bin/env python3
"""
在线地图估计的一致性检查：逐项关闭仿真器中的误差源，看地图 NEES 与地图误差的变化.

  OPENBLAS_NUM_THREADS=1 nice -n 19 python3 consistency.py      # 约 1 min

设置：3 个标记（0、0.6、1.2 m），粘贴误差 σ_p = 3 mm，估计横移比例，第一遍在线估计
（先验 σ = 5 mm），每种情形 40 次试验（两条录制轨迹交替）。地图 NEES = eᵀP⁻¹e（e 为两个
非参考标记的估计误差，P 为滤波器的地图协方差，不含系统误差下限），一致时均值为 2。

输出 consistency.csv；最后一行给出系统误差下限的标定：
    floor² = mean(e²) − mean(σ²)（全部误差源开启，每个布局单独计算）
"""
import csv
import os

import numpy as np

from common import HERE
from wall_sim import layouts, load_model, load_scenario, make_map, run_pass, Trial

N = 40
CASES = [
    ('all sources', {}),
    ('no strafe heading noise', {'h_seg': 0.0}),
    ('no laser station bias', {'l_psi_bias': 0.0, 'l_d_bias': 0.0}),
    ('no tag lateral bias (a + b*bearing)', {'t_a': 0.0, 't_b': 0.0}),
    ('no tag depth bias', {'t_dep_bias': 0.0}),
    ('no tag biases (lateral + depth)', {'t_a': 0.0, 't_b': 0.0, 't_dep_bias': 0.0}),
    ('no systematic errors', {'h_seg': 0.0, 'l_psi_bias': 0.0, 'l_d_bias': 0.0, 't_a': 0.0,
                              't_b': 0.0, 't_dep_bias': 0.0}),
]


def one(layout, overrides, seed0):
    model = load_model()
    design = layouts()[layout]
    nees, err, sig, emax = [], [], [], []
    for k in range(N):
        rng = np.random.default_rng(seed0 + k)
        true = {i: (u if i == 0 else u + 0.003 * rng.standard_normal())
                for i, u in design.items()}
        scn = load_scenario(('default_s1', 'default_s2')[k % 2])
        tr = Trial(scn, model, rng, true, overrides=overrides)
        r = run_pass(scn, tr, make_map(design), 'slam', True, prior_sigma=0.005)
        est = r['map']
        e = est.mean - np.array([true[i] for i in est.ids])
        nees.append(float(e @ np.linalg.solve(est.cov, e)))
        err += list(e)
        sig += list(est.sigma())
        emax.append(np.nanmax(np.abs(r['err'])))
    err, sig = np.array(err), np.array(sig)
    return dict(nees=np.mean(nees), dim=len(design) - 1, err_rms=np.sqrt(np.mean(err ** 2)),
                err_mean=np.mean(err), sigma_rms=np.sqrt(np.mean(sig ** 2)),
                floor=np.sqrt(max(0.0, np.mean(err ** 2) - np.mean(sig ** 2))),
                station_max=np.mean(emax))


def main():
    rows = []
    for name, ov in CASES:
        r = one(3, ov, 3000)
        rows.append(dict(case=name, layout=3, n=N, **r))
        print(name, {k: round(v, 5) for k, v in r.items()})
    for lay in (2, 6):
        r = one(lay, {}, 3000)
        rows.append(dict(case='all sources', layout=lay, n=N, **r))
        print(lay, {k: round(v, 5) for k, v in r.items()})
    with open(os.path.join(HERE, 'consistency.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        for r in rows:
            w.writerow({k: (f'{v:.5g}' if isinstance(v, float) else v) for k, v in r.items()})


if __name__ == '__main__':
    main()
