#!/usr/bin/env python3
"""
蒙特卡洛：标记粘贴误差 × 标记数量 × 是否估计横移比例 × 地图来源.

  OPENBLAS_NUM_THREADS=1 nice -n 19 python3 monte_carlo.py --layout 3   # 每个布局约 4–8 min
  OPENBLAS_NUM_THREADS=1 nice -n 19 python3 monte_carlo.py --layout 6 --scale 0   # 分两批
  python3 monte_carlo.py --summary                                      # 汇总已有结果

地图来源（每次试验共用同一组随机量，配对比较）：
  a_nominal   名义地图：估计器使用设计位置（σ_map = 2 mm 计入测量噪声，与默认配置相同）
  b_surveyed  实测地图：粘贴后测量，记录值 = 实际位置 + N(0, 0.5 mm)（σ_map = 0.5 mm；假设）
  c_online    第一遍在线估计：TagMapEKF，先验 = 设计位置、σ = 5 mm（与实际误差无关的固定值）
  d_pass2     第二遍：TagMapEKF，先验 = 第一遍结束时的估计（均值 + 协方差）；另一条录制轨迹、
              新的随机量，横移比例回到默认先验（第二遍可能在另一天、另一种地面）
  d_fixed     第二遍的另一种用法：把第一遍的估计当作已知地图（WallEKF，σ_map = 各标记的
              边缘标准差），不再在线估计
粘贴误差：每个非参考标记的实际位置 = 设计位置 + N(0, σ_p²)，σ_p ∈ {0, 1, 3, 5} mm。参考标记
定义墙面坐标原点，不加误差（它相对规划图纸的偏差对所有方法相同，见报告 2.4 节）。

输出：mc/mc_layout<L>.csv.gz（逐次试验、逐方法：11 站误差、σ_u、地图误差、NEES），
mc_summary.csv（每个单元：n、平均 / 95 分位 / 最大误差、每次运行最大误差的均值、NEES）。
"""
import argparse
import csv
import glob
import gzip
import math
import os
import time

import numpy as np

from common import HERE
from wall_sim import layouts, load_model, load_scenario, make_map, run_pass, Trial

SIGMA_P = (0.0, 0.001, 0.003, 0.005)
PRIOR_SIGMA = 0.005
SURVEY_SIGMA = 0.0005
MAP_FLOOR = 0.0005       # 相机系统误差的地图方差下限（consistency.py 在仿真器中标定：0.48 mm）
OUT = os.path.join(HERE, 'mc')
TRAJ = ('default_s1', 'default_s2')


def map_nees(est, true_tags, floor=MAP_FLOOR):
    """地图误差的 NEES（导出协方差，含系统误差下限）、不含下限的 NEES 与最大误差."""
    e = est.mean - np.array([true_tags[i] for i in est.ids])
    raw = est.cov - np.eye(len(e)) * floor ** 2
    return (float(e @ np.linalg.solve(est.cov, e)), float(e @ np.linalg.solve(raw, e)),
            float(np.max(np.abs(e))))


def run_cell(layout, sp, scale, trial, model):
    """一个单元的一次试验：返回各方法的结果行."""
    rng = np.random.default_rng([layout, int(sp * 1e4), int(scale), trial])
    design = layouts()[layout]
    true_tags = {i: (u if i == 0 else u + sp * rng.standard_normal())
                 for i, u in design.items()}
    k1 = trial % 2
    scn1, scn2 = load_scenario(TRAJ[k1]), load_scenario(TRAJ[1 - k1])
    tr1 = Trial(scn1, model, rng, true_tags)
    tr2 = Trial(scn2, model, rng, true_tags)
    nominal = make_map(design, 0.002)
    rows = []

    def row(method, r, scn, nees_map=math.nan, nees_raw=math.nan, map_err=math.nan):
        e = r['err']
        nees_u = float(np.nanmean((e / r['sigma']) ** 2))
        return dict(layout=layout, sigma_p_mm=sp * 1e3, scale=int(scale), trial=trial,
                    method=method, traj=scn['name'], max_mm=np.nanmax(np.abs(e)) * 1e3,
                    nees_u=nees_u, nees_map=nees_map, nees_map_raw=nees_raw,
                    map_err_mm=map_err * 1e3,
                    s_est=r['scale'][0], acc_moving=r['acc_moving'],
                    **{f'e{j}_mm': e[j] * 1e3 for j in range(len(e))},
                    **{f's{j}_mm': r['sigma'][j] * 1e3 for j in range(len(e))})

    rows.append(row('a_nominal', run_pass(scn1, tr1, nominal, 'known', scale), scn1))
    if layout == 1:
        return rows
    surveyed = make_map({i: (u if i == 0 else u + SURVEY_SIGMA * rng.standard_normal())
                         for i, u in true_tags.items()}, SURVEY_SIGMA)
    rows.append(row('b_surveyed', run_pass(scn1, tr1, surveyed, 'known', scale,
                                          sigma_map=SURVEY_SIGMA), scn1))
    rc = run_pass(scn1, tr1, nominal, 'slam', scale, prior_sigma=PRIOR_SIGMA,
                  map_floor=MAP_FLOOR)
    rows.append(row('c_online', rc, scn1, *map_nees(rc['map'], true_tags)))
    est = rc['map']
    rd = run_pass(scn2, tr2, nominal, 'slam', scale, prior=(est.tag_map(), est.cov),
                  map_floor=MAP_FLOOR)
    rows.append(row('d_pass2', rd, scn2, *map_nees(rd['map'], true_tags)))
    fixed = est.tag_map()
    rf = run_pass(scn2, tr2, fixed, 'known', scale,
                  sigma_map=float(np.max(est.sigma())))
    rows.append(row('d_fixed', rf, scn2))
    return rows


def run_layout(layout, n, model, scales=(False, True)):
    os.makedirs(OUT, exist_ok=True)
    suffix = '' if len(scales) == 2 else f'_scale{int(scales[0])}'
    path = os.path.join(OUT, f'mc_layout{layout}{suffix}.csv.gz')
    rows = []
    t0 = time.time()
    sps = (0.0,) if layout == 1 else SIGMA_P
    for sp in sps:
        for scale in scales:
            for k in range(n):
                rows += run_cell(layout, sp, scale, k, model)
            print(f'layout {layout} σ_p {sp * 1e3:.0f} mm scale {int(scale)}: '
                  f'{time.time() - t0:.0f} s', flush=True)
    with gzip.open(path, 'wt', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        for r in rows:
            w.writerow({k: (f'{v:.5g}' if isinstance(v, float) else v) for k, v in r.items()})
    print('写入', path)


def summarize():
    rows = []
    for path in sorted(glob.glob(os.path.join(OUT, 'mc_layout*.csv.gz'))):
        with gzip.open(path, 'rt') as f:
            rows += list(csv.DictReader(f))
    cells = {}
    for r in rows:
        key = (int(r['layout']), float(r['sigma_p_mm']), int(r['scale']), r['method'])
        cells.setdefault(key, []).append(r)
    out = []
    for key in sorted(cells):
        rs = cells[key]
        e = np.array([[float(r[f'e{j}_mm']) for j in range(11)] for r in rs])
        ae = np.abs(e)
        nees_u = np.array([float(r['nees_u']) for r in rs])
        nm = np.array([float(r['nees_map']) for r in rs])
        nr = np.array([float(r['nees_map_raw']) for r in rs])
        me = np.array([float(r['map_err_mm']) for r in rs])
        out.append(dict(
            layout=key[0], sigma_p_mm=key[1], scale=key[2], method=key[3], n_runs=len(rs),
            n_stations=ae.size,
            mean_mm=f'{np.mean(ae):.3f}', p95_mm=f'{np.percentile(ae, 95):.3f}',
            max_mm=f'{np.max(ae):.3f}', run_max_mean_mm=f'{np.mean(ae.max(axis=1)):.3f}',
            run_max_p95_mm=f'{np.percentile(ae.max(axis=1), 95):.3f}',
            signed_mean_mm=f'{np.mean(e):+.3f}',
            nees_u=f'{np.mean(nees_u):.3f}',
            nees_map=f'{np.nanmean(nm):.3f}' if np.isfinite(nm).any() else '',
            nees_map_raw=f'{np.nanmean(nr):.3f}' if np.isfinite(nr).any() else '',
            map_dim=len(layouts()[key[0]]) - 1,
            map_err_max_mean_mm=f'{np.nanmean(me):.3f}' if np.isfinite(me).any() else ''))
    with open(os.path.join(HERE, 'mc_summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(out[0]))
        w.writeheader()
        w.writerows(out)
    for o in out:
        print(o)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--layout', type=int, choices=(1, 2, 3, 6))
    ap.add_argument('-n', type=int, default=100)
    ap.add_argument('--scale', type=int, choices=(0, 1),
                    help='只跑一种（分批运行，结果文件加 _scale<0|1> 后缀）')
    ap.add_argument('--summary', action='store_true')
    a = ap.parse_args()
    if a.layout:
        scales = (False, True) if a.scale is None else (bool(a.scale),)
        run_layout(a.layout, a.n, load_model(), scales)
    if a.summary:
        summarize()


if __name__ == '__main__':
    main()
