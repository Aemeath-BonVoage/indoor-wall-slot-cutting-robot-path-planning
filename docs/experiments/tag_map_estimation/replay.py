#!/usr/bin/env python3
"""
用录制的 Gazebo 数据回放在线地图估计.

  OPENBLAS_NUM_THREADS=1 nice -n 19 python3 replay.py       # 约 10 s

两部分：

1. 同帧标记对的间距（只用仓库中已提交的数据）：along_wall_accuracy/runs/*/ekf_trace.csv
   中同一周期画面里有两个以上标记时，|p_i − p_j| 与朝向无关、与里程计无关，直接是间距的
   测量。按标记对统计均值、标准差与真值（世界中 0.3 m 的整数倍）之差。
   输出 replay_covisible.csv。

2. 完整回放（数据包 default_s1 / default_s2，bag_data/ 中的提取结果）：真实的轮式里程计、
   激光拟合与标记测量驱动 TagMapEKF。地图先验故意给错（ID 2 记为 +5 mm、ID 4 记为 −5 mm，
   σ = 5 mm），看估计能否回到真值。估计器发布的时间戳只有 0.1 s 分辨率（/clock 10 Hz）：
   标记位置是补偿到估计器内部时刻（最近一帧里程计）的，该时刻在 [t, t + 0.1 s) 内，
   横移中对齐误差可达 v·0.1 s ≈ 5 mm。试过把这段时间不确定度（均匀分布，σ = 29 ms × v）
   计入协方差、使用全部帧：地图仍偏 3 mm，且一次首次观测的重新初始化取到了错误的帧
   （ID 2 偏 19 mm），所以回放只在底盘静止（里程计速度 < 5 mm/s）的周期使用标记测量。
   另外从精对位接近阶段的最后 2 s 开始回放：接近阶段底盘边转边移、只看到参考标记，
   只用静止帧时两次静止观测之间的运动会被横移比例状态吸收（试运行中 s 被拉到 −13 %）；
   在线运行时标记每帧都参与更新，没有这一问题。激光按自身的时间戳（1 ms 分辨率）
   做延迟补偿。
   第二遍：default_s2 以 default_s1 回放结束时的地图估计为先验。
   输出 replay_map.csv（每次回放的地图估计）与 replay_stations.csv（逐站估计误差）。
"""
import csv
import glob
import math
import os

import numpy as np

from common import (AW_RUNS, HERE, load_bag, parse_tags, parse_trace_tags, read_csv,
                    relative_pose, to_wall, WORLD_TAG_U)
from scr_arm_controller.tag_map import TagMap
from scr_arm_controller.tag_map_slam import TagMapEKF
from scr_arm_controller.wall_ekf import WallEKF
from wall_sim import tag_R

PRIOR = {0: 0.0, 2: 0.605, 4: 1.195}
PRIOR_SIGMA = 0.005
MAP_FLOOR = 0.0005
STATIC_SPEED = 0.005
START_BEFORE_ALIGN_END = 2.0   # 从精对位阶段结束前这么多秒开始回放


def covisible():
    rows = []
    pairs = {}
    for path in sorted(glob.glob(os.path.join(AW_RUNS, '*', 'ekf_trace.csv'))):
        run = os.path.basename(os.path.dirname(path))
        for r in read_csv(path):
            tags = parse_trace_tags(r.get('tag_xy') or '')
            if len(tags) < 2:
                continue
            tags.sort(key=lambda t: WORLD_TAG_U[t[0]])
            for (i, xi, yi), (j, xj, yj) in zip(tags[:-1], tags[1:]):
                dist = math.hypot(xi - xj, yi - yj)
                moving = r['phase'] == 'move'
                pairs.setdefault((i, j), []).append((dist, moving, run))
    for (i, j), v in sorted(pairs.items(), key=lambda kv: WORLD_TAG_U[kv[0][0]]):
        true = WORLD_TAG_U[j] - WORLD_TAG_U[i]
        for label, sel in (('all', lambda m: True), ('static', lambda m: not m),
                           ('moving', lambda m: m)):
            d = np.array([x for x, m, _ in v if sel(m)])
            if len(d) < 3:
                continue
            runs = sorted({rn for _, m, rn in v if sel(m)})
            rows.append(dict(pair=f'{i}-{j}', true_m=true, frames=label, n=len(d),
                             runs=len(runs), mean_err_mm=(d.mean() - true) * 1e3,
                             std_mm=d.std(ddof=1) * 1e3,
                             sem_mm=d.std(ddof=1) / math.sqrt(len(d)) * 1e3))
    with open(os.path.join(HERE, 'replay_covisible.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        for r in rows:
            w.writerow({k: (f'{v:.4f}' if isinstance(v, float) else v) for k, v in r.items()})
    for r in rows:
        print(r)
    return rows


def replay(name, prior_map, prior_cov=None, estimate_scale=True):
    b = load_bag(name)
    est_rows = b['estimate']
    t = np.array([float(r['t']) for r in est_rows])
    odom = b['odom']
    wall = b['wall']
    R_wall = np.diag([np.median(wall[:, 3]), np.median(wall[:, 5])])
    f = TagMapEKF(prior_map, prior_sigma=PRIOR_SIGMA, prior_cov=prior_cov,
                  estimate_scale=estimate_scale, map_floor=MAP_FLOOR)
    u_known = False
    align_end = max(t1 for t0, t1, s_, p_ in b['phases'] if p_ == 'align')
    t_start = align_end - START_BEFORE_ALIGN_END
    last_pose = None
    last_scan = -1
    n_tag = 0
    trace = []
    st_end = {}
    for k, tk in enumerate(t):
        if tk < t_start:
            continue
        pose = odom.at(tk)
        if f.initialized and last_pose is not None:
            f.predict(*relative_pose(last_pose, pose))
        last_pose = pose
        # 最新的激光帧（时间戳 ≤ tk），按里程计补偿到 tk
        j = int(np.searchsorted(wall[:, 0], tk, side='right')) - 1
        if j >= 0 and j != last_scan and tk - wall[j, 0] < 0.3:
            last_scan = j
            ts = wall[j, 0]
            dx, dy, dyaw = relative_pose(pose, odom.at(ts))
            dy = f.correct_increment(dy) if f.initialized else dy
            phi_s = wall[j, 2]
            n_s = np.array([math.cos(phi_s), math.sin(phi_s)])
            c, s = math.cos(dyaw), math.sin(dyaw)
            n = np.array([[c, -s], [s, c]]) @ n_s
            dist = wall[j, 1] + float(n @ np.array([dx, dy]))
            phi = math.atan2(n[1], n[0])
            if not f.initialized:
                d0, u0, p0, _ = WallEKF.init_from((dist, phi))
                f.reset(d0, u0, p0)
            else:
                f.update_wall(dist, phi, R_wall)
        if not f.initialized:
            continue
        # 底盘速度（里程计 ±0.1 s）
        v = math.hypot(*relative_pose(odom.at(tk - 0.1), odom.at(tk + 0.1))[:2]) / 0.2
        obs = [(i, np.array([x, y])) for i, x, y, _ in parse_tags(est_rows[k]['tags'])
               if i in prior_map]
        if obs and v < STATIC_SPEED:
            if not u_known:
                i0, p0 = min(obs, key=lambda o: abs(o[1][1]))
                _, u0, _, _ = WallEKF.init_from((f.x[0], -f.x[2]), p0, u_tag=f.tag_u(i0))
                f.x[1] = u0
                f.P[1, :] = 0.0
                f.P[:, 1] = 0.0
                f.P[1, 1] = 0.01 ** 2
                u_known = True
            for i, p in obs:
                ok, _ = f.update_tag_id(p, tag_R(p), i)
                n_tag += int(ok)
        d_t, u_t, psi_t = to_wall(*b['truth'].at(tk))
        st, ph = -1, ''
        for t0, t1, s_, p_ in b['phases']:
            if t0 <= tk < t1 + 1e-6:
                st, ph = s_, p_
        if ph == 'arrived' and st >= 0 and u_known:
            st_end[st] = (f.x[1] - u_t, math.sqrt(f.P[1, 1]), f.x[2] - psi_t)
        trace.append((tk, f.x[1] - u_t, math.sqrt(f.P[1, 1]),
                      *[v for i in f.map_ids for v in (f.tag_u(i), f.tag_sigma(i))],
                      f.scale[0], f.scale[1]))
    return f, st_end, np.array(trace), n_tag


def main():
    covisible()
    prior = TagMap(PRIOR, reference_id=0, sigma_position=PRIOR_SIGMA)
    map_rows, st_rows = [], []
    traces = {}
    runs = [('default_s1', 'pass 1', prior, None), ('default_s2', 'pass 1', prior, None)]
    results = {}
    for name, label, pm, pc in runs:
        f, st_end, tr, n_tag = replay(name, pm, pc)
        results[name] = f
        traces[f'{name} {label}'] = tr
        est = f.map_estimate()
        for i, m, s in zip(est.ids, est.mean, est.sigma()):
            map_rows.append(dict(run=name, pass_=label, tag=i, prior_m=pm.u[i],
                                 estimate_m=m, sigma_mm=s * 1e3,
                                 err_mm=(m - WORLD_TAG_U[i]) * 1e3,
                                 scale=f.scale[0], tag_updates=n_tag))
        for st, (e, s, ep) in sorted(st_end.items()):
            st_rows.append(dict(run=name, pass_=label, station=st, err_mm=e * 1e3,
                                sigma_mm=s * 1e3, psi_err_deg=math.degrees(ep)))
    # 第二遍：default_s2，以 default_s1 第一遍的估计为先验
    est1 = results['default_s1'].map_estimate()
    f, st_end, tr, n_tag = replay('default_s2', est1.tag_map(), est1.cov)
    traces['default_s2 pass 2'] = tr
    est = f.map_estimate()
    for i, m, s in zip(est.ids, est.mean, est.sigma()):
        map_rows.append(dict(run='default_s2', pass_='pass 2 (prior: default_s1)', tag=i,
                             prior_m=est1.mean[est1.ids.index(i)], estimate_m=m,
                             sigma_mm=s * 1e3, err_mm=(m - WORLD_TAG_U[i]) * 1e3,
                             scale=f.scale[0], tag_updates=n_tag))
    for st, (e, s, ep) in sorted(st_end.items()):
        st_rows.append(dict(run='default_s2', pass_='pass 2 (prior: default_s1)', station=st,
                            err_mm=e * 1e3, sigma_mm=s * 1e3, psi_err_deg=math.degrees(ep)))
    for fn, rows in (('replay_map.csv', map_rows), ('replay_stations.csv', st_rows)):
        with open(os.path.join(HERE, fn), 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0]))
            w.writeheader()
            for r in rows:
                w.writerow({k: (f'{v:.6g}' if isinstance(v, float) else v)
                            for k, v in r.items()})
    for r in map_rows:
        print(r)
    for lbl in sorted({(r['run'], r['pass_']) for r in st_rows}):
        es = [abs(r['err_mm']) for r in st_rows if (r['run'], r['pass_']) == lbl]
        print(lbl, f'max {max(es):.2f} mm, mean {np.mean(es):.2f} mm, n = {len(es)}')
    np.savez_compressed(os.path.join(HERE, 'replay_traces.npz'),
                        **{k.replace(' ', '_'): v for k, v in traces.items()})


if __name__ == '__main__':
    main()
