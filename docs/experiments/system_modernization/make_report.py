#!/usr/bin/env python3
"""
系统层改造实验的汇总表与图（读取 runs/ 下的原始数据）.

  python3 docs/experiments/system_modernization/make_report.py

输出：
  nav_summary.csv / fig_nav_paths.png / fig_nav_metrics.png   导航对比（MPPI + Smac 2D vs NavFn + DWB）
  odom_summary.csv / fig_odom_drift.png                        定位融合（轮式里程计 vs EKF vs AMCL）
  base_summary.csv                                             底盘被控对象（力驱动 vs 轮子摩擦驱动）
  slot_summary.csv                                             多站执行（各配置）
"""
import csv
import glob
import math
import os

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, 'runs')
MAP_YAML = os.path.join(HERE, '..', '..', '..', 'src', 'scr_navigation', 'maps', 'my_map.yaml')

NAV = [('nav_navfn_dwb', 'NavFn + DWB'), ('nav_mppi_smac2d', 'Smac 2D + MPPI')]
CPU_KEYS = ('cpu_controller_serv', 'cpu_planner_server', 'cpu_bt_navigator', 'cpu_amcl',
            'cpu_system')


def read_csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return math.nan


def write_csv(path, rows):
    keys = list(rows[0])
    for r in rows:
        keys += [k for k in r if k not in keys]
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


def mean_std(vals):
    v = np.array([x for x in vals if not math.isnan(x)])
    return (float(v.mean()), float(v.std(ddof=1)) if len(v) > 1 else 0.0) if len(v) else \
        (math.nan, math.nan)


def setup_plot():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['font.family'] = ['Noto Sans CJK JP', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    return plt


def load_map():
    from PIL import Image
    with open(MAP_YAML) as f:
        m = yaml.safe_load(f)
    img = np.array(Image.open(os.path.join(os.path.dirname(MAP_YAML), m['image'])))
    h, w = img.shape
    r, (ox, oy, _) = m['resolution'], m['origin']
    return img, (ox, ox + w * r, oy, oy + h * r)


# ---------------------------------------------------------------------------
# 导航对比
# ---------------------------------------------------------------------------
def nav_report(plt):
    rows, legs = [], {}
    for label, name in NAV:
        p = os.path.join(RUNS, label, 'legs.csv')
        if not os.path.exists(p):
            continue
        legs[label] = read_csv(p)
        L = legs[label]
        ok = [r for r in L if r['success'] == '1']
        row = dict(config=name, legs=len(L), success=len(ok))
        for k in ('time_s', 'path_m', 'clearance_m', 'jerk_rms', 'dw_rms', 'cmd_dw_rms',
                  'final_err_m') + CPU_KEYS:
            m, s = mean_std([fnum(r.get(k)) for r in ok])
            row[f'{k}_mean'], row[f'{k}_std'] = round(m, 4), round(s, 4)
        row['clearance_m_min'] = round(min(fnum(r['clearance_m']) for r in ok), 4) if ok else ''
        row['time_total_s'] = round(sum(fnum(r['time_s']) for r in ok), 1)
        row['path_total_m'] = round(sum(fnum(r['path_m']) for r in ok), 2)
        rows.append(row)
    if not rows:
        return
    # 同口径：只统计两种配置在每一轮都成功的目标
    common = None
    for L in legs.values():
        ok_goals = {g for g in {r['goal'] for r in L}
                    if all(r['success'] == '1' for r in L if r['goal'] == g)}
        common = ok_goals if common is None else common & ok_goals
    for row, (label, _) in zip(rows, [n for n in NAV if n[0] in legs]):
        sel = [r for r in legs[label] if r['goal'] in common]
        row['common_goals'] = ' '.join(sorted(g.split('_')[0] for g in common))
        for k in ('time_s', 'path_m', 'clearance_m', 'jerk_rms', 'dw_rms', 'cmd_dw_rms',
                  'cpu_controller_serv'):
            m, s = mean_std([fnum(r[k]) for r in sel])
            row[f'common_{k}_mean'], row[f'common_{k}_std'] = round(m, 4), round(s, 4)
        row['common_time_total_s'] = round(sum(fnum(r['time_s']) for r in sel), 1)
        row['common_clearance_min'] = round(min(fnum(r['clearance_m']) for r in sel), 4)
    write_csv(os.path.join(HERE, 'nav_summary.csv'), rows)
    # 逐目标对比（各轮均值）
    goals = sorted({r['goal'] for L in legs.values() for r in L})
    per_goal = []
    for g in goals:
        row = dict(goal=g)
        for label, name in NAV:
            if label not in legs:
                continue
            ok = [r for r in legs[label] if r['goal'] == g and r['success'] == '1']
            for k in ('time_s', 'path_m', 'clearance_m', 'jerk_rms'):
                row[f'{label}_{k}'] = round(mean_std([fnum(r[k]) for r in ok])[0], 3)
        per_goal.append(row)
    write_csv(os.path.join(HERE, 'nav_per_goal.csv'), per_goal)

    # 轨迹图
    img, ext = load_map()
    fig, ax = plt.subplots(1, 2, figsize=(10, 4.6), sharey=True)
    for a, (label, name) in zip(ax, NAV):
        a.imshow(img, cmap='gray', extent=ext, origin='upper', vmin=0, vmax=255)
        tp = os.path.join(RUNS, label, 'trajectory.csv')
        if os.path.exists(tp):
            t = read_csv(tp)
            a.plot([fnum(r['gt_x']) for r in t], [fnum(r['gt_y']) for r in t], lw=0.8,
                   color='tab:red' if 'mppi' in label else 'tab:blue')
        a.set_title(f'{name}（3 圈真值轨迹）', fontsize=9)
        a.set_xlabel('x (m)')
        a.set_xlim(ext[0], ext[1])
        a.set_ylim(ext[2], ext[3])
    ax[0].set_ylabel('y (m)')
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, 'fig_nav_paths.png'), dpi=150)
    plt.close(fig)

    # 指标图：逐段对比
    keys = [('time_s', '用时 (s)'), ('path_m', '路径长度 (m)'), ('clearance_m', '最小间隙 (m)'),
            ('jerk_rms', '加加速度 RMS (m/s³)')]
    fig, ax = plt.subplots(1, 4, figsize=(12, 3))
    x = np.arange(len(goals))
    for a, (k, title) in zip(ax, keys):
        for i, (label, name) in enumerate(NAV):
            if label not in legs:
                continue
            m = [mean_std([fnum(r[k]) for r in legs[label] if r['goal'] == g
                           and r['success'] == '1']) for g in goals]
            a.bar(x + (i - 0.5) * 0.38, [v[0] for v in m], 0.38, yerr=[v[1] for v in m],
                  label=name, capsize=2,
                  color='tab:red' if 'mppi' in label else 'tab:blue', alpha=0.8)
        a.set_xticks(x)
        a.set_xticklabels([g.split('_')[0] for g in goals], fontsize=8)
        a.set_title(title, fontsize=9)
    ax[0].legend(fontsize=7, frameon=False)
    fig.suptitle('逐目标均值 ± 标准差（每个目标 3 次）', fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, 'fig_nav_metrics.png'), dpi=150)
    plt.close(fig)


# ---------------------------------------------------------------------------
# 定位融合：里程计漂移
# ---------------------------------------------------------------------------
def aligned_errors(t, prefix):
    """估计轨迹按起点与真值对齐后的位置误差（米）与航向误差（度）."""
    gx, gy, gyaw = (np.array([fnum(r[f'gt_{k}']) for r in t]) for k in ('x', 'y', 'yaw'))
    ex, ey, eyaw = (np.array([fnum(r[f'{prefix}_{k}']) for r in t]) for k in ('x', 'y', 'yaw'))
    ok = ~np.isnan(ex)
    if not ok.any():
        return None
    i0 = int(np.argmax(ok))
    if prefix == 'map':
        dx, dy, dyaw = ex - gx, ey - gy, eyaw - gyaw      # map 系直接比较（地图与世界系重合）
    else:
        th = gyaw[i0] - eyaw[i0]
        c, s = math.cos(th), math.sin(th)
        ax_ = gx[i0] + c * (ex - ex[i0]) - s * (ey - ey[i0])
        ay_ = gy[i0] + s * (ex - ex[i0]) + c * (ey - ey[i0])
        dx, dy, dyaw = ax_ - gx, ay_ - gy, eyaw + th - gyaw
    dyaw = np.degrees(np.arctan2(np.sin(dyaw), np.cos(dyaw)))
    dist = np.concatenate([[0], np.cumsum(np.hypot(np.diff(gx), np.diff(gy)))])
    return dist[ok], np.hypot(dx, dy)[ok], dyaw[ok]


def odom_report(plt):
    runs = sorted(glob.glob(os.path.join(RUNS, 'loc_seed*', 'trajectory.csv')))
    if not runs:
        return
    rows = []
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
    colors = {'wheel': 'tab:orange', 'odom': 'tab:green', 'map': 'tab:purple'}
    names = {'wheel': '仅轮式里程计', 'odom': 'EKF（轮式 + 陀螺）', 'map': 'AMCL（map 系）'}
    for k, p in enumerate(runs):
        label = os.path.basename(os.path.dirname(p))
        t = read_csv(p)
        for src in ('wheel', 'odom', 'map'):
            r = aligned_errors(t, src)
            if r is None:
                continue
            dist, e, yaw = r
            rows.append(dict(run=label, source=src, distance_m=round(float(dist[-1]), 1),
                             final_pos_err_m=round(float(e[-1]), 3),
                             max_pos_err_m=round(float(e.max()), 3),
                             drift_pct=round(float(100 * e[-1] / dist[-1]), 2),
                             final_yaw_err_deg=round(float(yaw[-1]), 2),
                             max_abs_yaw_err_deg=round(float(np.abs(yaw).max()), 2)))
            ax[0].plot(dist, e, color=colors[src], lw=0.9,
                       label=names[src] if k == 0 else None)
            ax[1].plot(dist, yaw, color=colors[src], lw=0.9,
                       label=names[src] if k == 0 else None)
    # 修正前（速度闭环与传感器用连杆速度）的同一路线，只进表不进图
    for p in sorted(glob.glob(os.path.join(RUNS, 'linkvel_loc_seed*', 'trajectory.csv'))):
        t = read_csv(p)
        for src in ('wheel', 'odom', 'map'):
            r = aligned_errors(t, src)
            if r is not None:
                dist, e, yaw = r
                rows.append(dict(run=os.path.basename(os.path.dirname(p)), source=src,
                                 distance_m=round(float(dist[-1]), 1),
                                 final_pos_err_m=round(float(e[-1]), 3),
                                 max_pos_err_m=round(float(e.max()), 3),
                                 drift_pct=round(float(100 * e[-1] / dist[-1]), 2),
                                 final_yaw_err_deg=round(float(yaw[-1]), 2),
                                 max_abs_yaw_err_deg=round(float(np.abs(yaw).max()), 2)))
    write_csv(os.path.join(HERE, 'odom_summary.csv'), rows)
    ax[0].set_xlabel('行驶距离 (m)')
    ax[0].set_ylabel('位置误差 (m)')
    ax[1].set_xlabel('行驶距离 (m)')
    ax[1].set_ylabel('航向误差 (°)')
    ax[0].legend(fontsize=8, frameon=False)
    fig.suptitle(f'定位漂移（起点对齐，{len(runs)} 次长路线导航，每次 3 圈）', fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, 'fig_odom_drift.png'), dpi=150)
    plt.close(fig)


# ---------------------------------------------------------------------------
# 底盘被控对象 / 多站执行
# ---------------------------------------------------------------------------
def base_report():
    rows = []
    for p in sorted(glob.glob(os.path.join(RUNS, 'base_*', 'summary.yaml'))):
        with open(p) as f:
            s = yaml.safe_load(f)
        segs = read_csv(os.path.join(os.path.dirname(p), 'segments.csv'))
        s = dict(run=os.path.basename(os.path.dirname(p)), **s)
        for r in segs:
            for k in ('vx', 'vy', 'wz'):
                v = fnum(r[f'ratio_{k}'])
                if not math.isnan(v):
                    s[f'{r["segment"]}_{k}'] = round(v, 3)
        rows.append(s)
    if rows:
        write_csv(os.path.join(HERE, 'base_summary.csv'), rows)


def slot_report():
    rows = []
    keys = ('stations_done', 'stations', 'all_ok', 'recoveries', 'stations_total_s',
            'station_lateral_mm_mean_abs', 'station_lateral_mm_max_abs',
            'station_standoff_mm_mean_abs', 'station_yaw_deg_mean_abs',
            'station_sensed_yaw_deg_mean_abs', 'path_err_mean_mm', 'path_err_p95_mm',
            'path_err_max_mm', 'coverage', 'station_base_drift_mm_max_abs')
    for p in sorted(glob.glob(os.path.join(RUNS, '*', 'summary.yaml'))):
        with open(p) as f:
            s = yaml.safe_load(f)
        if 'path_err_mean_mm' not in s and 'stations' not in s:
            continue
        st = read_csv(os.path.join(os.path.dirname(p), 'stations.csv'))
        lat = [fnum(r.get('lateral_mm')) for r in st]
        row = dict(run=os.path.basename(os.path.dirname(p)), **{k: s.get(k) for k in keys})
        row['lateral_mm_mean_signed'] = round(mean_std(lat)[0], 3)
        row['lateral_mm_std'] = round(mean_std(lat)[1], 3)
        rows.append(row)
    if rows:
        write_csv(os.path.join(HERE, 'slot_summary.csv'), rows)


def main():
    plt = setup_plot()
    nav_report(plt)
    odom_report(plt)
    base_report()
    slot_report()
    for f in sorted(os.listdir(HERE)):
        if f.endswith(('.csv', '.png')):
            print(f)


if __name__ == '__main__':
    main()
