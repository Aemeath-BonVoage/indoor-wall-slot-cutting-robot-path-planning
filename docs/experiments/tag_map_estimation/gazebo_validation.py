#!/usr/bin/env python3
"""
第 8 节 Gazebo 验证的汇总：与离线预期值（gazebo_predictions.csv）逐项对照.

  python3 gazebo_validation.py

读取 gazebo_runs/<运行名>/（run_gazebo.sh 的输出）与 along_wall_accuracy/runs/ 中的对照运行，
站点误差与离线仿真器的符号相同：e = û − u = −(lateral_mm − sensed_lateral_mm)。
输出：
  gazebo_runs.csv          每次运行：11 站补偿后刀尖沿墙误差（最大 / 带符号平均）、地图估计
                           （ID 2 / ID 4 的误差与导出 σ、归一化误差、2 维 NEES）、各标记的
                           接受率、覆盖率、恢复次数
  gazebo_vs_prediction.csv 每个配置：Gazebo 结果与预期（每次最大的平均与 5–95 分位、
                           带符号平均、地图误差）
  gazebo_map_history.csv   日志中每次写出地图时的估计（ID、均值、σ；墙钟时间，相对任务开始）
  fig_gazebo_stations.png/pdf  每次运行的最大误差与预期区间
  fig_gazebo_map.png/pdf       地图估计误差 ± 导出 σ（三个标记），全部在线估计运行的归一化误差
"""
import csv
import glob
import math
import os
import re
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, 'gazebo_runs')
AW_RUNS = os.path.join(HERE, '..', 'along_wall_accuracy', 'runs')
sys.path.insert(0, os.path.join(HERE, '..', '..'))
import figstyle as fs  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch, Rectangle  # noqa: E402

TRUTH = {0: 0.0, 2: 0.6, 4: 1.2}     # room.world 中的标记位置（相对参考标记 ID 0）

# 运行组 → (配置名, 预期值行, 说明)
GROUPS = [
    ('G1', 'known_err5', 'known_err5_p1', 'known map, recorded ±5 mm'),
    ('G2', 'slam_ok', 'slam_ok_p1', 'online map, recorded map correct'),
    ('G3', 'slam_err3', 'slam_err3_p1', 'online map, recorded ±3 mm'),
    ('G4', 'slam_err5', 'slam_err5_p1', 'online map, recorded ±5 mm'),
    ('G5', 'slam_err5_pass2', 'slam_err5_p2', 'second pass, prior = G4 estimate'),
    ('G6', 'slam_err3_pass2', 'slam_err3_p2', 'second pass, prior = G3 estimate'),
    ('G7', 'slam_err5_noscale', 'slam_err5_noscale_p1', 'online map ±5 mm, no strafe scale'),
    ('G8', 'tags_0.6_noodomerr', 'tags_0.6_noodomerr_p1', 'known map, no scale, odom error off'),
    ('G9', 'aw:tags_0.6', 'tags_0.6_p1', 'known map, no scale (along_wall runs)'),
    ('G10', 'slam2_err5', 'slam2_err5_p1', 'two markers, online map, ID 4 −5 mm'),
    ('G11', 'slam2_err5_noscale', 'slam2_err5_noscale_p1', 'two markers, online map, no scale'),
]
# 其他对照（along_wall 中已有的运行，不重跑）
REFERENCES = ['scale_tags_0.6', 'default', 'scale_tags_0.6_err3', 'scale_tags_1.2', 'tags_1.2']


def fnum(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return math.nan


def read_csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def run_dirs(config):
    """配置名 → [(种子, 目录)]；aw: 前缀表示 along_wall 中的运行."""
    if config.startswith('aw:'):
        base, config = AW_RUNS, config[3:]
    else:
        base = RUNS
    out = []
    for d in sorted(glob.glob(os.path.join(base, f'{config}_s[0-9]*'))):
        if re.fullmatch(re.escape(config) + r'_s\d+', os.path.basename(d)) and \
                os.path.exists(os.path.join(d, 'summary.yaml')):
            out.append((int(d.rsplit('_s', 1)[1]), d))
    return out


def tag_acceptance(d):
    """ekf_trace.csv：每个标记 被看到的帧数 / 接受的帧数."""
    seen, acc = {}, {}
    path = os.path.join(d, 'ekf_trace.csv')
    if not os.path.exists(path):
        return {}
    for r in read_csv(path):
        ids = [int(x) for x in r['tag_ids'].split()] if r.get('tag_ids') else []
        ok = [int(x) for x in r['tags_accepted'].split()] if r.get('tags_accepted') else []
        for i in ids:
            seen[i] = seen.get(i, 0) + 1
        for i in ok:
            acc[i] = acc.get(i, 0) + 1
    return {i: (seen[i], acc.get(i, 0)) for i in sorted(seen)}


def arrival_bias(d):
    """每站到站残差测量结束时的 û − u、ψ̂ − ψ 与 d·(ψ̂ − ψ)（与 bias_analysis.py 相同的取法）."""
    path = os.path.join(d, 'ekf_trace.csv')
    if not os.path.exists(path):
        return {}
    last = {}
    for r in read_csv(path):
        if r['phase'] == 'arrived' and int(r['station']) >= 0:
            last[int(r['station'])] = r
    if not last:
        return {}
    eu = np.array([fnum(r['u']) - fnum(r['true_u']) for r in last.values()]) * 1e3
    ep = np.array([fnum(r['psi']) - fnum(r['true_psi']) for r in last.values()])
    dp = np.array([fnum(r['true_d']) for r in last.values()]) * ep * 1e3
    return dict(arrive_u_err_mean_mm=float(eu.mean()),
                arrive_psi_err_mean_deg=float(np.degrees(ep.mean())),
                arrive_d_psi_err_mean_mm=float(dp.mean()))


def load_map(d):
    path = os.path.join(d, 'tag_map_estimate.yaml')
    if not os.path.exists(path):
        return None
    with open(path, encoding='utf-8') as f:
        m = yaml.safe_load(f)['tag_map']
    ids = m['covariance']['ids']
    cov = np.array(m['covariance']['matrix'], dtype=float)
    u = {t['id']: float(t['u']) for t in m['tags']}
    mean = np.array([u[i] for i in ids])
    return ids, mean, cov


def map_history(d):
    """task_layer.log 中“标记地图估计：ID i u = … ± … mm”的各行."""
    rows, t0 = [], None
    path = os.path.join(d, 'task_layer.log')
    if not os.path.exists(path):
        return rows
    pat = re.compile(r'ID (\d+) u = ([\d.\-]+) ± ([\d.]+) mm')
    for line in open(path, errors='replace'):
        m = re.search(r'\[(\d+\.\d+)\] \[task_skills\]: 标记地图估计：(.*)', line)
        if not m:
            continue
        t = float(m.group(1))
        t0 = t if t0 is None else t0
        for i, u, s in pat.findall(m.group(2)):
            rows.append((t - t0, int(i), float(u), float(s)))
    return rows


def run_metrics(group, config, seed, d):
    st = read_csv(os.path.join(d, 'stations.csv'))
    # 与离线仿真器相同的符号：e = û − u = −(lateral_mm − sensed_lateral_mm)（validate_sim.py）
    err = -np.array([fnum(s['lateral_mm']) - fnum(s['sensed_lateral_mm']) for s in st])
    with open(os.path.join(d, 'summary.yaml')) as f:
        sm = yaml.safe_load(f)
    row = dict(group=group, config=config.replace('aw:', ''), seed=seed,
               run=os.path.basename(d), stations=len(err),
               run_max_mm=float(np.nanmax(np.abs(err))) if len(err) else math.nan,
               signed_mean_mm=float(np.nanmean(err)) if len(err) else math.nan,
               all_ok=sm.get('all_ok'), recoveries=sm.get('recoveries'),
               coverage=sm.get('coverage'), path_err_max_mm=sm.get('path_err_max_mm'),
               final_lateral_scale=sm.get('final_lateral_scale'),
               stations_total_s=sm.get('stations_total_s'))
    row.update(arrival_bias(d))
    acc = tag_acceptance(d)
    for i in (0, 2, 4):
        n, a = acc.get(i, (0, 0))
        row[f'tag{i}_frames'] = n
        row[f'tag{i}_accept'] = round(a / n, 4) if n else ''
    mp = load_map(d)
    if mp is not None:
        ids, mean, cov = mp
        e = mean - np.array([TRUTH[i] for i in ids])
        for k, i in enumerate(ids):
            s = math.sqrt(cov[k, k])
            row[f'map_err_id{i}_mm'] = e[k] * 1e3
            row[f'map_sigma_id{i}_mm'] = s * 1e3
            row[f'map_z_id{i}'] = e[k] / s
        row['map_nees'] = float(e @ np.linalg.solve(cov, e))
        row['map_dof'] = len(ids)
    return row, err


def load_predictions():
    rows = read_csv(os.path.join(HERE, 'gazebo_predictions.csv'))
    return {r['config']: r for r in rows}


def verdict(g, rows, pred):
    """成立 / 不成立 / 未能确认：每次最大误差是否都落在预期 5–95 分位内."""
    if not rows or pred is None:
        return '未能确认'
    lo, hi = fnum(pred['run_max_p5_mm']), fnum(pred['run_max_p95_mm'])
    inside = [lo <= r['run_max_mm'] <= hi for r in rows]
    return f'{sum(inside)}/{len(inside)} in 5–95 %'


def main():
    preds = load_predictions()
    all_rows, errs, summary = [], {}, []
    for g, config, pkey, desc in GROUPS:
        rows = []
        for seed, d in run_dirs(config):
            r, e = run_metrics(g, config, seed, d)
            rows.append(r)
            errs[(g, seed)] = e
        all_rows += rows
        p = preds.get(pkey)
        s = dict(group=g, config=config.replace('aw:', ''), description=desc, n=len(rows))
        if rows:
            mx = [r['run_max_mm'] for r in rows]
            s.update(run_max_mm=' / '.join(f'{x:.2f}' for x in mx),
                     signed_mean_mm=' / '.join(f'{r["signed_mean_mm"]:+.2f}' for r in rows))
            for i in (2, 4):
                k = f'map_err_id{i}_mm'
                if k in rows[0]:
                    s[f'map_id{i}_mm'] = ' / '.join(
                        f'{r[k]:+.2f} ± {r[f"map_sigma_id{i}_mm"]:.2f}' for r in rows if k in r)
            if 'map_nees' in rows[0]:
                s['map_nees'] = ' / '.join(f'{r["map_nees"]:.1f}' for r in rows)
        if p is not None:
            band = f'{fnum(p["run_max_p5_mm"]):.2f}–{fnum(p["run_max_p95_mm"]):.2f}'
            s.update(pred_run_max_mm=f'{fnum(p["run_max_mean_mm"]):.2f} ({band})',
                     pred_signed_mean_mm=p['signed_mean_mm'],
                     pred_map_id2_mm=p['map_err_id2_mm'], pred_map_id4_mm=p['map_err_id4_mm'])
        s['inside_band'] = verdict(g, rows, p)
        summary.append(s)
    for c in REFERENCES:
        for seed, d in run_dirs('aw:' + c):
            r, e = run_metrics('ref', 'aw:' + c, seed, d)
            all_rows.append(r)
            errs[('ref:' + c, seed)] = e

    fields = list(dict.fromkeys(k for r in all_rows for k in r))
    with open(os.path.join(HERE, 'gazebo_runs.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in all_rows:
            w.writerow({k: (f'{v:.4f}' if isinstance(v, float) else v) for k, v in r.items()})
    fields = list(dict.fromkeys(k for r in summary for k in r))
    with open(os.path.join(HERE, 'gazebo_vs_prediction.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(summary)
    hist = []
    for g, config, _, _ in GROUPS:
        for seed, d in run_dirs(config):
            hist += [(g, seed, *h) for h in map_history(d)]
    with open(os.path.join(HERE, 'gazebo_map_history.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['group', 'seed', 'wall_t_s', 'id', 'u_m', 'sigma_mm'])
        w.writerows(hist)
    for s in summary:
        print(s)
    figures(all_rows, preds, hist)


def figures(rows, preds, hist):
    groups = [g for g in GROUPS if any(r['group'] == g[0] for r in rows)]
    # ---- 图 1：每次运行的最大误差与预期区间
    fig, ax = fs.subplots(width='double', height=2.6)
    for k, (g, config, pkey, desc) in enumerate(groups):
        p = preds.get(pkey)
        if p is not None:
            lo, hi, mu = (fnum(p[c]) for c in ('run_max_p5_mm', 'run_max_p95_mm',
                                               'run_max_mean_mm'))
            ax.add_patch(Rectangle((k - 0.3, lo), 0.6, hi - lo,
                                   color=fs.color('neutral'), alpha=0.3, lw=0))
            ax.plot([k - 0.3, k + 0.3], [mu, mu], color=fs.color('reference'), lw=1.0)
        for r in (r for r in rows if r['group'] == g):
            role = 'proposed' if 'slam' in config else 'baseline'
            ax.plot(k + (r['seed'] - 1.5) * 0.2, r['run_max_mm'], **fs.marker(role, ms=4))
    ax.set_xticks(range(len(groups)))
    ax.set_xticklabels([g[0] for g in groups])
    ax.set_yscale('log')
    ax.set_yticks([0.5, 1, 2, 5, 10, 20])
    ax.set_yticklabels(['0.5', '1', '2', '5', '10', '20'])
    ax.set_ylim(0.4, 25)
    ax.set_xlim(-0.6, len(groups) - 0.4)
    fs.light_grid(ax, 'y')
    ax.set_ylabel(fs.label('Max. along-wall error per run', 'mm'))
    ax.set_xlabel('Run group (two runs each, seeds 1 and 2)')
    h = [Line2D([], [], label='Online map estimation', **fs.marker('proposed')),
         Line2D([], [], label='Known (recorded) map', **fs.marker('baseline')),
         Line2D([], [], color=fs.color('reference'), lw=1.0, label='Offline prediction, mean'),
         Patch(color=fs.color('neutral'), alpha=0.3, label='Offline prediction, 5–95 %')]
    ax.legend(handles=h, loc='upper center', ncol=4, fontsize='small',
              bbox_to_anchor=(0.5, 1.13), frameon=False)
    fs.save(fig, os.path.join(HERE, 'fig_gazebo_stations'))

    # ---- 图 2：(a) 三个标记时的地图误差 ± 导出 σ；(b) 归一化误差 e / σ
    fig, (a1, a2) = fs.subplots(1, 2, width='double', height=2.8,
                                gridspec_kw=dict(width_ratios=[1.25, 1]))
    mrows = [r for r in rows if 'map_nees' in r]
    three = [r for r in mrows if r['config'].startswith('slam_')]
    for k, r in enumerate(three):
        for i, dx, role in ((2, -0.13, 'proposed'), (4, 0.13, 'alt')):
            a1.errorbar(k + dx, r[f'map_err_id{i}_mm'], yerr=r[f'map_sigma_id{i}_mm'],
                        capsize=2, label=f'ID {i} (truth {TRUTH[i]:.1f} m)' if k == 0 else None,
                        **fs.marker(role))
    a1.axhline(0, **fs.line('reference', lw=0.6))
    a1.set_xticks(range(len(three)))
    a1.set_xticklabels([f'{r["group"]}-s{r["seed"]}' for r in three], rotation=90)
    a1.set_ylabel(fs.label('Map error, estimate − truth', 'mm'))
    a1.set_ylim(top=2.7)
    a1.legend(fontsize='small', loc='upper center', ncol=2)
    fs.light_grid(a1, 'y')
    fs.panel_label(a1, 'a', 'Three markers, error ± exported 1σ')
    for k, r in enumerate(mrows):
        for i, dx, role in ((2, -0.13, 'proposed'), (4, 0.13, 'alt')):
            if f'map_z_id{i}' in r:
                a2.plot(k + dx, r[f'map_z_id{i}'], **fs.marker(role))
    for y in (-2, -1, 1, 2):
        a2.axhline(y, **fs.line('reference', lw=0.5))
    a2.axhline(0, color='k', lw=0.4)
    a2.set_yscale('symlog', linthresh=3.0)
    a2.set_yticks([-2, -1, 0, 1, 2, 3, 10])
    a2.set_yticklabels(['−2', '−1', '0', '1', '2', '3', '10'])
    a2.set_xticks(range(len(mrows)))
    a2.set_xticklabels([f'{r["group"]}-s{r["seed"]}' for r in mrows], rotation=90)
    a2.set_ylabel('Normalised map error e / σ')
    fs.panel_label(a2, 'b', 'All runs with online map')
    fs.save(fig, os.path.join(HERE, 'fig_gazebo_map'))


if __name__ == '__main__':
    main()
