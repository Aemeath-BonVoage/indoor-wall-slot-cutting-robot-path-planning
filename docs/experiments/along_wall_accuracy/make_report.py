#!/usr/bin/env python3
"""
沿墙精度实验的汇总表与图（读取 runs/ 下的原始数据）.

  python3 docs/experiments/along_wall_accuracy/make_report.py

输出：
  summary.csv / summary_table.md 每个配置：沿墙误差、趋势、刀尖轨迹误差、耗时、恢复 / 重新对位次数
  runs_summary.csv              每次运行的同一组指标
  stations_all.csv              全部运行的逐站数据（含配置与种子）
  fig_error_vs_distance.png     到站沿墙误差（真值）随停靠站离参考标记距离的变化
  fig_accuracy_time.png         最大沿墙误差 / 刀尖轨迹误差与总耗时
  fig_scale.png                 横移比例修正的在线估计（± 1σ）
  fig_estimate_error.png        估计的沿墙位置误差（估计 − 真值）随时间的变化
  tag_bias.csv                  标记测量误差（视线 / 垂直视线方向）随方位角的分布
"""
import csv
import glob
import math
import os
import re

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, 'runs')
MARKER_U = 1.505          # 参考标记（ID 0）在规划墙面坐标中的 u

# 配置 → (表格名称, 组别)
LABELS = {
    'single_off': ('一个标记，站间只用里程计（基线）', '基线'),
    'single_on_noobs': ('一个标记，横移时保留标记更新（不回观测位姿）', '基线'),
    'single_on': ('一个标记，横移时保留标记更新', '基线'),
    'tags_0.3_noobs': ('多标记 0.3 m，横移前不回观测位姿（试运行）', '多标记'),
    'tags_0.3': ('多标记，间距 0.3 m', '多标记'),
    'tags_0.6': ('多标记，间距 0.6 m', '多标记'),
    'tags_1.2': ('多标记，间距 1.2 m', '多标记'),
    'tags_0.6_err3': ('多标记 0.6 m，地图误差 ±3 mm', '多标记'),
    'realign_0.15': ('一个标记 + 行程 0.15 m 重新对位', '重新对位'),
    'realign_0.3': ('一个标记 + 行程 0.3 m 重新对位', '重新对位'),
    'realign_0.6': ('一个标记 + 行程 0.6 m 重新对位', '重新对位'),
    'scale_single': ('一个标记 + 横移比例修正', '比例修正'),
    'scale_realign_0.3': ('一个标记 + 比例修正 + 0.3 m 重新对位', '比例修正'),
    'scale_tags_1.2': ('多标记 1.2 m + 比例修正', '比例修正'),
    'scale_tags_0.6': ('多标记 0.6 m + 比例修正', '比例修正'),
    'scale_tags_0.6_err3': ('多标记 0.6 m + 比例修正，地图误差 ±3 mm', '比例修正'),
    'default': ('默认配置（复测，录制数据包）', '默认'),
    'single_off_bag': ('基线复测（录制数据包）', '默认'),
}
ORDER = list(LABELS)
VISIBLE = (-0.39, 0.44)   # 由 tag_window() 按数据更新


def color(c):
    import matplotlib.pyplot as plt
    return plt.get_cmap('tab20')(ORDER.index(c) % 20 if c in ORDER else 19)


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


def setup_plot():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['font.family'] = ['Noto Sans CJK JP', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    return plt


def load_runs():
    """runs/<配置>_s<种子>/ → 每次运行一条记录."""
    runs = []
    for d in sorted(glob.glob(os.path.join(RUNS, '*_s[0-9]*'))):
        m = re.match(r'(.+)_s(\d+)$', os.path.basename(d))
        if not m or not os.path.exists(os.path.join(d, 'summary.yaml')):
            continue
        with open(os.path.join(d, 'summary.yaml')) as f:
            summ = yaml.safe_load(f)
        st = read_csv(os.path.join(d, 'stations.csv'))
        ekf_path = os.path.join(d, 'ekf_trace.csv')
        runs.append(dict(config=m.group(1), seed=int(m.group(2)), dir=d, summary=summ,
                         stations=st, ekf=read_csv(ekf_path) if os.path.exists(ekf_path)
                         else []))
    return runs


def tip_errors(stations):
    """
    补偿后刀尖的沿墙误差（每站）= 到站真值误差 − 补偿所用的传感器残差.

    切割点按传感器测到的到站残差平移，因此刀尖在墙上的沿墙位置误差等于“残差测错了多少”，
    即估计误差的相反数。槽线多为水平段，刀尖轨迹误差（到规划折线的距离）对沿墙方向的
    偏移不敏感，所以单独给出这一项。
    """
    return np.array([fnum(s['lateral_mm']) - fnum(s['sensed_lateral_mm']) for s in stations])


def run_metrics(r):
    st = r['stations']
    dist = np.array([fnum(s['u_c']) - MARKER_U for s in st])
    lat = np.array([fnum(s['lateral_mm']) for s in st])
    tip = tip_errors(st)
    ok = ~np.isnan(lat)
    slope = float(np.polyfit(dist[ok], lat[ok], 1)[0]) / 10.0 if ok.sum() > 2 else math.nan
    s = r['summary']
    far = dist > 0.4
    return dict(
        config=r['config'], seed=r['seed'],
        stations_done=s.get('stations_done'), all_ok=s.get('all_ok'),
        lateral_mean_abs_mm=float(np.nanmean(np.abs(lat))),
        lateral_max_abs_mm=float(np.nanmax(np.abs(lat))),
        lateral_far_mean_abs_mm=float(np.nanmean(np.abs(lat[far]))) if far.any() else math.nan,
        lateral_slope_pct=slope,           # mm/m ÷ 10 = %
        tip_mean_abs_mm=float(np.nanmean(np.abs(tip))),
        tip_max_abs_mm=float(np.nanmax(np.abs(tip))),
        path_err_mean_mm=s.get('path_err_mean_mm'), path_err_p95_mm=s.get('path_err_p95_mm'),
        path_err_max_mm=s.get('path_err_max_mm'), coverage=s.get('coverage'),
        sim_time_s=s.get('sim_time_s'), stations_total_s=s.get('stations_total_s'),
        recoveries=s.get('recoveries'), realignments=s.get('realignments', 0),
        stations_with_tag=int(sum(1 for x in st if x.get('tags_used'))),
        final_lateral_scale=s.get('final_lateral_scale'),
        final_sigma_scale=s.get('final_sigma_scale'))


def aggregate(per_run):
    out = []
    keys = [k for k in per_run[0] if k not in ('config', 'seed', 'all_ok')]
    for c in ORDER:
        rs = [r for r in per_run if r['config'] == c]
        if not rs:
            continue
        row = dict(config=c, name=LABELS[c][0], runs=len(rs),
                   seeds=' '.join(str(r['seed']) for r in rs),
                   all_ok=all(bool(r['all_ok']) for r in rs))
        for k in keys:
            v = np.array([fnum(r[k]) for r in rs], dtype=float)
            if np.all(np.isnan(v)):
                continue
            row[k] = round(float(np.nanmean(v)), 4)
            row[k + '_min'] = round(float(np.nanmin(v)), 4)
            row[k + '_max'] = round(float(np.nanmax(v)), 4)
        out.append(row)
    return out


def fig_error_vs_distance(plt, runs):
    panels = [
        ('(a) 基线与多标记地图',
         ['single_off', 'single_on', 'tags_1.2', 'tags_0.6', 'tags_0.3', 'tags_0.6_err3']),
        ('(b) 重新对位与横移比例修正',
         ['single_on', 'realign_0.15', 'realign_0.3', 'scale_single', 'scale_tags_1.2',
          'scale_tags_0.6', 'default'])]
    fig, axes = plt.subplots(2, 2, figsize=(11, 8.0), sharey='row', sharex=True)
    for col_i, (title, cfgs) in enumerate(panels):
        for row_i in range(2):
            ax = axes[row_i, col_i]
            for c in cfgs:
                rs = [r for r in runs if r['config'] == c]
                if not rs:
                    continue
                col = color(c)
                for j, r in enumerate(rs):
                    d = [fnum(s['u_c']) - MARKER_U for s in r['stations']]
                    e = ([fnum(s['lateral_mm']) for s in r['stations']] if row_i == 0
                         else list(tip_errors(r['stations'])))
                    ax.plot(d, e, '-o' if j == 0 else '--s', color=col, ms=3.5, lw=1.2,
                            label=f'{LABELS[c][0]}（n = {len(rs)}）' if j == 0 else None)
            ax.axhline(0, color='0.5', lw=0.8)
            ax.axvspan(*VISIBLE, color='0.92', zorder=0)
            ax.grid(alpha=0.3)
            if row_i == 0:
                ax.set_title(title, fontsize=11)
                ax.legend(fontsize=7.5, loc='lower left')
            else:
                ax.set_xlabel('停靠站相对参考标记的沿墙距离 (m)')
    axes[0, 0].set_ylabel('到站沿墙误差（真值）(mm)')
    axes[1, 0].set_ylabel('补偿后刀尖沿墙误差 (mm)')
    fig.text(0.5, -0.01, '灰色区域：参考标记（ID 0）参与更新的底盘位置范围；实线 / 虚线为打滑种子 1 / 2',
             ha='center', fontsize=8.5)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, 'fig_error_vs_distance.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)


def fig_accuracy_time(plt, per_run):
    cfgs = [c for c in ORDER if any(r['config'] == c for r in per_run)]
    fig, axes = plt.subplots(1, 2, figsize=(11, 5.4))
    for c in cfgs:
        rs = [r for r in per_run if r['config'] == c]
        col = color(c)
        t = [fnum(r['sim_time_s']) for r in rs]
        axes[0].scatter(t, [fnum(r['lateral_max_abs_mm']) for r in rs], color=col, s=28,
                        label=LABELS[c][0])
        axes[1].scatter(t, [fnum(r['tip_max_abs_mm']) for r in rs], color=col, s=28)
    axes[0].set_ylabel('最大到站沿墙误差（真值）(mm)')
    axes[1].set_ylabel('补偿后刀尖最大沿墙误差 (mm)')
    for ax in axes:
        ax.set_xlabel('任务总耗时（仿真秒，含导航与精对位）')
        ax.set_yscale('log')
        ax.grid(alpha=0.3, which='both')
    h, lab = axes[0].get_legend_handles_labels()
    fig.legend(h, lab, loc='lower center', ncol=3, fontsize=8, bbox_to_anchor=(0.5, -0.02),
               frameon=False)
    fig.tight_layout(rect=(0, 0.17, 1, 1))
    fig.savefig(os.path.join(HERE, 'fig_accuracy_time.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)


def fig_scale(plt, runs):
    cfgs = [c for c in ('scale_single', 'scale_tags_1.2', 'scale_tags_0.6', 'default')
            if any(r['config'] == c for r in runs)]
    if not cfgs:
        return
    fig, ax = plt.subplots(figsize=(7.5, 3.8))
    for i, c in enumerate(cfgs):
        for j, r in enumerate(x for x in runs if x['config'] == c):
            e = r['ekf']
            if not e:
                continue
            t = np.array([fnum(x['t']) for x in e])
            s = np.array([fnum(x['lateral_scale']) for x in e]) * 100
            sg = np.array([fnum(x['sigma_scale']) for x in e]) * 100
            t = t - t[0]
            ax.plot(t, s, color=color(c), lw=1.0, ls='-' if j == 0 else '--',
                    label=LABELS[c][0] if j == 0 else None)
            if j == 0:
                ax.fill_between(t, s - sg, s + sg, color=color(c), alpha=0.15, lw=0)
    ax.axhline((1 / 1.03 - 1) * 100, color='k', lw=0.8, ls=':', label='编码器模型的横向打滑 3 %')
    ax.set_ylim(-6, 2)
    ax.set_xlabel('时间（仿真秒，自首次估计起）')
    ax.set_ylabel('横移比例修正 s (%)')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, 'fig_scale.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)


def fig_estimate_error(plt, runs):
    cfgs = [c for c in ('single_off', 'single_on', 'tags_0.6', 'scale_single', 'default')
            if any(r['config'] == c for r in runs)]
    fig, ax = plt.subplots(figsize=(7.5, 3.8))
    for i, c in enumerate(cfgs):
        r = next(x for x in runs if x['config'] == c)
        e = r['ekf']
        if not e:
            continue
        t = np.array([fnum(x['t']) for x in e])
        err = np.array([fnum(x['u']) - fnum(x['true_u']) for x in e]) * 1000
        ax.plot(t - t[0], err, color=color(c), lw=0.9, label=f'{LABELS[c][0]}（种子 {r["seed"]}）')
    ax.set_xlabel('时间（仿真秒，自首次估计起）')
    ax.set_ylabel('沿墙位置估计误差 û − u (mm)')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, 'fig_estimate_error.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)


TAG_U_WORLD = {5: -0.3, 0: 0.0, 1: 0.3, 2: 0.6, 3: 0.9, 4: 1.2}   # 世界中的实际位置


def tag_window(runs):
    """工作距离（d < 0.75 m）下标记被接受时，底盘相对该标记的沿墙位置范围（米）."""
    rel = [fnum(e['true_u']) - TAG_U_WORLD[int(i)] for r in runs for e in r['ekf']
           if fnum(e['true_d']) < 0.75 for i in e['tags_accepted'].split()
           if int(i) in TAG_U_WORLD]
    return (min(rel), max(rel)) if rel else VISIBLE


def tag_bias(runs):
    """
    标记测量误差：测得的标记中心（base_link）− 由真值位姿与世界中标记位置算出的值.

    分解为视线方向（深度）与垂直视线方向（方位）两个分量，并按方位角分组。只取底盘静止
    （到站后的残差测量阶段）的样本，避免运动中的时间对齐误差。
    """
    rows = []
    for r in runs:
        for e in r['ekf']:
            if e['phase'] != 'arrived' or not e.get('tag_xy'):
                continue
            d, u, psi = fnum(e['true_d']), fnum(e['true_u']), fnum(e['true_psi'])
            c, s = math.cos(psi), math.sin(psi)
            for item in e['tag_xy'].split():
                i, x, y = item.split(':')
                i = int(i)
                if i not in TAG_U_WORLD:
                    continue
                du = u - TAG_U_WORLD[i]
                px, py = d * c + du * s, -d * s + du * c
                ex, ey = float(x) - px, float(y) - py
                rng = math.hypot(px, py)
                lx, ly = px / rng, py / rng
                rows.append(dict(config=r['config'], seed=r['seed'], tag=i,
                                 bearing_deg=round(math.degrees(math.atan2(py, px)), 2),
                                 err_los_mm=round((ex * lx + ey * ly) * 1000, 3),
                                 err_perp_mm=round((-ex * ly + ey * lx) * 1000, 3)))
    if not rows:
        return []
    out = []
    edges = [-40, -25, -15, -5, 5, 15, 25, 40]
    for lo, hi in zip(edges[:-1], edges[1:]):
        sel = [x for x in rows if lo <= x['bearing_deg'] < hi]
        if not sel:
            continue
        los = np.array([x['err_los_mm'] for x in sel])
        perp = np.array([x['err_perp_mm'] for x in sel])
        out.append(dict(bearing_deg=f'{lo}…{hi}', samples=len(sel),
                        los_mean_mm=round(float(los.mean()), 3),
                        los_std_mm=round(float(los.std()), 3),
                        perp_mean_mm=round(float(perp.mean()), 3),
                        perp_std_mm=round(float(perp.std()), 3)))
    return out


def markdown_table(agg):
    """README 用的汇总表：均值（两次运行的范围）."""
    def num(v, nd):
        v = 0.0 if round(v, nd) == 0 else v           # 避免 “−0.00”
        return f'{v:.{nd}f}'.replace('-', '−')

    def f(r, k, nd=1):
        if k not in r:
            return '—'
        if r['runs'] > 1 and abs(r[k + '_max'] - r[k + '_min']) > 10 ** -nd / 2:
            return f"{num(r[k], nd)}（{num(r[k + '_min'], nd)} … {num(r[k + '_max'], nd)}）"
        return num(r[k], nd)
    lines = ['| 配置 | n | 到站沿墙误差 平均 / 最大 (mm) | 趋势 (%) | 补偿后刀尖沿墙误差 最大 (mm) '
             '| 刀尖轨迹误差 平均 / 95 分位 / 最大 (mm) | 覆盖率 | 总耗时 (s) | 恢复 / 重新对位 |',
             '|---|---|---|---|---|---|---|---|---|']
    for r in agg:
        lines.append(
            f"| `{r['config']}` {r['name']} | {r['runs']} | {f(r, 'lateral_mean_abs_mm')} / "
            f"{f(r, 'lateral_max_abs_mm')} | {f(r, 'lateral_slope_pct', 2)} | "
            f"{f(r, 'tip_max_abs_mm')} | {r['path_err_mean_mm']:.2f} / {r['path_err_p95_mm']:.2f}"
            f" / {r['path_err_max_mm']:.2f} | {r['coverage'] * 100:.1f} % | "
            f"{f(r, 'sim_time_s', 0)} | {r['recoveries']:.0f} / {r['realignments']:.0f} |")
    return '\n'.join(lines) + '\n'


def main():
    runs = load_runs()
    if not runs:
        print('runs/ 下没有数据')
        return
    per_run = [run_metrics(r) for r in runs]
    per_run.sort(key=lambda r: (ORDER.index(r['config']) if r['config'] in ORDER else 99,
                                r['seed']))
    write_csv(os.path.join(HERE, 'runs_summary.csv'),
              [{k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()}
               for r in per_run])
    agg = aggregate(per_run)
    write_csv(os.path.join(HERE, 'summary.csv'), agg)
    with open(os.path.join(HERE, 'summary_table.md'), 'w', encoding='utf-8') as f:
        f.write(markdown_table(agg))
    allst = []
    for r in runs:
        for s in r['stations']:
            allst.append(dict(config=r['config'], seed=r['seed'],
                              dist_from_tag_m=round(fnum(s['u_c']) - MARKER_U, 4), **s))
    write_csv(os.path.join(HERE, 'stations_all.csv'), allst)
    tb = tag_bias(runs)
    if tb:
        write_csv(os.path.join(HERE, 'tag_bias.csv'), tb)
        for row in tb:
            print('tag bias', row)
    global VISIBLE
    VISIBLE = tag_window(runs)
    print(f'标记参与更新的范围（底盘 − 标记）：{VISIBLE[0]:+.3f} … {VISIBLE[1]:+.3f} m')
    plt = setup_plot()
    fig_error_vs_distance(plt, runs)
    fig_accuracy_time(plt, per_run)
    fig_scale(plt, runs)
    fig_estimate_error(plt, runs)
    for r in aggregate(per_run):
        print(f"{r['config']:22s} n={r['runs']} max {r.get('lateral_max_abs_mm')} "
              f"mean {r.get('lateral_mean_abs_mm')} slope {r.get('lateral_slope_pct')} % "
              f"tip {r.get('tip_max_abs_mm')} p95 {r.get('path_err_p95_mm')} "
              f"t {r.get('sim_time_s')} s "
              f"realign {r.get('realignments')} rec {r.get('recoveries')}")


if __name__ == '__main__':
    main()
