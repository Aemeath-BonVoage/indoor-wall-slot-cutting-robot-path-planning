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

图使用 docs/figstyle 的样式（PDF + PNG）：颜色表示方法组（基线 / 多标记 / 重新对位 /
比例修正 / 默认），标记形状区分组内配置，实线 / 虚线区分打滑种子。
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

# 图中的名称
LABELS_EN = {
    'single_off': 'One marker, odometry only between stations (baseline)',
    'single_on_noobs': 'One marker, marker updates while strafing, no observe pose',
    'single_on': 'One marker, marker updates while strafing',
    'tags_0.3_noobs': 'Multi-marker 0.3 m, no observe pose (trial run)',
    'tags_0.3': 'Multi-marker map, spacing 0.3 m',
    'tags_0.6': 'Multi-marker map, spacing 0.6 m',
    'tags_1.2': 'Multi-marker map, spacing 1.2 m',
    'tags_0.6_err3': 'Multi-marker 0.6 m, map error ±3 mm',
    'realign_0.15': 'One marker + re-alignment every 0.15 m',
    'realign_0.3': 'One marker + re-alignment every 0.3 m',
    'realign_0.6': 'One marker + re-alignment every 0.6 m',
    'scale_single': 'One marker + strafe-scale correction',
    'scale_realign_0.3': 'One marker + scale correction + 0.3 m re-alignment',
    'scale_tags_1.2': 'Multi-marker 1.2 m + scale correction',
    'scale_tags_0.6': 'Multi-marker 0.6 m + scale correction',
    'scale_tags_0.6_err3': 'Multi-marker 0.6 m + scale correction, map error ±3 mm',
    'default': 'Default configuration (rerun, bag recorded)',
    'single_off_bag': 'Baseline rerun (bag recorded)',
}
GROUP_COLOR = {'基线': 'baseline', '多标记': 'alt', '重新对位': 'alt2', '比例修正': 'proposed',
               '默认': 'reference'}
GROUP_MARKERS = ('o', 's', '^', 'D', 'v', 'P')


def color(c):
    """颜色按方法组（docs/figstyle 的语义色）."""
    import figstyle as fs
    return fs.color(GROUP_COLOR[LABELS[c][1]])


def mark(c):
    """组内配置按出现顺序分配标记形状."""
    same = [k for k in ORDER if LABELS[k][1] == LABELS[c][1]]
    return GROUP_MARKERS[same.index(c) % len(GROUP_MARKERS)]


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
    import sys
    sys.path.insert(0, os.path.join(HERE, '..', '..'))
    import figstyle as fs
    return fs


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


def fig_error_vs_distance(fs, runs):
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    panels = [
        ('Baseline and multi-marker maps',
         ['single_off', 'single_on', 'tags_1.2', 'tags_0.6', 'tags_0.3', 'tags_0.6_err3']),
        ('Re-alignment and strafe-scale correction',
         ['single_on', 'realign_0.15', 'realign_0.3', 'scale_single', 'scale_tags_1.2',
          'scale_tags_0.6', 'default'])]
    fig, axes = fs.subplots(2, 2, width='double', height=5.6, sharey='row', sharex=True)
    for col_i, (title, cfgs) in enumerate(panels):
        handles = []
        for row_i in range(2):
            ax = axes[row_i, col_i]
            for c in cfgs:
                rs = [r for r in runs if r['config'] == c]
                if not rs:
                    continue
                for j, r in enumerate(rs):
                    d = [fnum(s['u_c']) - MARKER_U for s in r['stations']]
                    e = ([fnum(s['lateral_mm']) for s in r['stations']] if row_i == 0
                         else list(tip_errors(r['stations'])))
                    ax.plot(d, e, color=color(c), marker=mark(c), ms=2.8, lw=0.8,
                            mfc='white' if j else color(c), ls='-' if j == 0 else (0, (3, 1.5)))
                if row_i == 0:
                    handles.append(Line2D([], [], color=color(c), marker=mark(c), ms=2.8, lw=0.8,
                                          label=f'{LABELS_EN[c]} (n = {len(rs)})'))
            ax.axhline(0, color=fs.GREY, lw=0.5)
            ax.axvspan(*VISIBLE, color='0.92', lw=0, zorder=0)
            fs.light_grid(ax)
            if row_i == 1:
                ax.set_xlabel(fs.label('Station distance from the reference marker', 'm'))
        handles += [Line2D([], [], color=fs.BLACK, lw=0.8, label='Slip seed 1'),
                    Line2D([], [], color=fs.BLACK, lw=0.8, ls=(0, (3, 1.5)), marker='o',
                           ms=2.8, mfc='white', label='Slip seed 2'),
                    Patch(color='0.92', label='Reference marker (ID 0) in view')]
        axes[1, col_i].legend(handles=handles, loc='upper center', bbox_to_anchor=(0.5, -0.2),
                              ncol=1)
    axes[0, 0].set_ylabel(fs.label('Along-wall error on arrival', 'mm'))
    axes[1, 0].set_ylabel(fs.label('Tool-tip along-wall error\nafter compensation', 'mm'))
    fs.panel_labels(axes.flat, ['Baseline and multi-marker maps: arrival error',
                                'Re-alignment and scale correction: arrival error',
                                'Baseline and multi-marker maps: tool tip',
                                'Re-alignment and scale correction: tool tip'])
    fs.save(fig, os.path.join(HERE, 'fig_error_vs_distance.png'))


def fig_accuracy_time(fs, per_run):
    from matplotlib.lines import Line2D
    cfgs = [c for c in ORDER if any(r['config'] == c for r in per_run)]
    fig, axes = fs.subplots(1, 2, width='double', height=4.0)
    handles = []
    for c in cfgs:
        rs = [r for r in per_run if r['config'] == c]
        t = [fnum(r['sim_time_s']) for r in rs]
        kw = dict(color=color(c), marker=mark(c), ls='none', ms=3.5, mfc='none', mew=0.9)
        axes[0].plot(t, [fnum(r['lateral_max_abs_mm']) for r in rs], **kw)
        axes[1].plot(t, [fnum(r['tip_max_abs_mm']) for r in rs], **kw)
        handles.append(Line2D([], [], label=LABELS_EN[c], **kw))
    axes[0].set_ylabel(fs.label('Max. along-wall error on arrival', 'mm'))
    axes[1].set_ylabel(fs.label('Max. tool-tip along-wall error', 'mm'))
    for ax in axes:
        ax.set_xlabel(fs.label('Total task time incl. navigation and alignment', 's'))
        ax.set_yscale('log')
        fs.plain_log(ax)
        fs.light_grid(ax)
    fs.panel_labels(axes)
    fig.legend(handles=handles, loc='outside lower center', ncol=2)
    fs.save(fig, os.path.join(HERE, 'fig_accuracy_time.png'))


def fig_scale(fs, runs):
    cfgs = [c for c in ('scale_single', 'scale_tags_1.2', 'scale_tags_0.6', 'default')
            if any(r['config'] == c for r in runs)]
    if not cfgs:
        return
    fig, ax = fs.subplots(width='single', height=3.0)
    for i, c in enumerate(cfgs):
        for j, r in enumerate(x for x in runs if x['config'] == c):
            e = r['ekf']
            if not e:
                continue
            t = np.array([fnum(x['t']) for x in e])
            s = np.array([fnum(x['lateral_scale']) for x in e]) * 100
            sg = np.array([fnum(x['sigma_scale']) for x in e]) * 100
            t = t - t[0]
            lo, hi = -6.0, 2.0          # 纵轴范围；范围外的线段不画，±1σ 带截到范围边界
            ax.plot(t, np.where((s >= lo) & (s <= hi), s, np.nan), color=color(c), lw=0.8,
                    ls='-' if j == 0 else (0, (3, 1.5)), label=LABELS_EN[c] if j == 0 else None)
            if j == 0:
                ax.fill_between(t, np.clip(s - sg, lo, hi), np.clip(s + sg, lo, hi),
                                color=color(c), alpha=0.15, lw=0)
    ax.axhline((1 / 1.03 - 1) * 100, **fs.line('reference', lw=0.8,
                                               label='Encoder model: 3 % lateral slip'))
    ax.set_ylim(-6, 2)
    ax.set_xlabel(fs.label('Time since first estimate', 's'))
    ax.set_ylabel(fs.label('Strafe-scale correction s', '%'))
    fs.light_grid(ax)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.22), ncol=1)
    fs.save(fig, os.path.join(HERE, 'fig_scale.png'))


def fig_estimate_error(fs, runs):
    cfgs = [c for c in ('single_off', 'single_on', 'tags_0.6', 'scale_single', 'default')
            if any(r['config'] == c for r in runs)]
    fig, ax = fs.subplots(width='single', height=3.0)
    styles = ['-', (0, (3, 1.5)), (0, (4, 1, 1, 1)), (0, (1, 1)), '-']
    for i, c in enumerate(cfgs):
        r = next(x for x in runs if x['config'] == c)
        e = r['ekf']
        if not e:
            continue
        t = np.array([fnum(x['t']) for x in e])
        err = np.array([fnum(x['u']) - fnum(x['true_u']) for x in e]) * 1000
        ax.plot(t - t[0], err, color=color(c), lw=0.8, ls=styles[i % len(styles)],
                label=f'{LABELS_EN[c]} (seed {r["seed"]})')
    ax.set_xlabel(fs.label('Time since first estimate', 's'))
    ax.set_ylabel(fs.label('Along-wall estimate error û − u', 'mm'))
    fs.light_grid(ax)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.22), ncol=1)
    fs.save(fig, os.path.join(HERE, 'fig_estimate_error.png'))


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
    fs = setup_plot()
    fig_error_vs_distance(fs, runs)
    fig_accuracy_time(fs, per_run)
    fig_scale(fs, runs)
    fig_estimate_error(fs, runs)
    for r in aggregate(per_run):
        print(f"{r['config']:22s} n={r['runs']} max {r.get('lateral_max_abs_mm')} "
              f"mean {r.get('lateral_mean_abs_mm')} slope {r.get('lateral_slope_pct')} % "
              f"tip {r.get('tip_max_abs_mm')} p95 {r.get('path_err_p95_mm')} "
              f"t {r.get('sim_time_s')} s "
              f"realign {r.get('realignments')} rec {r.get('recoveries')}")


if __name__ == '__main__':
    main()
