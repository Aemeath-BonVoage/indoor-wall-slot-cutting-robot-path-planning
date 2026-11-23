#!/usr/bin/env python3
"""
Gazebo 验证数据的整理与作图（不需要 ROS；读取 runs/ 下已有的记录）.

  python3 docs/experiments/cutting_force/gazebo_analysis.py [--only step|stations|feed]

1. step：外力阶跃试验（runs/step/*.csv，hold_step_test.py 的输出）与离线模型的对比。对每个
   试验用两个版本的对象模型各算一次预测：第一版（里程计按连续积分）与修正后的模型
   （里程计按控制周期采样累加）。输出 gazebo_step.csv、fig_gazebo_step.png；
2. stations：带切割力的整墙试验（runs/<配置>_<规划>_s<种子>/，run_matrix.sh 的输出）的
   指标汇总 station_runs.csv、按配置的统计 station_summary.csv 与 fig_gazebo_stations.png；
3. feed：切割力模型给出的进给上限（各材料、各项约束）feed_limit.csv，以及进给速度对
   联合规划收益的影响（读取 benchmark/ 下的汇总，存在时才画）fig_feed_limit.png。
"""
import argparse
import csv
import glob
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', '..'))

import plant_sim as ps  # noqa: E402
import station_metrics as sm  # noqa: E402
from plant_sim import cf  # noqa: E402

TIP_X = ps.PF.tool_tip(cf.SLOT_DEPTH)[0]
MODE = {'p': 'P', 'pi': 'PI', 'pid': 'PID', 'dob': 'DOB', 'lqg': 'LQG'}
ROLE = {'P': 'baseline', 'PI': 'alt2', 'PID': 'alt', 'DOB': 'proposed', 'LQG': 'alt4',
        'DOB+FF': 'alt3'}
DURATION, TAIL = 6.0, 3.0


def write_csv(path, rows):
    fields = []
    for r in rows:
        fields += [k for k in r if k not in fields]
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)


def r(v, n=3):
    return round(float(v), n) if v == v else ''


# ---------------------------------------------------------------------------
# 1. 外力阶跃
# ---------------------------------------------------------------------------
def brick_wrench(feed):
    """烧结砖横槽在给定进给下的名义底盘力旋量."""
    return cf.cut_wrench(cf.BladeParams(), ps.PF, cf.SLOT_DEPTH, feed, 'horizontal').planar


def step_case(name):
    """文件名 <控制律>_<外力>[_<限幅>] → (控制律, 力旋量, 限幅)."""
    parts = name.split('_')
    mode, what = parts[0], parts[1]
    limits = parts[2] if len(parts) > 2 else 'default'
    if what.startswith('fy'):
        w = (0.0, float(what[2:]), 0.0)
    elif what.startswith('mz'):
        w = (0.0, 0.0, float(what[2:]))
    else:
        w = brick_wrench(float(what[5:]) * 1e-3)
    return mode, w, limits


def predict(mode, wrench, limits, first_model=False):
    """离线模型对阶跃试验的预测记录（时间以加力时刻为零）."""
    hold, _ = ps.make_hold(MODE[mode], limits)
    lim = ps.LIMITS[limits]
    kw = dict(arm_compliance=False, odom_period=0.0) if first_model else {}
    plant = ps.ForcePlant(lim['a_max'], lim['alpha_max'], **kw)
    rec = ps.simulate(hold, ps.ConstantWrench(wrench, 1.0, 1.0 + DURATION),
                      1.0 + DURATION + TAIL, plant)
    return rec['t'] - 1.0, np.hypot(rec['tipx'], rec['tipy']) * 1e3, rec


def load_step(path):
    a = np.loadtxt(path, delimiter=',', skiprows=1)
    tip = np.hypot(a[:, 1], a[:, 2] + TIP_X * a[:, 3]) * 1e3
    return a, tip


def step_tests(fs, out):
    rows, traces = [], {}
    for path in sorted(glob.glob(os.path.join(HERE, 'runs', 'step', '*.csv'))):
        name = os.path.basename(path)[:-4]
        if name.startswith('wheels_'):
            continue
        mode, w, limits = step_case(name)
        a, tip = load_step(path)
        t = a[:, 0]
        on = (t > DURATION - 1.0) & (t < DURATION)
        tp, tipp, rec = predict(mode, w, limits)
        _, tip1, _ = predict(mode, w, limits, first_model=True)
        onp = (tp > DURATION - 1.0) & (tp < DURATION)
        traces[name] = (t, tip, tp, tipp)
        rows.append(dict(
            case=name, mode=mode, fx_N=r(w[0], 2), fy_N=r(w[1], 2), mz_Nm=r(w[2], 2),
            limits=limits, samples=len(t),
            peak_mm=r(tip.max(), 2),
            predicted_peak_mm=r(tipp.max(), 2),
            first_model_peak_mm=r(tip1.max(), 2),
            release_peak_mm=r(tip[t >= DURATION + 0.04].max(), 2),
            predicted_release_peak_mm=r(tipp[tp >= DURATION + 0.04].max(), 2),
            steady_y_mm=r(a[on, 2].mean() * 1e3, 3),
            predicted_steady_y_mm=r(rec['y'][onp].mean() * 1e3, 3),
            steady_yaw_deg=r(math.degrees(a[on, 3].mean()), 4),
            predicted_steady_yaw_deg=r(math.degrees(rec['psi'][onp].mean()), 4)))
    for x in rows:
        m, p, q = x['peak_mm'], x['predicted_peak_mm'], x['first_model_peak_mm']
        x['error_pct'] = r(100.0 * (p - m) / m, 1)
        x['first_model_error_pct'] = r(100.0 * (q - m) / m, 1)
    write_csv(os.path.join(out, 'gazebo_step.csv'), rows)

    fig, ax = fs.subplots(1, 3, width='double', height=2.5)
    for mode in ('p', 'pi', 'pid', 'dob'):
        key = f'{mode}_mz2'
        if key not in traces:
            continue
        t, tip, tp, tipp = traces[key]
        ax[0].plot(t, tip, label=MODE[mode], **fs.line(ROLE[MODE[mode]]))
        ax[0].plot(tp, tipp, color='k', ls=':', lw=0.6)
    ax[0].set_xlim(-0.3, 3.0)
    ax[0].set_xlabel(fs.label('Time since the step', 's'))
    ax[0].set_ylabel(fs.label('Tool-tip deviation', 'mm'))
    ax[0].legend(loc='center right')
    fs.note(ax[0], 'black dotted: model', x=0.98, y=0.3)

    lo, hi = 3.0, 150.0
    ax[1].plot([lo, hi], [lo, hi], **fs.line('reference'))
    ax[1].plot([x['first_model_peak_mm'] for x in rows], [x['peak_mm'] for x in rows],
               label='first model', **fs.marker('neutral'))
    ax[1].plot([x['predicted_peak_mm'] for x in rows], [x['peak_mm'] for x in rows],
               label='sampled odometry', **fs.marker('proposed'))
    ax[1].set_xscale('log')
    ax[1].set_yscale('log')
    fs.plain_log(ax[1], 'x')
    fs.plain_log(ax[1], 'y')
    ax[1].set_xlim(lo, hi)
    ax[1].set_ylim(lo, hi)
    ax[1].set_xlabel(fs.label('Predicted peak', 'mm'))
    ax[1].set_ylabel(fs.label('Measured peak in Gazebo', 'mm'))
    ax[1].legend(loc='upper left')

    for mode in ('p', 'pi', 'dob'):
        key = f'{mode}_mz8.6_default'
        if key not in traces:
            continue
        t, tip, tp, tipp = traces[key]
        ax[2].plot(t, tip, label=MODE[mode], **fs.line(ROLE[MODE[mode]]))
        ax[2].plot(tp, tipp, color='k', ls=':', lw=0.6)
    ax[2].axvline(DURATION, color='0.6', lw=0.5)
    ax[2].set_xlim(-0.3, DURATION + TAIL)
    ax[2].set_yscale('log')
    fs.plain_log(ax[2], 'y')
    ax[2].set_ylim(0.3, 300)
    ax[2].set_xlabel(fs.label('Time since the step', 's'))
    ax[2].set_ylabel(fs.label('Tool-tip deviation', 'mm'))
    ax[2].legend(loc='lower left')
    fs.note(ax[2], 'release', x=0.69, y=0.97, ha='left')
    fs.panel_labels(ax, ['2 N·m yaw step', f'Peak deviation, {len(rows)} step tests',
                         '8.6 N·m, default limits'])
    fs.save(fig, os.path.join(out, 'fig_gazebo_step.png'))
    return rows


# ---------------------------------------------------------------------------
# 2. 带切割力的整墙试验
# ---------------------------------------------------------------------------
CONFIGS = [   # (配置名, 图中的标签, 角色)
    ('p_f5.5', 'P', 'baseline'),
    ('dob_f5.5', 'DOB', 'proposed'),
    ('dobff_f5.5', 'DOB + FF', 'alt3'),
    ('dob_f8.8', 'DOB\n8.8 mm/s', 'proposed'),
    ('dobff_f8.8', 'DOB + FF\n8.8 mm/s', 'alt3'),
]
STAT_KEYS = ('tip_peak_horizontal_mm', 'tip_rms_horizontal_mm', 'tip_peak_vertical_mm',
             'tip_rms_vertical_mm', 'tip_peak_all_mm', 'yaw_peak_deg', 'path_err_mean_mm',
             'path_err_max_mm', 'coverage', 'base_drift_max_mm', 'stations_total_s',
             'fy_abs_mean_N', 'expected_fy_abs_mean_N', 'mz_abs_mean_Nm',
             'expected_mz_abs_mean_Nm', 'force_peak_N')


def split_name(run):
    """<配置>_<规划>_s<种子> → (配置, 规划, 种子)."""
    cfg, plan, seed = run.rsplit('_', 2)
    return cfg, plan, int(seed[1:])


def station_trace(run, station):
    """一站的 (时间, 刀尖偏移 mm, 阶段)，时间以该站第一个切割样本为零；以及力旋量记录."""
    trace = sm.read_csv(os.path.join(run, 'tool_trace.csv'))
    dev = sm.tip_deviation(trace)
    idx = [k for k, x in enumerate(trace) if int(x['station']) == station
           and x['phase'] in ('move', 'cut')]
    t = np.array([float(trace[k]['t']) for k in idx])
    cut = np.array([trace[k]['phase'] == 'cut' for k in idx])
    t0 = t[cut][0]
    mag = np.hypot(dev[idx, 0], dev[idx, 1])
    w = sm.read_csv(os.path.join(run, 'wrench.csv'))
    wr = None
    if w:
        wr = np.array([[float(x[k]) for k in ('t', 'mz', 'expected_mz', 'fy', 'expected_fy')]
                       for x in w])
        wr = wr[(wr[:, 0] >= t[0]) & (wr[:, 0] <= t[-1])]
        wr[:, 0] -= t0
    return t - t0, mag, cut, wr


def longest_horizontal_station(run):
    trace = sm.read_csv(os.path.join(run, 'tool_trace.csv'))
    best = {}
    for st, a, b, o in sm.cut_runs(trace):
        if o == 'h':
            best[st] = best.get(st, 0) + (b - a)
    return max(best, key=best.get)


def cut_statistics(runs, after=3.0):
    """
    按配置汇总逐刀的刀尖偏移峰值（毫米）：横槽、竖槽，以及每一刀结束后 after 秒内（退刀）.

    与离线仿真的统计口径相同（每一刀一个峰值，取均值 ± 标准差）。
    """
    peaks = {}
    for d in runs:
        cfg = split_name(os.path.basename(d))[0]
        trace = sm.read_csv(os.path.join(d, 'tool_trace.csv'))
        dev = sm.tip_deviation(trace)
        mag = np.hypot(dev[:, 0], dev[:, 1])
        t = np.array([float(x['t']) for x in trace])
        p = peaks.setdefault(cfg, dict(horizontal=[], vertical=[], after_cut=[]))
        for _, a, b, o in sm.cut_runs(trace):
            p['horizontal' if o == 'h' else 'vertical'].append(np.nanmax(mag[a:b]))
            k = int(np.searchsorted(t, t[b - 1] + after))
            if k > b and not np.all(np.isnan(mag[b:k])):
                p['after_cut'].append(np.nanmax(mag[b:k]))
    rows = []
    for cfg in sorted(peaks):
        row = dict(config=cfg)
        for key, v in peaks[cfg].items():
            if v:
                row.update({f'{key}_cuts': len(v), f'{key}_peak_mean_mm': r(np.mean(v), 2),
                            f'{key}_peak_std_mm': r(np.std(v), 2),
                            f'{key}_peak_max_mm': r(np.max(v), 2)})
        rows.append(row)
    return rows


def station_tests(fs, out):
    runs = sorted(d for d in glob.glob(os.path.join(HERE, 'runs', '*_s[0-9]'))
                  if os.path.exists(os.path.join(d, 'summary.yaml')))
    rows = []
    for d in runs:
        m = sm.run_metrics(d)
        cfg, plan, seed = split_name(m['run'])
        rows.append(dict(config=cfg, plan=plan, seed=seed, **m))
    write_csv(os.path.join(out, 'station_runs.csv'),
              [{k: (f'{v:.4g}' if isinstance(v, float) else v) for k, v in x.items()}
               for x in rows])
    summary = []
    for cfg in sorted({x['config'] for x in rows}):
        sel = [x for x in rows if x['config'] == cfg]
        s = dict(config=cfg, n=len(sel), runs=' '.join(f"{x['plan']}{x['seed']}" for x in sel),
                 all_ok=sum(x['all_ok'] for x in sel))
        for k in STAT_KEYS:
            v = [x[k] for x in sel if x.get(k) is not None]
            if v:
                s[k + '_mean'], s[k + '_min'], s[k + '_max'] = (
                    f'{np.mean(v):.4g}', f'{min(v):.4g}', f'{max(v):.4g}')
        summary.append(s)
    write_csv(os.path.join(out, 'station_summary.csv'), summary)
    write_csv(os.path.join(out, 'station_cuts.csv'), cut_statistics(runs))

    fig, ax = fs.subplots(1, 3, width='double', height=2.5)
    ref = os.path.join(HERE, 'runs', 'dobff_f5.5_joint_s1')
    st = longest_horizontal_station(ref)
    t, mag, cut, wr = station_trace(ref, st)
    ax[0].plot(wr[:, 0], np.abs(wr[:, 1]), label='applied', lw=0.6, **{
        k: v for k, v in fs.line('proposed').items() if k != 'lw'})
    ax[0].plot(wr[:, 0], np.abs(wr[:, 2]), label='expected', **fs.line('reference'))
    ax[0].set_xlabel(fs.label('Time since first engagement', 's'))
    ax[0].set_ylabel(fs.label('Yaw moment on the base $|M_z|$', 'N·m'))
    ax[0].legend(loc='upper right')
    for cfg, label, role in CONFIGS[:3]:
        d = os.path.join(HERE, 'runs', f'{cfg}_joint_s1')
        if not os.path.exists(os.path.join(d, 'summary.yaml')):
            continue
        t, mag, cut, _ = station_trace(d, st)
        ax[1].plot(t, np.maximum(mag, 1e-2), label=label, **fs.line(role))
    ax[1].axhline(1.0, **fs.line('reference'))
    ax[1].set_yscale('log')
    fs.plain_log(ax[1], 'y')
    ax[1].set_ylim(0.03, 900)
    ax[1].set_xlabel(fs.label('Time since first engagement', 's'))
    ax[1].set_ylabel(fs.label('Tool-tip deviation', 'mm'))
    ax[1].legend(loc='upper left', ncol=3, columnspacing=0.8, handlelength=1.6)
    for a in ax[:2]:
        a.set_xlim(wr[0, 0], wr[-1, 0])

    for j, (cfg, label, role) in enumerate(CONFIGS):
        sel = [x for x in rows if x['config'] == cfg]
        for key, dx, filled in (('tip_peak_horizontal_mm', -0.14, True),
                                ('tip_rms_horizontal_mm', 0.14, False)):
            v = [x[key] for x in sel]
            kw = fs.marker(role)
            kw['mfc'] = kw['color'] if filled else 'white'
            ax[2].plot([j + dx] * len(v), v, **kw)
    ax[2].axhline(1.0, **fs.line('reference'))
    ax[2].set_yscale('log')
    fs.plain_log(ax[2], 'y')
    ax[2].set_ylim(0.5, 150)
    ax[2].set_xticks(range(len(CONFIGS)))
    ax[2].set_xticklabels([c[1] for c in CONFIGS], fontsize=7)
    ax[2].set_xlim(-0.5, len(CONFIGS) - 0.5)
    ax[2].set_ylabel(fs.label('Tip deviation in horizontal cuts', 'mm'))
    fs.note(ax[2], 'filled: peak, open: RMS\none marker per run', x=0.98, y=0.97)
    fs.panel_labels(ax, [f'Station {st}: cutting moment', f'Station {st}: tip deviation',
                         'All runs, 11 stations each'])
    fs.save(fig, os.path.join(out, 'fig_gazebo_stations.png'))
    return summary


# ---------------------------------------------------------------------------
# 3. 进给上限与规划收益
# ---------------------------------------------------------------------------
BENCHMARKS = [   # (时间模型预设, 400 面墙对比的统计结果目录, 图中的标签)
    ('align_v3', os.path.join('..', 'task_planning', 'align_v3'), 'point to point\n(dry run)'),
    ('traj_v1', os.path.join('..', 'arm_trajectory', 'benchmark'),
     'continuous, 50 mm/s\n(dry run)'),
    ('cut_brick_v1', 'benchmark', 'continuous, 5.5 mm/s\n(brick, force limit)'),
]


def planning_gain(out):
    """各时间模型下 400 面墙对比的配对统计（task_planning_report.py 的 paired_tests.csv）."""
    rows = []
    for preset, d, label in BENCHMARKS:
        base = os.path.join(HERE, d)
        if not os.path.exists(os.path.join(base, 'paired_tests.csv')):
            continue
        with open(os.path.join(base, 'paired_tests.csv')) as f:
            tests = {(x['a'], x['b'], x['group']): x for x in csv.DictReader(f)}
        with open(os.path.join(base, 'summary.csv')) as f:
            summ = {x['method']: x for x in csv.DictReader(f)}
        j = tests[('current', 'joint', '非承重墙')]
        m = tests[('per_slot', 'current', '全部')]
        rows.append(dict(
            time_model=preset, label=label, n_nonbearing=int(j['n']),
            wins=int(j['wins']), ties=int(j['ties']), losses=int(j['losses']),
            joint_vs_rule_s=float(j['mean_s']), joint_vs_rule_pct=float(j['mean_pct']),
            joint_vs_rule_pct_ci=j['mean_pct_ci'], n_all=int(m['n']),
            rule_vs_per_slot_s=float(m['mean_s']), rule_vs_per_slot_pct=float(m['mean_pct']),
            t_mean_rule_s=float(summ['current']['t_mean']),
            t_mean_joint_s=float(summ['joint']['t_mean'])))
    write_csv(os.path.join(out, 'planning_gain.csv'),
              [{k: v.replace('\n', ' ') if isinstance(v, str) else v for k, v in x.items()}
               for x in rows])
    return rows


def feed_limits(fs, out):
    hold_default = ps.IZ * ps.K_IN * ps.LIMITS['default']['hold'][2] / 1.3
    hold_raised = ps.IZ * ps.K_IN * ps.LIMITS['raised']['hold'][2] / 1.3
    rows = []
    for material in ('aerated_block', 'brick', 'concrete'):
        blade = cf.BladeParams.for_material(material)
        for label, kw in (
                ('full power only', dict()),
                ('power 67 %', dict(power_fraction=0.67)),
                ('power 67 %, friction 0.3 / 1.0', dict(power_fraction=0.67, mu=0.3)),
                ('power 67 %, friction 0.3 / 1.5',
                 dict(power_fraction=0.67, mu=0.3, friction_margin=1.5)),
                ('power 67 %, friction 0.3 / 1.5, raised hold limits',
                 dict(power_fraction=0.67, mu=0.3, friction_margin=1.5,
                      max_yaw_moment=hold_raised)),
                ('power 67 %, friction 0.3 / 1.5, default hold limits',
                 dict(power_fraction=0.67, mu=0.3, friction_margin=1.5,
                      max_yaw_moment=hold_default))):
            lim = cf.feed_limit(blade, **kw)
            w = cf.cut_wrench(blade, cf.Platform(tool_z=cf.WORST_TOOL_Z, tool_y=cf.WORST_TOOL_Y),
                              cf.SLOT_DEPTH, lim.feed, 'horizontal').planar
            rows.append(dict(
                material=material, constraints=label, feed_mm_s=r(lim.feed * 1e3, 2),
                plunge_mm_s=r(lim.plunge * 1e3, 2), binding=lim.binding,
                power_mm_s=r(lim.power * 1e3, 2),
                friction_mm_s='' if lim.friction is None else r(lim.friction * 1e3, 2),
                hold_mm_s='' if lim.hold is None else r(lim.hold * 1e3, 2),
                fy_N=r(abs(w[1]), 1), mz_Nm=r(abs(w[2]), 1)))
    write_csv(os.path.join(out, 'feed_limit.csv'), rows)

    gain = planning_gain(out)
    ncol = 2 if len(gain) == len(BENCHMARKS) else 1
    fig, ax = fs.subplots(1, ncol, width='double' if ncol == 2 else 'single', height=2.4)
    ax = np.atleast_1d(ax)
    feeds = np.logspace(math.log10(0.5e-3), math.log10(0.1), 90)
    pf_worst = cf.Platform(tool_z=cf.WORST_TOOL_Z, tool_y=cf.WORST_TOOL_Y)
    for material, role, label in (('aerated_block', 'alt', 'aerated block'),
                                  ('brick', 'proposed', 'brick'),
                                  ('concrete', 'baseline', 'concrete')):
        blade = cf.BladeParams.for_material(material)
        p_lim = cf.max_feed_rate(blade, cf.SLOT_DEPTH)
        v = feeds[feeds <= p_lim]
        mz = [abs(cf.cut_wrench(blade, pf_worst, cf.SLOT_DEPTH, x, 'horizontal').planar[2])
              for x in v]
        ax[0].plot(v * 1e3, mz, label=label, **fs.line(role))
        lim = cf.feed_limit(blade, power_fraction=0.67, mu=0.3, friction_margin=1.5)
        m_lim = abs(cf.cut_wrench(blade, pf_worst, cf.SLOT_DEPTH, lim.feed,
                                  'horizontal').planar[2])
        ax[0].plot([lim.feed * 1e3], [m_lim], **fs.marker(role))
    ax[0].axhline(hold_default, **fs.line('reference'))
    fs.note(ax[0], 'default hold limits', x=0.03, y=0.33, ha='left')
    ax[0].set_xscale('log')
    ax[0].set_yscale('log')
    fs.plain_log(ax[0], 'x')
    fs.plain_log(ax[0], 'y')
    ax[0].set_xlabel(fs.label('Feed rate', 'mm/s'))
    ax[0].set_ylabel(fs.label('Yaw moment, horizontal cut', 'N·m'))
    ax[0].legend(loc='lower right')
    fs.note(ax[0], 'curves end at 820 W\nmarker: feed limit', x=0.03, y=0.97, ha='left')
    if ncol == 2:
        x = np.arange(len(gain))
        ax[1].bar(x - 0.2, [q['joint_vs_rule_pct'] for q in gain], 0.4,
                  label='joint planning vs. rule route + DP', **fs.bar('proposed'))
        ax[1].bar(x + 0.2, [q['rule_vs_per_slot_pct'] for q in gain], 0.4,
                  label='station merging vs. per-slot baseline', **fs.bar('neutral'))
        for xi, q in zip(x, gain):
            ax[1].text(xi - 0.2, q['joint_vs_rule_pct'] + 0.25, f"{q['joint_vs_rule_s']:.1f} s",
                       ha='center', va='bottom', fontsize=7)
        ax[1].set_xticks(x)
        ax[1].set_xticklabels([q['label'] for q in gain], fontsize=7)
        ax[1].set_ylabel(fs.label('Mean saving in predicted time', '%'))
        ax[1].set_ylim(0, 16)
        ax[1].legend(loc='upper right')
        fs.panel_labels(ax, ['Cutting moment and feed limit', 'Planning gain, 400 walls'])
    fs.save(fig, os.path.join(out, 'fig_feed_limit.png'))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('--out', default=HERE)
    ap.add_argument('--only', default='', help='step | stations | feed')
    args = ap.parse_args()
    import figstyle as fs

    if args.only in ('', 'step'):
        rows = step_tests(fs, args.out)
        print(f'step tests: {len(rows)} → gazebo_step.csv, fig_gazebo_step.png')
    if args.only in ('', 'stations'):
        rows = station_tests(fs, args.out)
        print(f'station runs: {sum(x["n"] for x in rows)} in {len(rows)} configurations → '
              'station_runs.csv, station_summary.csv, fig_gazebo_stations.png')
    if args.only in ('', 'feed'):
        rows = feed_limits(fs, args.out)
        print(f'feed limits: {len(rows)} rows → feed_limit.csv, fig_feed_limit.png')


if __name__ == '__main__':
    main()
