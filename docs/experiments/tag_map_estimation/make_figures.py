#!/usr/bin/env python3
"""
本实验的图（docs/figstyle 样式，PDF + PNG）.

  python3 docs/experiments/tag_map_estimation/make_figures.py

读取 validation.csv、mc_summary.csv、consistency.csv、replay_*.csv / replay_traces.npz、
bias_gazebo.csv、bias_sim.csv、marker_free.csv。
"""
import csv
import os

import numpy as np

from common import HERE, WORLD_TAG_U
import figstyle as fs

METHODS = [('a_nominal', 'Nominal map', 'baseline'),
           ('b_surveyed', 'Surveyed map', 'reference'),
           ('c_online', 'Online map, pass 1', 'proposed'),
           ('d_pass2', 'Online map, pass 2', 'alt')]


def rd(name):
    with open(os.path.join(HERE, name), newline='') as f:
        return list(csv.DictReader(f))


def fig_validation():
    rows = rd('validation.csv')
    fig, ax = fs.subplots(width='single', height=2.5)
    y = np.arange(len(rows))[::-1]
    for yi, r in zip(y, rows):
        lo, hi = float(r['sim_max_p5_mm']), float(r['sim_max_p95_mm'])
        ax.plot([lo, hi], [yi, yi], color=fs.color('proposed'), lw=1.2)
        ax.plot(float(r['sim_max_mean_mm']), yi, **fs.marker('proposed'),
                label='Simulation (mean, 5-95 %)' if yi == y[0] else None)
        for k, v in enumerate(r['gazebo_max_mm'].split(' / ')):
            ax.plot(float(v), yi + 0.18, **fs.marker('baseline'),
                    label='Gazebo runs' if (yi == y[0] and k == 0) else None)
    ax.set_yticks(y)
    ax.set_yticklabels([r['config'] for r in rows])
    ax.set_xscale('log')
    fs.plain_log(ax, 'x')
    ax.set_xlabel(fs.label('Max. along-wall error over 11 stations', 'mm'))
    ax.legend(loc='lower right')
    fs.save(fig, os.path.join(HERE, 'fig_validation.png'))


def fig_methods():
    rows = rd('mc_summary.csv')
    fig, axes = fs.subplots(1, 2, width='double', height=2.4, sharey=True)
    for ax, scale, title in zip(axes, (0, 1), ('without strafe-scale state',
                                                'with strafe-scale state')):
        for k, (m, lab, role) in enumerate(METHODS):
            pts = sorted((float(r['sigma_p_mm']), float(r['run_max_mean_mm']),
                          float(r['run_max_p95_mm']))
                         for r in rows if r['method'] == m and int(r['layout']) == 3
                         and int(r['scale']) == scale)
            if not pts:
                continue
            x = np.array([p[0] for p in pts]) + (k - 1.5) * 0.08
            ax.errorbar(x, [p[1] for p in pts],
                        yerr=[np.zeros(len(pts)), [p[2] - p[1] for p in pts]],
                        capsize=2, label=lab, **fs.line(role, marker=True))
        ax.set_xlabel(fs.label('Marker placement error (1-sigma)', 'mm'))
        ax.set_xticks([0, 1, 3, 5])
        fs.panel_label(ax, 'ab'[scale], title)
    axes[0].set_ylabel(fs.label('Max. station error per run', 'mm'))
    axes[0].legend(loc='upper left')
    fs.save(fig, os.path.join(HERE, 'fig_mc_methods.png'))


def fig_layouts():
    rows = rd('mc_summary.csv')
    fig, axes = fs.subplots(1, 2, width='double', height=2.3, sharey=True)
    layouts = (1, 2, 3, 6)
    for ax, scale in zip(axes, (0, 1)):
        for k, (m, lab, role) in enumerate(METHODS):
            vals, errs = [], []
            for L in layouts:
                sp = 0.0 if L == 1 else 3.0
                if L == 1 and m != 'a_nominal':     # 只有参考标记时没有地图可估计
                    vals.append(np.nan)
                    errs.append(0.0)
                    continue
                r = [r for r in rows if r['method'] == m and int(r['layout']) == L
                     and int(r['scale']) == scale and float(r['sigma_p_mm']) == sp]
                vals.append(float(r[0]['run_max_mean_mm']) if r else np.nan)
                errs.append(float(r[0]['run_max_p95_mm']) - vals[-1] if r else 0.0)
            x = np.arange(len(layouts)) + (k - 1.5) * 0.2
            ax.bar(x, vals, width=0.18, yerr=[np.zeros(len(vals)), errs], capsize=1.5,
                   label=lab, **fs.bar(role))
        ax.set_xticks(np.arange(len(layouts)))
        ax.set_xticklabels(['1 (ref. only)', '2 (1.2 m)', '3 (0.6 m)', '6 (0.3 m)'])
        ax.set_xlabel('Markers on the wall (spacing)')
        ax.set_yscale('log')
        ax.set_ylim(0.5, 40)
        fs.plain_log(ax, 'y')
        fs.panel_label(ax, 'ab'[scale], ('without', 'with')[scale] + ' strafe-scale state')
    axes[0].set_ylabel(fs.label('Max. station error per run', 'mm'))
    axes[1].legend(loc='upper right', ncol=2)
    fs.save(fig, os.path.join(HERE, 'fig_mc_layouts.png'))


def fig_replay():
    tr = np.load(os.path.join(HERE, 'replay_traces.npz'))
    fig, axes = fs.subplots(2, 1, width='single', height=3.4, sharex=True)
    for key, role, lab in (('default_s1_pass_1', 'proposed', 'Run 1, pass 1'),
                           ('default_s2_pass_2', 'alt', 'Run 2, pass 2 (prior: run 1)')):
        a = tr[key]
        t = a[:, 0] - a[0, 0]
        for ax, col, tag in ((axes[0], 3, 2), (axes[1], 5, 4)):
            e = (a[:, col] - WORLD_TAG_U[tag]) * 1e3
            s = a[:, col + 1] * 1e3
            ax.plot(t, e, label=lab, **fs.line(role))
            ax.fill_between(t, e - s, e + s, color=fs.color(role), alpha=0.15, lw=0)
    for ax, tag in zip(axes, (2, 4)):
        ax.axhline(0.0, **fs.line('reference'))
        ax.set_ylabel(fs.label(f'ID {tag} map error', 'mm'))
        fs.panel_label(ax, 'ab'[tag // 4], f'marker ID {tag} (prior {"+5" if tag == 2 else "-5"} mm)')
    axes[1].set_xlabel(fs.label('Time since start of alignment', 's'))
    axes[0].legend(loc='upper right')
    fs.save(fig, os.path.join(HERE, 'fig_replay.png'))


def fig_bias():
    gz = rd('bias_gazebo.csv')
    sim = rd('bias_sim.csv')
    fig, axes = fs.subplots(1, 2, width='double', height=2.4)
    ax = axes[0]
    for cfg, role, lab in (('tags_0.6', 'baseline', 'tags 0.6 m, no scale state'),
                           ('tags_0.3', 'alt2', 'tags 0.3 m, no scale state'),
                           ('scale_tags_0.6', 'proposed', 'tags 0.6 m + scale state')):
        rs = [r for r in gz if r['config'] == cfg]
        ax.plot([float(r['d_times_psi_err_mm']) for r in rs],
                [float(r['u_err_mm']) for r in rs], label=lab,
                **fs.marker(role))
    lim = [-1.0, 3.0]
    ax.plot(lim, lim, **fs.line('reference'), label='$y = x$')
    ax.set_xlabel(fs.label(r'$d\,(\hat\psi-\psi)$ at station', 'mm'))
    ax.set_ylabel(fs.label(r'$\hat u - u$ at station', 'mm'))
    ax.legend(loc='upper left')
    fs.panel_label(ax, 'a', 'Gazebo, 11 stations x 2 runs')
    ax = axes[1]
    rs = [r for r in sim if r['sweep'] == 'lateral scale error']
    x = np.array([float(r['value']) * 100 for r in rs])
    ax.plot(x, [float(r['u_err_mean_mm']) for r in rs], label=r'$\hat u - u$ (mean)',
            **fs.line('baseline', marker=True))
    ax.plot(x, [float(r['d_psi_mean_mm']) for r in rs], label=r'$d\,(\hat\psi-\psi)$ (mean)',
            **fs.line('alt', marker=True))
    gzm = np.mean([float(r['u_err_mm']) for r in gz if r['config'] == 'tags_0.6'])
    ax.plot([2.8], [gzm], label='Gazebo tags 0.6 m (mean)',
            **fs.marker('reference'))
    ax.set_xlabel(fs.label('Odometry over-reads strafe by', '%'))
    ax.set_ylabel(fs.label('Station bias', 'mm'))
    ax.legend(loc='upper left')
    fs.panel_label(ax, 'b', 'simulation, no scale state')
    fs.save(fig, os.path.join(HERE, 'fig_bias.png'))


def fig_marker_free():
    rows = rd('marker_free.csv')
    fig, ax = fs.subplots(width='single', height=2.4)
    for feat, order, role, lab in (
            ('corner', 'noise→quantize', 'proposed', 'Corner, per scan'),
            ('corner', 'quantize→noise', 'alt', 'Corner, static 30-scan bias (quantise first)'),
            ('opening edge', 'analytic', 'baseline', 'Opening edge, per scan')):
        rs = [r for r in rows if r['feature'] == feat and r['order'] == order]
        key = 'static30_sigma_mm' if 'static' in lab else 'frame_sigma_mm'
        ax.plot([float(r['distance_m']) for r in rs], [float(r[key]) for r in rs], label=lab,
                **fs.line(role, marker=True))
    ax.axhline(1.5, **fs.line('reference'))
    fs.note(ax, 'tag map + scale state: max 1.5 mm (Gazebo)', x=0.02, y=0.25, ha='left', va='top')
    ax.set_ylim(0.3, 200)
    ax.set_xlabel(fs.label('Along-wall distance to the feature', 'm'))
    ax.set_ylabel(fs.label('Along-wall error (1-sigma)', 'mm'))
    ax.set_yscale('log')
    fs.plain_log(ax, 'y')
    ax.legend(loc='upper right')
    fs.save(fig, os.path.join(HERE, 'fig_marker_free.png'))


def main():
    fs.setup()
    fig_validation()
    fig_methods()
    fig_layouts()
    fig_replay()
    fig_bias()
    fig_marker_free()


if __name__ == '__main__':
    main()
