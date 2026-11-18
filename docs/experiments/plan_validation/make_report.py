#!/usr/bin/env python3
"""
规划验证实验的汇总表与图：时间模型重标定（align_v3）+ Gazebo 中联合规划与规则路线的对比.

  python3 docs/experiments/plan_validation/make_report.py

读取：
  标定数据   docs/experiments/along_wall_accuracy/runs/{default_s1,default_s2,
             scale_tags_0.6_s1,scale_tags_0.6_s2}（默认配置执行 11 站规划，44 站）
  验证运行   runs/dp11_s3（同一份 11 站规划，新运行，留出）
             runs/rule_s*、runs/joint_s*（room3 南墙示例任务，align_v3 下的两份规划）
             runs/demo_joint（演示录制：联合规划 + 墙面外观实体 + 演示相机）
输出：calibration.csv、runs_summary.csv、stations_all.csv、fig_time_model.png、
     fig_joint_vs_rule.png（样式见 docs/figstyle，PDF + PNG）
"""
import csv
import glob
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'src', 'scr_planner', 'scripts'))
sys.path.insert(0, os.path.join(REPO, 'src', 'scr_planner', 'python'))
sys.path.insert(0, os.path.join(REPO, 'docs'))
import calibrate_time_model as cal  # noqa: E402
import slot_plan as sp  # noqa: E402

CAL_RUNS = [os.path.join(REPO, 'docs', 'experiments', 'along_wall_accuracy', 'runs', r)
            for r in ('default_s1', 'default_s2', 'scale_tags_0.6_s1', 'scale_tags_0.6_s2')]
PRESETS = ('align_v1', 'align_v2', 'align_v3')
METHODS = ('joint', 'phases', 'phases-reg')


def tm_tuple(name):
    tm = sp.TimeModel.preset(name)
    return tm.t_settle, tm.v_strafe, tm.t_point


def run_rows(d):
    rows = cal.load([d])
    with open(os.path.join(d, 'summary.yaml'), encoding='utf-8') as f:
        s = yaml.safe_load(f)
    return rows, s


def calibration_table():
    rows = cal.load(CAL_RUNS)
    a = cal.arrays(rows)
    out = []
    for m in METHODS:
        p = cal.FIT[m](a)
        st = cal.stats(p, rows)
        loo = []
        for k in sorted({r['run'] for r in rows}):
            q = cal.FIT[m](cal.arrays([r for r in rows if r['run'] != k]))
            test = [r for r in rows if r['run'] == k]
            loo.append(cal.stats(q, test)['rmse'] ** 2 * len(test))
        out.append(dict(source=f'fit:{m}', t_settle=p[0], v_strafe=p[1], t_point=p[2],
                        rmse_s=st['rmse'], bias_s=st['bias'], r2=st['r2'],
                        loo_rmse_s=float(np.sqrt(sum(loo) / len(rows))),
                        max_abs_total_err_s=max(abs(t) for t in st['total_err'])))
    for name in PRESETS:
        p = tm_tuple(name)
        st = cal.stats(p, rows)
        out.append(dict(source=f'preset:{name}', t_settle=p[0], v_strafe=p[1], t_point=p[2],
                        rmse_s=st['rmse'], bias_s=st['bias'], r2=st['r2'], loo_rmse_s='',
                        max_abs_total_err_s=max(abs(t) for t in st['total_err'])))
    return out


def summarize(d, group):
    rows, s = run_rows(d)
    a = cal.arrays(rows)
    lat = np.array([abs(float(r['lateral_mm'])) for r in rows if r.get('lateral_mm')])
    res = dict(run=os.path.basename(d), group=group, all_ok=s.get('all_ok'),
               stations=len(rows), points=int(a['n'].sum()), travel_m=float(a['d'].sum()),
               measured_stations_s=float(a['y'].sum()), sim_time_s=s['sim_time_s'],
               nav_align_s=s['sim_time_s'] - float(a['y'].sum()))
    for name in PRESETS:
        pred = float(cal.predict(tm_tuple(name), a).sum())
        res[f'pred_{name}_s'] = pred
        res[f'err_{name}_s'] = pred - float(a['y'].sum())
    res['station_rmse_v3_s'] = cal.stats(tm_tuple('align_v3'), rows)['rmse']
    res.update(lateral_mm_mean=float(lat.mean()), lateral_mm_max=float(lat.max()),
               path_err_mean_mm=s['path_err_mean_mm'], path_err_p95_mm=s['path_err_p95_mm'],
               path_err_max_mm=s['path_err_max_mm'], coverage=s['coverage'],
               scale=s.get('final_lateral_scale'), recoveries=s.get('recoveries'))
    return res, rows


def write_csv(path, rows):
    keys = list(rows[0].keys())
    for r in rows:
        keys += [k for k in r if k not in keys]
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: (f'{v:.4g}' if isinstance(v, float) else v) for k, v in r.items()})


def plot_time_model(cal_rows, val_rows, out):
    import figstyle as fs
    fig, axes = fs.subplots(1, 2, width='double', height=3.1)
    for ax, name in zip(axes, ('align_v2', 'align_v3')):
        p = tm_tuple(name)
        for rows, role, lab in ((cal_rows, 'neutral', 'Calibration data'),
                                (val_rows, 'proposed', 'Validation data')):
            lab += f' ({len({r["run"] for r in rows})} runs, {len(rows)} stations)'
            a = cal.arrays(rows)
            mk = 'o' if role == 'neutral' else '^'
            ax.plot(a['y'], cal.predict(p, a), ls='none', marker=mk, ms=3.2, mfc='none',
                    mec=fs.color(role), mew=0.7, label=lab)
        lim = [5, 40]
        ax.plot(lim, lim, label='Prediction = measurement', **fs.line('reference', lw=0.7))
        ax.set_xlim(lim)
        ax.set_ylim(lim)
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel(fs.label('Measured time per station (simulation)', 's'))
        ax.set_ylabel(fs.label('Predicted time per station', 's'))
        e = cal.predict(p, cal.arrays(val_rows)) - cal.arrays(val_rows)['y']
        bias = f'{np.mean(e):+.2f}'.replace('-', '−')
        fs.note(ax, f'Validation: RMSE {np.sqrt(np.mean(e ** 2)):.2f} s,\n'
                    f'mean bias {bias} s', x=0.97, y=0.03, va='bottom')
        ax.legend(loc='upper left')
    fs.panel_labels(axes, ['Time model align_v2', 'Time model align_v3 (recalibrated)'])
    fs.save(fig, out)


def plot_joint_vs_rule(summ, out):
    import figstyle as fs
    fig, (ax1, ax2) = fs.subplots(1, 2, width='double', height=2.5,
                                  gridspec_kw=dict(width_ratios=[1, 1.3]))
    groups = [('rule', 'Rule-based routes\n(88 points)', 'baseline'),
              ('joint', 'Joint planning\n(80 points)', 'proposed')]
    for i, (g, lab, role) in enumerate(groups):
        rs = [r for r in summ if r['group'] == g]
        y = [r['measured_stations_s'] for r in rs]
        ax1.plot([i] * len(y), y, zorder=3, **fs.marker(role, ms=4))
        ax1.hlines(rs[0]['pred_align_v3_s'], i - 0.25, i + 0.25, color=fs.BLACK, lw=1.2,
                   label='Predicted, align_v3' if i == 0 else None)
        ax1.hlines(rs[0]['pred_align_v2_s'], i - 0.25, i + 0.25, color=fs.GREY, lw=1.0,
                   ls=(0, (3, 1.5)), label='Predicted, align_v2' if i == 0 else None)
    ax1.plot([], [], ls='none', marker='o', ms=4, color=fs.GREY,
             label='Measured (one point per run)')
    ax1.set_xticks(range(len(groups)))
    ax1.set_xticklabels([lab for _, lab, _ in groups])
    ax1.set_xlim(-0.6, 1.6)
    ax1.set_ylabel(fs.label('Sum of station times (simulation)', 's'))
    ax1.legend(loc='upper right')
    for g, role, name in (('rule', 'baseline', 'Rule-based routes'),
                          ('joint', 'proposed', 'Joint planning')):
        for r in (r for r in summ if r['group'] == g):
            rows = cal.load([os.path.join(HERE, 'runs', r['run'])])
            ax2.plot([int(x['station']) + 1 for x in rows],
                     [float(x['station_s']) for x in rows], lw=0.7,
                     label=name if r['run'].endswith('s1') else None,
                     **fs.line(role, marker=True, ms=2.6))
    ax2.set_xlabel('Station (execution order)')
    ax2.set_ylabel(fs.label('Time per station', 's'))
    ax2.legend(loc='upper right')
    fs.panel_labels([ax1, ax2], ['11 stations: measured vs. predicted',
                                 'Time per station (one line per run)'])
    fs.save(fig, out)


def main():
    calib = calibration_table()
    write_csv(os.path.join(HERE, 'calibration.csv'), calib)
    for r in calib:
        print(r)
    summ, all_rows, val_rows = [], [], []
    groups = [('dp11', 'dp11_s*'), ('rule', 'rule_s*'), ('joint', 'joint_s*'),
              ('demo', 'demo_joint')]
    for g, pat in groups:
        for d in sorted(glob.glob(os.path.join(HERE, 'runs', pat))):
            if not os.path.exists(os.path.join(d, 'stations.csv')):
                continue
            res, rows = summarize(d, g)
            summ.append(res)
            for r in rows:
                all_rows.append(dict(group=g, **r))
            if g != 'demo':
                val_rows += rows
    write_csv(os.path.join(HERE, 'runs_summary.csv'), summ)
    write_csv(os.path.join(HERE, 'stations_all.csv'), all_rows)
    for r in summ:
        print({k: (round(v, 2) if isinstance(v, float) else v) for k, v in r.items()})
    if val_rows:
        st = cal.stats(tm_tuple('align_v3'), val_rows)
        print(f'验证数据（{len(val_rows)} 站）align_v3：每站 RMSE {st["rmse"]:.2f} s，'
              f'平均偏差 {st["bias"]:+.2f} s')
        plot_time_model(cal.load(CAL_RUNS), val_rows,
                        os.path.join(HERE, 'fig_time_model.png'))
    if any(r['group'] == 'joint' for r in summ) and any(r['group'] == 'rule' for r in summ):
        plot_joint_vs_rule([r for r in summ if r['group'] in ('rule', 'joint')],
                           os.path.join(HERE, 'fig_joint_vs_rule.png'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
