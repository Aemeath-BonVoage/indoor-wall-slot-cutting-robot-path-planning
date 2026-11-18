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
     fig_joint_vs_rule.png（中文字体自动选择）
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


def cjk_font():
    from matplotlib import font_manager
    for name in ('Noto Sans CJK SC', 'Noto Sans CJK JP', 'Noto Serif CJK SC', 'WenQuanYi Zen Hei',
                 'AR PL UKai CN'):
        if any(f.name == name for f in font_manager.fontManager.ttflist):
            return name
    return None


def plot_time_model(cal_rows, val_rows, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    f = cjk_font()
    if f:
        plt.rcParams['font.family'] = f
    plt.rcParams['axes.unicode_minus'] = False
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.2))
    for ax, name in zip(axes, ('align_v2', 'align_v3')):
        p = tm_tuple(name)
        for rows, mk, lab in ((cal_rows, 'o', '标定数据'), (val_rows, '^', '验证数据')):
            lab += f'（{len({r["run"] for r in rows})} 次运行，{len(rows)} 站）'
            a = cal.arrays(rows)
            ax.scatter(a['y'], cal.predict(p, a), s=18, marker=mk, alpha=0.7, label=lab)
        lim = [5, 40]
        ax.plot(lim, lim, 'k--', lw=0.8)
        ax.set_xlim(lim)
        ax.set_ylim(lim)
        ax.set_xlabel('实测每站耗时（s，仿真时间）')
        ax.set_ylabel('预测每站耗时（s）')
        e = np.concatenate([cal.predict(p, cal.arrays(r)) - cal.arrays(r)['y']
                            for r in (val_rows,)])
        ax.set_title(f'{name}：验证数据每站 RMSE {np.sqrt(np.mean(e ** 2)):.2f} s，'
                     f'平均偏差 {np.mean(e):+.2f} s', fontsize=10)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc='upper left')
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def plot_joint_vs_rule(summ, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    f = cjk_font()
    if f:
        plt.rcParams['font.family'] = f
    plt.rcParams['axes.unicode_minus'] = False
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4.0),
                                   gridspec_kw=dict(width_ratios=[1.3, 1]))
    groups = [('rule', '规则路线\n（88 点）'), ('joint', '联合规划\n（80 点）')]
    for i, (g, lab) in enumerate(groups):
        rs = [r for r in summ if r['group'] == g]
        y = [r['measured_stations_s'] for r in rs]
        ax1.scatter([i] * len(y), y, color='C0', zorder=3,
                    label='实测（每次运行）' if i == 0 else None)
        ax1.hlines(rs[0]['pred_align_v3_s'], i - 0.25, i + 0.25, color='C3', lw=2,
                   label='align_v3 预测' if i == 0 else None)
        ax1.hlines(rs[0]['pred_align_v2_s'], i - 0.25, i + 0.25, color='C7', lw=1.5, ls='--',
                   label='align_v2 预测' if i == 0 else None)
    ax1.set_xticks(range(len(groups)))
    ax1.set_xticklabels([lab for _, lab in groups])
    ax1.set_xlim(-0.6, 1.6)
    ax1.set_ylabel('各站耗时之和（s，仿真时间）')
    ax1.set_title('11 个停靠站的执行时间：实测 vs 预测', fontsize=10)
    ax1.grid(alpha=0.3, axis='y')
    ax1.legend(fontsize=8)
    for g, c in (('rule', 'C0'), ('joint', 'C1')):
        for r in (r for r in summ if r['group'] == g):
            rows = cal.load([os.path.join(HERE, 'runs', r['run'])])
            ax2.plot([int(x['station']) + 1 for x in rows],
                     [float(x['station_s']) for x in rows], 'o-', color=c, ms=3, lw=0.8,
                     alpha=0.8, label={'rule': '规则路线', 'joint': '联合规划'}[g]
                     if r['run'].endswith('s1') else None)
    ax2.set_xlabel('停靠站（执行顺序）')
    ax2.set_ylabel('每站耗时（s）')
    ax2.set_title('每站耗时（每条线一次运行）', fontsize=10)
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


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
