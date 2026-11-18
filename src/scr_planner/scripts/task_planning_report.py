#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务级联合规划实验的统计与作图（读取 task_planning_benchmark.py 的输出）.

  python3 src/scr_planner/scripts/task_planning_report.py [data_dir] [out_dir] [--time-model=NAME]

data_dir 默认 docs/experiments/task_planning/align_v3/data，out_dir 默认为 data_dir 的上一级。
--time-model 只用于重新规划示例墙（fig_example.png），须与生成 data_dir 时所用的预设一致：
align_v3/data 用 align_v3（默认），align_v2/data 用 align_v2，data/ 用 align_v1。

统计口径
  配对样本：per_slot、current、joint 三者都得到可行方案的墙（current 可行时 joint 必可行）。
  差值 d = T_A - T_B（秒，正值表示 B 更快），相对节省 = d / T_A。
  Wilcoxon 符号秩检验（双侧，零差值剔除），效应量为配对秩二列相关
  r = (W+ - W-) / (W+ + W-)；均值与中位数的 95 % 置信区间用配对 bootstrap
  （10000 次重采样，种子 0）。|d| < 0.05 s 记为持平。
  与下界的差距 (T - LB) / LB；T - LB ≤ 0.01 s 记为“已证明最优”。

输出：summary.csv、paired_tests.csv、gap.csv、lee_summary.csv、sensitivity_summary.csv、
      fig_results.png、fig_example.png（及同名 PDF，样式见 docs/figstyle），以及打印到
      终端的汇总（报告中的数字来自这里）。
"""
import csv
import os
import sys
import warnings
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.realpath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
for d in (os.path.join(HERE, '..', 'python'),
          os.path.join(HERE, '..', '..', 'share', 'scr_planner', 'python')):
    if os.path.isdir(d):
        sys.path.insert(0, d)

METHODS = ('per_slot', 'current', 'joint')
ABLATIONS = ('lattice_indep', 'lattice_seq', 'lattice_cd', 'joint')
NAMES = {'per_slot': '逐槽划分（基线）', 'current': '先选线后 DP（现有）', 'joint': '联合规划',
         'current_v0': '现有（修正前评估）', 'lattice_indep': '站感知·各槽独立',
         'lattice_seq': '站感知·共享窗口', 'lattice_cd': '站感知·共享+坐标下降',
         'lower_bound': '下界'}
# 图中的名称与语义角色（docs/figstyle）：逐槽划分为中性参照，先选线后 DP（规则路线）为基线，
# 联合规划为本项目方法
NAMES_EN = {'per_slot': 'Per-slot partition (naive)', 'current': 'Rule-based routes + DP',
            'joint': 'Joint planning', 'lower_bound': 'Lower bound'}
ROLES = {'per_slot': 'neutral', 'current': 'baseline', 'joint': 'proposed'}
OBST_EN = {'梁': 'Beam', '柱': 'Column', '门': 'Door', '窗': 'Window', '飘窗': 'Bay window',
           '窄墙垛': 'Pier', '开关': 'Switch', '接线盒': 'Junction box', '弱电线槽': 'Data trunking'}
TIE, CERT = 0.05, 0.01


def load(path):
    with open(path, encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k, v in r.items():
            if v not in ('', None) and k not in ('method', 'source', 'model'):
                r[k] = float(v)
    return rows


def by_seed(rows, key='method'):
    out = defaultdict(dict)
    for r in rows:
        out[int(r['seed'])][r[key]] = r
    return out


def boot_ci(x, fn, n=10000, seed=0):
    rng = np.random.default_rng(seed)
    x = np.asarray(x)
    idx = rng.integers(0, len(x), size=(n, len(x)))
    vals = fn(x[idx], axis=1)
    return float(np.percentile(vals, 2.5)), float(np.percentile(vals, 97.5))


def paired(ta, tb):
    """ta, tb：同一组墙上两种方法的时间 → 统计 dict（d = ta - tb）."""
    from scipy.stats import rankdata, wilcoxon
    ta, tb = np.asarray(ta), np.asarray(tb)
    d = ta - tb
    rel = d / ta * 100
    nz = d[np.abs(d) >= TIE]
    out = dict(n=len(d), wins=int((d >= TIE).sum()), ties=int((np.abs(d) < TIE).sum()),
               losses=int((d <= -TIE).sum()),
               mean_s=float(d.mean()), median_s=float(np.median(d)),
               mean_pct=float(rel.mean()), median_pct=float(np.median(rel)),
               max_pct=float(rel.max()))
    out['mean_pct_ci'] = boot_ci(rel, np.mean)
    out['mean_s_ci'] = boot_ci(d, np.mean)
    out['median_pct_ci'] = boot_ci(rel, np.median)
    if len(nz) >= 1:
        ranks = rankdata(np.abs(nz))
        wp, wm = ranks[nz > 0].sum(), ranks[nz < 0].sum()
        out['r_rb'] = float((wp - wm) / (wp + wm))
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            out['p'] = float(wilcoxon(nz).pvalue) if len(nz) >= 2 else 1.0
    else:
        out['r_rb'], out['p'] = 0.0, 1.0
    out['n_nonzero'] = int(len(nz))
    return out


def fmt_p(p):
    return '< 1e-10' if p < 1e-10 else f'{p:.2g}'


def fmt_pair(s):
    lo, hi = s['mean_pct_ci']
    return (f"n = {s['n']}，快 / 持平 / 慢 = {s['wins']} / {s['ties']} / {s['losses']}，"
            f"平均节省 {s['mean_s']:.1f} s（{s['mean_pct']:.2f} %，95 % CI [{lo:.2f}, {hi:.2f}] %），"
            f"中位 {s['median_pct']:.2f} %，最大 {s['max_pct']:.1f} %，"
            f"Wilcoxon p = {fmt_p(s['p'])}（非零 {s['n_nonzero']}），r_rb = {s['r_rb']:.2f}")


def write_csv(path, rows):
    keys = []
    for r in rows:
        keys += [k for k in r if k not in keys]
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


def setup_mpl():
    sys.path.insert(0, os.path.join(REPO, 'docs'))
    import figstyle as fs
    return fs


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    kw = dict(a[2:].split('=', 1) for a in sys.argv[1:] if a.startswith('--'))
    data = args[0] if args else os.path.join(
        REPO, 'docs', 'experiments', 'task_planning', 'align_v3', 'data')
    out = args[1] if len(args) > 1 else os.path.dirname(os.path.abspath(data))
    time_model = kw.get('time-model', 'align_v3')
    rows = load(os.path.join(data, 'results.csv'))
    S = by_seed(rows)
    n_walls = len(S)
    print(f'墙数 {n_walls}')

    # --- 可行性 ---------------------------------------------------------------
    feas = {m: sum(S[s][m]['feasible'] == 1 for s in S) for m in NAMES}
    print('可行方案数：', {m: f'{feas[m]}/{n_walls}' for m in feas})
    rescued = [s for s in S
               if S[s]['current']['feasible'] == 0 and S[s]['joint']['feasible'] == 1]
    neither = [s for s in S if S[s]['joint']['feasible'] == 0]
    print(f'现有方法无解而联合规划有解：{len(rescued)} 面；联合规划也无解：{len(neither)} 面'
          f'（其中 {sum(S[s]["joint"]["slots"] == 4 for s in neither)} 面为 4 条槽）')

    P = sorted(s for s in S if all(S[s][m]['feasible'] == 1 for m in METHODS))
    print(f'配对样本 {len(P)} 面（承重墙 {sum(S[s]["joint"]["load_bearing"] == 1 for s in P)} 面）')

    def T(m, seeds):
        return np.array([S[s][m]['t_exact'] for s in seeds])

    # --- 汇总表 ---------------------------------------------------------------
    summary = []
    for m in ('per_slot', 'current_v0', 'current', 'lattice_indep', 'lattice_seq', 'lattice_cd',
              'joint', 'lower_bound'):
        rs = [S[s][m] for s in P]
        row = dict(method=m, name=NAMES[m], n=len(rs),
                   t_mean=np.mean([r['t_exact'] for r in rs]),
                   stations_mean=np.mean([r['stations'] for r in rs]),
                   points_mean=np.mean([r['points'] for r in rs]),
                   travel_m_mean=np.mean([r['travel_m'] for r in rs]))
        if m != 'lower_bound':
            row.update(length_m_mean=np.mean([r['length_m'] for r in rs]),
                       turns_mean=np.mean([r['turns'] for r in rs]),
                       violations=int(sum(r['violations'] for r in rs)),
                       plan_ms_median=np.median([r['plan_ms'] for r in rs]),
                       plan_ms_p95=np.percentile([r['plan_ms'] for r in rs], 95),
                       plan_ms_max=max(r['plan_ms'] for r in rs))
        summary.append({k: (round(v, 3) if isinstance(v, float) else v) for k, v in row.items()})
    write_csv(os.path.join(out, 'summary.csv'), summary)
    print('\n方法汇总（配对样本均值）')
    for r in summary:
        print('  ', r)

    # --- 配对检验 -------------------------------------------------------------
    nlb = [s for s in P if S[s]['joint']['load_bearing'] == 0]
    tests = []
    pairs = [('current', 'joint', P, '全部'), ('current', 'joint', nlb, '非承重墙'),
             ('per_slot', 'current', P, '全部'), ('per_slot', 'joint', P, '全部'),
             ('current_v0', 'current', P, '全部')]
    pairs += [('current', m, nlb, '非承重墙') for m in ABLATIONS[:-1]]
    for k in (1, 2, 3, 4):
        pairs.append(('current', 'joint', [s for s in nlb if S[s]['joint']['slots'] == k],
                      f'非承重墙 {k} 条槽'))
    print('\n配对检验（A → B，节省 = T_A - T_B）')
    for a, b, seeds, grp in pairs:
        if not seeds:
            continue
        st = paired(T(a, seeds), T(b, seeds))
        print(f'  {NAMES[a]} → {NAMES[b]} [{grp}]：{fmt_pair(st)}')
        flat = {k: (f'{v[0]:.3f}..{v[1]:.3f}' if isinstance(v, tuple) else
                    (round(v, 4) if isinstance(v, float) else v)) for k, v in st.items()}
        tests.append(dict(a=a, b=b, group=grp, **flat))
    write_csv(os.path.join(out, 'paired_tests.csv'), tests)

    # 指标级配对差（联合 vs 现有，非承重墙）
    print('\n联合 vs 现有（非承重墙）各指标平均差（联合 - 现有）：')
    for k in ('stations', 'points', 'travel_m', 'length_m', 'turns'):
        d = np.array([S[s]['joint'][k] - S[s]['current'][k] for s in nlb])
        print(f'  {k}: {d.mean():+.3f}（增加 {int((d > 1e-9).sum())} 面，减少 '
              f'{int((d < -1e-9).sum())} 面）')
    src = defaultdict(int)
    for s in P:
        src['rule' if S[s]['joint']['source'] == 'rule' else 'lattice'] += 1
    print('  联合规划最终方案来源：', dict(src))

    # --- 与下界的差距 ---------------------------------------------------------------
    gaps = {}
    gap_rows = []
    allj = sorted(s for s in S if S[s]['joint']['feasible'] == 1 and
                  S[s]['lower_bound']['feasible'] == 1)
    print('\n与下界的差距 (T - LB) / LB')
    for m, seeds in (('per_slot', P), ('current', P), ('joint', P), ('joint', allj)):
        lb = T('lower_bound', seeds)
        t = T(m, seeds)
        assert (t >= lb - 1e-6).all(), f'{m}: 低于下界'
        g = (t - lb) / lb * 100
        cert = (t - lb) <= CERT
        gaps[(m, len(seeds))] = g
        row = dict(method=m, n=len(seeds), certified=int(cert.sum()),
                   certified_pct=round(100 * cert.mean(), 1), gap_mean_pct=round(g.mean(), 2),
                   gap_median_pct=round(float(np.median(g)), 2),
                   gap_p95_pct=round(float(np.percentile(g, 95)), 2),
                   gap_max_pct=round(float(g.max()), 2))
        gap_rows.append(row)
        print('  ', NAMES[m], row)
    write_csv(os.path.join(out, 'gap.csv'), gap_rows)
    for k in (1, 2, 3, 4):
        seeds = [s for s in P if S[s]['joint']['slots'] == k]
        c = [(S[s]['joint']['t_exact'] - S[s]['lower_bound']['t_exact']) <= CERT for s in seeds]
        print(f'  联合规划 {k} 条槽：证明最优 {sum(c)}/{len(c)}')

    # --- Lee 基线 ------------------------------------------------------------------
    lee = load(os.path.join(data, 'lee.csv'))
    eq = [abs(r['lee_cost'] - r['astar_relaxed_cost']) <= 1e-6 for r in lee]
    nl = [r for r in lee if r['load_bearing'] == 0]
    lee_sum = dict(
        slots=len(lee), cost_equal=sum(eq),
        astar_rule_ok=int(sum(r['astar_rule_ok'] for r in lee)),
        astar_rule_viol_slots=int(sum(r['astar_rule_viol'] > 0 for r in lee)),
        lee_viol_slots=int(sum(r['lee_viol'] > 0 for r in lee)),
        lee_minbend_viol_slots=int(sum(r['lee_minbend_viol'] > 0 for r in lee)),
        lee_viol_slots_partition=int(sum(r['lee_viol'] > 0 for r in nl)),
        partition_slots=len(nl),
        rule_cost_over_relaxed_pct=round(float(np.mean(
            [(r['astar_rule_cost'] / r['astar_relaxed_cost'] - 1) * 100 for r in nl
             if r['astar_rule_ok']])), 2),
        astar_relaxed_ms_median=round(float(np.median([r['astar_relaxed_ms'] for r in lee])), 2),
        lee_ms_median=round(float(np.median([r['lee_ms'] for r in lee])), 1))
    write_csv(os.path.join(out, 'lee_summary.csv'), [lee_sum])
    print('\nLee 布线对照：', lee_sum)

    # --- 时间模型敏感性 -------------------------------------------------------------
    sens_path = os.path.join(data, 'sensitivity.csv')
    sens_rows = []
    if os.path.exists(sens_path):
        sens = load(sens_path)
        print('\n时间模型敏感性（非承重墙，现有 → 联合）')
        for model in dict.fromkeys(r['model'] for r in sens):
            Sm = by_seed([r for r in sens if r['model'] == model])
            seeds = [s for s in Sm if Sm[s]['current']['feasible'] == 1 and
                     Sm[s]['current']['load_bearing'] == 0]
            st = paired([Sm[s]['current']['t_exact'] for s in seeds],
                        [Sm[s]['joint']['t_exact'] for s in seeds])
            cert = np.mean([(Sm[s]['joint']['t_exact'] - Sm[s]['lower_bound']['t_exact']) <= CERT
                            for s in seeds]) * 100
            print(f'  {model}: {fmt_pair(st)}；联合规划证明最优 {cert:.0f} %')
            lo, hi = st['mean_pct_ci']
            sens_rows.append(dict(model=model, n=st['n'], mean_pct=round(st['mean_pct'], 2),
                                  ci=f'{lo:.2f}..{hi:.2f}', wins=st['wins'], ties=st['ties'],
                                  losses=st['losses'], p=st['p'], certified_pct=round(cert, 1)))
        write_csv(os.path.join(out, 'sensitivity_summary.csv'), sens_rows)

    plot_results(S, P, nlb, gaps, out)
    plot_example(S, nlb, out, time_model)


def plot_results(S, P, nlb, gaps, out):
    fs = setup_mpl()
    fig, axs = fs.subplots(2, 2, width='double', height=5.0)

    # (a) 按槽数的相对节省（非承重墙）
    ax = axs[0, 0]
    rng = np.random.default_rng(0)
    for k in (1, 2, 3, 4):
        seeds = [s for s in nlb if S[s]['joint']['slots'] == k]
        for j, (a, m) in enumerate((('per_slot', 'current'), ('current', 'joint'))):
            v = np.array([(S[s][a]['t_exact'] - S[s][m]['t_exact']) / S[s][a]['t_exact'] * 100
                          for s in seeds])
            x = k + (j - 0.5) * 0.36
            ax.plot(x + rng.uniform(-0.09, 0.09, len(v)), v, ls='none',
                    marker=fs.MARKER[ROLES[m]], ms=2.2, mew=0.5, mfc='none',
                    mec=fs.color(ROLES[m]), alpha=0.6, rasterized=True)
            ax.plot([x - 0.13, x + 0.13], [v.mean()] * 2, color=fs.BLACK, lw=1.4)
    ax.set_xticks([1, 2, 3, 4], [f'{k} slot{"s" if k > 1 else ""}\n'
                                 f'(n = {sum(S[s]["joint"]["slots"] == k for s in nlb)})'
                                 for k in (1, 2, 3, 4)])
    ax.set_ylabel(fs.label('Predicted execution-time saving', '%'))
    ax.plot([], [], ls='none', marker=fs.MARKER['baseline'], mfc='none',
            mec=fs.color('baseline'), label='Per-slot → rule-based routes + DP')
    ax.plot([], [], ls='none', marker=fs.MARKER['proposed'], mfc='none',
            mec=fs.color('proposed'), label='Rule-based routes + DP → joint planning')
    ax.plot([], [], color=fs.BLACK, lw=1.4, label='Mean')
    ax.legend(loc='upper left')
    fs.panel_label(ax, 'a', 'Saving per wall (partition walls)')

    # (b) 与下界差距的经验分布
    ax = axs[0, 1]
    for m in METHODS:
        g = np.sort(gaps[(m, len(P))])
        ax.step(np.concatenate([[0], g]), np.arange(len(g) + 1) / len(g), where='post',
                label=NAMES_EN[m], **fs.line(ROLES[m], lw=1.1))
    ax.set_xscale('symlog', linthresh=1)
    fs.plain_ticks(ax, 'x')
    ax.set_xlim(0, max(gaps[('per_slot', len(P))].max() * 1.1, 10))
    ax.set_xlabel(fs.label('Gap to lower bound (T − LB) / LB', '%'))
    ax.set_ylabel('Fraction of walls (cumulative)')
    ax.legend(loc='lower right')
    fs.panel_label(ax, 'b', f'Distance from optimum (paired walls, n = {len(P)})')

    # (c) 可行方案数（全部墙，按槽数）
    ax = axs[1, 0]
    ks = (1, 2, 3, 4)
    tot = [sum(S[s]['joint']['slots'] == k for s in S) for k in ks]
    for j, m in enumerate(('current', 'joint')):
        v = [sum(S[s]['joint']['slots'] == k and S[s][m]['feasible'] == 1 for s in S)
             for k in ks]
        x = np.array(ks) + (j - 0.5) * 0.34
        ax.bar(x, v, width=0.3, label=NAMES_EN[m], **fs.bar(ROLES[m]))
        for xi, vi in zip(x, v):
            ax.text(xi, vi + max(tot) * 0.02, f'{vi}', ha='center', va='bottom', fontsize=7)
    ax.set_xticks(ks, [f'{k} slot{"s" if k > 1 else ""}\n(of {t} walls)'
                       for k, t in zip(ks, tot)])
    ax.set_ylabel('Walls with a compliant plan')
    ax.set_ylim(0, max(tot) * 1.3)
    ax.legend(loc='upper right')
    fs.panel_label(ax, 'c', f'Feasibility (all {len(S)} walls)')

    # (d) 规划耗时
    ax = axs[1, 1]
    for j, m in enumerate(METHODS):
        for k in (1, 2, 3, 4):
            v = [S[s][m]['plan_ms'] / 1000 for s in P if S[s]['joint']['slots'] == k]
            x = k + (j - 1) * 0.25
            q = np.percentile(v, [5, 50, 95])
            ax.plot([x, x], [q[0], q[2]], color=fs.color(ROLES[m]), lw=1.2)
            ax.plot([x], [q[1]], zorder=3, label=NAMES_EN[m] if k == 1 else None,
                    **fs.marker(ROLES[m], ms=3.5))
    ax.set_yscale('log')
    ax.set_ylim(2e-4, 300)
    fs.plain_log(ax)
    ax.set_xticks([1, 2, 3, 4], ['1 slot', '2 slots', '3 slots', '4 slots'])
    ax.set_ylabel(fs.label('Planning time', 's'))
    ax.legend(loc='upper left')
    fs.note(ax, 'Marker: median; bar: 5th–95th percentile', x=0.98, y=0.02, va='bottom')
    fs.panel_label(ax, 'd', 'Planning time (single thread, Python + C++ search)')
    print('\n已保存', fs.save(fig, os.path.join(out, 'fig_results.png')))


def plot_example(S, nlb, out, time_model='align_v3'):
    """节省最多的一面 2–3 条槽的非承重墙：现有方法与联合规划的槽线和停靠站对比."""
    import random

    import matplotlib.patches as mpatches
    import slot_plan as sp
    import task_plan as tp
    import wall_gen as wg
    from scr_planner_py import OrientedAStar
    fs = setup_mpl()
    cand = [s for s in nlb if S[s]['joint']['slots'] in (2, 3)]
    seed = max(cand, key=lambda s: S[s]['current']['t_exact'] - S[s]['joint']['t_exact'])
    pl, robot = OrientedAStar(), sp.RobotModel()
    scene, meta = wg.residential_wall(seed, routable=tp.standalone_routable(pl, robot))
    su = random.Random(seed * 7919 + 1).uniform(0.3, scene.width / 100 - 0.3)
    wall = sp.WallFrame((0.0, 0.0), (6.0, 0.0))
    tm = sp.TimeModel.preset(time_model)
    cur = sp.plan_task(scene, wall, robot, pl, start_u=su, tm=tm)
    joint = sp.plan_task(scene, wall, robot, pl, start_u=su, routes='joint', tm=tm)
    us = [p[0] for s in cur.slots + joint.slots for p in s.cut_path]
    u0, u1 = min(us + [su]) - 0.35, max(us + [su]) + 0.35
    band = robot.reach_band
    fig, axs = fs.subplots(2, 1, width='double', height=3.0, sharex=True)
    slot_col = (fs.ORANGE, fs.PURPLE, fs.SKY)
    for i, (ax, plan, m) in enumerate(((axs[0], cur, 'current'), (axs[1], joint, 'joint'))):
        for o in scene.obstacles:
            x, y, w, h = o.x / 100, o.y / 100, o.w / 100, o.h / 100
            if x + w < u0 or x > u1 or y > 1.0:
                continue
            ax.add_patch(mpatches.Rectangle((x, y), w, h, color=fs.LIGHT, lw=0))
            if w >= 0.08 and y < 0.9:
                name = OBST_EN.get(o.label, o.kind.replace('_', ' ').capitalize())
                ax.text(x + w / 2, min(y + h, 0.9) - 0.03, name, ha='center', va='top',
                        fontsize=7, color=fs.BLACK)
        for k, bs in enumerate(scene.task_boxes):
            for b in bs:
                ax.add_patch(mpatches.Rectangle((b.x / 100, b.y / 100), b.w / 100, b.h / 100,
                                                fill=False, ec=fs.GREY, lw=0.6))
        ax.axhspan(band[0], band[1], color='#EEF4FA', lw=0, zorder=0)
        for st in plan.stations:
            ax.add_patch(mpatches.Rectangle((st.u_lo, band[0]), st.u_hi - st.u_lo,
                                            band[1] - band[0], fill=False,
                                            ec=fs.color(ROLES[m]), lw=0.5))
            ax.plot([st.u_c], [band[0] - 0.035], marker='^', color=fs.color(ROLES[m]), ms=3.5)
        for k, s in enumerate(plan.slots):
            p = np.array(s.cut_path)
            ax.plot(p[:, 0], p[:, 1], color=slot_col[k % 3], lw=1.6, solid_capstyle='round',
                    label=f'Slot {k + 1} ({"low-voltage" if s.circuit == "weak" else "power"})')
        ax.axvline(su, color=fs.GREY, ls=(0, (1, 1.5)), lw=0.8)
        pr = plan.prediction
        fs.panel_label(ax, 'ab'[i], f'{NAMES_EN[m]}: {pr["stations"]} stations, '
                       f'{pr["points"]} arm points, strafe {pr["travel_m"]:.2f} m, '
                       f'predicted {pr["t_total"]:.1f} s')
        ax.set_ylim(band[0] - 0.08, 0.95)
        ax.set_xlim(u0, u1)
        ax.set_ylabel(fs.label('Height v', 'm'))
        ax.set_aspect('equal')
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    h, lab = axs[0].get_legend_handles_labels()
    h += [Line2D([], [], color=fs.GREY, ls=(0, (1, 1.5)), lw=0.8, label='Reference marker'),
          Line2D([], [], color=fs.BLACK, ls='none', marker='^', ms=3.5, label='Station'),
          Patch(fill=False, ec=fs.BLACK, lw=0.5, label='Cutting window per station'),
          Patch(color=fs.LIGHT, label='Obstacle')]
    fig.legend(handles=h, loc='outside lower center', ncol=4)
    axs[1].set_xlabel(fs.label('Along-wall position u', 'm'))
    fs.save(fig, os.path.join(out, 'fig_example.png'))
    print(f'示例墙：seed {seed}，{len(scene.tasks)} 条槽，现有 {cur.prediction} → 联合 '
          f'{joint.prediction}')
    print('已保存', os.path.join(out, 'fig_example.png'))


if __name__ == '__main__':
    main()
