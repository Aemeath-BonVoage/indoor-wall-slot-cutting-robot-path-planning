#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""房间级规划与无解墙面替代方案的统计与作图.

  python3 src/scr_planner/scripts/room_planning_report.py [out_dir]

读取 out_dir/data/ 下 infeasible_walls_benchmark.py 与 room_planning_benchmark.py 的输出
（out_dir 默认 docs/experiments/room_planning），写出：
  fallback_summary.csv、room_summary.csv、room_paired_tests.csv、transit_fit.csv
  fig_fallback.png、fig_fallback_example.png、fig_room_example.png、fig_room_results.png、
  fig_transit_model.png（及同名 PDF，样式见 docs/figstyle）
并把报告正文引用的数字打印到终端。

统计口径与 task_planning_report.py 相同：配对差值 d = T_A − T_B，Wilcoxon 符号秩检验
（双侧，|d| < 0.05 s 视为持平并剔除），效应量为配对秩二列相关，均值的 95 % 置信区间来自
配对 bootstrap（10000 次，种子 0）。
"""
import csv
import os
import sys
from collections import Counter, defaultdict

import numpy as np

HERE = os.path.dirname(os.path.realpath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, HERE)
for d in (os.path.join(HERE, '..', 'python'),
          os.path.join(HERE, '..', '..', 'share', 'scr_planner', 'python')):
    if os.path.isdir(d):
        sys.path.insert(0, d)

from task_planning_report import boot_ci, fmt_pair, paired, setup_mpl, write_csv  # noqa: E402

TIERS = ('refeed', 'floor', 'ceiling', 'relocate')
TIER_EN = {'refeed': 'Re-feed', 'floor': 'Floor feed', 'ceiling': 'Ceiling feed',
           'relocate': 'Move box'}
TIER_ROLE = {'refeed': 'proposed', 'floor': 'alt', 'ceiling': 'alt2', 'relocate': 'alt3'}
CAUSES = ('staircase_in_band', 'band_only', 'strong_weak_spacing', 'slot_spacing', 'unresolved')
CAUSE_EN = {'staircase_in_band': 'Staircase\nroutes', 'band_only': 'Reach band\nonly',
            'strong_weak_spacing': 'Power / LV\nspacing', 'slot_spacing': 'Slot\nspacing',
            'unresolved': 'No single\nrule'}
METHODS = ('fixed', 'nearest', 'fixed_order_best_dir', 'best_order_fixed_dir', 'optimal')
METHOD_EN = {'fixed': 'Fixed order', 'nearest': 'Nearest neighbour',
             'fixed_order_best_dir': 'Fixed order, best directions',
             'best_order_fixed_dir': 'Best order, fixed directions', 'optimal': 'Exact (subset DP)'}
METHOD_ROLE = {'fixed': 'baseline', 'nearest': 'alt', 'fixed_order_best_dir': 'alt2',
               'best_order_fixed_dir': 'alt3', 'optimal': 'proposed'}


def load(path):
    with open(path, encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k, v in r.items():
            try:
                r[k] = float(v) if v != '' else None
            except ValueError:
                pass
    return rows


def mean_ci(x):
    x = np.asarray(x, float)
    lo, hi = boot_ci(x, np.mean)
    return float(x.mean()), lo, hi


# ---------------------------------------------------------------------------
# A：无解墙面
# ---------------------------------------------------------------------------
def fallback_stats(data, out):
    W = load(os.path.join(data, 'infeasible_walls.csv'))
    A = load(os.path.join(data, 'infeasible_alternatives.csv'))
    n = len(W)
    print(f'== 无解墙面：{n} 面 ==')
    print('槽数', sorted(Counter(int(w['slots']) for w in W).items()))
    print('最少拿掉的槽数', sorted(Counter(int(w['min_remove']) for w in W).items()))
    print('最小冲突集大小', Counter(len(str(w['core']).split()) for w in W))
    print('冲突回路', Counter(w['conflict'] for w in W))
    print('原因', Counter(w['cause'] for w in W))
    print('放松后可行', Counter(w['relax'] or '(无)' for w in W).most_common())
    rows = []
    for c in CAUSES:
        ws = [w for w in W if w['cause'] == c]
        if ws:
            rows.append(dict(item='cause', name=c, walls=len(ws),
                             **{f'slots{k}': sum(int(w['slots']) == k for w in ws)
                                for k in (2, 3, 4)}))
    feas = defaultdict(set)
    for a in A:
        if a['feasible']:
            feas[int(a['seed'])].add(a['tier'])
    for t in TIERS:
        r = [a for a in A if a['tier'] == t and a['feasible']]
        if not r:
            rows.append(dict(item='tier', name=t, walls=0))
            continue
        cost = [a['installer_cost'] for a in r]
        rows.append(dict(
            item='tier', name=t, walls=len(r),
            first_choice=sum(w['best_tier'] == t for w in W),
            changed1=sum(a['changed'] == 1 for a in r), changed2=sum(a['changed'] == 2 for a in r),
            changed3=sum(a['changed'] >= 3 for a in r),
            cost_median=round(float(np.median(cost)), 2), cost_mean=round(float(np.mean(cost)), 2),
            cost_max=round(max(cost), 2),
            manual_m_mean=round(float(np.mean([a['manual_m'] for a in r])), 3),
            conduit_m_mean=round(float(np.mean([a['conduit_m'] for a in r])), 3),
            robot_stub_m_mean=round(float(np.mean([a['robot_stub_m'] for a in r])), 3),
            moved_cm_mean=round(float(np.mean([a['moved_cm'] for a in r])), 2),
            t_pred_mean=round(float(np.mean([a['t_pred'] for a in r])), 1),
            violations=int(sum(a['violations'] for a in r))))
    cum, got = [], set()
    for t in TIERS:
        got |= {s for s, v in feas.items() if t in v}
        cum.append((t, len(got)))
    rows.append(dict(item='any', name='any_tier', walls=len(feas)))
    rows += [dict(item='cumulative', name=t, walls=k) for t, k in cum]
    left = [w for w in W if int(w['seed']) not in feas]
    rows.append(dict(item='remaining', name=' '.join(str(int(w['seed'])) for w in left),
                     walls=len(left)))
    write_csv(os.path.join(out, 'fallback_summary.csv'), rows)
    for r in rows:
        print('  ', {k: v for k, v in r.items() if v is not None})
    print('各级组合', Counter(tuple(sorted(v)) for v in feas.values()).most_common())
    print('refeed 中互换 / 改接',
          sum(1 for a in A if a['tier'] == 'refeed' and a['feasible'] and a['changed'] == 2),
          sum(1 for a in A if a['tier'] == 'refeed' and a['feasible'] and a['changed'] == 1))
    print('耗时：诊断 %.1f s，枚举 %.1f s（最大 %.1f s）每面墙' % (
        np.mean([w['diag_s'] for w in W]), np.mean([w['alt_s'] for w in W]),
        max(w['alt_s'] for w in W)))
    return W, A, feas


def plot_fallback(W, A, feas, out):
    fs = setup_mpl()
    fig, axs = fs.subplots(1, 3, width='double', height=2.5)
    ax = axs[0]
    causes = [c for c in CAUSES if any(w['cause'] == c for w in W)]
    bottom = np.zeros(len(causes))
    for k, role in ((2, 'neutral'), (3, 'alt2'), (4, 'baseline')):
        h = np.array([sum(w['cause'] == c and int(w['slots']) == k for w in W) for c in causes])
        ax.bar(range(len(causes)), h, bottom=bottom, label=f'{k} slots', **fs.bar(role))
        bottom += h
    for i, b in enumerate(bottom):
        ax.text(i, b + 1, f'{int(b)}', ha='center', va='bottom', fontsize=7)
    ax.set_xticks(range(len(causes)))
    ax.set_xticklabels([CAUSE_EN[c] for c in causes], fontsize=7)
    ax.set_ylabel('Walls')
    ax.set_ylim(0, max(bottom) * 1.2)
    ax.legend(loc='upper right')
    fs.panel_label(ax, 'a', 'Binding rule')
    ax = axs[1]
    n = len(W)
    cnt = [sum(1 for v in feas.values() if t in v) for t in TIERS]
    for i, (t, c) in enumerate(zip(TIERS, cnt)):
        ax.bar(i, c, **fs.bar(TIER_ROLE[t]))
    ax.bar(len(TIERS), len(feas), **fs.bar('neutral'))
    ax.axhline(n, color=fs.BLACK, ls=(0, (1, 1.5)), lw=0.8)
    ax.text(len(TIERS) + 0.4, n + 2, f'{n} walls without a plan', ha='right', va='bottom',
            fontsize=7)
    ax.set_xticks(range(len(TIERS) + 1))
    ax.set_xticklabels([f'{TIER_EN[t].replace(" ", chr(10))}\n({c})' for t, c in zip(TIERS, cnt)]
                       + [f'Any\n\n({len(feas)})'], fontsize=7)
    ax.set_ylabel('Walls made feasible')
    ax.set_ylim(0, n * 1.22)
    fs.panel_label(ax, 'b', 'Per alternative')
    ax = axs[2]
    rng = np.random.default_rng(0)
    for i, t in enumerate(TIERS):
        c = np.array([a['installer_cost'] for a in A if a['tier'] == t and a['feasible']])
        if len(c) == 0:
            continue
        ax.plot(i + rng.uniform(-0.18, 0.18, len(c)), c, alpha=0.6,
                **fs.marker(TIER_ROLE[t], ms=2.5))
        ax.plot([i - 0.3, i + 0.3], [np.median(c)] * 2, color=fs.BLACK, lw=1.0)
    ax.set_yscale('log')
    fs.plain_log(ax)
    ax.set_xticks(range(len(TIERS)))
    ax.set_xticklabels([TIER_EN[t].replace(' ', '\n') for t in TIERS], fontsize=7)
    ax.set_ylabel(fs.label('Installer cost', 'equiv. min'))
    fs.panel_label(ax, 'c', 'Cost (assumed rates)')
    fs.save(fig, os.path.join(out, 'fig_fallback.png'))


def draw_wall(ax, fs, scene, robot, slots=(), stations=(), stubs=(), u_range=None, vmax=0.95):
    import matplotlib.patches as mp
    band = robot.reach_band
    u0, u1 = u_range or (0.0, scene.width / 100)
    ax.axhspan(band[0], band[1], color='#EEF4FA', lw=0, zorder=0)
    for o in scene.obstacles:
        x, y, w, h = o.x / 100, o.y / 100, o.w / 100, o.h / 100
        if x + w < u0 or x > u1 or y > vmax or o.kind in ('strong_line', 'weak_line'):
            continue
        ax.add_patch(mp.Rectangle((x, y), w, h, color=fs.LIGHT, lw=0))
    for bs in scene.task_boxes:
        for b in bs:
            if b.kind == 'box':
                ax.add_patch(mp.Rectangle((b.x / 100, b.y / 100), b.w / 100, b.h / 100,
                                          fill=False, ec=fs.GREY, lw=0.6))
    for st in stations:
        ax.add_patch(mp.Rectangle((st.u_lo, band[0]), st.u_hi - st.u_lo, band[1] - band[0],
                                  fill=False, ec=fs.BLUE, lw=0.4))
    cols = (fs.ORANGE, fs.PURPLE, fs.SKY, fs.GREEN, fs.VERMILLION, fs.BLUE)
    for k, s in enumerate(slots):
        if not s.cut_path:
            continue
        p = np.array(s.cut_path)
        ax.plot(p[:, 0], p[:, 1], color=cols[k % len(cols)], lw=1.6, solid_capstyle='round')
    for x, y0, y1, circ, k in stubs:
        ax.plot([(x + 0.5) / 100] * 2, [y0 / 100, min(y1 / 100, band[0])], color=fs.BLACK,
                ls=(0, (2, 1.2)), lw=1.0)
    ax.set_xlim(u0, u1)
    ax.set_ylim(0.0, vmax)
    ax.set_aspect('equal')
    ax.set_ylabel(fs.label('Height v', 'm'))


def plot_fallback_example(W, out, seed=None):
    """一面无解的墙：原任务（冲突的两条槽各自的规则路线）、换引线点、走地."""
    import plan_alternatives as pa
    import slot_plan as sp
    import task_plan as tp
    import wall_gen as wg
    from scr_planner_py import OrientedAStar
    fs = setup_mpl()
    pl, robot = OrientedAStar(), sp.RobotModel()
    wall = sp.WallFrame((0.0, 0.0), (6.0, 0.0))
    if seed is None:
        seed = next(int(w['seed']) for w in W if int(w['slots']) == 2 and w['best_tier'] == 'refeed'
                    and w['alternatives'] >= 3)
    scene, _ = wg.residential_wall(seed, routable=tp.standalone_routable(pl, robot))
    alts = pa.alternatives(scene, robot, pl)
    picks = [next((a for a in alts if pa.TIERS[a.tier] == t), None) for t in ('refeed', 'floor')]
    alone = []
    for k, (s, g, _, _) in enumerate(scene.tasks):
        grid = sp.route_grid(scene, k, [], robot)
        alone.append(sp.make_slot(scene, k, pl.plan(grid, s, g, **scene.params())))
    us = [p[0] for sc in [scene] + [a.scene for a in picks if a] for t in sc.tasks for p in t[:2]]
    ur = (min(us) / 100 - 0.4, max(us) / 100 + 0.4)
    fig, axs = fs.subplots(3, 1, width='double', height=4.6, sharex=True)
    draw_wall(axs[0], fs, scene, robot, alone, u_range=ur)
    fs.panel_label(axs[0], 'a', 'Original task: each slot routed alone (the routes cross; '
                   'no compliant pair of routes found)')
    for ax, a, letter in zip(axs[1:], picks, 'bc'):
        if a is None:
            continue
        plan = pa.finalize(a, wall, robot, pl)
        s = a.summary()
        draw_wall(ax, fs, a.check_scene or a.scene, robot, plan.slots, plan.stations, a.stubs,
                  u_range=ur)
        extra = (f'{s["conduit_m"]:.2f} m conduit in the screed, {s["manual_m"]:.2f} m cut by '
                 f'hand' if a.stubs else 'no material outside the wall')
        fs.panel_label(ax, letter, f'{TIER_EN[pa.TIERS[a.tier]]}: {extra}; '
                       f'{plan.prediction["stations"]} stations, predicted '
                       f'{plan.prediction["t_total"]:.0f} s')
        print(f'示例墙 {seed} {pa.TIERS[a.tier]}: {s}, 预测 {plan.prediction}, 违规 '
              f'{sum(pa.check(a, scene, robot).values())}')
    axs[2].set_xlabel(fs.label('Along-wall position u', 'm'))
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    h = [Line2D([], [], color=fs.ORANGE, lw=1.6, label='Slot (robot)'),
         Line2D([], [], color=fs.BLACK, ls=(0, (2, 1.2)), lw=1.0,
                label='Vertical chase below the reach band (manual)'),
         Patch(fill=False, ec=fs.GREY, lw=0.6, label='Box'),
         Patch(fill=False, ec=fs.BLUE, lw=0.4, label='Cutting window per station'),
         Patch(color='#EEF4FA', label='Arm reach band'), Patch(color=fs.LIGHT, label='Obstacle')]
    fig.legend(handles=h, loc='outside lower center', ncol=3)
    fs.save(fig, os.path.join(out, 'fig_fallback_example.png'))
    print('示例墙（替代方案）seed', seed)


# ---------------------------------------------------------------------------
# B：房间
# ---------------------------------------------------------------------------
def transit_fit(out):
    legs = load(os.path.join(REPO, 'docs', 'experiments', 'system_modernization', 'runs',
                             'nav_mppi_smac2d', 'legs.csv'))
    legs = [g for g in legs if g['success'] == 1]
    d = np.array([g['path_m'] for g in legs])
    t = np.array([g['time_s'] for g in legs])
    A = np.c_[np.ones_like(d), d]
    (t0, k), *_ = np.linalg.lstsq(A, t, rcond=None)
    rmse = float(np.sqrt(np.mean((A @ [t0, k] - t) ** 2)))
    loo = []
    for i in range(len(d)):
        m = np.arange(len(d)) != i
        c, *_ = np.linalg.lstsq(A[m], t[m], rcond=None)
        loo.append(A[i] @ c - t[i])
    row = dict(n=len(d), t0=round(float(t0), 3), v=round(float(1 / k), 4), rmse=round(rmse, 3),
               loo_rmse=round(float(np.sqrt(np.mean(np.square(loo)))), 3),
               d_min=round(float(d.min()), 2), d_max=round(float(d.max()), 2))
    write_csv(os.path.join(out, 'transit_fit.csv'), [row])
    print('导航时间拟合', row)
    fs = setup_mpl()
    fig, ax = fs.subplots(width='single', height=2.3)
    ax.plot(d, t, label=f'Nav2 legs (n = {len(d)})', **fs.marker('neutral'))
    x = np.linspace(0, d.max() * 1.05, 50)
    ax.plot(x, t0 + k * x, label=f't = {t0:.2f} + d / {1 / k:.3f}', **fs.line('proposed'))
    ax.axvspan(0, d.min(), color=fs.LIGHT, alpha=0.5, lw=0)
    ax.text(d.min() / 2, t.max() * 0.93, 'extrapolated', ha='center', va='top', fontsize=7)
    ax.set_xlabel(fs.label('Path length d', 'm'))
    ax.set_ylabel(fs.label('Navigation time', 's'))
    ax.set_xlim(0, d.max() * 1.05)
    ax.set_ylim(0, t.max() * 1.1)
    ax.legend(loc='lower right')
    fs.save(fig, os.path.join(out, 'fig_transit_model.png'))
    return row


def room_stats(data, out):
    R = load(os.path.join(data, 'rooms.csv'))
    Wl = load(os.path.join(data, 'walls.csv'))
    n = len(R)
    print(f'\n== 房间：{n} 个 ==')
    print('形状', Counter(r['shape'] for r in R), '有任务的墙数',
          sorted(Counter(int(r['active_walls']) for r in R).items()))
    print('墙：', Counter(w['status'] for w in Wl), '替代方案级别',
          Counter(w['tier'] for w in Wl if w['tier']))
    print('跨墙角回路', Counter(r['link_method'] for r in R if r['link_method']))
    print('违规合计', sum(r['violations'] for r in R), '站不进去的停靠站',
          sum(r['blocked_stations'] for r in R))
    bt = [r for r in R if r['t_brute'] is not None]
    print('与全枚举一致', sum(abs(r['t_brute'] - r['t_optimal']) < 1e-6 for r in bt), '/', len(bt))
    tests, summary = [], []
    groups = [('all', R)] + [(f'{k} walls', [r for r in R if int(r['active_walls']) == k])
                             for k in sorted({int(r['active_walls']) for r in R})]
    for name, rows in groups:
        if len(rows) < 2:
            continue
        opt = np.array([r['t_optimal'] for r in rows])
        tr = np.array([r['t_nav'] + r['t_align'] for r in rows])
        srow = dict(group=name, rooms=len(rows), t_optimal_mean=round(float(opt.mean()), 1),
                    stations_mean=round(float(np.mean([r['stations'] for r in rows])), 1),
                    nav_m_mean=round(float(np.mean([r['nav_m'] for r in rows])), 2))
        for key, vals in (('service', [r['t_service'] for r in rows]),
                          ('nav', [r['t_nav'] for r in rows]),
                          ('align', [r['t_align'] for r in rows])):
            m, lo, hi = mean_ci(np.array(vals) / opt * 100)
            srow.update({f'share_{key}_pct': round(m, 2), f'share_{key}_lo': round(lo, 2),
                         f'share_{key}_hi': round(hi, 2)})
        srow['transit_s_mean'] = round(float(tr.mean()), 1)
        for m in METHODS:
            srow[f't_{m}_mean'] = round(float(np.mean([r[f't_{m}'] for r in rows])), 1)
            srow[f'optimal_{m}'] = sum(abs(r[f't_{m}'] - r['t_optimal']) < 0.05 for r in rows)
        summary.append(srow)
        for m in METHODS[:-1]:
            s = paired([r[f't_{m}'] for r in rows], opt)
            # 顺序主要影响转场：再给出节省占该方法“作业以外时间”的比例（作业时间取最优
            # 方案的值；不同方向的作业时间略有差别，所以这是近似）
            tra = np.array([r[f't_{m}'] - r['t_service'] for r in rows])
            rel_tr = (np.array([r[f't_{m}'] for r in rows]) - opt) / np.maximum(tra, 1e-9) * 100
            lo, hi = boot_ci(rel_tr, np.mean)
            tests.append(dict(group=name, a=m, b='optimal', **{
                k: (round(v, 4) if isinstance(v, float) else v) for k, v in s.items()
                if not isinstance(v, tuple)}, mean_pct_lo=round(s['mean_pct_ci'][0], 3),
                mean_pct_hi=round(s['mean_pct_ci'][1], 3),
                mean_s_lo=round(s['mean_s_ci'][0], 2), mean_s_hi=round(s['mean_s_ci'][1], 2),
                of_transit_pct=round(float(rel_tr.mean()), 2), of_transit_lo=round(lo, 2),
                of_transit_hi=round(hi, 2)))
            if name == 'all' or m in ('fixed', 'nearest'):
                print(f'  [{name}] {m} → optimal: {fmt_pair(s)}；占该方法转场时间 '
                      f'{rel_tr.mean():.1f} % [{lo:.1f}, {hi:.1f}]')
    write_csv(os.path.join(out, 'room_summary.csv'), summary)
    write_csv(os.path.join(out, 'room_paired_tests.csv'), tests)
    for s in summary:
        print('  ', s)
    print('规划耗时（每个房间）：墙面规划 中位 %.2f s（95 分位 %.2f，最大 %.2f）；回路 %.2f s；'
          '转场矩阵 %.3f s；DP %.3f ms（最大 %.3f）' % (
              np.median([r['walls_s'] for r in R]), np.percentile([r['walls_s'] for r in R], 95),
              max(r['walls_s'] for r in R), np.median([r['link_s'] for r in R]),
              np.median([r['instance_s'] for r in R]), np.median([r['ms_optimal'] for r in R]),
              max(r['ms_optimal'] for r in R)))
    legs = sum(int(r['active_walls']) for r in R)
    print('最优方案的转场段：%d 段，其中直线可达 %d 段' % (legs, sum(r['direct_legs'] for r in R)))
    return R, Wl


def plot_room_results(R, out):
    fs = setup_mpl()
    fig, axs = fs.subplots(1, 4, width='double', height=2.5)
    ks = sorted({int(r['active_walls']) for r in R})
    rng = np.random.default_rng(0)
    ax = axs[0]
    for j, (m, off) in enumerate((('fixed', -0.2), ('nearest', 0.2))):
        for k in ks:
            d = np.array([r[f't_{m}'] - r['t_optimal'] for r in R if int(r['active_walls']) == k])
            ax.plot(k + off + rng.uniform(-0.12, 0.12, len(d)), d, alpha=0.55,
                    label=METHOD_EN[m] if k == ks[0] else None, **fs.marker(METHOD_ROLE[m], ms=2.3))
            ax.plot([k + off - 0.17, k + off + 0.17], [d.mean()] * 2, color=fs.BLACK, lw=1.0)
    ax.set_xticks(ks)
    ax.set_xlabel('Walls with tasks')
    ax.set_ylabel(fs.label('Extra time vs. optimum', 's'))
    ax.set_ylim(top=ax.get_ylim()[1] * 1.28)
    ax.legend(loc='upper left')
    fs.panel_label(ax, 'a')
    ax = axs[1]
    bottom = np.zeros(len(ks))
    for key, role, name in (('t_service', 'proposed', 'On-wall work'),
                            ('t_align', 'alt2', 'First alignment'),
                            ('t_nav', 'baseline', 'Navigation')):
        h = np.array([np.mean([r[key] / r['t_optimal'] * 100 for r in R
                               if int(r['active_walls']) == k]) for k in ks])
        ax.bar(ks, h, bottom=bottom, label=name, **fs.bar(role))
        bottom += h
    ax.set_xticks(ks)
    ax.set_xlabel('Walls with tasks')
    ax.set_ylabel(fs.label('Share of predicted time', '%'))
    ax.set_ylim(0, 128)
    ax.set_yticks(range(0, 101, 20))
    ax.legend(loc='upper center', ncol=1)
    fs.panel_label(ax, 'b')
    ax = axs[2]
    for m in METHODS[:-1]:
        g = np.sort([(r[f't_{m}'] - r['t_optimal']) / r['t_optimal'] * 100 for r in R])
        ax.step(np.r_[0, g], np.r_[0, np.arange(1, len(g) + 1) / len(g)], where='post',
                label=METHOD_EN[m], **fs.line(METHOD_ROLE[m]))
    ax.set_xlabel(fs.label('Gap to optimum', '%'))
    ax.set_ylabel('Fraction of rooms')
    ax.set_ylim(0, 1.02)
    ax.set_xlim(0, 8)
    ax.legend(loc='lower right')
    fs.panel_label(ax, 'c')
    ax = axs[3]
    w = np.array([r['walls_s'] + r['link_s'] for r in R])
    dp = np.array([r['ms_optimal'] / 1000 for r in R])
    inst = np.array([r['instance_s'] for r in R])
    for i, (v, role) in enumerate(((w, 'proposed'), (inst, 'alt'), (dp, 'alt2'))):
        ax.plot(i + rng.uniform(-0.2, 0.2, len(v)), np.maximum(v, 1e-5), alpha=0.5,
                **fs.marker(role, ms=2.3))
        ax.plot([i - 0.3, i + 0.3], [np.median(v)] * 2, color=fs.BLACK, lw=1.0)
    ax.set_yscale('log')
    fs.plain_log(ax)
    ax.set_xticks(range(3))
    ax.set_xticklabels(['Per-wall\nplans', 'Transit\nmatrix', 'Subset\nDP'], fontsize=7)
    ax.set_ylabel(fs.label('Planning time', 's'))
    fs.panel_label(ax, 'd')
    fs.save(fig, os.path.join(out, 'fig_room_results.png'))


def plot_room_example(R, out, seed=None):
    """一个房间的俯视图：固定顺序与最优顺序的转场路线."""
    import matplotlib.patches as mp
    import room_gen as rg
    import room_plan as rp
    import slot_plan as sp
    import task_plan as tp
    from scr_planner_py import OrientedAStar
    fs = setup_mpl()
    pl, robot = OrientedAStar(), sp.RobotModel()
    if seed is None:
        cand = [r for r in R if int(r['active_walls']) >= 4]
        seed = int(max(cand, key=lambda r: r['t_fixed'] - r['t_optimal'])['seed'])
    room = rg.residential_room(seed, routable=tp.standalone_routable(pl, robot))
    plan = rp.plan_room(room, robot, pl)
    by = {r.index: r for r in plan.walls}
    floor = rp.FloorMap(room.boundary, room.floor_obstacles)
    poly = np.array(room.boundary + [room.boundary[0]])
    fig, axs = fs.subplots(1, 2, width='double', height=3.6, sharey=True)
    for ax, m, letter in ((axs[0], 'fixed', 'a'), (axs[1], 'optimal', 'b')):
        ax.plot(poly[:, 0], poly[:, 1], color=fs.BLACK, lw=1.2)
        for o in room.floor_obstacles:
            ax.add_patch(mp.Rectangle((o[0], o[1]), o[2], o[3], color=fs.LIGHT, lw=0))
        for i, w in enumerate(room.walls):
            mid = w.frame.to_map(w.frame.length / 2, -0.22)
            ax.text(mid[0], mid[1], w.name + (' (LB)' if w.scene.load_bearing else ''),
                    ha='center', va='center', fontsize=7)
            for t in w.scene.tasks:
                a, b = w.frame.to_map(t[0][0] / 100), w.frame.to_map(t[1][0] / 100)
                ax.plot([a[0], b[0]], [a[1], b[1]], color=fs.ORANGE, lw=3.0,
                        solid_capstyle='butt')
        prev = room.start_pose[:2]
        ax.plot(*prev, marker='*', color=fs.BLACK, ms=7, ls='none')
        for step, (i, d) in enumerate(plan.methods[m]['sequence']):
            o = by[i].options[d]
            sts = np.array([st.pose[:2] for st in o.plan.stations])
            ax.plot(sts[:, 0], sts[:, 1], **fs.marker(METHOD_ROLE[m], ms=2.5))
            mk = room.walls[i].frame.to_map(o.marker_u, 0.0)
            ax.plot(mk[0], mk[1], marker='s', color=fs.GREEN, ms=3.5, ls='none')
            path = np.array(floor.path(prev, o.entry))
            ax.plot(path[:-1, 0], path[:-1, 1], color=fs.color(METHOD_ROLE[m]), lw=0.9)
            ax.annotate('', xy=path[-1], xytext=path[-2],
                        arrowprops=dict(arrowstyle='->', color=fs.color(METHOD_ROLE[m]), lw=0.9))
            ax.annotate('', xy=sts[0], xytext=o.entry,
                        arrowprops=dict(arrowstyle='-', color=fs.GREY, lw=0.6, ls=':'))
            if len(sts) > 1:
                ax.annotate('', xy=sts[-1], xytext=sts[0],
                            arrowprops=dict(arrowstyle='->', color=fs.GREY, lw=0.7))
            ax.text(o.entry[0], o.entry[1], f' {step + 1}', fontsize=8, ha='left', va='bottom')
            prev = o.exit
        inst = plan.instance
        parts = inst.parts(plan.methods[m]['seq'])
        fs.panel_label(ax, letter, f'{METHOD_EN[m]}: {plan.methods[m]["t_total"]:.0f} s '
                       f'(transit {parts["transit"]:.0f} s)')
        ax.set_aspect('equal')
        ax.set_xlabel(fs.label('x', 'm'))
        pad = 0.45
        ax.set_xlim(poly[:, 0].min() - pad, poly[:, 0].max() + pad)
        ax.set_ylim(poly[:, 1].min() - pad, poly[:, 1].max() + pad + 0.5)
    axs[0].set_ylabel(fs.label('y', 'm'))
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    h = [Line2D([], [], color=fs.ORANGE, lw=3.0, label='Slot extent on the wall'),
         Line2D([], [], color=fs.BLACK, marker='*', ms=7, ls='none', label='Start pose'),
         Line2D([], [], color=fs.GREEN, marker='s', ms=3.5, ls='none', label='Entry marker'),
         Line2D([], [], color=fs.GREY, marker='o', ms=2.5, ls='none', label='Station'),
         Line2D([], [], color=fs.GREY, lw=0.9, label='Navigation leg (numbered) / sweep'),
         Patch(color=fs.LIGHT, label='Floor obstacle')]
    fig.legend(handles=h, loc='outside lower center', ncol=3)
    fs.save(fig, os.path.join(out, 'fig_room_example.png'))
    print(f'示例房间 seed {seed}: {room.meta}; ' +
          '; '.join(f'{m} {v["t_total"]:.1f} s ' +
                    ' '.join(f'{room.walls[i].name}{d}' for i, d in v['sequence'])
                    for m, v in plan.methods.items()))
    print('  ', {k: round(v, 1) for k, v in plan.totals.items()}, plan.link_notes,
          [(r.name, r.status) for r in plan.walls if r.status != 'no_task'])


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    kw = dict(a[2:].split('=', 1) for a in sys.argv[1:] if a.startswith('--') and '=' in a)
    out = args[0] if args else os.path.join(REPO, 'docs', 'experiments', 'room_planning')
    data = os.path.join(out, 'data')
    if os.path.exists(os.path.join(data, 'infeasible_walls.csv')):
        W, A, feas = fallback_stats(data, out)
        plot_fallback(W, A, feas, out)
        plot_fallback_example(W, out, int(kw['wall']) if 'wall' in kw else None)
    transit_fit(out)
    if os.path.exists(os.path.join(data, 'rooms.csv')):
        R, _ = room_stats(data, out)
        plot_room_results(R, out)
        plot_room_example(R, out, int(kw['room']) if 'room' in kw else None)


if __name__ == '__main__':
    main()
