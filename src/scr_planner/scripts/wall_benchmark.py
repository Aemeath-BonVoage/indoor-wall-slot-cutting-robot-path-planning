#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""墙面开槽规划基准：第一版 Python 增强 A* vs C++ 方向感知 A*（无规范 / 有规范）.

随机生成 N 面墙（非承重 / 承重），每面墙一项开槽任务，三种方法：

  py_enhanced  src/path_planning/improved_astar.py 的 EnhancedAStar，
               第一版墙面实验参数 turn_penalty=8、diagonal_penalty=15
  cpp_plain    C++ OrientedAStar，只避障（与第一版规划器同一问题），turn_penalty=4
  cpp_rules    C++ OrientedAStar + 开槽规范（承重墙禁横槽、横槽 ≤ 50 cm、竖槽优先、
               弯曲半径 12 cm、电上水下）

所有路径都用 wall_rules.WallScene.violations（独立于 C++ 的检查）对照完整规范打分。

用法（仓库根目录，先 colcon build --packages-select scr_planner）:
  PYTHONUNBUFFERED=1 python3 src/scr_planner/scripts/wall_benchmark.py
  python3 src/scr_planner/scripts/wall_benchmark.py --plot-only   # 只由已有 CSV 重画图

--plot-only 读取 --out 下的 wall_results.csv / wall_summary.csv，按同一种子重新生成场景，
只重新规划图中的三个示例场景（并核对其指标与 CSV 一致），输出 wall_comparison.png / .pdf
（docs/figstyle 统一样式，英文）。
"""
import argparse
import csv
import os
import random
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.realpath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(HERE, '..', 'python'))
sys.path.insert(0, os.path.join(REPO, 'src', 'path_planning'))

import wall_rules as wr  # noqa: E402
from scr_planner_py import OrientedAStar  # noqa: E402

METHODS = ('py_enhanced', 'cpp_plain', 'cpp_rules')
VIOLATION_KEYS = ('diagonal', 'horizontal_on_load_bearing', 'horizontal_too_long', 'short_leg',
                  'arc_blocked', 'blocked_cell')


class _GridAdapter:
    """给第一版规划器 EnhancedAStar 用的 is_free 接口."""

    def __init__(self, free):
        self.free = free
        self.height, self.width = free.shape

    def is_free(self, x, y, use_inflated=True):
        return 0 <= x < self.width and 0 <= y < self.height and bool(self.free[y, x])


def run_python(ref, scene, plain):
    algo = ref.EnhancedAStar(_GridAdapter(plain < 253), turn_penalty=8.0, diagonal_penalty=15.0,
                             use_manhattan=True, smooth_path=False)
    t0 = time.perf_counter()
    path, _ = algo.search(scene.start, scene.goal)
    return [tuple(p) for p in path], (time.perf_counter() - t0) * 1e3


def metrics(scene, cells):
    if not cells:
        return dict(found=0, length_cm=np.nan, turns=np.nan, J_plain=np.nan, compliant=0,
                    **{k: np.nan for k in VIOLATION_KEYS})
    v = scene.violations(cells)
    segs = wr.segments(cells)
    length = sum(np.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(cells[:-1], cells[1:]))
    turns = len(segs) - 1
    return dict(found=1, length_cm=round(float(length), 2), turns=turns,
                J_plain=round(float(length + wr.TURN_PENALTY * turns), 2),
                compliant=int(sum(v.values()) == 0), **v)


def make_scenes(args, cpp):
    """按种子生成场景；只避障都不可达的场景丢弃并计数."""
    rng = random.Random(args.seed)
    scenes, discarded = [], 0
    for lb, n in ((False, args.partition), (True, args.load_bearing)):
        while sum(s.load_bearing == lb for s in scenes) < n:
            s = wr.random_scene(rng, load_bearing=lb)
            if s is None:
                continue
            # 只避障都不可达的场景（障碍 + 边距把墙面切断）没有比较意义，丢弃并计数
            if not cpp.plan(s.grid(False), s.start, s.goal, **s.params(rules=False))['ok']:
                discarded += 1
                continue
            scenes.append(s)
    return scenes, discarded


def plan_scene(ref, cpp, sc, use_python=True):
    """一个场景上的三种方法：{method: (cells, time_ms, C++ 结果或 None)}."""
    plain = sc.grid(top_preference=False)
    ruled = sc.grid(top_preference=True)
    out = {}
    if use_python:
        cells, t = run_python(ref, sc, plain)
        out['py_enhanced'] = (cells, t, None)
    r = cpp.plan(plain, sc.start, sc.goal, **sc.params(rules=False))
    out['cpp_plain'] = (wr.densify(r['corners']) if r['ok'] else [], r['time_ms'], r)
    r = cpp.plan(ruled, sc.start, sc.goal, **sc.params(rules=True))
    out['cpp_rules'] = (wr.densify(r['corners']) if r['ok'] else [], r['time_ms'], r)
    return out


def _num(v):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return v
    return int(f) if f.is_integer() else f


def plot_only(args, ref, cpp):
    """由已提交的 CSV 重画图：只重新规划示例场景，并核对指标与 CSV 一致."""
    with open(os.path.join(args.out, 'wall_results.csv'), encoding='utf-8') as f:
        rows = [{k: (_num(v) if k not in ('wall', 'start', 'goal', 'task', 'method') else v)
                 for k, v in r.items()} for r in csv.DictReader(f)]
    with open(os.path.join(args.out, 'wall_summary.csv'), encoding='utf-8') as f:
        summary = [{k: (_num(v) if k not in ('wall', 'method') else v) for k, v in r.items()}
                   for r in csv.DictReader(f)]
    scenes, _ = make_scenes(args, cpp)
    by = {(r['scene'], r['method']): r for r in rows}
    for i, sc in enumerate(scenes):         # 场景与 CSV 一致（起终点）
        assert by[(i, 'cpp_rules')]['start'] == str(sc.start), f'scene {i}: start mismatch'
        assert by[(i, 'cpp_rules')]['goal'] == str(sc.goal), f'scene {i}: goal mismatch'
    paths = {}
    for i in pick_examples(scenes, rows):
        out = plan_scene(ref, cpp, scenes[i])
        paths[i] = {m: v[0] for m, v in out.items()}
        for m, (cells, _, _) in out.items():
            got, ref_row = metrics(scenes[i], cells), by[(i, m)]
            for k in ('found', 'length_cm', 'turns', 'compliant') + VIOLATION_KEYS:
                if ref_row['found']:
                    assert abs(float(got[k]) - float(ref_row[k])) < 1e-6, \
                        f'scene {i} {m} {k}: {got[k]} != {ref_row[k]}'
        print(f'示例场景 {i}：重新规划的指标与 wall_results.csv 一致')
    plot(scenes, paths, rows, summary, args.out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--partition', type=int, default=100, help='非承重墙场景数')
    ap.add_argument('--load-bearing', type=int, default=40, help='承重墙场景数')
    ap.add_argument('--seed', type=int, default=2026)
    ap.add_argument('--out', default=os.path.join(REPO, 'docs', 'experiments', 'cpp_planner'))
    ap.add_argument('--no-python', action='store_true', help='跳过第一版规划器（快速调试）')
    ap.add_argument('--plot-only', action='store_true', help='只由已有 CSV 重画图')
    args = ap.parse_args()

    import improved_astar as ref  # noqa: E402  第一版规划器
    cpp = OrientedAStar()
    if args.plot_only:
        return plot_only(args, ref, cpp)
    scenes, discarded = make_scenes(args, cpp)
    print(f'场景: {len(scenes)} 个，丢弃只避障也不可达的 {discarded} 个')

    rows, paths = [], []
    for i, sc in enumerate(scenes):
        plain = sc.grid(top_preference=False)
        out = plan_scene(ref, cpp, sc, not args.no_python)
        # 承重墙只能走竖槽：合规路径存在 <=> 入墙点到线盒的竖直线全程可开槽（独立判定）
        feasible = ''
        if sc.load_bearing:
            x = sc.start[0]
            y0, y1 = sorted((sc.start[1], sc.goal[1]))
            feasible = int(bool((plain[y0:y1 + 1, x] < 253).all()))
            assert feasible == int(out['cpp_rules'][2]['ok']), f'scene {i}: feasibility mismatch'
        for m, (cells, t, r) in out.items():
            row = dict(scene=i, rule_feasible=feasible, wall='load_bearing' if sc.load_bearing else 'partition',
                       width_cm=sc.width, start=sc.start, goal=sc.goal, task=sc.goal_label,
                       method=m, time_ms=round(t, 3),
                       expansions=r['expansions'] if r else np.nan, **metrics(sc, cells))
            rows.append(row)
        paths.append({m: v[0] for m, v in out.items()})
        if m == 'cpp_rules':
            assert not cells or row['compliant'] == 1, f'scene {i}: C++ rules path violates {row}'
        short = ' '.join(f"{m}:{'ok' if rr['found'] else '--'}/{'合规' if rr['compliant'] else '违规'}"
                         f"/{rr['time_ms']:.1f}ms" for m, rr in
                         ((x['method'], x) for x in rows[-len(out):]))
        print(f"[{i:3d}] {'承重' if sc.load_bearing else '非承重'} W={sc.width} {short}")

    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, 'wall_results.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    # --- 汇总 ---------------------------------------------------------------
    summary = []
    for wall in ('partition', 'load_bearing'):
        for m in METHODS:
            rs = [r for r in rows if r['wall'] == wall and r['method'] == m]
            if not rs:
                continue
            found = [r for r in rs if r['found']]
            d = dict(wall=wall, method=m, n=len(rs), found=len(found),
                     compliant=sum(r['compliant'] for r in rs),
                     violating=sum(r['found'] and not r['compliant'] for r in rs),
                     no_path=sum(not r['found'] for r in rs),
                     length_cm_mean=round(np.mean([r['length_cm'] for r in found]), 1) if found else '',
                     turns_mean=round(np.mean([r['turns'] for r in found]), 2) if found else '',
                     time_ms_median=round(float(np.median([r['time_ms'] for r in rs])), 3),
                     time_ms_max=round(float(np.max([r['time_ms'] for r in rs])), 3))
            for k in VIOLATION_KEYS:
                d['paths_with_' + k] = sum(1 for r in found if r[k] > 0)
            summary.append(d)
    with open(os.path.join(args.out, 'wall_summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0].keys()))
        w.writeheader()
        w.writerows(summary)
    for d in summary:
        print(d)

    # 第一版规划器 vs C++ 只避障：同一问题上的最优性（公共目标 J = 长度 + 4 × 转弯）
    if not args.no_python:
        py = {r['scene']: r for r in rows if r['method'] == 'py_enhanced' and r['found']}
        cp = {r['scene']: r for r in rows if r['method'] == 'cpp_plain' and r['found']}
        both = sorted(set(py) & set(cp))
        worse = sum(py[i]['J_plain'] > cp[i]['J_plain'] + 1e-6 for i in both)
        better = sum(py[i]['J_plain'] < cp[i]['J_plain'] - 1e-6 for i in both)
        more_turns = sum(py[i]['turns'] > cp[i]['turns'] for i in both)
        speed = [py[i]['time_ms'] / max(cp[i]['time_ms'], 1e-3) for i in both]
        print(f'\n同一问题（只避障）{len(both)} 个场景：第一版规划器 J 更差 {worse}，更好 {better}，'
              f'转弯更多 {more_turns}；加速比中位 {np.median(speed):.0f}x')

    plot(scenes, paths, rows, summary, args.out)


def pick_examples(scenes, rows):
    """图中的三个示例：横槽过长、弯放不下、承重墙上的横槽（各取第一个满足条件的场景）."""
    by = {(r['scene'], r['method']): r for r in rows}

    def pick(pred):
        for i, sc in enumerate(scenes):
            if pred(i, sc):
                return i
        return None

    examples = [
        pick(lambda i, sc: not sc.load_bearing and by.get((i, 'py_enhanced'), {}).get(
            'horizontal_too_long', 0) > 0 and by[(i, 'cpp_rules')]['found']),
        pick(lambda i, sc: not sc.load_bearing and by.get((i, 'py_enhanced'), {}).get(
            'short_leg', 0) > 0 and by[(i, 'cpp_rules')]['found'] and
            by[(i, 'cpp_rules')]['turns'] >= 2),
        pick(lambda i, sc: sc.load_bearing and by.get((i, 'py_enhanced'), {}).get(
            'horizontal_on_load_bearing', 0) > 0),
    ]
    return [e for e in examples if e is not None][:3]


def plot(scenes, paths, rows, summary, out):
    sys.path.insert(0, os.path.join(REPO, 'docs'))
    import figstyle as fs
    from matplotlib.patches import Rectangle

    by = {(r['scene'], r['method']): r for r in rows}
    examples = pick_examples(scenes, rows)
    kind_color = {'door': '#BFBFBF', 'window': '#C6DBEF', 'beam': '#8C8C8C',
                  'column': '#8C8C8C', 'box': fs.GREY, 'weak_line': '#A1D99B', 'pier': '#DADAEB'}
    viol_en = {'horizontal_too_long': 'horizontal run > 50 cm', 'short_leg': 'no room for bend',
               'horizontal_on_load_bearing': 'horizontal slot on load-bearing wall',
               'arc_blocked': 'bend hits obstacle', 'diagonal': 'diagonal slot',
               'blocked_cell': 'crosses obstacle'}
    styles = (('py_enhanced', 'baseline', 'First-version planner (Python)'),
              ('cpp_rules', 'proposed', 'C++ planner + slotting rules'))
    import textwrap
    fig = fs.figure(width='double', height=4.6)
    gs = fig.add_gridspec(2, max(len(examples), 1), height_ratios=[1.5, 0.8])
    wmax = max(scenes[i].width for i in examples)
    hmax = max(scenes[i].height for i in examples)
    axes = [fig.add_subplot(gs[0, k]) for k in range(len(examples))]
    axes.append(fig.add_subplot(gs[1, :]))
    for n, (ax, i) in enumerate(zip(axes, examples)):
        verdicts = []
        sc = scenes[i]
        blocked = sc.grid(top_preference=False) >= 253
        ax.imshow(np.where(blocked, 0.9, 1.0), cmap='gray', vmin=0, vmax=1, origin='lower',
                  extent=(0, sc.width, 0, sc.height), interpolation='nearest')
        for o in sc.obstacles:
            ax.add_patch(Rectangle((o.x, o.y), o.w, o.h, color=kind_color[o.kind], lw=0))
        for m, role, lab in styles:
            cells = paths[i].get(m) or []
            r = by.get((i, m))
            short = 'First version' if role == 'baseline' else 'C++ + rules'
            if not cells:
                verdicts.append(f'{short}: no compliant path exists')
                continue
            tag = 'compliant' if r['compliant'] else 'violates ' + ', '.join(
                viol_en[k] for k in VIOLATION_KEYS if r[k] > 0)
            verdicts.append(f'{short}: {tag}')
            if m == 'cpp_rules':  # 画出半径 12 cm 的弯管圆弧
                segs = wr.segments(cells)
                pts = [segs[0][0]] + [s[1] for s in segs]
                R = wr.BEND_RADIUS
                xs, ys = [pts[0][0]], [pts[0][1]]
                for k in range(1, len(pts) - 1):
                    a, c, b = np.array(pts[k - 1]), np.array(pts[k]), np.array(pts[k + 1])
                    u = (c - a) / np.linalg.norm(c - a)
                    v = (b - c) / np.linalg.norm(b - c)
                    p0, p1 = c - R * u, c + R * v
                    ctr = p0 + R * v
                    a0 = np.arctan2(*(p0 - ctr)[::-1])
                    da = np.arctan2(*(p1 - ctr)[::-1]) - a0
                    da = (da + np.pi) % (2 * np.pi) - np.pi
                    t = np.linspace(a0, a0 + da, 20)
                    xs += list(ctr[0] + R * np.cos(t))
                    ys += list(ctr[1] + R * np.sin(t))
                xs.append(pts[-1][0])
                ys.append(pts[-1][1])
            else:
                xs, ys = [c[0] for c in cells], [c[1] for c in cells]
            ax.plot(xs, ys, **fs.line(role, lw=1.4 if role == 'baseline' else 1.1))
        ax.plot(*sc.start, 's', color=fs.BLACK, ms=3.5)
        ax.plot(*sc.goal, 'o', color=fs.BLACK, ms=3.5, mfc='white')
        fs.panel_label(ax, 'abcd'[n], f"Scene #{i} "
                       f"({'load-bearing' if sc.load_bearing else 'partition'} wall)")
        ax.add_patch(Rectangle((0, 0), sc.width, sc.height, fill=False, ec=fs.BLACK, lw=0.5))
        ax.set_xlim(0, wmax)          # 三个示例用同一坐标范围，面板等大
        ax.set_ylim(0, hmax)
        ax.set_aspect('equal')
        ax.set_xlabel(fs.label('u', 'cm'))
        ax.annotate('\n'.join(textwrap.fill(v, 46) for v in verdicts), xy=(0.5, 0),
                    xycoords='axes fraction', xytext=(0, -24), textcoords='offset points',
                    ha='center', va='top', fontsize=7)
    axes[0].set_ylabel(fs.label('v', 'cm'))
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    handles = [Line2D([], [], label=lab, **fs.line(role)) for _, role, lab in styles]
    handles += [Line2D([], [], color=fs.BLACK, marker='s', ls='none', ms=3.5,
                       label='Wall entry point'),
                Line2D([], [], color=fs.BLACK, marker='o', mfc='white', ls='none', ms=3.5,
                       label='Target box'),
                Patch(color='0.9', label='Blocked (obstacle + margin)')]
    fig.legend(handles=handles, loc='outside upper center', ncol=5)

    ax = axes[-1]
    labels = {'py_enhanced': 'First-version planner (Python)',
              'cpp_plain': 'C++, obstacle avoidance only', 'cpp_rules': 'C++ + slotting rules'}
    roles = {'py_enhanced': 'baseline', 'cpp_plain': 'alt2', 'cpp_rules': 'proposed'}
    x = np.arange(2)
    for k, m in enumerate(METHODS):
        vals = []
        for wall in ('partition', 'load_bearing'):
            d = next((d for d in summary if d['wall'] == wall and d['method'] == m), None)
            vals.append(100.0 * d['violating'] / d['n'] if d else 0)
        bars = ax.bar(x + (k - 1) * 0.2, vals, 0.19, label=labels[m], **fs.bar(roles[m]))
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, v + 1.5, f'{v:.0f}%', ha='center',
                    va='bottom', fontsize=7)
    ax.set_xticks(x)
    ax.set_xticklabels(['Partition walls (n = 100)', 'Load-bearing walls (n = 40)'])
    ax.set_xlim(-0.5, 2.6)
    ax.set_ylabel(fs.label('Scenes with a\nrule-violating path', '%'))
    ax.set_ylim(0, 118)
    ax.legend(loc='center right')
    fs.panel_label(ax, 'abcd'[len(examples)], 'Rule violations (lower is better)')
    print('图已保存:', fs.save(fig, os.path.join(out, 'wall_comparison.png')))


if __name__ == '__main__':
    main()
