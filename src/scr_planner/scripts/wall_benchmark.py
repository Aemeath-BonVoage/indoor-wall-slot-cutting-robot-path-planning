#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""墙面开槽规划基准：第一版规划器 Python 增强 A* vs C++ 方向感知 A*（无规范 / 有规范）.

随机生成 N 面墙（非承重 / 承重），每面墙一项开槽任务，三种方法：

  py_enhanced  src/path_planning/improved_astar.py 的 EnhancedAStar，
               第一版墙面实验参数 turn_penalty=8、diagonal_penalty=15
  cpp_plain    C++ OrientedAStar，只避障（与第一版规划器同一问题），turn_penalty=4
  cpp_rules    C++ OrientedAStar + 开槽规范（承重墙禁横槽、横槽 ≤ 50 cm、竖槽优先、
               弯曲半径 12 cm、电上水下）

所有路径都用 wall_rules.WallScene.violations（独立于 C++ 的检查）对照完整规范打分。

用法（仓库根目录，先 colcon build --packages-select scr_planner）:
  PYTHONUNBUFFERED=1 python3 src/scr_planner/scripts/wall_benchmark.py
"""
import argparse
import csv
import os
import random
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--partition', type=int, default=100, help='非承重墙场景数')
    ap.add_argument('--load-bearing', type=int, default=40, help='承重墙场景数')
    ap.add_argument('--seed', type=int, default=2026)
    ap.add_argument('--out', default=os.path.join(REPO, 'docs', 'experiments', 'cpp_planner'))
    ap.add_argument('--no-python', action='store_true', help='跳过第一版规划器（快速调试）')
    args = ap.parse_args()

    import improved_astar as ref  # noqa: E402  第一版规划器
    cpp = OrientedAStar()
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
    print(f'场景: {len(scenes)} 个，丢弃只避障也不可达的 {discarded} 个')

    rows, paths = [], []
    for i, sc in enumerate(scenes):
        plain = sc.grid(top_preference=False)
        ruled = sc.grid(top_preference=True)
        out = {}
        if not args.no_python:
            cells, t = run_python(ref, sc, plain)
            out['py_enhanced'] = (cells, t, None)
        r = cpp.plan(plain, sc.start, sc.goal, **sc.params(rules=False))
        out['cpp_plain'] = (wr.densify(r['corners']) if r['ok'] else [], r['time_ms'], r)
        r = cpp.plan(ruled, sc.start, sc.goal, **sc.params(rules=True))
        out['cpp_rules'] = (wr.densify(r['corners']) if r['ok'] else [], r['time_ms'], r)

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


def plot(scenes, paths, rows, summary, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Arc, Rectangle
    matplotlib.rcParams['font.sans-serif'] = ['Noto Sans CJK SC', 'Noto Sans CJK JP',
                                              'Noto Serif CJK JP', 'DejaVu Sans']
    matplotlib.rcParams['axes.unicode_minus'] = False

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
    examples = [e for e in examples if e is not None][:3]
    colors = {'kind': {'door': '#8E9AAF', 'window': '#9FC5E8', 'beam': '#7F7F7F',
                       'column': '#7F7F7F', 'box': '#E06666', 'weak_line': '#6AA84F',
                       'pier': '#B4A7D6'}}
    fig, axes = plt.subplots(1, len(examples) + 1, squeeze=False, figsize=(5.2 * (len(examples) + 1), 4.6),
                             gridspec_kw={'width_ratios': [1.15] * len(examples) + [1.0]})
    axes = axes[0]
    for ax, i in zip(axes, examples):
        sc = scenes[i]
        blocked = sc.grid(top_preference=False) >= 253
        ax.imshow(np.where(blocked, 0.86, 1.0), cmap='gray', vmin=0, vmax=1, origin='lower',
                  extent=(0, sc.width, 0, sc.height))
        for o in sc.obstacles:
            ax.add_patch(Rectangle((o.x, o.y), o.w, o.h, color=colors['kind'][o.kind], alpha=0.9))
        for m, col, ls, lw, lab in (('py_enhanced', '#E67E22', '-', 2.4, '第一版规划器 Python'),
                                    ('cpp_rules', '#1F4E9C', '-', 1.8, 'C++ + 开槽规范')):
            cells = paths[i].get(m) or []
            r = by.get((i, m))
            if not cells:
                ax.plot([], [], color=col, label=f'{lab}: 无合规路径')
                continue
            tag = '合规' if r['compliant'] else '违规: ' + ', '.join(
                {'horizontal_too_long': '横槽>50cm', 'short_leg': '弯放不下',
                 'horizontal_on_load_bearing': '承重墙横槽', 'arc_blocked': '弯碰障碍',
                 'diagonal': '斜槽', 'blocked_cell': '穿障碍'}[k]
                for k in VIOLATION_KEYS if r[k] > 0)
            if m == 'cpp_rules':  # 画出半径 12 cm 的弯管圆弧
                segs = wr.segments(cells)
                pts = [segs[0][0]] + [s[1] for s in segs]
                R = wr.BEND_RADIUS
                xs, ys = [], []
                xs.append(pts[0][0]); ys.append(pts[0][1])
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
                    xs += list(ctr[0] + R * np.cos(t)); ys += list(ctr[1] + R * np.sin(t))
                xs.append(pts[-1][0]); ys.append(pts[-1][1])
                ax.plot(xs, ys, ls, color=col, lw=lw, label=f'{lab}: {tag}')
            else:
                ax.plot([c[0] for c in cells], [c[1] for c in cells], ls, color=col, lw=lw,
                        label=f'{lab}: {tag}')
        ax.plot(*sc.start, 's', color='k', ms=6)
        ax.plot(*sc.goal, 'o', color='#C00000', ms=7)
        ax.set_title(f"场景 #{i}（{'承重墙' if sc.load_bearing else '非承重墙'}，"
                     f"{sc.start_label}→{sc.goal_label}）", fontsize=10)
        ax.set_xlim(0, sc.width)
        ax.set_ylim(0, sc.height)
        ax.set_aspect('equal')
        ax.set_xlabel('cm')
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.12), fontsize=8, frameon=False)

    ax = axes[-1]
    labels = {'py_enhanced': '第一版规划器 Python', 'cpp_plain': 'C++ 只避障', 'cpp_rules': 'C++ + 规范'}
    x = np.arange(2)
    for k, m in enumerate(METHODS):
        vals = []
        for wall in ('partition', 'load_bearing'):
            d = next((d for d in summary if d['wall'] == wall and d['method'] == m), None)
            vals.append(100.0 * d['violating'] / d['n'] if d else 0)
        bars = ax.bar(x + (k - 1) * 0.26, vals, 0.25, label=labels[m],
                      color=('#E67E22', '#9B59B6', '#1F4E9C')[k])
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, v + 1, f'{v:.0f}%', ha='center', fontsize=8)
    ax.set_xticks(x)
    ax.set_xticklabels(['非承重墙', '承重墙'])
    ax.set_ylabel('输出违规路径的场景占比 (%)')
    ax.set_ylim(0, 110)
    ax.set_title('违反开槽规范的输出（越低越好）', fontsize=10)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'wall_comparison.png'), dpi=110, bbox_inches='tight')
    print('图已保存:', os.path.join(out, 'wall_comparison.png'))


if __name__ == '__main__':
    main()
