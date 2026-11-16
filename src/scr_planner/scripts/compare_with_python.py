#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""C++ 增强 A*（scr_planner）与 Python 参考实现的对比基准.

三种实现在 *同一张膨胀栅格* 上、同一组起终点下运行：

  py_enhanced  src/path_planning/improved_astar.py 的 EnhancedAStar（第一版规划器，原样调用）
               8 邻域 + 斜向惩罚 8.0 + 转向惩罚 5.0（90° 折算 2.5），状态 (x, y)
  py_oriented  本脚本里的纯 Python 版方向感知 A*：状态 (x, y, 朝向)，4 邻域，
               代价 = 步数 + 2.5 × 转弯数 —— 与 C++ 版同一算法，用来交叉校验最优代价
  cpp          scr_planner 的 OrientedAStar（bench_grid 可执行文件），参数同上

统一指标（都在稠密逐格路径上计算）：长度、转弯数、横平竖直比例、
公共目标 J = 长度(格) + 2.5 × 转弯数、耗时、扩展节点数。

用法（在仓库根目录）:
  PYTHONUNBUFFERED=1 python3 src/scr_planner/scripts/compare_with_python.py \
      --bench build/scr_planner/bench_grid --out docs/experiments/cpp_planner
"""
import argparse
import csv
import heapq
import math
import os
import random
import subprocess
import sys
import tempfile
import time
from collections import deque

import numpy as np

TURN = 2.5  # 公共转弯代价（格）：Python 版 turn_penalty=5.0 的 90° 折算值


# ----------------------------------------------------------------------------
# 纯 Python 方向感知 A*（与 C++ OrientedAStar 同一算法）
# ----------------------------------------------------------------------------
DX = (1, 0, -1, 0)
DY = (0, 1, 0, -1)


def _min_turns(h, dx, dy):
    need_x = 0 if dx > 0 else (2 if dx < 0 else -1)
    need_y = 1 if dy > 0 else (3 if dy < 0 else -1)
    n = (need_x >= 0) + (need_y >= 0)
    if n == 0:
        return 0
    return n - 1 if h in (need_x, need_y) else n


def oriented_astar(free, start, goal, tp=TURN):
    """free: 2D bool array [y, x]; start/goal: (x, y). 返回 (cells, cost, expansions)."""
    hgt, wid = free.shape
    gx, gy = goal

    def heur(x, y, h):
        dx, dy = gx - x, gy - y
        return abs(dx) + abs(dy) + tp * _min_turns(h, dx, dy)

    g = {}
    parent = {}
    closed = set()
    openq = []
    sx, sy = start
    for h in range(4):
        s = (sx, sy, h)
        g[s] = 0.0
        parent[s] = None
        heapq.heappush(openq, (heur(sx, sy, h), 0.0, s))
    expansions = 0
    while openq:
        f, neg_g, s = heapq.heappop(openq)
        if s in closed:
            continue
        closed.add(s)
        expansions += 1
        x, y, h = s
        if x == gx and y == gy:
            cells = []
            while s is not None:
                cells.append((s[0], s[1]))
                s = parent[s]
            return cells[::-1], g[(x, y, h)], expansions
        for nh in range(4):
            if nh == (h + 2) % 4:
                continue
            nx, ny = x + DX[nh], y + DY[nh]
            if not (0 <= nx < wid and 0 <= ny < hgt) or not free[ny, nx]:
                continue
            ns = (nx, ny, nh)
            if ns in closed:
                continue
            ng = g[s] + 1.0 + (tp if nh != h else 0.0)
            if ng < g.get(ns, math.inf):
                g[ns] = ng
                parent[ns] = s
                heapq.heappush(openq, (ng + heur(nx, ny, nh), -ng, ns))
    return [], math.inf, expansions


# ----------------------------------------------------------------------------
# 指标
# ----------------------------------------------------------------------------
def metrics(cells, res):
    if len(cells) < 2:
        return dict(length_m=0.0, turns=0, hv_ratio=1.0, J=0.0)
    length = 0.0
    turns = 0
    hv = 0
    prev = None
    for (x0, y0), (x1, y1) in zip(cells[:-1], cells[1:]):
        dx, dy = x1 - x0, y1 - y0
        length += math.hypot(dx, dy)
        n = max(abs(dx), abs(dy))
        d = (dx // n, dy // n)
        if dx == 0 or dy == 0:
            hv += 1
        if prev is not None and d != prev:
            turns += 1
        prev = d
    return dict(length_m=length * res, turns=turns, hv_ratio=hv / (len(cells) - 1),
                J=length + TURN * turns)


def densify(corners):
    cells = [corners[0]]
    for (x0, y0), (x1, y1) in zip(corners[:-1], corners[1:]):
        assert x0 == x1 or y0 == y1, 'C++ path segment not axis-aligned'
        n = abs(x1 - x0) + abs(y1 - y0)
        sx = (x1 > x0) - (x1 < x0)
        sy = (y1 > y0) - (y1 < y0)
        cells += [(x0 + sx * k, y0 + sy * k) for k in range(1, n + 1)]
    return cells


# ----------------------------------------------------------------------------
# 起终点采样：最大 4 连通自由区域内、曼哈顿距离足够远的随机点对
# ----------------------------------------------------------------------------
def largest_component(free):
    hgt, wid = free.shape
    label = -np.ones(free.shape, dtype=np.int32)
    best, best_cells = -1, []
    cur = 0
    for y in range(hgt):
        for x in range(wid):
            if not free[y, x] or label[y, x] >= 0:
                continue
            comp = []
            q = deque([(x, y)])
            label[y, x] = cur
            while q:
                cx, cy = q.popleft()
                comp.append((cx, cy))
                for k in range(4):
                    nx, ny = cx + DX[k], cy + DY[k]
                    if 0 <= nx < wid and 0 <= ny < hgt and free[ny, nx] and label[ny, nx] < 0:
                        label[ny, nx] = cur
                        q.append((nx, ny))
            if len(comp) > len(best_cells):
                best, best_cells = cur, comp
            cur += 1
    return best_cells


def sample_pairs(cells, n, min_dist, seed):
    rng = random.Random(seed)
    pairs = []
    while len(pairs) < n:
        a, b = rng.choice(cells), rng.choice(cells)
        if abs(a[0] - b[0]) + abs(a[1] - b[1]) >= min_dist:
            pairs.append((a, b))
    return pairs


def write_pgm(path, arr):
    hgt, wid = arr.shape
    with open(path, 'wb') as f:
        f.write(f'P5\n{wid} {hgt}\n255\n'.encode())
        f.write(arr.astype(np.uint8).tobytes())


# ----------------------------------------------------------------------------
def main():
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.abspath(os.path.join(here, '..', '..', '..'))
    ap = argparse.ArgumentParser()
    ap.add_argument('--repo', default=repo)
    ap.add_argument('--bench', default=os.path.join(repo, 'build', 'scr_planner', 'bench_grid'))
    ap.add_argument('--out', default=os.path.join(repo, 'docs', 'experiments', 'cpp_planner'))
    ap.add_argument('--pairs', type=int, default=30)
    ap.add_argument('--min-dist', type=int, default=80, help='最小曼哈顿距离（格）')
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--cpp-repeats', type=int, default=20)
    ap.add_argument('--py-repeats', type=int, default=3)
    args = ap.parse_args()

    sys.path.insert(0, os.path.join(args.repo, 'src', 'path_planning'))
    import improved_astar as ref  # noqa: E402  (第一版规划器 Python 实现)

    map_dir = os.path.join(args.repo, 'src', 'scr_navigation', 'maps')
    gm = ref.GridMap(os.path.join(map_dir, 'my_map.pgm'), os.path.join(map_dir, 'my_map.yaml'))
    free = gm.inflated_grid == 0
    res = gm.resolution

    comp = largest_component(free)
    print(f'最大连通自由区域: {len(comp)} 格')
    thesis_pair = ref.find_valid_points(gm)  # 毕设论文里用的那一对
    pairs = [tuple(map(tuple, thesis_pair))] + sample_pairs(comp, args.pairs, args.min_dist,
                                                            args.seed)

    os.makedirs(args.out, exist_ok=True)
    tmp = tempfile.mkdtemp(prefix='scr_bench_')
    grid_pgm = os.path.join(tmp, 'grid.pgm')
    write_pgm(grid_pgm, np.where(free, 0, 254))
    pairs_txt = os.path.join(tmp, 'pairs.txt')
    with open(pairs_txt, 'w') as f:
        for (sx, sy), (gx, gy) in pairs:
            f.write(f'{sx} {sy} {gx} {gy}\n')
    paths_txt = os.path.join(tmp, 'paths.txt')

    # --- C++ ---------------------------------------------------------------
    out = subprocess.run([args.bench, grid_pgm, pairs_txt, str(TURN), str(args.cpp_repeats),
                          paths_txt], check=True, capture_output=True, text=True).stdout
    cpp_rows = list(csv.DictReader(out.splitlines()))
    with open(paths_txt) as f:
        cpp_paths = [[tuple(map(int, p.split(','))) for p in line.strip().split(';')]
                     for line in f]

    rows = []
    for i, (s, t) in enumerate(pairs):
        # --- Python 第一版规划器 -------------------------------------------------
        algo = ref.EnhancedAStar(gm, turn_penalty=5.0, diagonal_penalty=8.0, smooth_path=False)
        times = []
        for _ in range(args.py_repeats):
            t0 = time.perf_counter()
            path_e, info_e = algo.search(s, t)
            times.append((time.perf_counter() - t0) * 1e3)
        m_e = metrics(path_e, res)
        # --- Python 方向感知版 ---------------------------------------------
        times_o = []
        for _ in range(args.py_repeats):
            t0 = time.perf_counter()
            path_o, cost_o, exp_o = oriented_astar(free, s, t)
            times_o.append((time.perf_counter() - t0) * 1e3)
        m_o = metrics(path_o, res)
        # --- C++ -----------------------------------------------------------
        cr = cpp_rows[i]
        assert cr['status'] == 'success', cr
        path_c = densify(cpp_paths[i])
        assert path_c[0] == s and path_c[-1] == t
        assert all(free[y, x] for x, y in path_c), 'C++ path through blocked cell'
        m_c = metrics(path_c, res)
        assert abs(float(cr['cost']) - cost_o) < 1e-6, (i, cr['cost'], cost_o)
        assert abs(m_c['J'] - float(cr['cost'])) < 1e-6

        for name, m, t_ms, exp in (
                ('py_enhanced', m_e, float(np.median(times)), info_e['expanded']),
                ('py_oriented', m_o, float(np.median(times_o)), exp_o),
                ('cpp', m_c, float(cr['time_ms_median']), int(cr['expansions']))):
            rows.append(dict(pair=i, sx=s[0], sy=s[1], gx=t[0], gy=t[1], impl=name,
                             length_m=round(m['length_m'], 3), turns=m['turns'],
                             hv_ratio=round(m['hv_ratio'], 4), J=round(m['J'], 3),
                             time_ms=round(t_ms, 3), expansions=exp))
        print(f'[{i:2d}] {s}->{t}  turns py/cpp {m_e["turns"]:2d}/{m_c["turns"]:2d}  '
              f'J {m_e["J"]:7.1f}/{m_c["J"]:7.1f}  time {np.median(times):7.1f} / '
              f'{np.median(times_o):7.1f} / {float(cr["time_ms_median"]):6.2f} ms')

    with open(os.path.join(args.out, 'results.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    # --- 汇总 ---------------------------------------------------------------
    by = {k: [r for r in rows if r['impl'] == k] for k in ('py_enhanced', 'py_oriented', 'cpp')}
    e, c = by['py_enhanced'], by['cpp']
    n = len(c)
    worse_turns = sum(a['turns'] > b['turns'] for a, b in zip(e, c))
    worse_J = sum(a['J'] > b['J'] + 1e-6 for a, b in zip(e, c))
    better_J = sum(a['J'] < b['J'] - 1e-6 for a, b in zip(e, c))
    summary = []
    for k, rs in by.items():
        summary.append(dict(
            impl=k, n=len(rs),
            length_m_mean=round(np.mean([r['length_m'] for r in rs]), 3),
            turns_mean=round(np.mean([r['turns'] for r in rs]), 2),
            turns_total=int(np.sum([r['turns'] for r in rs])),
            hv_ratio_min=round(min(r['hv_ratio'] for r in rs), 4),
            J_mean=round(np.mean([r['J'] for r in rs]), 2),
            time_ms_median=round(float(np.median([r['time_ms'] for r in rs])), 3),
            time_ms_mean=round(float(np.mean([r['time_ms'] for r in rs])), 3),
            time_ms_max=round(float(np.max([r['time_ms'] for r in rs])), 3),
            expansions_mean=int(np.mean([r['expansions'] for r in rs]))))
    with open(os.path.join(args.out, 'summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0].keys()))
        w.writeheader()
        w.writerows(summary)
    print()
    for s_ in summary:
        print(s_)
    speed = [a['time_ms'] / b['time_ms'] for a, b in zip(e, c)]
    speed_o = [a['time_ms'] / b['time_ms'] for a, b in zip(by['py_oriented'], c)]
    print(f'\npy_enhanced 转弯更多: {worse_turns}/{n}, J 更差: {worse_J}/{n}, J 更好: {better_J}/{n}')
    print(f'加速比 py_enhanced/cpp: 中位 {np.median(speed):.0f}x (min {min(speed):.0f}x)')
    print(f'加速比 py_oriented/cpp: 中位 {np.median(speed_o):.0f}x (min {min(speed_o):.0f}x)')

    # --- 图 -----------------------------------------------------------------
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    matplotlib.rcParams['font.sans-serif'] = ['Noto Sans CJK SC', 'Noto Sans CJK JP', 'Noto Serif CJK JP',
                                              'WenQuanYi Micro Hei', 'DejaVu Sans']
    matplotlib.rcParams['axes.unicode_minus'] = False

    # 选 J 差距最大的两对 + 毕设那一对做路径示例
    gaps = sorted(range(n), key=lambda i: e[i]['J'] - c[i]['J'], reverse=True)
    show = [0] + [i for i in gaps if i != 0][:2]
    fig, axes = plt.subplots(1, len(show) + 1, figsize=(5 * (len(show) + 1), 5))
    disp = np.where(gm.grid == 1, 0.0, np.where(gm.grid == 0, 1.0, 0.75))
    disp = np.where((gm.inflated_grid == 1) & (gm.grid == 0), 0.88, disp)
    for ax, i in zip(axes, show):
        s, t = pairs[i]
        algo = ref.EnhancedAStar(gm, turn_penalty=5.0, diagonal_penalty=8.0, smooth_path=False)
        pe, _ = algo.search(s, t)
        pc = densify(cpp_paths[i])
        ax.imshow(disp, cmap='gray', vmin=0, vmax=1, origin='upper')
        ax.plot([p[0] for p in pe], [p[1] for p in pe], '-', color='#E67E22', lw=2.2,
                label=f'Python 增强A*: {e[i]["turns"]} 弯, J={e[i]["J"]:.0f}')
        ax.plot([p[0] for p in pc], [p[1] for p in pc], '--', color='#2471A3', lw=1.6,
                label=f'C++ 方向感知A*: {c[i]["turns"]} 弯, J={c[i]["J"]:.0f}')
        ax.plot(*s, 'o', color='green', ms=7)
        ax.plot(*t, '*', color='red', ms=11)
        ax.set_title(f'起终点对 #{i}' + ('（毕设原始用例）' if i == 0 else ''))
        ax.legend(loc='lower left', fontsize=8)
        ax.set_xticks([])
        ax.set_yticks([])
    ax = axes[-1]
    x = np.arange(n)
    for k, col, lab in (('py_enhanced', '#E67E22', 'Python 增强A*（第一版规划器）'),
                        ('py_oriented', '#8E44AD', 'Python 方向感知A*'),
                        ('cpp', '#2471A3', 'C++ scr_planner')):
        ax.semilogy(x, [r['time_ms'] for r in by[k]], 'o', color=col, ms=4, label=lab)
    ax.set_xlabel('起终点对编号')
    ax.set_ylabel('单次规划耗时 (ms, 对数)')
    ax.set_title('规划耗时')
    ax.grid(True, which='both', alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'comparison.png'), dpi=110)
    print('图已保存:', os.path.join(args.out, 'comparison.png'))


if __name__ == '__main__':
    main()
