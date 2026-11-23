# -*- coding: utf-8 -*-
"""住宅房间随机生成器：矩形或 L 形房间，每面墙由 wall_gen.residential_wall 生成.

    room = room_gen.residential_room(seed, routable=...)

同一种子生成相同的房间（只用 random.Random）。尺寸与布局都是 [假设] 的规则化设定，
不是实测户型：
  平面     70 % 矩形（边长 3.0–6.0 m），30 % L 形（外包 4.5–6.5 m，缺角每边 1.5 m 以上）
  墙       长度不小于 MIN_TASK_WALL 的墙各有 0–3 项开槽任务（概率 0.25 / 0.35 / 0.25 / 0.15），
           至少两面墙有任务；30 % 的墙为承重墙
  可站立   任务两端离墙角不小于 CORNER_KEEP（机器人半径使它站不进墙角）
  地面障碍 0–2 个：靠墙的柜子 / 堆料（挡住一段墙前空间，不与该墙任务的停靠范围重叠）
           或房间中部的堆料（只影响转场路径）
  定位标记 有任务的墙每 0.6 m 一个
  跨墙角回路  40 % 的房间有一条：墙角一侧墙上的已有插座 → 相邻墙上的新点位
  起始位姿 房间内随机的可通行位置
"""
import math
import random

import room_plan as rp
import slot_plan as sp
import wall_gen as wg
import wall_rules as wr

MIN_TASK_WALL = 2.4      # 短于此长度的墙不布置任务（米）
CORNER_KEEP = 55         # 任务端点离墙角的最小距离（厘米）
N_SLOTS_P = ((0, 0.25), (1, 0.35), (2, 0.25), (3, 0.15))


def _polygon(rng):
    """逆时针顶点列表（米）."""
    r = lambda lo, hi: rng.randrange(int(lo * 10), int(hi * 10) + 1) / 10.0  # noqa: E731
    if rng.random() < 0.7:
        a, b = r(3.0, 6.0), r(3.0, 6.0)
        return [(0, 0), (a, 0), (a, b), (0, b)], 'rect'
    a, b = r(4.5, 6.5), r(4.5, 6.5)
    ca, cb = r(1.5, a - 2.4), r(1.5, b - 2.4)
    return [(0, 0), (a, 0), (a, b - cb), (a - ca, b - cb), (a - ca, b), (0, b)], 'L'


def _frames(poly):
    """逆时针多边形的每条边 → WallFrame（面向墙时左端 = 逆时针方向的后一个顶点）.
    返回的墙按顺时针排列：第 i 面墙的右端是第 i+1 面墙的左端."""
    n = len(poly)
    edges = [(poly[(i + 1) % n], poly[i]) for i in range(n)]
    return [sp.WallFrame(a, b) for a, b in reversed(edges)]


def _choice(rng, table):
    x, acc = rng.random(), 0.0
    for v, p in table:
        acc += p
        if x < acc:
            return v
    return table[-1][0]


def _tasks_clear_of_corners(scene):
    return all(CORNER_KEEP <= p[0] <= scene.width - CORNER_KEEP
               for t in scene.tasks for p in t[:2])


def _wall(seed, k, frame, n, routable):
    """第 k 面墙：n 项任务（n = 0 时只保留结构与已有点位）."""
    W = int(round(frame.length * 100))
    if frame.length < MIN_TASK_WALL:                 # 短墙：不布置门窗与点位
        lb = random.Random(seed * 100 + k).random() < 0.3
        return rp.RoomWall(f'w{k}', frame, wr.WallScene(W, 280, lb, []), [],
                           dict(seed=seed * 100 + k, n_slots=0, load_bearing=lb))
    scene, meta = wg.residential_wall(seed * 100 + k, n_slots=max(n, 1), width=W,
                                      routable=routable if n else None,
                                      accept=_tasks_clear_of_corners if n else None)
    if n == 0:
        scene.obstacles += [b for bs in scene.task_boxes for b in bs]
        scene.tasks, scene.circuits, scene.task_boxes = [], [], []
    markers = rp.default_markers(frame.length) if n else []
    return rp.RoomWall(f'w{k}', frame, scene, markers, dict(meta, n_slots=n))


def _span(scene):
    us = [p[0] for t in scene.tasks for p in t[:2]]
    return (min(us) / 100.0, max(us) / 100.0) if us else None


def _floor_obstacles(rng, walls, poly):
    out = []
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    for _ in range(rng.randrange(0, 3)):
        w, h = rng.randrange(4, 13) / 10.0, rng.randrange(4, 7) / 10.0
        if rng.random() < 0.5:                       # 靠墙：贴着某面墙放
            wall = rng.choice(walls)
            L = wall.frame.length
            if L < w + 1.2:
                continue
            u0 = rng.randrange(0, int((L - w) * 10) + 1) / 10.0
            span = _span(wall.scene)
            if span and u0 < span[1] + 0.7 and span[0] - 0.7 < u0 + w:
                continue
            p, q = wall.frame.to_map(u0, 0.0), wall.frame.to_map(u0 + w, h)
            x0, y0 = min(p[0], q[0]), min(p[1], q[1])
            out.append((round(float(x0), 3), round(float(y0), 3),
                        round(abs(float(q[0] - p[0])), 3), round(abs(float(q[1] - p[1])), 3),
                        '靠墙柜'))
        else:                                        # 房间中部
            ax, ay = int(min(xs) * 10) + 18, int(min(ys) * 10) + 18
            x0 = rng.randrange(ax, max(int(max(xs) * 10) - 18 - int(w * 10), ax + 1)) / 10.0
            y0 = rng.randrange(ay, max(int(max(ys) * 10) - 18 - int(h * 10), ay + 1)) / 10.0
            out.append((x0, y0, w, h, '堆料'))
    return out


def _link(rng, walls):
    """一条跨墙角回路：第 i 面墙右端墙角附近的已有插座 → 第 i+1 面墙左端附近的新点位."""
    n = len(walls)
    for _ in range(20):
        i = rng.randrange(n)
        a, b = walls[i], walls[(i + 1) % n]
        if min(a.frame.length, b.frame.length) < MIN_TASK_WALL:
            continue
        if rp._corner_side(a, b) != ('right', 'left'):
            continue
        xa = a.scene.width - rng.randrange(60, 151)
        xb, yb = rng.randrange(60, 151), rng.choice((wg.SOCKET_H, wg.CABINET_H, wg.BEDSIDE_H))

        def free(scene, x, y):
            probe = rp.pa._clone(scene)
            probe.obstacles += [bx for bs in scene.task_boxes for bx in bs]
            ends = [p for t in scene.tasks for p in t[:2]]
            return wg._free_box(probe, 'strong', x, y, []) and \
                all(abs(x - ex) >= 25 or abs(y - ey) >= 25 for ex, ey in ends)

        if free(a.scene, xa, wg.SOCKET_H) and free(b.scene, xb, yb):
            return rp.Link(i, (xa, wg.SOCKET_H), (i + 1) % n, (xb, yb), 'strong', '已有插座',
                           {wg.SOCKET_H: '新插座', wg.CABINET_H: '电视柜插座',
                            wg.BEDSIDE_H: '床头插座'}[yb])
    return None


def residential_room(seed, routable=None, max_tries=50):
    """按种子生成房间任务（rp.Room）."""
    rng = random.Random(seed)
    for attempt in range(max_tries):
        poly, shape = _polygon(rng)
        frames = _frames(poly)
        ns = [(_choice(rng, N_SLOTS_P) if f.length >= MIN_TASK_WALL else 0) for f in frames]
        if sum(n > 0 for n in ns) < 2:
            continue
        try:
            walls = [_wall(seed + 7919 * attempt, k, f, n, routable)
                     for k, (f, n) in enumerate(zip(frames, ns))]
        except RuntimeError:
            continue
        obstacles = _floor_obstacles(rng, walls, poly)
        link = _link(rng, walls) if rng.random() < 0.4 else None
        floor = rp.FloorMap(poly, obstacles)
        robot = sp.RobotModel()
        # 任务范围内的站位与标记前的导航目标都要可站立，且彼此连通
        pts = []
        ok = True
        for w in walls:
            span = _span(w.scene)
            if not span:
                continue
            n = int((span[1] - span[0] + 0.2) / 0.05) + 1
            ok &= all(rp.standable(w, floor, robot, span[0] - 0.1 + 0.05 * k) for k in range(n + 1))
            pts += [tuple(w.frame.to_map(u, robot.standoff)) for u in span]
            ok &= any(floor.is_free(w.frame.to_map(m, rp.NAV_STANDOFF)) for m in w.markers)
        if not ok:
            continue
        rr, cc = floor.free.nonzero()
        if len(rr) == 0:
            continue
        for _ in range(200):
            k = rng.randrange(len(rr))
            start = floor._xy((int(rr[k]), int(cc[k])))
            if all(math.isfinite(floor.dist(start, p)) for p in pts):
                break
        else:
            continue
        yaw = rng.choice((0.0, 1.571, 3.142, -1.571))
        return rp.Room(walls, poly, obstacles, (round(start[0], 3), round(start[1], 3), yaw),
                       [link] if link else [], f'room{seed}',
                       dict(seed=seed, shape=shape, n_walls=len(walls),
                            n_task_walls=sum(n > 0 for n in ns), n_slots=sum(ns),
                            n_floor_obstacles=len(obstacles), link=int(link is not None)))
    raise RuntimeError(f'seed {seed}: no room after {max_tries} tries')
