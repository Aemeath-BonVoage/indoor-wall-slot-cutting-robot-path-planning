# -*- coding: utf-8 -*-
"""墙面开槽规范 → 规划栅格 / 规划参数 / 合规检查，以及随机墙面场景生成.

单位：厘米；栅格 1 cm/格，grid[y, x]，y = 0 为地面、向上为正。

每条规则都注明出处与可信程度：
  [国标]  已核对条文原文或多个转载一致
  [参照]  国标中针对相近情形的数值，借用于开槽
  [行业]  家装行业通行做法，未在国标原文中核对到
  [假设]  本项目的工程假设，可调
"""
import math
import random
from dataclasses import dataclass, field

import numpy as np

# ---------------------------------------------------------------------------
# 规则常量
# ---------------------------------------------------------------------------
CONDUIT_OD = 2.0            # [假设] Φ20 PVC 电线管外径
BEND_RADIUS = 6 * CONDUIT_OD  # [国标] GB 50303-2015 12.2.1：弯曲半径 ≥ 6 × 管外径 → 12 cm
SLOT_DEPTH = CONDUIT_OD + 1.5  # [国标] GB 50303-2015 12.1.3 / 12.2.3：保护层 / 埋深 ≥ 15 mm
SLOT_WIDTH = CONDUIT_OD + 1.5  # [行业] 槽宽 = 管径 + 10~20 mm，取中值
H_RUN_MAX = 50              # [行业] 非承重墙横槽连续长度 ≤ 500 mm（有资料称出自 GB 50327，未核对原文）
PIER_MIN = 50               # [国标] GB 50203-2011 3.0.11：截面长边 < 500 mm 的承重墙体、独立柱内不应埋设管线
OPENING_MARGIN = 20         # [参照] GB 50203-2011 3.0.9：门窗洞口两侧 200 mm 内不得设脚手眼
BEAM_MARGIN = 50            # [参照] GB 50203-2011 3.0.9：梁或梁垫下及其左右 500 mm 内不得设脚手眼
WEAK_CURRENT_SPACING = 30   # [行业] 强弱电平行间距 ≥ 300 mm
BOX_MARGIN = 5              # [假设] 与其他线盒的净距
EDGE_MARGIN = 10            # [假设] 与墙边 / 阴阳角的净距
SLOT_SPACING = 5            # [假设] 同一面墙上两条同类（强电）槽之间的净距
TURN_PENALTY = 4.0          # [假设] 每个弯折算 4 cm 槽长（= 第一版规划器 turn_penalty 8.0 的 90° 折算值）
WEIGHT_X_PARTITION = 2.0    # [假设] 非承重墙“竖槽为主”：横槽每厘米代价 ×2
TOP_PREFERENCE = 0.5        # [行业→假设] “电上水下”：越靠下每格加价越多，最多 +0.5
# 承重墙：[国标] GB 50203-2011 3.0.11 未经设计同意不得开凿水平沟槽 → 禁止横向
# 梁、柱本体：[行业] 主要受力构件严禁开槽 → 不可通行

BLOCKED = 254


@dataclass
class Rect:
    x: int
    y: int
    w: int
    h: int
    kind: str          # door / window / beam / column / box / weak_line / strong_line / pier / slot
    label: str = ''

    def grown(self, m):
        return Rect(self.x - m, self.y - m, self.w + 2 * m, self.h + 2 * m, self.kind, self.label)


@dataclass
class WallScene:
    width: int
    height: int
    load_bearing: bool
    obstacles: list = field(default_factory=list)
    start: tuple = (0, 0)
    goal: tuple = (0, 0)
    start_label: str = ''
    goal_label: str = ''
    # 多条槽：[(start, goal, start_label, goal_label)]；为空时只有 start → goal 一条
    tasks: list = field(default_factory=list)
    # 每条槽的回路类型 'strong' / 'weak'（为空时全部为强电）
    circuits: list = field(default_factory=list)
    # 每条槽两端线盒 [(Rect, Rect)]：只作为其他槽的障碍（本槽从线盒中心出发）
    task_boxes: list = field(default_factory=list)

    # -- 规则 → 栅格 -------------------------------------------------------
    def circuit_of(self, k):
        return self.circuits[k] if k < len(self.circuits) else 'strong'

    def margin_of(self, o, circuit='strong'):
        """障碍外扩距离.  circuit 为正在规划的槽的回路类型：弱电槽与已有弱电线槽只需
        同类槽间距；'slot' / 'slot_x' 为已规划的同类 / 异类（强弱电）槽."""
        if o.kind == 'weak_line':
            return SLOT_SPACING if circuit == 'weak' else WEAK_CURRENT_SPACING
        if o.kind == 'strong_line':         # 已有 / 墙外引入的强电线管（plan_alternatives 的引上段）
            return SLOT_SPACING if circuit == 'strong' else WEAK_CURRENT_SPACING
        return {'door': OPENING_MARGIN, 'window': OPENING_MARGIN, 'beam': BEAM_MARGIN,
                'column': BEAM_MARGIN, 'box': BOX_MARGIN, 'pier': 0, 'slot': SLOT_SPACING,
                'slot_x': WEAK_CURRENT_SPACING}[o.kind]

    def grid(self, top_preference=True, circuit='strong'):
        """uint8 栅格：254 = 不可开槽；其余为“电上水下”代价（0 顶部 ~ 252 底部）."""
        g = np.zeros((self.height, self.width), dtype=np.uint8)
        if top_preference:
            ys = np.arange(self.height, dtype=float)
            g[:] = np.round(252 * (1.0 - ys / (self.height - 1)))[:, None].astype(np.uint8)
        blocked = np.zeros_like(g, dtype=bool)
        e = EDGE_MARGIN
        blocked[:, :e] = blocked[:, -e:] = True
        blocked[:e, :] = blocked[-e:, :] = True
        for o in self.obstacles:
            r = o.grown(self.margin_of(o, circuit))
            x0, y0 = max(0, r.x), max(0, r.y)
            x1, y1 = min(self.width, r.x + r.w), min(self.height, r.y + r.h)
            if x1 > x0 and y1 > y0:
                blocked[y0:y1, x0:x1] = True
        g[blocked] = BLOCKED
        return g

    def params(self, rules=True):
        """C++ 规划器参数.  rules=False：只避障（与第一版规划器同一问题）."""
        p = dict(turn_penalty=TURN_PENALTY, allow_unknown=0, obstacle_threshold=253)
        if not rules:
            return p
        p.update(bend_radius=int(BEND_RADIUS), cost_weight=TOP_PREFERENCE)
        if self.load_bearing:
            p.update(allow_x=0)
        else:
            p.update(weight_x=WEIGHT_X_PARTITION, max_run_x=H_RUN_MAX)
        return p

    # -- 合规检查（与 C++ 实现无关的独立检查） -----------------------------
    def violations(self, cells, circuit='strong'):
        """逐格路径 → 违规项计数 dict."""
        v = dict(diagonal=0, horizontal_on_load_bearing=0, horizontal_too_long=0,
                 short_leg=0, arc_blocked=0, blocked_cell=0)
        if len(cells) < 2:
            return v
        blocked = self.grid(top_preference=False, circuit=circuit) >= 253
        segs = segments(cells)
        for (a, b, kind) in segs:
            if kind == 'diag':
                v['diagonal'] += 1
            elif kind == 'h':
                if self.load_bearing:
                    v['horizontal_on_load_bearing'] += 1
                elif abs(b[0] - a[0]) > H_RUN_MAX:
                    v['horizontal_too_long'] += 1
        r = int(BEND_RADIUS)
        n = len(segs)
        for i, (a, b, kind) in enumerate(segs):
            length = max(abs(b[0] - a[0]), abs(b[1] - a[1]))
            bends = (i > 0) + (i < n - 1)
            if length < bends * r:
                v['short_leg'] += 1
        for i in range(1, n):
            a0, c, k0 = segs[i - 1]
            _, b1, k1 = segs[i]
            if 'diag' in (k0, k1):
                continue
            for dx, dy in arc_cells(heading(a0, c), heading(c, b1), r):
                x, y = c[0] + dx, c[1] + dy
                if not (0 <= x < self.width and 0 <= y < self.height) or blocked[y, x]:
                    v['arc_blocked'] += 1
                    break
        for x, y in cells[1:-1]:
            if blocked[y, x]:
                v['blocked_cell'] += 1
        return v


# ---------------------------------------------------------------------------
# 几何工具
# ---------------------------------------------------------------------------
DIRS = ((1, 0), (0, 1), (-1, 0), (0, -1))


def heading(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    if abs(dx) >= abs(dy):
        return 0 if dx >= 0 else 2
    return 1 if dy > 0 else 3


def segments(cells):
    """逐格路径 → [(起点, 终点, 'h'|'v'|'diag')]，按方向变化切分."""
    out = []
    start = cells[0]
    prev_d = None
    for p, q in zip(cells[:-1], cells[1:]):
        d = (np.sign(q[0] - p[0]), np.sign(q[1] - p[1]))
        if prev_d is not None and d != prev_d:
            out.append((start, p, _kind(prev_d)))
            start = p
        prev_d = d
    out.append((start, cells[-1], _kind(prev_d)))
    return out


def _kind(d):
    if d[0] != 0 and d[1] != 0:
        return 'diag'
    return 'h' if d[1] == 0 else 'v'


def arc_cells(frm, to, r):
    """与 C++ OrientedAStar::arcCells 相同的离散化（拐角在原点）."""
    if r <= 0 or frm % 2 == to % 2:
        return []
    ux, uy = DIRS[frm]
    vx, vy = DIRS[to]
    p0, p1 = (-r * ux, -r * uy), (r * vx, r * vy)
    cx, cy = p0[0] + p1[0], p0[1] + p1[1]
    a0 = math.atan2(p0[1] - cy, p0[0] - cx)
    da = math.atan2(p1[1] - cy, p1[0] - cx) - a0
    while da > math.pi:
        da -= 2 * math.pi
    while da < -math.pi:
        da += 2 * math.pi
    n = max(2, math.ceil(abs(da) * r / 0.25) + 1)
    out = []
    for i in range(n):
        a = a0 + da * i / (n - 1)
        c = (_round(cx + r * math.cos(a)), _round(cy + r * math.sin(a)))
        if c not in out:
            out.append(c)
    return out


def _round(v):  # 与 std::lround 一致（.5 远离 0）
    return int(math.floor(v + 0.5)) if v >= 0 else -int(math.floor(-v + 0.5))


def densify(corners):
    cells = [tuple(corners[0])]
    for (x0, y0), (x1, y1) in zip(corners[:-1], corners[1:]):
        n = abs(x1 - x0) + abs(y1 - y0)
        sx = (x1 > x0) - (x1 < x0)
        sy = (y1 > y0) - (y1 < y0)
        cells += [(x0 + sx * k, y0 + sy * k) for k in range(1, n + 1)]
    return cells


# ---------------------------------------------------------------------------
# 随机墙面场景
# ---------------------------------------------------------------------------
def _overlaps(r, others, gap=0):
    for o in others:
        if (r.x < o.x + o.w + gap and o.x < r.x + r.w + gap and
                r.y < o.y + o.h + gap and o.y < r.y + r.h + gap):
            return True
    return False


def random_scene(rng: random.Random, load_bearing=None, max_tries=200, band=None, n_tasks=1):
    """生成一面带门窗 / 梁柱 / 已有线盒 / 弱电线槽的墙，以及一项开槽任务.

    非承重墙：从上部接线盒引线到新插座 / 开关 / 空调插座。
    承重墙：从顶部（楼板 / 吊顶）引下到新线盒，管线入墙点在线盒正上方
    （横槽被禁止，这是承重墙上唯一的合规走法）。
    返回 None 表示多次尝试仍放不下任务线盒。

    band=(lo, hi)（厘米）：任务线盒全部放在机械臂可切高度带内（墙面布局按机器人能力
    调整，“可切高度”是任务约束）。此时非承重墙为“已有线盒 → 新插座”的低位改造，承重墙
    为同一竖线上上下两个线盒（承重墙只能开竖槽）；n_tasks 条槽共用一面墙。
    band=None 为 2a 规范对比实验用的全高度任务（保持原样以便复现）。
    """
    for _ in range(max_tries):
        lb = rng.random() < 0.5 if load_bearing is None else load_bearing
        W, H = rng.randrange(300, 501, 10), 280
        obs = []
        if rng.random() < 0.5:                                   # 梁（墙顶通长）
            bh = rng.randrange(40, 61, 5)
            obs.append(Rect(0, H - bh, W, bh, 'beam', '梁'))
        if rng.random() < 0.3:                                   # 端部构造柱
            cw = rng.randrange(30, 51, 5)
            obs.append(Rect(0 if rng.random() < 0.5 else W - cw, 0, cw, H, 'column', '柱'))
        if rng.random() < 0.6:                                   # 门
            dw = rng.randrange(80, 101, 10)
            for _ in range(20):
                r = Rect(rng.randrange(EDGE_MARGIN, W - dw - EDGE_MARGIN), 0, dw, 210, 'door', '门')
                if not _overlaps(r, obs, 30):
                    obs.append(r)
                    break
        if rng.random() < 0.6:                                   # 窗
            ww, wh = rng.randrange(100, 161, 10), rng.randrange(120, 151, 10)
            for _ in range(20):
                r = Rect(rng.randrange(EDGE_MARGIN, W - ww - EDGE_MARGIN), 90, ww, wh, 'window', '窗')
                if not _overlaps(r, obs, 30):
                    obs.append(r)
                    break
        if lb:                                                   # 承重墙窄墙垛
            spans = sorted([(o.x, o.x + o.w) for o in obs if o.kind in ('door', 'window', 'column')]
                           + [(-1, 0), (W, W + 1)])
            for (a0, a1), (b0, b1) in zip(spans[:-1], spans[1:]):
                if 0 < b0 - a1 < PIER_MIN:
                    obs.append(Rect(a1, 0, b0 - a1, H, 'pier', '窄墙垛'))
        if rng.random() < 0.5:                                   # 已有弱电线槽（竖向）
            x = rng.randrange(EDGE_MARGIN, W - EDGE_MARGIN)
            r = Rect(x, 30, 2, H - 30, 'weak_line', '弱电线槽')
            if not _overlaps(r, obs, 0):
                obs.append(r)
        for _ in range(rng.randrange(2, 6)):                     # 已有线盒
            y = rng.choice((30, 130, 220))
            for _ in range(20):
                r = Rect(rng.randrange(EDGE_MARGIN, W - 9 - EDGE_MARGIN), y, 9, 9, 'box', '线盒')
                if not _overlaps(r, obs, 15):
                    obs.append(r)
                    break

        scene = WallScene(W, H, lb, obs)
        grid = scene.grid(top_preference=False)
        free = grid < 253

        def free_box(cx, cy):
            return (0 <= cx < W and 0 <= cy < H and
                    free[max(0, cy - 6):cy + 7, max(0, cx - 6):cx + 7].all())

        if band is not None:
            tasks = _in_band_tasks(rng, scene, free_box, band, n_tasks)
            if tasks:
                scene.tasks = tasks
                scene.start, scene.goal, scene.start_label, scene.goal_label = tasks[0]
                return scene
            continue
        goal_y, goal_label = rng.choice(((34, '新插座'), (134, '新开关'), (204, '空调插座')))
        if lb:
            tops = [o.y for o in obs if o.kind == 'beam']
            feed_y = (min(tops) - BEAM_MARGIN - 1) if tops else H - EDGE_MARGIN - 1
            for _ in range(50):
                gx = rng.randrange(EDGE_MARGIN, W - EDGE_MARGIN)
                if free_box(gx, goal_y) and free[feed_y, gx] and feed_y - goal_y > 2 * BEND_RADIUS:
                    scene.start, scene.goal = (gx, feed_y), (gx, goal_y)
                    scene.start_label, scene.goal_label = '顶部入墙点', goal_label
                    return scene
        else:
            for _ in range(50):
                sx, sy = rng.randrange(EDGE_MARGIN, W - EDGE_MARGIN), rng.randrange(150, H - 20)
                gx = rng.randrange(EDGE_MARGIN, W - EDGE_MARGIN)
                if (free_box(sx, sy) and free_box(gx, goal_y) and abs(sx - gx) >= 60
                        and abs(sy - goal_y) > 2 * BEND_RADIUS):
                    scene.start, scene.goal = (sx, sy), (gx, goal_y)
                    scene.start_label, scene.goal_label = '接线盒', goal_label
                    return scene
    return None


def _in_band_tasks(rng, scene, free_box, band, n_tasks):
    """在可切高度带内生成 n_tasks 项开槽任务（线盒中心离带边至少 6 cm：半个 86 盒 + 余量）."""
    lo, hi = math.ceil(band[0] + 6), math.floor(band[1] - 6)
    W = scene.width
    heights = [h for h in (34, 45, 60, 72) if lo <= h <= hi] or [lo]
    boxes, tasks = [], []

    def far(x, y):
        return all(abs(x - bx) >= 25 or abs(y - by) >= 25 for bx, by in boxes)

    for _ in range(n_tasks):
        for _ in range(80):
            if scene.load_bearing:
                gx = rng.randrange(EDGE_MARGIN + 10, W - EDGE_MARGIN - 10)
                sy, gy = sorted(rng.sample(heights, 2)) if len(heights) > 1 else (lo, hi)
                if gy - sy <= 2 * BEND_RADIUS:
                    sy, gy = lo, hi
                s, g = (gx, sy), (gx, gy)
                labels = ('已有插座', '上方新插座')
            else:
                s = (rng.randrange(EDGE_MARGIN + 10, W - EDGE_MARGIN - 10), rng.choice(heights))
                g = (rng.randrange(EDGE_MARGIN + 10, W - EDGE_MARGIN - 10), rng.choice(heights))
                if not 40 <= abs(s[0] - g[0]) <= 160:
                    continue
                labels = ('已有插座', '新插座')
            if free_box(*s) and free_box(*g) and far(*s) and far(*g):
                boxes += [s, g]
                tasks.append((s, g) + labels)
                break
        else:
            return None
    return tasks
