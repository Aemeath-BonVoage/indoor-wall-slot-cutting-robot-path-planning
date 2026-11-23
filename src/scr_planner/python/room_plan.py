# -*- coding: utf-8 -*-
"""房间级任务规划：各面墙的开槽方案 + 墙的访问顺序 + 每面墙的扫掠方向 + 墙间转场.

房间任务（Room）
  walls            若干面墙（map 中的两个端点、是否承重、墙面障碍、开槽任务，即单面墙任务配置）；
                   每面墙有若干定位标记（沿墙位置），机器人到一面墙后先在某个标记前精对位
  boundary         房间地面的边界多边形；floor_obstacles 为地面上的矩形障碍
  start_pose       机器人起始位姿
  links            跨墙角的回路：线盒 A 在一面墙上，线盒 B 在相邻墙上，线管必须连续

每面墙（plan_wall）
  联合规划（task_plan.joint_routes）；没有合规方案时取 plan_alternatives 排序第一的替代方案。
  对两个扫掠方向 d ∈ {+（u 增大）, −} 分别求：入口标记（离该方向第一站最近、且标记前的
  导航目标可站立的标记）、预测作业时间 s(i, d)、入口位姿 e(i, d)（标记前 nav_standoff）、
  出口位姿 x(i, d)（最后一站）。

墙间转场（TransitModel）
  t = t0 + 路径长度 / v + t_align。t0、v 由 system_modernization 的 Nav2（Smac 2D + MPPI）
  18 段导航实测拟合；t_align 取 plan_validation 中“导航 + 首次精对位”的实测总时间减去
  导航段。路径长度：两点连线在可通行区域内时取直线距离，否则取 5 cm 栅格上的最短路
  （FloorMap）。这些都是模型预测，没有在房间内的短距离转场上标定。

顺序问题（solve）
  n 面有任务的墙，每面两个方向：访问全部墙各一次、每面墙选一个方向，使
      T = Σ s(i, d_i) + c0(第一面墙) + Σ c(x(i, d_i) → e(j, d_j))
  最小。这是每簇两个节点的广义旅行商问题（开放路径，不要求回到起点）。n 很小（一个房间
  的墙数），用子集动态规划精确求解：f[S][i, d] = 已完成集合 S、最后停在 (i, d) 的最短时间，
  O(2^n · (2n)^2)。基线：固定顺序（按墙的列出顺序、一律 + 方向）、最近邻（每步去转场时间
  最短的 (墙, 方向)）。固定顺序 + 最优方向、最优顺序 + 固定方向两个消融用同一个 DP 加限制。

跨墙角回路（apply_links）
  A. 墙角过线：两面墙各开一条从线盒到“过线口”的槽，过线口在离墙角 PORT_OFFSET 处（机器人
     在墙角站不进去，够不到更近的位置），两面墙取同一高度 h；过线口到墙角的一小段横槽由
     人工开。只用于两面都是非承重墙的情形（承重墙禁止横槽）。
  B. 走地：两端线盒各开竖槽到地面，线管在垫层内绕过墙角（plan_alternatives.box_stub）。
  先试 A（两面墙加上过线槽后都仍可规划），不行用 B。两种做法线管都是连续的。
"""
import copy
import itertools
import math
from dataclasses import dataclass, field

import numpy as np

import plan_alternatives as pa
import slot_plan as sp
import task_plan as tp
import wall_rules as wr

# -- 机器人与场地常量 ---------------------------------------------------------
ROBOT_RADIUS = 0.45      # Nav2 代价地图的 robot_radius 0.4 m + 0.05 m 余量
NAV_STANDOFF = 1.2       # 导航目标离墙距离（scr_task task_executor.yaml：nav_standoff）
MARKER_SPACING = 0.6     # [假设] 每面有任务的墙按此间距布置定位标记（room3 南墙的标记地图间距）
PORT_OFFSET = 40         # 过线口离墙角的距离（厘米）= ROBOT_RADIUS − 机械臂向一侧的可达量
GRID = 0.05              # 地面栅格分辨率（米）


@dataclass
class TransitModel:
    """墙间转场时间（仿真时间，秒）：Nav2 导航 + 到新墙面后的首次精对位.

    t0、v：docs/experiments/system_modernization/runs/nav_mppi_smac2d/legs.csv 的 18 段导航
    （路径 3.7–13.1 m）对 t = t0 + d / v 的最小二乘拟合，残差 RMS 1.28 s；房间内转场多为
    1–6 m，低于拟合范围的部分是外推。
    t_align：docs/experiments/plan_validation 中“导航 + 首次精对位”为 42.2–43.6 s（n = 4，
    另有一次因 Nav2 终点收敛多用 15 s），按任务层日志其中导航约 28 s，余下约 15 s。
    """
    t0: float = 3.07
    v: float = 0.417
    t_align: float = 15.0

    def nav(self, d):
        return self.t0 + d / self.v

    def leg(self, d):
        return self.nav(d) + self.t_align


# ---------------------------------------------------------------------------
# 房间任务
# ---------------------------------------------------------------------------
@dataclass
class RoomWall:
    name: str
    frame: sp.WallFrame
    scene: wr.WallScene            # tasks 可以为空（没有任务的墙）
    markers: list = field(default_factory=list)     # 定位标记沿墙位置 u（米）
    meta: dict = field(default_factory=dict)

    @property
    def has_tasks(self):
        return bool(self.scene.tasks)


@dataclass
class Link:
    """跨墙角回路：wall_a 上的线盒 box_a（栅格坐标，厘米）→ wall_b 上的 box_b."""
    wall_a: int
    box_a: tuple
    wall_b: int
    box_b: tuple
    circuit: str = 'strong'
    label_a: str = '已有插座'
    label_b: str = '新插座'


@dataclass
class Room:
    walls: list
    boundary: list                                  # [(x, y)] 地面边界多边形
    floor_obstacles: list = field(default_factory=list)   # [(x, y, w, h, label)] 左下角 + 宽高
    start_pose: tuple = (0.0, 0.0, 0.0)
    links: list = field(default_factory=list)
    name: str = 'room'
    meta: dict = field(default_factory=dict)


# ---------------------------------------------------------------------------
# 地面可通行区域与最短路
# ---------------------------------------------------------------------------
class FloorMap:
    """边界多边形内、离边界与地面障碍不少于 radius 的区域；dist() 给出两点间的路径长度."""

    def __init__(self, boundary, obstacles=(), radius=ROBOT_RADIUS, res=GRID):
        self.res, self.radius = res, radius
        pts = np.array(boundary, float)
        self.x0, self.y0 = pts.min(axis=0)
        nx = int(math.ceil((pts[:, 0].max() - self.x0) / res)) + 1
        ny = int(math.ceil((pts[:, 1].max() - self.y0) / res)) + 1
        xs = self.x0 + np.arange(nx) * res
        ys = self.y0 + np.arange(ny) * res
        X, Y = np.meshgrid(xs, ys)
        inside = np.zeros(X.shape, bool)
        dmin = np.full(X.shape, np.inf)
        n = len(pts)
        for i in range(n):
            (ax, ay), (bx, by) = pts[i], pts[(i + 1) % n]
            cond = (ay > Y) != (by > Y)
            with np.errstate(divide='ignore', invalid='ignore'):
                xi = ax + (Y - ay) * (bx - ax) / (by - ay)
            inside ^= cond & (X < xi)
            dmin = np.minimum(dmin, _seg_dist(X, Y, ax, ay, bx, by))
        free = inside & (dmin >= radius - 1e-9)
        for (ox, oy, ow, oh, *_) in obstacles:
            dx = np.maximum(np.maximum(ox - X, X - (ox + ow)), 0.0)
            dy = np.maximum(np.maximum(oy - Y, Y - (oy + oh)), 0.0)
            free &= np.hypot(dx, dy) >= radius - 1e-9
        self.free = free
        self.shape = free.shape
        self._graph = None
        self._cache = {}

    def cell(self, p):
        return (int(round((p[1] - self.y0) / self.res)), int(round((p[0] - self.x0) / self.res)))

    def is_free(self, p):
        r, c = self.cell(p)
        return 0 <= r < self.shape[0] and 0 <= c < self.shape[1] and bool(self.free[r, c])

    def snap(self, p):
        """离 p 最近的可通行格（p 本身可通行时就是它）."""
        r, c = self.cell(p)
        if 0 <= r < self.shape[0] and 0 <= c < self.shape[1] and self.free[r, c]:
            return r, c
        rr, cc = np.nonzero(self.free)
        k = np.argmin((rr - r) ** 2 + (cc - c) ** 2)
        return int(rr[k]), int(cc[k])

    def line_free(self, p, q):
        n = max(2, int(math.dist(p, q) / (self.res / 2)) + 1)
        for k in range(n + 1):
            t = k / n
            if not self.is_free((p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1]))):
                return False
        return True

    def _build(self):
        from scipy.sparse import coo_matrix
        H, W = self.shape
        idx = np.arange(H * W).reshape(H, W)
        f = self.free
        rows, cols, vals = [], [], []

        def add(a, b, ok, w):
            rows.append(a[ok])
            cols.append(b[ok])
            vals.append(np.full(int(ok.sum()), w))

        add(idx[:, :-1], idx[:, 1:], f[:, :-1] & f[:, 1:], self.res)
        add(idx[:-1, :], idx[1:, :], f[:-1, :] & f[1:, :], self.res)
        d = self.res * math.sqrt(2)
        add(idx[:-1, :-1], idx[1:, 1:],
            f[:-1, :-1] & f[1:, 1:] & f[:-1, 1:] & f[1:, :-1], d)
        add(idx[:-1, 1:], idx[1:, :-1],
            f[:-1, 1:] & f[1:, :-1] & f[:-1, :-1] & f[1:, 1:], d)
        r, c, v = np.concatenate(rows), np.concatenate(cols), np.concatenate(vals)
        self._graph = coo_matrix((v, (r, c)), shape=(H * W, H * W)).tocsr()

    def dist(self, p, q):
        """p → q 的路径长度（米）；不连通时为 inf.

        连线可通行：直线距离。否则：8 邻接栅格最短路（比真实最短路长，最多约 8 %）。
        """
        if self.line_free(p, q):
            return math.dist(p, q)
        from scipy.sparse.csgraph import dijkstra
        if self._graph is None:
            self._build()
        a, b = self.snap(p), self.snap(q)
        src = a[0] * self.shape[1] + a[1]
        if src not in self._cache:
            self._cache[src] = dijkstra(self._graph, directed=False, indices=src)
        d = self._cache[src][b[0] * self.shape[1] + b[1]]
        return float(d) + math.dist(p, self._xy(a)) + math.dist(q, self._xy(b))

    def path(self, p, q):
        """p → q 的路径折线（作图用）：直线可通行时为两点，否则为栅格最短路上的格点."""
        if self.line_free(p, q):
            return [tuple(p), tuple(q)]
        from scipy.sparse.csgraph import dijkstra
        if self._graph is None:
            self._build()
        a, b = self.snap(p), self.snap(q)
        W = self.shape[1]
        _, pred = dijkstra(self._graph, directed=False, indices=a[0] * W + a[1],
                           return_predecessors=True)
        k, cells = b[0] * W + b[1], []
        while k >= 0 and len(cells) < pred.size:
            cells.append(self._xy(divmod(int(k), W)))
            k = pred[k]
        return [tuple(p)] + cells[::-1] + [tuple(q)]

    def _xy(self, rc):
        return (self.x0 + rc[1] * self.res, self.y0 + rc[0] * self.res)


def _seg_dist(X, Y, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    L2 = vx * vx + vy * vy
    t = np.clip(((X - ax) * vx + (Y - ay) * vy) / L2, 0.0, 1.0) if L2 > 0 else 0.0
    return np.hypot(X - (ax + t * vx), Y - (ay + t * vy))


def standable(wall: RoomWall, floor: FloorMap, robot: sp.RobotModel, u):
    """机器人能否停在这面墙的沿墙位置 u（站位，离墙 standoff）."""
    return floor.is_free(wall.frame.to_map(u, robot.standoff))


# ---------------------------------------------------------------------------
# 跨墙角回路
# ---------------------------------------------------------------------------
def _corner_side(wa: RoomWall, wb: RoomWall, tol=0.02):
    """两面墙的公共墙角分别在各自的哪一端（'left' / 'right'）；不相邻时返回 None."""
    ends = lambda w: (('left', w.frame.to_map(0.0)), ('right', w.frame.to_map(w.frame.length)))  # noqa: E731,E501
    for sa, pa_ in ends(wa):
        for sb, pb in ends(wb):
            if np.linalg.norm(pa_ - pb) < tol:
                return sa, sb
    return None


def _row_clear(scene, x0, x1, y, circuit, skip=()):
    for o in scene.obstacles:
        if any(o is b for b in skip):
            continue
        g = o.grown(scene.margin_of(o, circuit))
        if g.y <= y < g.y + g.h and g.x < x1 and x0 < g.x + g.w:
            return False
    return True


def _port_task(wall: RoomWall, side, box_xy, box, label, circuit, h):
    """在墙面上加一条“线盒 → 过线口”的槽；返回 (新 scene, 任务序号) 或 None.

    过线口在列 p（离墙角 PORT_OFFSET），高度 h。过线口到墙角的人工横槽作为障碍；过线口
    朝房间一侧的相邻格封掉，使槽线只能竖直到达过线口（线管在过线口处转成水平，墙角一侧
    的横段长 PORT_OFFSET，不超过横槽长度上限）。
    """
    sc = wall.scene
    if sc.load_bearing:
        return None
    W = sc.width
    p = PORT_OFFSET if side == 'left' else W - 1 - PORT_OFFSET
    x0, x1 = (0, p) if side == 'left' else (p + 1, W)
    probe = pa._clone(sc)
    probe.obstacles += [b for bs in sc.task_boxes for b in bs]
    if not _row_clear(probe, max(x0, wr.EDGE_MARGIN), min(x1, W - wr.EDGE_MARGIN), h, circuit,
                      skip=(box,)):
        return None
    line = 'strong_line' if circuit == 'strong' else 'weak_line'
    s = pa._clone(sc)
    s.obstacles.append(wr.Rect(p + (1 if side == 'left' else -1), h, 1, 1, 'pier', '过线口挡格'))
    s.tasks.append((tuple(box_xy), (p, h), label, '墙角过线'))
    s.circuits.append(circuit)
    s.task_boxes.append((box, wr.Rect(x0, h, x1 - x0, 1, line, '墙角过线横槽')))
    pa._finish(s)
    return s, len(s.tasks) - 1


def _port_route_ok(slot):
    """过线槽的最后一段必须竖直，且长度满足过线口处那个弯的弯曲半径要求."""
    c = tp.path_cells(slot)
    if len(c) < 2 or c[-1][0] != c[-2][0]:
        return False
    leg = abs(c[-1][1] - c[-2][1])
    return leg >= (2 if len(c) > 2 else 1) * wr.BEND_RADIUS


def apply_links(room: Room, robot, planner):
    """把跨墙角回路落实到两面墙上；返回 (新的墙列表, 每面墙的固定竖槽, 说明列表).

    说明项：dict(link, method = corner / floor / ceiling / unresolved, h_cm, manual_m,
    conduit_m, robot_m, protected = {墙序号: 任务序号})。
    """
    walls = [copy.copy(w) for w in room.walls]
    fixed = {i: [] for i in range(len(walls))}
    protected = {i: set() for i in range(len(walls))}
    notes = []
    lo = math.ceil(robot.reach_band[0] / sp.RES)
    hi = math.floor(robot.reach_band[1] / sp.RES)
    for n, ln in enumerate(room.links):
        wa, wb = walls[ln.wall_a], walls[ln.wall_b]
        sides = _corner_side(wa, wb)
        boxes = (pa.wg.box_at(*ln.box_a, ln.label_a), pa.wg.box_at(*ln.box_b, ln.label_b))
        note = dict(link=n, walls=[wa.name, wb.name], method='unresolved', manual_m=0.0,
                    conduit_m=0.0, robot_m=0.0)
        done = False
        if sides is not None:
            best = None
            for h in range(lo + 1, hi + 1):
                ra = _port_task(wa, sides[0], ln.box_a, boxes[0], ln.label_a, ln.circuit, h)
                rb = _port_task(wb, sides[1], ln.box_b, boxes[1], ln.label_b, ln.circuit, h)
                if ra is None or rb is None:
                    continue
                cost, ok = 0.0, True
                for (s, k) in (ra, rb):
                    slots = pa.routable(s, robot, planner)
                    if slots is None or not _port_route_ok(slots[k]):
                        ok = False
                        break
                    cost += sum(sp.polyline_length(x.cut_path) for x in slots)
                if ok and (best is None or cost < best[0] - 1e-9):
                    best = (cost, h, ra, rb)
            if best is not None:
                _, h, (sa, ka), (sb, kb) = best
                wa.scene, wb.scene = sa, sb
                protected[ln.wall_a].add(ka)
                protected[ln.wall_b].add(kb)
                note.update(method='corner', h_cm=h, manual_m=2 * PORT_OFFSET / 100.0)
                done = True
        if not done and sides is not None:
            for side in ('floor', 'ceiling'):
                stubs = []
                for w, xy, box, lab in ((wa, ln.box_a, boxes[0], ln.label_a),
                                        (wb, ln.box_b, boxes[1], ln.label_b)):
                    probe = pa._clone(w.scene)
                    probe.obstacles += [b for bs in w.scene.task_boxes for b in bs]
                    stubs.append(pa.box_stub(probe, xy[0], xy[1], box, lab, ln.circuit, side,
                                             lo, hi, skip=(box,)))
                if any(s is None for s in stubs):
                    continue
                ends = ((ln.wall_a, wa, stubs[0], boxes[0], sides[0], ln.box_a),
                        (ln.wall_b, wb, stubs[1], boxes[1], sides[1], ln.box_b))
                for i, w, st, box, sd, xy in ends:
                    w.scene = pa._clone(w.scene)
                    w.scene.obstacles += [box, st['whole']]
                    if st['slot'] is not None:
                        fixed[i].append(dict(st, box=box, circuit=ln.circuit))
                    note['manual_m'] += st['manual_m']
                    note['robot_m'] += st['robot_m']
                    note['conduit_m'] += (xy[0] if sd == 'left' else w.scene.width - xy[0]) / 100.0
                note.update(method=side)
                done = True
                break
        notes.append({k: (round(v, 3) if isinstance(v, float) else v) for k, v in note.items()})
    return walls, fixed, protected, notes


# ---------------------------------------------------------------------------
# 每面墙：方案 + 两个扫掠方向
# ---------------------------------------------------------------------------
@dataclass
class WallOption:
    direction: str               # '+'：u 增大；'-'：u 减小
    marker_u: float
    plan: sp.SlotPlan
    service: float               # 预测作业时间（秒）
    entry: tuple                 # 导航目标（标记前 NAV_STANDOFF）
    exit: tuple                  # 最后一站的位置
    blocked: int = 0             # 站不进去的停靠站数


@dataclass
class WallResult:
    index: int
    name: str
    status: str                  # success / fallback / no_task / infeasible（没有合规且站得进去的方案）
    options: dict = field(default_factory=dict)    # {'+': WallOption, '-': WallOption}
    alternative: object = None
    n_alternatives: int = 0
    plan_s: float = 0.0
    check_scene: object = None   # 含全部固定竖槽任务的墙面（合规检查与导出用）

    def violations(self, robot, direction=None):
        """独立检查器的违规计数（task_plan.plan_violations + 引线竖槽检查）."""
        d = direction or next(iter(self.options))
        slots = self.options[d].plan.slots
        v = dict(tp.plan_violations(self.check_scene, slots, robot))
        v.update(pa.stub_violations(self.alternative, self.original, slots))
        return v

    original: object = None      # 落实跨墙角回路之前的墙面


def exact_time(stations, start_u, tm):
    u, t = start_u, 0.0
    for st in stations:
        t += tm.move(st.u_c - u) + tm.cut(len(st.points), st.cut_length)
        u = st.u_c
    return t


def directed_plan(slots, wall: RoomWall, floor, robot, tm, direction):
    """给定槽线：按方向选入口标记、划分停靠站并按该方向排序."""
    frame = wall.frame
    base = tp.evaluate(slots, frame, robot, tm, 0.0)
    if not base.stations:
        return None
    cs = sorted(st.u_c for st in base.stations)
    first = cs[0] if direction == '+' else cs[-1]
    ok = [m for m in wall.markers if floor.is_free(frame.to_map(m, NAV_STANDOFF))]
    marker = min(ok or wall.markers or [first], key=lambda m: abs(m - first))
    plan = tp.evaluate(slots, frame, robot, tm, marker)
    plan.stations = sorted(plan.stations, key=lambda s: s.u_c, reverse=(direction == '-'))
    for k, st in enumerate(plan.stations):
        st.index = k
    plan.order = 'sweep' + direction
    plan.routes = 'joint'
    plan.prediction = sp.predict_time(plan.stations, marker, tm)
    last = plan.stations[-1]
    blocked = sum(not standable(wall, floor, robot, st.u_c) for st in plan.stations)
    return WallOption(direction, marker, plan, exact_time(plan.stations, marker, tm),
                      tuple(frame.to_map(marker, NAV_STANDOFF)), (last.pose[0], last.pose[1]),
                      blocked)


def plan_wall(index, wall: RoomWall, floor, robot, planner, tm, extra_fixed=(), protected=()):
    """一面墙的方案与两个方向的选项；extra_fixed 为 apply_links 给出的固定竖槽项."""
    import time
    t0 = time.perf_counter()
    res = WallResult(index, wall.name, 'no_task', original=wall.scene)
    if not wall.has_tasks and not extra_fixed:
        return res
    plan, alt, alts = pa.plan_with_fallback(wall.scene, wall.frame, robot, planner, tm,
                                            extra_fixed=[e['slot'] for e in extra_fixed],
                                            protected=protected,
                                            accept=lambda p: all(standable(wall, floor, robot,
                                                                           st.u_c)
                                                                 for st in p.stations))
    res.plan_s = time.perf_counter() - t0
    res.n_alternatives = len(alts)
    if plan is None:
        res.status = 'infeasible'
        return res
    res.alternative = alt
    cs = pa._clone(alt.check_scene or alt.scene)
    for e in extra_fixed:                 # 固定竖槽在检查用的墙面里是任务，不是障碍
        cs.obstacles = [o for o in cs.obstacles if o is not e['whole'] and o is not e['box']]
        cs.tasks.append(e['task'])
        cs.circuits.append(e['circuit'])
        cs.task_boxes.append((e['box'], e['rest']))
    res.check_scene = pa._finish(cs)
    res.status = 'fallback' if alt.changes else 'success'
    for d in '+-':
        opt = directed_plan(plan.slots, wall, floor, robot, tm, d)
        if opt is not None:
            res.options[d] = opt
    if not res.options:
        res.status = 'no_task'           # 全部改走墙外且带内没有竖槽
    res.plan_s = time.perf_counter() - t0
    return res


# ---------------------------------------------------------------------------
# 顺序问题：每簇两个节点的广义旅行商（开放路径）
# ---------------------------------------------------------------------------
@dataclass
class Instance:
    """n 个簇（墙），每簇若干选项（方向）。service[i][d]、start[i][d]、trans[i][d][j][e]."""
    service: list
    start: list
    trans: list

    @property
    def n(self):
        return len(self.service)

    def cost(self, seq):
        """seq = [(i, d), ...] 的总时间."""
        t, prev = 0.0, None
        for i, d in seq:
            t += (self.start[i][d] if prev is None else self.trans[prev[0]][prev[1]][i][d])
            t += self.service[i][d]
            prev = (i, d)
        return t

    def parts(self, seq):
        s = sum(self.service[i][d] for i, d in seq)
        return dict(service=s, transit=self.cost(seq) - s)


def solve_dp(inst: Instance, options=None, order=None):
    """子集动态规划求精确最优 → (总时间, [(墙, 选项), ...]).

    options[i]：墙 i 允许的选项下标（默认全部）；order：给定时固定访问顺序，只优化选项。
    """
    n = inst.n
    if n == 0:
        return 0.0, []
    opts = [list(options[i]) if options else list(range(len(inst.service[i]))) for i in range(n)]
    if order is not None:
        f = {d: (inst.start[order[0]][d] + inst.service[order[0]][d], [(order[0], d)])
             for d in opts[order[0]]}
        for a, b in zip(order[:-1], order[1:]):
            f = {e: min(((t + inst.trans[a][d][b][e] + inst.service[b][e], seq + [(b, e)])
                         for d, (t, seq) in f.items()), key=lambda x: x[0]) for e in opts[b]}
        return min(f.values(), key=lambda x: x[0])
    INF = math.inf
    f = [dict() for _ in range(1 << n)]
    for i in range(n):
        for d in opts[i]:
            f[1 << i][(i, d)] = (inst.start[i][d] + inst.service[i][d], None)
    for S in range(1 << n):
        for (i, d), (t, _) in list(f[S].items()):
            for j in range(n):
                if S >> j & 1:
                    continue
                T = S | 1 << j
                for e in opts[j]:
                    c = t + inst.trans[i][d][j][e] + inst.service[j][e]
                    if c < f[T].get((j, e), (INF,))[0] - 1e-12:
                        f[T][(j, e)] = (c, (S, (i, d)))
    full = (1 << n) - 1
    key = min(f[full], key=lambda k: f[full][k][0])
    total, seq, S = f[full][key][0], [], full
    while key is not None:
        seq.append(key)
        back = f[S][key][1]
        S, key = (back if back is not None else (0, None))
    return total, seq[::-1]


def solve_fixed(inst: Instance, option=0):
    """基线：按墙的列出顺序，每面墙取同一个选项."""
    seq = [(i, min(option, len(inst.service[i]) - 1)) for i in range(inst.n)]
    return inst.cost(seq), seq


def solve_nearest(inst: Instance):
    """基线：最近邻，每一步去转场时间最短的 (墙, 选项)."""
    left, seq, prev = set(range(inst.n)), [], None
    while left:
        cands = [(i, d) for i in left for d in range(len(inst.service[i]))]
        nxt = min(cands, key=lambda k: (inst.start[k[0]][k[1]] if prev is None
                                        else inst.trans[prev[0]][prev[1]][k[0]][k[1]], k))
        seq.append(nxt)
        left.remove(nxt[0])
        prev = nxt
    return inst.cost(seq), seq


def solve_brute(inst: Instance):
    """全枚举（只用于测试对照）."""
    best = (math.inf, [])
    for perm in itertools.permutations(range(inst.n)):
        for ds in itertools.product(*[range(len(inst.service[i])) for i in perm]):
            seq = list(zip(perm, ds))
            c = inst.cost(seq)
            if c < best[0] - 1e-12:
                best = (c, seq)
    return best


METHODS = ('fixed', 'nearest', 'fixed_order_best_dir', 'best_order_fixed_dir', 'optimal')


def solve(inst: Instance, method='optimal'):
    if method == 'optimal':
        return solve_dp(inst)
    if method == 'fixed':
        return solve_fixed(inst)
    if method == 'nearest':
        return solve_nearest(inst)
    if method == 'fixed_order_best_dir':
        return solve_dp(inst, order=list(range(inst.n)))
    if method == 'best_order_fixed_dir':
        return solve_dp(inst, options=[[0]] * inst.n)
    raise ValueError(method)


# ---------------------------------------------------------------------------
# 整个房间
# ---------------------------------------------------------------------------
@dataclass
class RoomPlan:
    room: Room
    walls: list                       # WallResult（全部墙）
    active: list                      # 参与排序的墙在 walls 中的下标
    instance: Instance
    sequence: list                    # [(walls 下标, 方向 '+'/'-')]
    totals: dict
    methods: dict                     # {方法: dict(t_total, sequence)}
    link_notes: list = field(default_factory=list)
    transit: TransitModel = None
    legs: list = field(default_factory=list)   # 每段转场 dict(to, distance_m, t_nav, t_align)


DIRS = '+-'


def build_instance(room, results, floor, transit):
    active = [r.index for r in results if r.options]
    by = {r.index: r for r in results}
    opt = lambda i, d: by[i].options[DIRS[d]]  # noqa: E731
    start = room.start_pose[:2]
    service = [[opt(i, d).service for d in range(2)] for i in active]
    st = [[transit.leg(floor.dist(start, opt(i, d).entry)) for d in range(2)] for i in active]
    trans = [[[[0.0 if i == j else transit.leg(floor.dist(opt(i, d).exit, opt(j, e).entry))
                for e in range(2)] for j in active] for d in range(2)] for i in active]
    return active, Instance(service, st, trans)


def plan_room(room: Room, robot=None, planner=None, tm=None, transit=None, method='optimal'):
    """房间规划 → RoomPlan（methods 中同时给出各基线的总时间，便于比较）."""
    robot = robot or sp.RobotModel()
    tm = tm or sp.TimeModel()
    transit = transit or TransitModel()
    floor = FloorMap(room.boundary, room.floor_obstacles)
    walls, fixed, protected, notes = apply_links(room, robot, planner)
    results = [plan_wall(i, w, floor, robot, planner, tm, fixed[i], protected[i])
               for i, w in enumerate(walls)]
    return sequence_room(room, results, floor, transit, method, notes)


def sequence_room(room, results, floor, transit, method='optimal', notes=()):
    active, inst = build_instance(room, results, floor, transit)
    methods = {}
    for m in METHODS:
        t, seq = solve(inst, m)
        methods[m] = dict(t_total=t, sequence=[(active[i], DIRS[d]) for i, d in seq], seq=seq)
    t, seq = methods[method]['t_total'], methods[method]['seq']
    by = {r.index: r for r in results}
    legs, prev = [], room.start_pose[:2]
    for i, d in seq:
        o = by[active[i]].options[DIRS[d]]
        dist = floor.dist(prev, o.entry)
        legs.append(dict(to=by[active[i]].name, direction=DIRS[d], distance_m=round(dist, 3),
                         t_nav=round(transit.nav(dist), 2), t_align=transit.t_align,
                         t_service=round(o.service, 2)))
        prev = o.exit
    parts = inst.parts(seq)
    totals = dict(t_total=t, t_service=parts['service'],
                  t_nav=sum(g['t_nav'] for g in legs), t_align=transit.t_align * len(legs),
                  nav_m=sum(g['distance_m'] for g in legs), walls=len(active),
                  stations=sum(len(by[active[i]].options[DIRS[d]].plan.stations) for i, d in seq))
    return RoomPlan(room, results, active, inst, methods[method]['sequence'], totals, methods,
                    list(notes), transit, legs)


# ---------------------------------------------------------------------------
# 配置读取 / 结果导出
# ---------------------------------------------------------------------------
def room_from_config(cfg):
    """房间任务配置（米）→ Room。每面墙的条目就是单面墙任务配置（wall / obstacles / tasks），
    另有 name 与 markers；示例见 config/room_task_room3.yaml."""
    walls = []
    for k, w in enumerate(cfg['walls']):
        entry = dict(w)
        if not entry.get('tasks'):
            # scene_from_config 需要至少一项任务：先放一项占位任务，读完后清空
            entry['tasks'] = [dict(start=[0.5, 0.5], goal=[0.5, 0.6])]
            scene, frame = sp.scene_from_config(entry)
            scene.tasks, scene.circuits, scene.task_boxes = [], [], []
        else:
            scene, frame = sp.scene_from_config(entry)
            if not scene.task_boxes:
                scene.task_boxes = [(pa.wg.box_at(*s, sl), pa.wg.box_at(*g, gl))
                                    for s, g, sl, gl in scene.tasks]
        markers = list(w.get('markers') or default_markers(frame.length))
        walls.append(RoomWall(w.get('name', f'wall{k}'), frame, scene, markers))
    names = [w.name for w in walls]
    cm = lambda p: (int(round(p[0] * 100)), int(round(p[1] * 100)))  # noqa: E731
    links = [Link(names.index(ln['from']['wall']), cm(ln['from']['box']),
                  names.index(ln['to']['wall']), cm(ln['to']['box']), ln.get('circuit', 'strong'),
                  ln['from'].get('label', '已有插座'), ln['to'].get('label', '新插座'))
             for ln in cfg.get('links', [])]
    boundary = cfg.get('boundary') or [tuple(w.frame.to_map(w.frame.length)) for w in walls]
    obstacles = [(o['x'], o['y'], o['w'], o['h'], o.get('label', '')) for o in
                 cfg.get('floor_obstacles', [])]
    return Room(walls, [tuple(p) for p in boundary], obstacles, tuple(cfg['start_pose']), links,
                cfg.get('name', 'room'))


def default_markers(length):
    n = int(math.floor((length - 2 * MARKER_SPACING) / MARKER_SPACING + 1e-9))
    return [round(MARKER_SPACING * (k + 1), 3) for k in range(max(1, n + 1))]


def room_to_config(room: Room):
    import wall_gen as wg
    walls = []
    for w in room.walls:
        f = w.frame
        cfg = wg.scene_to_config(w.scene) if w.has_tasks else dict(
            wall={}, tasks=[], obstacles=[dict(kind=o.kind, label=o.label, u=o.x / 100, v=o.y / 100,
                                               w=o.w / 100, h=o.h / 100)
                                          for o in w.scene.obstacles])
        cfg['wall'] = dict(frame_id=f.frame_id, left_end=[round(float(c), 4) for c in f.p0],
                           right_end=[round(float(c), 4) for c in f.to_map(f.length)],
                           height=w.scene.height / 100.0, load_bearing=bool(w.scene.load_bearing))
        walls.append(dict(name=w.name, markers=[float(m) for m in w.markers], **cfg))
    return dict(
        name=room.name, start_pose=[round(float(v), 3) for v in room.start_pose],
        boundary=[[round(float(x), 4), round(float(y), 4)] for x, y in room.boundary],
        floor_obstacles=[dict(x=o[0], y=o[1], w=o[2], h=o[3], label=o[4] if len(o) > 4 else '')
                         for o in room.floor_obstacles],
        walls=walls,
        links=[{'from': dict(wall=room.walls[ln.wall_a].name,
                             box=[ln.box_a[0] / 100, ln.box_a[1] / 100], label=ln.label_a),
                'to': dict(wall=room.walls[ln.wall_b].name,
                           box=[ln.box_b[0] / 100, ln.box_b[1] / 100], label=ln.label_b),
                'circuit': ln.circuit} for ln in room.links])


def plan_to_dicts(rp: RoomPlan, robot, duration=2.0):
    """→ (房间级摘要 dict, {墙名: 单面墙规划 dict})。单面墙规划与 slot_plan.plan_to_dict 的
    格式相同（执行器可逐面墙直接执行），marker_u 为该墙的入口标记."""
    by = {r.index: r for r in rp.walls}
    r2 = lambda v: round(float(v), 2)  # noqa: E731
    files, seq = {}, []
    for k, ((i, d), leg) in enumerate(zip(rp.sequence, rp.legs)):
        res, wall = by[i], rp.room.walls[i]
        o = res.options[d]
        files[wall.name] = sp.plan_to_dict(o.plan, wall.frame, robot, duration,
                                           scene=res.check_scene,
                                           marker_u=o.marker_u)
        seq.append(dict(step=k + 1, wall=wall.name, file=f'wall_{wall.name}.yaml', direction=d,
                        marker_u=r2(o.marker_u),
                        nav_goal=dict(x=r2(o.entry[0]), y=r2(o.entry[1]),
                                      yaw=r2(o.plan.stations[0].pose[2])),
                        nav_distance_m=leg['distance_m'], t_nav=leg['t_nav'],
                        t_align=leg['t_align'], t_wall=r2(o.service),
                        stations=len(o.plan.stations), status=res.status,
                        blocked_stations=o.blocked))
    changes = []
    for r in rp.walls:
        if r.alternative is not None and r.alternative.changes:
            changes.append(dict(wall=r.name, **r.alternative.summary()))
    summary = dict(
        room=rp.room.name, status='success' if all(r.status != 'infeasible' for r in rp.walls)
        else 'partial',
        prediction={k: (v if isinstance(v, int) else r2(v)) for k, v in rp.totals.items()},
        transit_model=dict(t0=rp.transit.t0, v=rp.transit.v, t_align=rp.transit.t_align),
        sequence=seq,
        methods={m: dict(t_total=r2(v['t_total']),
                         sequence=[f'{rp.room.walls[i].name}{d}' for i, d in v['sequence']])
                 for m, v in rp.methods.items()},
        walls=[dict(name=r.name, status=r.status) for r in rp.walls],
        installer_changes=changes, links=rp.link_notes)
    return summary, files
