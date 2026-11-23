# -*- coding: utf-8 -*-
"""任务级联合规划：槽线走向 + 停靠站划分 + 访问顺序，最小化预测执行时间.

问题
  同一组开槽任务往往有多条同样合规、长度和弯数相近的正交走法，但机械臂沿墙只覆盖
  w = 12 cm（RobotModel.window_width），不同走法需要的停靠站数和机械臂点数可能差很多。
  逐条槽按规范代价 J 选线、再划分停靠站（slot_plan.plan_task，routes='rule'）不考虑这一点。

执行时间（slot_plan.TimeModel，Gazebo 标定）
  T = Σ_站 (t_settle + 横移距离 / v) + t_point × 机械臂点数 + 槽线长度 / v_feed
  最后一项只在机械臂以连续轨迹执行时存在（逐点执行时 v_feed = inf，切割耗时含在 t_point 内）。
  每个停靠站内，一段被窗口截出的子折线有 s 个直线小段时需要 4s 个点（退刀 / 下刀 /
  每小段终点 / 每次转向退刀-转刀-下刀 / 末尾退刀，见 slot_plan.count_points），所以
      机械臂点数 = 4 × Σ_槽线直线段 (该段跨过的停靠站窗口数)。
  竖直段只落在一个窗口里；横向段每跨过一条窗口边界多一个小段（多 4 个点：逐点执行时
  约 8 s，连续轨迹时约 2 s）。

方法（plan_task_joint）
  1. 窗口格：取所有停靠站窗口落在相位 φ 的等宽格上（边界列 c ≡ φ mod 12）。给定 φ，
     一条路线的机械臂点数沿路线可加：每段 4 t_point，横向段每穿过一条边界再加 4 t_point；
     新开一个窗口的代价（t_settle + w / v）记在进入该窗口的那一步上；已被其他槽打开的
     窗口不再计。这些都是“列代价”，加到 OrientedAStar 的分段搜索里（step_cost_* /
     through_cost_x）；槽线长度的耗时（每格 RES / v_feed）加在逐格的轴权重上。开槽规范仍是
     硬约束，原规范代价 J 乘 ε 作为同时间下的次序。
  2. 对每个相位 φ（12 个），按任务顺序逐条规划，再做两轮坐标下降（固定其他槽、重规划
     第 k 条，只接受让精确预测时间下降的修改）。
  3. 每个候选（含原规则路线）都用精确的停靠站 DP（slot_plan.partition_dp）+ 一维最优
     扫掠顺序评估，取预测时间最短者；因此结果不劣于 routes='rule'。
  访问顺序：所有停靠站都在一条直线上，从起点出发访问全部点的最短开放路径就是“先到较近
  一端再扫到另一端”（slot_plan.order_stations），所以跨槽排序（GTSP）在这里退化为闭式解，
  联合问题的难点只剩槽线选择与窗口划分。

下界（lower_bound）
  三个松弛，各自对任何合规方案都成立：
    - 每条槽单独规划（忽略其他槽的间距约束），横向段长 L 至少占 ceil(L / w) 个窗口；
      连续轨迹的时间模型下，同一次搜索同时最小化“小段数 × 4 t_point + 长度 / v_feed”；
    - 每条槽的 u 投影至少覆盖两端线盒之间的区间，停靠站数 ≥ 这些区间并集的最少窗口覆盖数；
    - 横移距离 ≥ 覆盖上述区间的最左、最右站之间的一维最短扫掠距离。
  任何方法的预测时间 T 与下界之比给出“离最优还差多少”的上界；T 等于下界时即为证明最优。

经典基线（lee_route）
  Lee 迷宫布线（Lee 1961）加转弯代价：状态 (x, y, 方向) 的逐格 Dijkstra。它不能表达
  “整段”约束（弯曲半径对两侧直段长度的要求、横槽最大长度），用来（a）在关闭这些约束时
  校验 OrientedAStar 的单槽最优性，（b）统计逐格布线违反开槽规范的比例。
"""
import heapq
import itertools
import math
import time

import numpy as np

import slot_plan as sp
import wall_rules as wr

EPS_RULE = 1e-3          # 秒 / 规范代价单位：规范代价 J 只在预测时间相同的路线之间排序


# ---------------------------------------------------------------------------
# 窗口格与路线的窗口占用
# ---------------------------------------------------------------------------
def window_cells(robot: sp.RobotModel):
    return int(round(robot.window_width / sp.RES))


def path_cells(slot):
    """槽线拐点（米，格心）→ 栅格拐点."""
    return [(int(round(u / sp.RES - 0.5)), int(round(v / sp.RES - 0.5))) for u, v in slot.cut_path]


def windows_used(corners, phi, wc):
    """给定相位 φ 的等宽窗口格，一条路线（栅格拐点）占用的窗口下标集合.

    竖直段在列 c → 窗口 ⌊(c - φ)/wc⌋；横向段 a→b 占用列 min..max-1 的窗口
    （终点恰在边界上时不进入右侧窗口，与 slot_plan 的半开窗口一致）。
    """
    win = set()
    for (x0, y0), (x1, y1) in zip(corners[:-1], corners[1:]):
        if x0 == x1:
            win.add((x0 - phi) // wc)
        else:
            lo, hi = sorted((x0, x1))
            win.update(range((lo - phi) // wc, (hi - 1 - phi) // wc + 1))
    if len(corners) == 1:
        win.add((corners[0][0] - phi) // wc)
    return win


def subsegments(corners, phi, wc):
    """给定窗口格时路线被截成的直线小段数（机械臂点数 = 4 × 此数）."""
    n = 0
    for (x0, y0), (x1, y1) in zip(corners[:-1], corners[1:]):
        if x0 == x1:
            n += 1
        else:
            lo, hi = sorted((x0, x1))
            n += (hi - 1 - phi) // wc - (lo - phi) // wc + 1
    return n


def time_params(scene, tm: sp.TimeModel):
    """开槽规范参数 → 以秒计的搜索参数.

    每段 4 t_point，每格槽线 RES / v_feed（逐点执行时为 0），规范代价 J × EPS_RULE 作次序。
    """
    p = scene.params()
    q = dict(p)
    cell = sp.RES * tm.t_per_metre
    q['turn_penalty'] = 4 * tm.t_point + EPS_RULE * p['turn_penalty']
    q['weight_x'] = cell + EPS_RULE * p.get('weight_x', 1.0)
    q['weight_y'] = cell + EPS_RULE * p.get('weight_y', 1.0)
    q['cost_weight'] = EPS_RULE * p.get('cost_weight', 0.0)
    return q


def lattice_costs(width, phi, wc, opened, tm: sp.TimeModel, robot: sp.RobotModel):
    """相位 φ 的窗口格 → OrientedAStar 列代价（秒）.

    边界列 b（b ≡ φ mod wc）属于右侧窗口。
      向右从 c 走到 c+1：若 c+1 是边界列，进入窗口 ⌊(c+1-φ)/wc⌋；
      向左从 c 走到 c-1：若 c 是边界列，进入窗口 ⌊(c-1-φ)/wc⌋；
      横向段内部穿过边界列：多截出一个小段（4 t_point）。
    新窗口的代价 = t_settle + w / v（一站的固定开销 + 横移一个窗宽）；opened 中的窗口免计。
    """
    s_open = tm.t_settle + robot.window_width / tm.v_strafe
    cols = np.arange(width)
    boundary = (cols - phi) % wc == 0
    win = (cols - phi) // wc
    new = np.array([w not in opened for w in win])
    pos = np.zeros(width)
    neg = np.zeros(width)
    pos[:-1] = np.where(boundary[1:] & new[1:], s_open, 0.0)
    neg[1:] = np.where(boundary[1:] & new[:-1], s_open, 0.0)
    through = np.where(boundary, 4 * tm.t_point, 0.0)
    return pos, neg, through


# ---------------------------------------------------------------------------
# 精确评估：停靠站 DP + 一维最优扫掠顺序
# ---------------------------------------------------------------------------
def evaluate(slots, wall, robot, tm, start_u, partition='dp', order='sweep',
             retract_between_turns=True):
    return sp.assemble_plan(slots, wall, robot, partition=partition, order=order,
                            start_u=start_u, tm=tm, retract_between_turns=retract_between_turns)


def _t(plan):
    if plan is None or plan.status != "success":
        return math.inf
    return plan.prediction.get("t_total", math.inf)


# ---------------------------------------------------------------------------
# 联合规划
# ---------------------------------------------------------------------------
def plan_slot_on_lattice(scene, k, others, robot, planner, tm, phi, share=True,
                         restrict_to_reach=True):
    """固定其他槽，在相位 φ 的窗口格上为第 k 条槽求“预测时间”最短的合规路线."""
    wc = window_cells(robot)
    start, goal, _, _ = sp.scene_tasks(scene)[k]
    grid = sp.route_grid(scene, k, others, robot, restrict_to_reach)
    opened = set()
    if share:
        for s_ in others:
            if s_.cut_path:
                opened |= windows_used(path_cells(s_), phi, wc)
    pos, neg, through = lattice_costs(grid.shape[1], phi, wc, opened, tm, robot)
    res = planner.plan(grid, start, goal, step_cost_pos=pos, step_cost_neg=neg,
                       through_cost_x=through, **time_params(scene, tm))
    return sp.make_slot(scene, k, res)


def _sequential(scene, order, robot, planner, tm, phi, share, restrict_to_reach):
    """按 order 逐条规划；返回 (槽线列表或 None, 失败的槽号或 None, 搜索次数)."""
    slots = [None] * len(order)
    n = 0
    for k in order:
        others = [s for s in slots if s is not None]
        s = plan_slot_on_lattice(scene, k, others, robot, planner, tm, phi, share,
                                 restrict_to_reach)
        n += 1
        if s.status != 'success':
            return None, k, n
        slots[k] = s
    return slots, None, n


def sequential_with_retry(scene, robot, planner, tm, phi, share=True, restrict_to_reach=True,
                          first=None, exhaustive=False):
    """逐条规划；某条槽被先规划的槽堵死时把它提到最前重排（rip-up and reroute 的简化版），
    最多尝试 K + 1 种顺序；exhaustive=True 时再穷举其余排列（K ≤ 4 时至多 24 种）。
    first：优先尝试的顺序。返回 (槽线列表或 None, 成功的顺序或 None, 搜索次数)."""
    K = len(sp.scene_tasks(scene))
    order, tried, n = list(first or range(K)), set(), 0
    while tuple(order) not in tried and len(tried) <= K:
        tried.add(tuple(order))
        slots, failed, m = _sequential(scene, order, robot, planner, tm, phi, share,
                                       restrict_to_reach)
        n += m
        if slots is not None:
            return slots, tuple(order), n
        if order[0] == failed:                 # 单独规划也不可行
            break
        order = [failed] + [k for k in order if k != failed]
    if exhaustive:
        for order in itertools.permutations(range(K)):
            if order in tried:
                continue
            slots, _, m = _sequential(scene, order, robot, planner, tm, phi, share,
                                      restrict_to_reach)
            n += m
            if slots is not None:
                return slots, order, n
    return None, None, n


def joint_routes(scene, wall, robot, planner, tm, start_u, phases=None, passes=2, share=True,
                 restrict_to_reach=True, include_rule=True, stats=None):
    """返回 (槽线列表, 精确评估后的 SlotPlan)；没有任何可行候选时返回 (None, None).

    phases：窗口格相位（栅格列，默认 0..wc-1）；passes：坐标下降轮数；share：是否把其他槽
    已打开的窗口算作免费（关闭即“各槽独立的站感知选线”消融）；include_rule：是否把规则
    路线也作为候选（保证结果不劣于 routes='rule'）。
    """
    wc = window_cells(robot)
    K = len(sp.scene_tasks(scene))
    phases = range(wc) if phases is None else phases
    best_slots, best_plan = None, None
    st = stats if stats is not None else {}
    st.setdefault('evaluations', 0)
    st.setdefault('searches', 0)

    def consider(slots, tag):
        nonlocal best_slots, best_plan
        plan = evaluate(slots, wall, robot, tm, start_u)
        st['evaluations'] += 1
        if _t(plan) < _t(best_plan) - 1e-9:
            best_slots, best_plan = slots, plan
            st['source'] = tag
        return plan

    if include_rule:
        rule = sp.plan_routes(scene, robot, planner, restrict_to_reach)
        st['searches'] += K
        if all(s.status == 'success' for s in rule):
            consider(rule, 'rule')
    good_order, exhaustive = None, True
    for phi in phases:
        # 第一个相位穷举槽顺序以找到可行顺序；之后的相位先试这个顺序
        slots, order, n = sequential_with_retry(scene, robot, planner, tm, phi, share,
                                                restrict_to_reach, good_order, exhaustive)
        exhaustive = False
        st['searches'] += n
        good_order = order or good_order
        if slots is None:
            continue
        cur = consider(slots, f'phi={phi}')
        t_cur = _t(cur)
        for _ in range(passes):
            improved = False
            for k in range(K):
                others = [s for j, s in enumerate(slots) if j != k]
                s = plan_slot_on_lattice(scene, k, others, robot, planner, tm, phi, share,
                                         restrict_to_reach)
                st['searches'] += 1
                if s.status != 'success' or s.cut_path == slots[k].cut_path:
                    continue
                trial = slots[:k] + [s] + slots[k + 1:]
                plan = consider(trial, f'phi={phi}+cd')
                if _t(plan) < t_cur - 1e-9:
                    slots, t_cur, improved = trial, _t(plan), True
            if not improved:
                break
    return best_slots, best_plan


def plan_task_joint(scene, wall, robot, planner, tm=None, start_u=None, restrict_to_reach=True,
                    retract_between_turns=True, partition='dp', order='sweep', **kw):
    """与 slot_plan.plan_task 相同的输出（SlotPlan），槽线由 joint_routes 选取."""
    tm = tm or sp.TimeModel()
    if start_u is None:
        tasks = sp.scene_tasks(scene)
        start_u = min(min(s[0], g[0]) for s, g, _, _ in tasks) * sp.RES + sp.RES / 2
    slots, _ = joint_routes(scene, wall, robot, planner, tm, start_u,
                            restrict_to_reach=restrict_to_reach, **kw)
    if slots is None:                      # 没有可行候选：退回规则路线以给出失败原因
        slots = sp.plan_routes(scene, robot, planner, restrict_to_reach)
    plan = evaluate(slots, wall, robot, tm, start_u, partition, order, retract_between_turns)
    plan.routes = 'joint'
    return plan


# ---------------------------------------------------------------------------
# 下界
# ---------------------------------------------------------------------------
def _cover_count(intervals, w, tol=sp.REACH_TOL):
    """覆盖若干区间所需的最少宽 w 窗口数（从左到右贪心，经典最优）.

    区间 (a, b)：a < b 时需覆盖 [a, b)（右端点处只有横向段的末端）；a == b 时为一个点
    （竖直槽）。窗口 [lo, lo + w] 按机械臂行程取闭区间（容差 tol，与 partition_dp 一致）。
    """
    n, covered_to = 0, -math.inf
    for a, b in sorted(intervals):
        if a == b:
            if a > covered_to + tol:
                n, covered_to = n + 1, a + w
            continue
        if b <= covered_to + tol:
            continue
        x = max(a, covered_to)
        k = math.ceil((b - x) / w - 1e-9)
        n, covered_to = n + k, x + k * w
    return n


def lower_bound(scene, robot, planner, tm, start_u, restrict_to_reach=True):
    """预测执行时间的下界（秒）与各项分量；任一槽单独都不可行时返回 None.

    机械臂项：对每条槽单独求 min(小段数 × 4 t_point + 长度 / v_feed)。小段数与长度由同一条
    路线同时取到，所以它不小于“最少小段数 × 4 t_point + 最短长度 / v_feed”，仍是任何合规
    方案的下界。逐点执行的时间模型（v_feed = inf）下长度项为零，退化为最少小段数。
    """
    wc = window_cells(robot)
    w = robot.window_width
    p4 = 4 * tm.t_point
    cell = sp.RES * tm.t_per_metre
    run = np.array([p4 * (math.ceil(k / wc) - 1) if k > 0 else 0.0 for k in range(2000)])
    seg_lb, t_arm = 0, 0.0
    for k, (start, goal, _, _) in enumerate(sp.scene_tasks(scene)):
        grid = sp.route_grid(scene, k, [], robot, restrict_to_reach)
        q = time_params(scene, tm)
        q.update(turn_penalty=p4, weight_x=1e-9, weight_y=1e-9, cost_weight=0.0)
        res = planner.plan(grid, start, goal, run_cost_x=run, **q)
        if not res['ok']:
            return None
        seg_lb += round((res['cost'] + p4) / p4)          # 小段数（去掉 1e-9 的长度权）
        if cell > 0.0:
            q.update(weight_x=cell, weight_y=cell)
            res = planner.plan(grid, start, goal, run_cost_x=run, **q)
            t_arm += res['cost'] + p4
    if cell <= 0.0:
        t_arm = p4 * seg_lb
    us = [((min(s[0], g[0]) + 0.5) * sp.RES, (max(s[0], g[0]) + 0.5) * sp.RES)
          for s, g, _, _ in sp.scene_tasks(scene)]
    n_st = _cover_count(us, w)
    off = robot.y_range[1] - robot.margin                 # 站中心 = 窗口左端 + off
    lo, hi = min(a for a, _ in us), max(b for _, b in us)
    # 最左站中心 ≤ lo + off，最右站中心 ≥ hi - w + off（见 slot_plan.place_centres）
    travel = sp.sweep_travel(lo + off, hi - w + off, start_u)
    t = n_st * tm.t_settle + travel / tm.v_strafe + t_arm
    return dict(t_total=t, stations=n_st, points=4 * seg_lb, travel_m=travel)


# ---------------------------------------------------------------------------
# 经典基线：Lee 迷宫布线 + 转弯代价
# ---------------------------------------------------------------------------
def lee_route(grid, start, goal, turn_penalty=0.0, weight_x=1.0, weight_y=1.0, cost_weight=0.0,
              allow_x=True, obstacle_threshold=253, **_):
    """逐格 Dijkstra，状态 (x, y, 方向)；代价 = Σ(轴权重 + cost_weight·格代价/252) + 转弯代价.

    返回 dict(ok, corners, cost, turns, time_ms)。与 OrientedAStar 关闭弯曲半径和
    横槽长度约束时的目标函数完全相同。
    """
    t0 = time.perf_counter()
    H, W = grid.shape
    blocked = grid >= obstacle_threshold
    cellc = np.minimum(grid.astype(float), 252.0) / 252.0 * cost_weight
    dx, dy = (1, 0, -1, 0), (0, 1, 0, -1)
    wt = (weight_x, weight_y)
    INF = math.inf
    dist = {}
    parent = {}
    pq = []
    sx, sy = start
    for d in range(4):
        if d % 2 == 0 and not allow_x:
            continue
        nx, ny = sx + dx[d], sy + dy[d]
        if 0 <= nx < W and 0 <= ny < H and not blocked[ny, nx]:
            c = wt[d % 2] + cellc[ny, nx]
            key = (nx, ny, d)
            if c < dist.get(key, INF):
                dist[key] = c
                parent[key] = None
                heapq.heappush(pq, (c, nx, ny, d))
    end = None
    while pq:
        c, x, y, d = heapq.heappop(pq)
        if c > dist.get((x, y, d), INF):
            continue
        if (x, y) == tuple(goal):
            end = (x, y, d)
            break
        for nd in (d, (d + 1) % 4, (d + 3) % 4):
            if nd % 2 == 0 and not allow_x:
                continue
            nx, ny = x + dx[nd], y + dy[nd]
            if not (0 <= nx < W and 0 <= ny < H) or blocked[ny, nx]:
                continue
            nc = c + wt[nd % 2] + cellc[ny, nx] + (turn_penalty if nd != d else 0.0)
            key = (nx, ny, nd)
            if nc < dist.get(key, INF) - 1e-12:
                dist[key] = nc
                parent[key] = (x, y, d)
                heapq.heappush(pq, (nc, nx, ny, nd))
    ms = (time.perf_counter() - t0) * 1000
    if end is None:
        return dict(ok=tuple(start) == tuple(goal), corners=[tuple(start)], cost=0.0, turns=0,
                    time_ms=ms)
    cells = []
    k = end
    while k is not None:
        cells.append((k[0], k[1]))
        k = parent[k]
    cells.append(tuple(start))
    cells.reverse()
    corners = [cells[0]] + [cells[i] for i in range(1, len(cells) - 1)
                            if wr.heading(cells[i - 1], cells[i]) !=
                            wr.heading(cells[i], cells[i + 1])] + [cells[-1]]
    return dict(ok=True, corners=corners, cost=dist[end], turns=len(corners) - 2, time_ms=ms)


# ---------------------------------------------------------------------------
# 合规检查（与规划器无关的独立实现）
# ---------------------------------------------------------------------------
def plan_violations(scene, slots, robot, restrict_to_reach=True):
    """整组槽线的违规计数：单槽规范（wall_rules.violations）、高度带、槽间间距、他槽线盒."""
    v = dict(rule=0, out_of_band=0, spacing=0, box=0)
    lo, hi = (math.ceil(robot.reach_band[0] / sp.RES), math.floor(robot.reach_band[1] / sp.RES))
    cells = [wr.densify(path_cells(s)) if s.cut_path else [] for s in slots]
    for k, (s, c) in enumerate(zip(slots, cells)):
        if not c:
            continue
        v['rule'] += sum(scene.violations(c, s.circuit).values())
        if restrict_to_reach:
            v['out_of_band'] += sum(1 for _, y in c if not lo <= y <= hi)
        for j, boxes in enumerate(scene.task_boxes):
            if j == k:
                continue
            for b in boxes:
                g = b.grown(wr.BOX_MARGIN)
                v['box'] += sum(1 for x, y in c if g.x <= x < g.x + g.w and g.y <= y < g.y + g.h)
    for j, k in itertools.combinations(range(len(slots)), 2):
        if not cells[j] or not cells[k]:
            continue
        m = wr.SLOT_SPACING if slots[j].circuit == slots[k].circuit else wr.WEAK_CURRENT_SPACING
        for o in sp._slot_obstacles(slots[j]):
            g = o.grown(m)
            if any(g.x <= x < g.x + g.w and g.y <= y < g.y + g.h for x, y in cells[k]):
                v['spacing'] += 1
    return v


def standalone_routable(planner, robot, restrict_to_reach=True):
    """wall_gen.residential_wall 的 routable 回调：第 k 条槽单独规划是否可行."""
    def check(scene, k):
        start, goal, _, _ = sp.scene_tasks(scene)[k]
        grid = sp.route_grid(scene, k, [], robot, restrict_to_reach)
        return planner.plan(grid, start, goal, **scene.params())['ok']
    return check
