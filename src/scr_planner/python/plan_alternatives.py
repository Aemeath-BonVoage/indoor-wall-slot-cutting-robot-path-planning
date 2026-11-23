# -*- coding: utf-8 -*-
"""一面墙没有合规方案时的诊断与替代方案.

联合规划（task_plan.joint_routes）在部分多槽墙面上找不到“全部槽都合规”的方案：每条槽
单独可行，但可切高度带只有 53 cm，几条槽互相挡住。这里不放松任何开槽规范，而是改任务
本身，按“对施工改动由小到大”给出三级替代方案：

  refeed    换引线点：新点位改从同墙另一个同类已有点位引线，或两条槽互换起点。
            墙外不增加材料，改变的只是回路的接线拓扑。                          [假设] 回路容量允许
  floor /   墙外引线：这条槽不再沿墙横走，两端线盒各开一段竖槽到地面（或顶棚），线管在
  ceiling   地面垫层（或吊顶 / 楼板）内连通。墙上只剩竖槽——承重墙与非承重墙都允许
            （GB 50203-2011 3.0.11 只限制水平沟槽）。增加：墙外线管长度；高度带以外的
            竖槽机械臂够不到，需人工开槽。                                      [行业] 点位之间走地 / 走顶
  relocate  移动线盒：新点位沿墙平移不超过 RELOCATE_MAX。改变的是设计位置。     [假设] 容许范围

每个方案都重新过一遍合规检查（task_plan.plan_violations 与本模块的 stub_violations，均与
规划器无关），并带有一个可比较的代价（installer_cost，单位：折算的人工分钟；各项单价是
[假设]，见常量注释）。排序先按级别（改动大小），再按改动的槽数，最后按代价。

筛选阶段只用规则路线、穷举槽的规划顺序判断可行性（routable），比联合规划快得多但不完备：
被判为不可行的候选可能其实有解。最终方案再用联合规划求预测时间。
"""
import copy
import itertools
import math
from dataclasses import dataclass, field

import slot_plan as sp
import task_plan as tp
import wall_gen as wg
import wall_rules as wr

# -- 代价单价（全部为 [假设]，只用于在替代方案之间排序） -----------------------
COST_MANUAL_PER_M = 10.0     # 人工开槽（手持开槽机 + 清槽）每米折算分钟
COST_CONDUIT_PER_M = 3.0     # 墙外（垫层 / 吊顶内）敷管每米折算分钟
COST_MOVE_PER_CM = 1.0       # 线盒每偏离设计位置 1 cm 折算分钟
COST_REFEED = 2.0            # 每改一条回路的引线点（核对接线）折算分钟
RELOCATE_MAX = 20            # [假设] 线盒沿墙平移的容许范围（厘米）
RELOCATE_STEP = 5
MIN_ROBOT_STUB = 10          # 引上 / 引下竖槽在高度带内不足此长度（厘米）时整段人工开
TIERS = ('refeed', 'floor', 'ceiling', 'relocate')
FEED_LABELS = {'已有插座': 'strong', '已有弱电口': 'weak'}


@dataclass
class Change:
    slot: int                  # 原任务序号
    kind: str                  # refeed / floor / ceiling / relocate
    label: str = ''
    moved_cm: float = 0.0      # 线盒移动距离
    conduit_m: float = 0.0     # 墙外线管长度
    manual_m: float = 0.0      # 需人工开的竖槽长度
    robot_m: float = 0.0       # 仍由机器人开的竖槽长度
    detail: dict = field(default_factory=dict)


@dataclass
class Alternative:
    changes: list
    scene: object              # 改动后的墙面（任务列表可能与原来不同）
    stubs: list = field(default_factory=list)    # 墙外引线的整段竖槽 [(x, y0, y1, circuit, slot)]
    slots: list = None         # 筛选阶段的规则路线（含固定竖槽）
    fixed: list = field(default_factory=list)    # 引线竖槽在高度带内、由机器人开的部分
    check_scene: object = None                   # 含固定竖槽任务的墙面（合规检查用）
    plan: object = None        # finalize 后的 SlotPlan

    @property
    def tier(self):
        return max((TIERS.index(c.kind) for c in self.changes), default=-1)

    @property
    def installer_cost(self):
        return sum(COST_MANUAL_PER_M * c.manual_m + COST_CONDUIT_PER_M * c.conduit_m +
                   COST_MOVE_PER_CM * c.moved_cm + (COST_REFEED if c.kind == 'refeed' else 0.0)
                   for c in self.changes)

    @property
    def key(self):
        return (self.tier, len(self.changes), round(self.installer_cost, 6))

    def summary(self):
        return dict(tier=TIERS[self.tier] if self.changes else 'none',
                    changed_slots=sorted({c.slot for c in self.changes}),
                    moved_cm=sum(c.moved_cm for c in self.changes),
                    conduit_m=round(sum(c.conduit_m for c in self.changes), 3),
                    manual_m=round(sum(c.manual_m for c in self.changes), 3),
                    robot_stub_m=round(sum(c.robot_m for c in self.changes), 3),
                    installer_cost=round(self.installer_cost, 2),
                    changes=[dict(slot=c.slot, kind=c.kind, label=c.label, **c.detail)
                             for c in self.changes])


# ---------------------------------------------------------------------------
# 可行性筛选：规则路线 + 槽顺序的深度优先穷举
# ---------------------------------------------------------------------------
def routable(scene, robot, planner, restrict_to_reach=True, stats=None):
    """存在某个规划顺序使全部槽的规则路线都合规时返回槽线列表，否则 None.

    深度优先枚举顺序前缀：前缀失败时它的所有延伸都被剪掉，K = 4 时最多 64 次搜索。
    """
    tasks = sp.scene_tasks(scene)
    K = len(tasks)
    if K == 0:
        return []
    params = scene.params()

    def dfs(done, slots):
        if len(done) == K:
            return slots
        for k in range(K):
            if k in done:
                continue
            grid = sp.route_grid(scene, k, [s for s in slots if s is not None], robot,
                                 restrict_to_reach)
            res = planner.plan(grid, tasks[k][0], tasks[k][1], **params)
            if stats is not None:
                stats['searches'] = stats.get('searches', 0) + 1
            if not res['ok']:
                continue
            nxt = list(slots)
            nxt[k] = sp.make_slot(scene, k, res)
            out = dfs(done | {k}, nxt)
            if out is not None:
                return out
        return None

    return dfs(frozenset(), [None] * K)


def subscene(scene, keep):
    """只保留 keep 中的任务（其余任务的线盒留作障碍）."""
    s = copy.copy(scene)
    s.tasks = [scene.tasks[k] for k in keep]
    s.circuits = [scene.circuit_of(k) for k in keep]
    s.task_boxes = [scene.task_boxes[k] for k in keep]
    s.obstacles = list(scene.obstacles) + [b for k in range(len(scene.tasks)) if k not in keep
                                           for b in scene.task_boxes[k]]
    if s.tasks:
        s.start, s.goal, s.start_label, s.goal_label = s.tasks[0]
    return s


# ---------------------------------------------------------------------------
# 诊断：为什么没有合规方案
# ---------------------------------------------------------------------------
RELAXATIONS = (('band', None), ('WEAK_CURRENT_SPACING', wr.SLOT_SPACING), ('H_RUN_MAX', 10000),
               ('BEND_RADIUS', 0), ('SLOT_SPACING', 0))


def diagnose(scene, robot, planner):
    """无解墙面的原因归类（只做诊断：放松规则得到的方案不作为输出）.

    min_remove / blockers：最少拿掉几条槽、拿掉哪几条，其余槽即可全部合规（在 routable
        的意义下）——说明冲突集中在哪条槽上；
    conflict：最小冲突集里出现的回路组合（strong-strong / strong-weak / weak-weak）；
    relax：逐项单独放松后变为可行的规则：band（不限可切高度带）、WEAK_CURRENT_SPACING
        （强弱电间距 30 → 5 cm）、H_RUN_MAX（不限横槽长度）、BEND_RADIUS（不要求弯曲半径）、
        SLOT_SPACING（同类槽间距 5 → 0 cm）；
    cause：按上面的结果归成一类（见 _cause）。
    """
    K = len(sp.scene_tasks(scene))
    out = dict(slots=K, feasible=routable(scene, robot, planner) is not None)
    if out['feasible']:
        return out
    best = None
    for m in range(1, K):
        for rem in itertools.combinations(range(K), m):
            keep = [k for k in range(K) if k not in rem]
            if routable(subscene(scene, keep), robot, planner) is not None:
                best = rem
                break
        if best is not None:
            break
    out.update(min_remove=len(best) if best is not None else K,
               blockers=list(best) if best is not None else list(range(K)))
    # 最小冲突集：含 blockers[0] 的最小不可行子集
    core = list(range(K))
    for m in range(2, K + 1):
        hit = next((c for c in itertools.combinations(range(K), m)
                    if routable(subscene(scene, list(c)), robot, planner) is None), None)
        if hit is not None:
            core = list(hit)
            break
    circ = sorted({scene.circuit_of(k) for k in core})
    out.update(core=core, conflict='-'.join(circ if len(circ) > 1 else circ * 2))
    relax = []
    for name, value in RELAXATIONS:
        if name == 'band':
            ok = routable(scene, robot, planner, restrict_to_reach=False) is not None
        else:
            old = getattr(wr, name)
            setattr(wr, name, value)
            try:
                ok = routable(scene, robot, planner) is not None
            finally:
                setattr(wr, name, old)
        if ok:
            relax.append(name)
    out.update(relax=relax, cause=_cause(relax))
    return out


def _cause(relax):
    """归类：哪条规则是约束性的（按“放松它就可行”判断；多条同时成立时取列表中靠前的）."""
    r = set(relax)
    if 'WEAK_CURRENT_SPACING' in r:
        return 'strong_weak_spacing'       # 强弱电 30 cm 间距在 53 cm 高度带里放不下
    if r & {'H_RUN_MAX', 'BEND_RADIUS'}:
        return 'staircase_in_band'         # 横槽 ≤ 50 cm + 弯曲半径迫使的阶梯走法互相挡住
    if 'SLOT_SPACING' in r:
        return 'slot_spacing'
    if 'band' in r:
        return 'band_only'                 # 只有放开高度带（走到带外）才有解
    return 'unresolved'                    # 任何单项放松都不够


# ---------------------------------------------------------------------------
# 候选替代方案的构造
# ---------------------------------------------------------------------------
def unused_feeds(scene):
    """墙上未被任务使用的已有带内点位 [(cx, cy, circuit, label, Rect)]（按标签识别）."""
    out = []
    for o in scene.obstacles:
        if o.kind == 'box' and o.label in FEED_LABELS and o.w == wg.BOX and o.h == wg.BOX:
            out.append((o.x + wg.BOX // 2, o.y + wg.BOX // 2, FEED_LABELS[o.label], o.label, o))
    return out


def _clone(scene):
    s = copy.copy(scene)
    s.obstacles = list(scene.obstacles)
    s.tasks = list(scene.tasks)
    s.circuits = [scene.circuit_of(k) for k in range(len(scene.tasks))]
    s.task_boxes = list(scene.task_boxes)
    return s


def _finish(s):
    if s.tasks:
        s.start, s.goal, s.start_label, s.goal_label = s.tasks[0]
    return s


def refeed_candidates(scene):
    """单条槽改接到未用的同类已有点位，或两条同类槽互换起点."""
    out = []
    K = len(scene.tasks)
    for k in range(K):
        (start, goal, _, gl), circ = scene.tasks[k], scene.circuit_of(k)
        for cx, cy, c, label, rect in unused_feeds(scene):
            if c != circ:
                continue
            s = _clone(scene)
            s.obstacles = [o for o in s.obstacles if o is not rect]
            s.obstacles.append(scene.task_boxes[k][0])          # 原起点线盒变成普通障碍
            s.tasks[k] = ((cx, cy), goal, label, gl)
            s.task_boxes[k] = (rect, scene.task_boxes[k][1])
            ch = Change(k, 'refeed', f'{gl} 改由 u = {cx} cm 处的{label}引线',
                        detail=dict(old_start=list(start), new_start=[cx, cy]))
            out.append(Alternative([ch], _finish(s)))
    for j, k in itertools.combinations(range(K), 2):
        if scene.circuit_of(j) != scene.circuit_of(k):
            continue
        s = _clone(scene)
        (sj, gj, slj, glj), (sk, gk, slk, glk) = scene.tasks[j], scene.tasks[k]
        s.tasks[j], s.tasks[k] = (sk, gj, slk, glj), (sj, gk, slj, glk)
        s.task_boxes[j] = (scene.task_boxes[k][0], scene.task_boxes[j][1])
        s.task_boxes[k] = (scene.task_boxes[j][0], scene.task_boxes[k][1])
        out.append(Alternative(
            [Change(j, 'refeed', f'{glj} 与 {glk} 互换引线点',
                    detail=dict(old_start=list(sj), new_start=list(sk))),
             Change(k, 'refeed', f'{glk} 与 {glj} 互换引线点',
                    detail=dict(old_start=list(sk), new_start=list(sj)))], _finish(s)))
    return out


def relocate_candidates(scene):
    """单条槽的新点位沿墙平移 ±RELOCATE_STEP … ±RELOCATE_MAX."""
    out = []
    K = len(scene.tasks)
    ends = [(p[0], p[1], scene.circuit_of(j), j) for j in range(K) for p in scene.tasks[j][:2]]
    for k in range(K):
        (start, goal, sl, gl), circ = scene.tasks[k], scene.circuit_of(k)
        probe = _clone(scene)
        probe.obstacles += [b for j in range(K) if j != k for b in scene.task_boxes[j]]
        for d in range(RELOCATE_STEP, RELOCATE_MAX + 1, RELOCATE_STEP):
            for sgn in (-1, 1):
                gx = goal[0] + sgn * d
                if not wg._free_box(probe, circ, gx, goal[1], []):
                    continue
                if any(abs(gx - ex) < (wg.WEAK_SEP if ec != circ else 25) and
                       abs(goal[1] - ey) < (wg.WEAK_SEP if ec != circ else 25)
                       for ex, ey, ec, j in ends if j != k):
                    continue
                if goal[1] == start[1] and abs(gx - start[0]) < 2 * wr.BEND_RADIUS + 10:
                    continue
                s = _clone(scene)
                s.tasks[k] = (start, (gx, goal[1]), sl, gl)
                s.task_boxes[k] = (scene.task_boxes[k][0], wg.box_at(gx, goal[1], gl))
                out.append(Alternative([Change(k, 'relocate', f'{gl} 沿墙平移 {sgn * d:+d} cm',
                                               moved_cm=d, detail=dict(old_goal=list(goal),
                                                                       new_goal=[gx, goal[1]]))],
                                       _finish(s)))
    return out


def _stub_span(scene, y, side):
    """线盒（中心高 y）到地面 / 顶棚的竖槽占的行 [y0, y1)（不含线盒本身）."""
    half = wg.BOX // 2
    return (0, y - half) if side == 'floor' else (y + half + 1, scene.height)


def _column_clear(scene, x, y0, y1, circuit, skip=()):
    """列 x、行 [y0, y1) 的竖槽与 skip 以外的全部障碍保持规定净距."""
    if not wr.EDGE_MARGIN <= x < scene.width - wr.EDGE_MARGIN:
        return False
    for o in scene.obstacles:
        if any(o is b for b in skip):
            continue
        g = o.grown(scene.margin_of(o, circuit))
        if g.x <= x < g.x + g.w and g.y < y1 and y0 < g.y + g.h:
            return False
    return True


def offwall_candidates(scene, robot, side, sizes=None):
    """把槽的子集 S 改为墙外引线（side = floor / ceiling）；每个子集一个候选."""
    K = len(scene.tasks)
    lo = math.ceil(robot.reach_band[0] / sp.RES)
    hi = math.floor(robot.reach_band[1] / sp.RES)
    out = []
    for m in (sizes or range(1, K + 1)):
        for S in itertools.combinations(range(K), m):
            alt = _offwall(scene, S, side, lo, hi)
            if alt is not None:
                out.append(alt)
    return out


def _offwall(scene, S, side, lo, hi):
    """S 中的槽改走墙外.

    规划用的墙面（Alternative.scene）里只剩其余的槽，引线竖槽整段作为障碍；竖槽落在高度带
    内的部分是固定的竖直槽（Alternative.fixed），不经过搜索，规划完成后并入方案一起划分
    停靠站。check_scene 把这些固定槽也列为任务，供独立检查器使用。
    """
    K = len(scene.tasks)
    keep = [k for k in range(K) if k not in S]
    s = _clone(scene)
    s.tasks = [scene.tasks[k] for k in keep]
    s.circuits = [scene.circuit_of(k) for k in keep]
    s.task_boxes = [scene.task_boxes[k] for k in keep]
    full = _clone(s)
    changes, stubs, fixed = [], [], []
    probe = _clone(scene)
    probe.obstacles += [b for k in range(K) for b in scene.task_boxes[k]]
    for k in S:
        (start, goal, sl, gl), circ = scene.tasks[k], scene.circuit_of(k)
        ch = Change(k, side, f'{sl}→{gl} 改走{"地面" if side == "floor" else "顶棚"}',
                    conduit_m=abs(goal[0] - start[0]) / 100.0, detail=dict(stubs=[]))
        for (x, y), box, lab in ((start, scene.task_boxes[k][0], sl),
                                 (goal, scene.task_boxes[k][1], gl)):
            st = box_stub(probe, x, y, box, lab, circ, side, lo, hi, skip=scene.task_boxes[k])
            if st is None:
                return None
            stubs.append(st['span'] + (circ, k))
            s.obstacles += [box, st['whole']]
            if st['slot'] is not None:
                fixed.append(st['slot'])
                full.tasks.append(st['task'])
                full.circuits.append(circ)
                full.task_boxes.append((box, st['rest']))
            else:
                full.obstacles += [box, st['whole']]
            ch.robot_m += st['robot_m']
            ch.manual_m += st['manual_m']
            ch.detail['stubs'].append(dict(u=x, v0=st['span'][1], v1=st['span'][2]))
        changes.append(ch)
    # 引线竖槽之间也要保持间距
    for (xa, a0, a1, ca, ka), (xb, b0, b1, cb, kb) in itertools.combinations(stubs, 2):
        gap = wr.SLOT_SPACING if ca == cb else wr.WEAK_CURRENT_SPACING
        if abs(xa - xb) <= gap and a0 < b1 + gap and b0 < a1 + gap:
            return None
    return Alternative(changes, _finish(s), stubs, fixed=fixed, check_scene=_finish(full))


def box_stub(probe, x, y, box, label, circuit, side, lo, hi, skip=()):
    """线盒（中心 x, y）到地面 / 顶棚的引线竖槽；与障碍净距不足时返回 None.

    probe：含全部线盒障碍的墙面；skip：不参与净距检查的线盒（本槽自己的两端）。
    返回 dict：span = (x, y0, y1) 整段竖槽；whole = 整段的障碍矩形；高度带内不短于
    MIN_ROBOT_STUB 的部分由机器人开：slot（固定竖直槽）、task、rest（其余人工段的矩形），
    否则 slot 为 None、整段人工；robot_m / manual_m 为两部分的长度（米）。
    """
    line = 'strong_line' if circuit == 'strong' else 'weak_line'
    y0, y1 = _stub_span(probe, y, side)
    if not _column_clear(probe, x, y0, y1, circuit, skip=skip):
        return None
    out = dict(span=(x, y0, y1), whole=wr.Rect(x, y0, 1, y1 - y0, line, f'{label}引线竖槽'),
               slot=None, task=None, rest=None, robot_m=0.0, manual_m=(y1 - y0) / 100.0)
    r0, r1 = (lo, y) if side == 'floor' else (y, hi)
    if r1 - r0 >= MIN_ROBOT_STUB:
        m0, m1 = (y0, lo) if side == 'floor' else (hi + 1, y1)
        port = (x, lo) if side == 'floor' else (x, hi)
        task = (port, (x, y), '地面引上' if side == 'floor' else '顶棚引下', label)
        out.update(slot=_fixed_slot(task, circuit), task=task,
                   rest=wr.Rect(x, m0, 1, m1 - m0, line, f'{label}引线竖槽'),
                   robot_m=(r1 - r0) / 100.0, manual_m=(m1 - m0) / 100.0)
    return out


def _fixed_slot(task, circuit):
    (port, goal, sl, gl) = task
    slot = sp.Slot(f'{sl}→{gl}', 'success', circuit=circuit,
                   planner=dict(turns=0, cost=0.0, expansions=0, time_ms=0.0, fixed=True))
    slot.corners_m = sp.corners_to_m([port, goal])
    slot.cut_path = [tuple(p) for p in slot.corners_m]
    slot.conduit = list(slot.cut_path)
    return slot


# ---------------------------------------------------------------------------
# 合规检查（独立于规划器）
# ---------------------------------------------------------------------------
def stub_violations(alt, original, slots):
    """墙外引线竖槽的违规计数：与障碍的净距、与各条槽线的间距、竖槽之间的间距."""
    v = dict(stub_obstacle=0, stub_slot=0, stub_stub=0)
    K = len(original.tasks)
    boxes = [(k, b) for k in range(K) for b in original.task_boxes[k]]
    # 固定竖槽就是引线竖槽自己在高度带内的一段，不与竖槽比较
    cells = [(s.circuit, wr.densify(tp.path_cells(s))) for s in slots
             if s.cut_path and not s.planner.get('fixed')]
    for x, y0, y1, circ, k in alt.stubs:
        for o in list(original.obstacles) + [b for j, b in boxes if j != k]:
            m = wr.BOX_MARGIN if o.kind == 'box' else original.margin_of(o, circ)
            g = o.grown(m)
            if g.x <= x < g.x + g.w and g.y < y1 and y0 < g.y + g.h:
                v['stub_obstacle'] += 1
        for c, cs in cells:
            gap = wr.SLOT_SPACING if c == circ else wr.WEAK_CURRENT_SPACING
            if any(abs(cx - x) <= gap and y0 - gap <= cy < y1 + gap for cx, cy in cs):
                v['stub_slot'] += 1
    for (xa, a0, a1, ca, ka), (xb, b0, b1, cb, kb) in itertools.combinations(alt.stubs, 2):
        gap = wr.SLOT_SPACING if ca == cb else wr.WEAK_CURRENT_SPACING
        if abs(xa - xb) <= gap and a0 < b1 + gap and b0 < a1 + gap:
            v['stub_stub'] += 1
    return v


def check(alt, original, robot, slots=None):
    """替代方案的全部违规计数（应全为 0）."""
    slots = slots if slots is not None else (alt.plan.slots if alt.plan else alt.slots)
    v = dict(tp.plan_violations(alt.check_scene or alt.scene, slots, robot))
    v.update(stub_violations(alt, original, slots))
    return v


# ---------------------------------------------------------------------------
# 总入口
# ---------------------------------------------------------------------------
def alternatives(scene, robot, planner, tiers=TIERS, stats=None, protected=()):
    """全部可行的替代方案，按 (级别, 改动槽数, 代价) 排序.

    墙外引线按子集大小递增枚举，某一大小出现可行方案后不再枚举更大的子集（改动更多、
    代价更高）。protected：不允许改动的任务序号（例如房间规划中的跨墙过线槽）。
    返回 [] 表示这几级替代方案都不可行。
    """
    out = []

    def screen(cands):
        ok = []
        for alt in cands:
            if any(c.slot in protected for c in alt.changes):
                continue
            alt.slots = routable(alt.scene, robot, planner, stats=stats)
            if alt.slots is not None:
                alt.slots = alt.slots + alt.fixed
                ok.append(alt)
        return ok

    if 'refeed' in tiers:
        out += screen(refeed_candidates(scene))
    for side in ('floor', 'ceiling'):
        if side not in tiers:
            continue
        for m in range(1, len(scene.tasks) + 1):
            ok = screen(offwall_candidates(scene, robot, side, sizes=(m,)))
            if ok:
                out += ok
                break
    if 'relocate' in tiers:
        out += screen(relocate_candidates(scene))
    return sorted(out, key=lambda a: a.key)


def finalize(alt, wall, robot, planner, tm=None, start_u=None, extra_fixed=()):
    """给选定的替代方案求最终规划：其余槽用联合规划选线，再与固定竖槽合并划分停靠站；
    与筛选阶段的规则路线比较后取预测时间短者."""
    tm = tm or sp.TimeModel()

    def cell(p):
        return (int(round(p[0] / sp.RES - 0.5)), int(round(p[1] / sp.RES - 0.5)))

    tasks = list(alt.scene.tasks) + [(cell(s.cut_path[0]), cell(s.cut_path[-1]))
                                     for s in list(alt.fixed) + list(extra_fixed)]
    if not tasks:                                   # 全部改走墙外且带内没有竖槽：机器人无任务
        alt.plan = sp.SlotPlan(status='success', slots=[], routes='joint', start_u=start_u or 0.0,
                               prediction=dict(stations=0, points=0, travel_m=0.0, t_move=0.0,
                                               t_cut=0.0, t_total=0.0))
        return alt.plan
    if start_u is None:
        start_u = min(min(t[0][0], t[1][0]) for t in tasks) * sp.RES + sp.RES / 2
    extra = list(extra_fixed)
    cands = [alt.slots + extra] if alt.slots else []
    if alt.scene.tasks:
        slots, _ = tp.joint_routes(alt.scene, wall, robot, planner, tm, start_u)
        if slots is not None:
            cands.append(list(slots) + alt.fixed + extra)
    elif not cands:
        cands.append(list(alt.fixed) + extra)
    plan = min((tp.evaluate(c, wall, robot, tm, start_u) for c in cands), key=tp._t)
    plan.routes = 'joint'
    alt.plan = plan
    return plan


def plan_with_fallback(scene, wall, robot, planner, tm=None, start_u=None, tiers=TIERS,
                       extra_fixed=(), protected=(), accept=None):
    """先联合规划；没有合规方案时返回排序第一的替代方案.

    extra_fixed：已经确定、不经搜索的固定竖槽（房间规划中跨墙走地的引上段），与规划出的
    槽线合并划分停靠站。accept(plan) -> bool：方案的附加条件（房间规划中要求每个停靠站
    机器人都站得进去）；不满足时按排序试下一个替代方案。返回 (SlotPlan 或 None,
    Alternative, 全部可行替代方案)；原任务可行时 Alternative 没有任何改动（changes 为空）。
    """
    tm = tm or sp.TimeModel()
    none = Alternative([], scene, fixed=[])
    has_tasks = bool(scene.tasks)
    if has_tasks:
        # 先用便宜的顺序穷举判断可行性；它失败时联合规划的站感知走法偶尔仍能解出
        slots = routable(scene, robot, planner)
        if slots is None:
            slots, _ = tp.joint_routes(scene, wall, robot, planner, tm,
                                       start_u if start_u is not None else 0.0)
        if slots is not None:
            none.slots = list(slots)
    else:
        none.slots = []
    if none.slots is not None:
        plan = finalize(none, wall, robot, planner, tm, start_u, extra_fixed)
        if accept is None or accept(plan):
            return plan, none, []
    alts = alternatives(scene, robot, planner, tiers, protected=protected)
    for alt in alts:
        finalize(alt, wall, robot, planner, tm, start_u, extra_fixed)
        if accept is None or accept(alt.plan):
            return alt.plan, alt, alts
    return None, none, alts
