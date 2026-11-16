# -*- coding: utf-8 -*-
"""开槽任务规划：墙面障碍 → 正交槽线 → 底盘停靠站 → 机械臂切割点.

切割只沿正交折线进行：机械臂为直角坐标解耦结构（joint1 = Z、joint2 = Y、joint3 = X 切深、
joint4 = 刀片方向），每一刀只动一个轴，刀片只取 0°（竖切）/ 90°（横切）。线管弯曲半径
（GB 50303-2015 12.2.1）只作为规划约束（两弯间直线段 ≥ 2r、拐角内侧圆弧无障碍），
保证管子能敷进直角槽并在拐角处弯过去；圆弧本身不切，仅用于显示。

坐标系
  墙面系 (u, v)：u 沿墙水平向右（面对墙面时），v 为离地高度，单位米。
  map 系：墙由 left_end / right_end 两端点给出（面对墙面时的左、右端）。
  机械臂点 (x, y, z, rotation)：与 scr_arm_controller 的 cartesian_cutting /
  navigation_cutting_task 任务 YAML 中的 points 完全相同（相对 base_link，rotation 为度）。

机器人参数全部来自现有代码 / URDF（见 RobotModel 注释），tests/test_slot_plan.py
会解析 cartesian_cutting.py 校验它们没有漂移。
"""
import math
from dataclasses import dataclass, field

import numpy as np

import wall_rules as wr

RES = 0.01  # 规划栅格分辨率 1 cm
# 窗口边界容差：站 k 负责 u ∈ [lo - EPS, hi - EPS)，落在边界上的竖切只归右侧站
EPS = 1e-7


# ---------------------------------------------------------------------------
# 机器人模型
# ---------------------------------------------------------------------------
@dataclass
class RobotModel:
    # cartesian_cutting.py：初始位置（所有关节 = 0）与关节限位
    x0: float = 0.53677
    y0: float = -0.00015
    z0: float = 0.50679
    j1: tuple = (-0.35, 0.2)      # Z：z = z0 + j1
    j2: tuple = (-0.08, 0.06)     # Y：y = y0 - j2
    j3: tuple = (-0.045, 0.02)    # X：x = x0 - j3
    j4: tuple = (-1.57, 1.57)     # 刀片旋转
    # visual_servo_ros.tool_front_x：刀具最前端 = link4 原点 x + 0.1594
    tool_offset: float = 0.1594
    # URDF：轮轴在 base_link 下 0.052484 m，轮半径 0.05 m → base_link 离地 0.1025 m
    base_height: float = 0.1025
    # task_at_target.yaml：对位距离（base_link 中心到墙面）
    standoff: float = 0.70
    # 现有切割示例中的退刀位置 x = 0.520（刀尖距墙约 2 cm）
    x_retract: float = 0.520
    # 对位容差 1 cm（task_at_target.yaml position_tolerance）：工作窗口每侧留出
    margin: float = 0.01

    @property
    def x_range(self):
        return (self.x0 - self.j3[1], self.x0 - self.j3[0])

    @property
    def y_range(self):
        return (self.y0 - self.j2[1], self.y0 - self.j2[0])

    @property
    def z_range(self):
        return (self.z0 + self.j1[0], self.z0 + self.j1[1])

    @property
    def window_width(self):
        """一个停靠站可切的墙面宽度（扣除两侧对位余量）."""
        lo, hi = self.y_range
        return (hi - lo) - 2 * self.margin

    @property
    def reach_band(self):
        """机械臂可达的离地高度范围（扣除余量）."""
        lo, hi = self.z_range
        return (lo + self.base_height + self.margin, hi + self.base_height - self.margin)

    def x_cut(self, depth):
        return self.standoff + depth - self.tool_offset

    def reachable(self, x, y, z, rotation_deg, tol=1e-3):
        """与 CartesianCuttingController.inverse_kinematics 相同的判定."""
        j = (z - self.z0, -(y - self.y0), -(x - self.x0), math.radians(rotation_deg))
        return all(lo - tol <= v <= hi + tol for v, (lo, hi) in
                   zip(j, (self.j1, self.j2, self.j3, self.j4)))


# ---------------------------------------------------------------------------
# 墙面在 map 中的位姿
# ---------------------------------------------------------------------------
@dataclass
class WallFrame:
    left_end: tuple       # map (x, y)，面对墙面时的左端
    right_end: tuple
    frame_id: str = 'map'

    def __post_init__(self):
        p0, p1 = np.array(self.left_end, float), np.array(self.right_end, float)
        self.length = float(np.linalg.norm(p1 - p0))
        self.t = (p1 - p0) / self.length            # 沿墙向右
        self.n = np.array([self.t[1], -self.t[0]])  # 指向房间（机器人一侧）
        self.p0 = p0

    def to_map(self, u, depth_from_wall=0.0):
        return self.p0 + u * self.t + depth_from_wall * self.n

    def station_pose(self, u_c, standoff):
        """机器人正对墙面、base_link 中心离墙 standoff 时的 map 位姿 (x, y, yaw)."""
        p = self.to_map(u_c, standoff)
        yaw = math.atan2(-self.n[1], -self.n[0])
        return float(p[0]), float(p[1]), yaw

    def arm_point(self, u, v, u_c, robot: RobotModel):
        """墙面点 → 停靠站 u_c 处的机械臂 (y, z)：base 左方 = -t，故 y = -(u - u_c)."""
        return -(u - u_c), v - robot.base_height


# ---------------------------------------------------------------------------
# 槽线几何
# ---------------------------------------------------------------------------
def corners_to_m(corners):
    return [((x + 0.5) * RES, (y + 0.5) * RES) for x, y in corners]


def conduit_polyline(corners_m, r, chord_deg=15.0):
    """正交拐点 → 管线中心线：每个拐角换成半径 r 的四分之一圆弧（以弦逼近）."""
    pts = [np.array(p, float) for p in corners_m]
    if len(pts) < 3 or r <= 0:
        return [tuple(p) for p in pts]
    out = [pts[0]]
    n = max(1, int(round(90.0 / chord_deg)))
    for i in range(1, len(pts) - 1):
        a, c, b = pts[i - 1], pts[i], pts[i + 1]
        u = (c - a) / np.linalg.norm(c - a)
        w = (b - c) / np.linalg.norm(b - c)
        p0, p1 = c - r * u, c + r * w
        ctr = p0 + r * w
        a0 = math.atan2(*(p0 - ctr)[::-1])
        da = math.atan2(*(p1 - ctr)[::-1]) - a0
        da = (da + math.pi) % (2 * math.pi) - math.pi
        out += [ctr + r * np.array([math.cos(a0 + da * k / n), math.sin(a0 + da * k / n)])
                for k in range(n + 1)]
    out.append(pts[-1])
    dedup = [out[0]]
    for p in out[1:]:
        if np.linalg.norm(p - dedup[-1]) > 1e-9:
            dedup.append(p)
    return [tuple(map(float, p)) for p in dedup]


def polyline_length(pts):
    return float(sum(math.dist(a, b) for a, b in zip(pts[:-1], pts[1:])))


def clip_polyline(pts, lo, hi, axis):
    """保留 pts 中 lo <= p[axis] <= hi 的部分，返回若干连续子折线."""
    pieces, cur = [], []

    def inside(p):
        return lo - 1e-9 <= p[axis] <= hi + 1e-9

    def cut(a, b, bound):
        t = (bound - a[axis]) / (b[axis] - a[axis])
        return tuple(a[k] + t * (b[k] - a[k]) for k in range(2))

    for a, b in zip(pts[:-1], pts[1:]):
        ia, ib = inside(a), inside(b)
        if ia and not cur:
            cur = [a]
        if ia and ib:
            cur.append(b)
        elif ia and not ib:
            cur.append(cut(a, b, hi if b[axis] > hi else lo))
            pieces.append(cur)
            cur = []
        elif not ia and ib:
            cur = [cut(a, b, lo if a[axis] < lo else hi), b]
        elif a[axis] < lo < b[axis] or b[axis] < lo < a[axis] or \
                a[axis] < hi < b[axis] or b[axis] < hi < a[axis]:
            # 两端都在外但穿过区间
            e1 = cut(a, b, lo if a[axis] < lo else hi)
            e2 = cut(a, b, hi if a[axis] < lo else lo)
            if math.dist(e1, e2) > 1e-9:
                pieces.append([e1, e2])
    if len(cur) >= 2:
        pieces.append(cur)
    return [p for p in pieces if polyline_length(p) > 1e-6]


# ---------------------------------------------------------------------------
# 停靠站划分
# ---------------------------------------------------------------------------
def cover_stations(intervals, width):
    """一维定长窗口覆盖（窗口为半开区间 [lo, lo + width)）.

    intervals：槽线各段在 u 轴上的投影 [(s, e)]，竖直段为退化区间 s == e。
    贪心：从最左未覆盖处放窗口，连续部分紧接上一个窗口。对“用最少定长区间
    覆盖直线上的点集 / 区间并集”这是经典最优算法。
    """
    windows = []
    last = -math.inf
    for s, e in sorted(intervals):
        if s == e:
            if s >= last - EPS:
                windows.append((s, s + width))
                last = s + width
            continue
        pos = s if s >= last else last
        while e > last - EPS:
            windows.append((pos, pos + width))
            last = pos + width
            pos = last
    return windows


@dataclass
class Station:
    index: int
    u_lo: float
    u_hi: float
    u_c: float
    pose: tuple                 # map (x, y, yaw)
    pieces: list = field(default_factory=list)   # 墙面系子折线
    points: list = field(default_factory=list)   # 机械臂点 dict(x, y, z, rotation)
    cut_length: float = 0.0


def blade_angle_deg(dy, dz):
    """切割方向 (dy, dz) → joint4 角度（度）.

    URDF 中 joint4 轴为 -x(base)；rotation = 0 时刀片沿 z（竖切），90 沿 y（横切），
    与现有 wall_cutting.yaml 一致。刀片对称，角度折到 (-90, 90]。
    """
    a = math.degrees(math.atan2(dy, dz))
    while a > 90.0 + 1e-9:
        a -= 180.0
    while a <= -90.0 + 1e-9:
        a += 180.0
    return round(a, 3)


def station_points(st: Station, wall: WallFrame, robot: RobotModel, depth,
                   retract_between_turns=True):
    """把一个停靠站内的各段子折线转成机械臂点序列（退刀 → 下刀 → 沿线切 → 退刀）."""
    xc, xr = robot.x_cut(depth), robot.x_retract
    pts = []

    def P(x, y, z, rot):
        pts.append(dict(x=round(x, 5), y=round(y, 5), z=round(z, 5), rotation=rot))

    for piece in st.pieces:
        arm = [wall.arm_point(u, v, st.u_c, robot) for u, v in piece]
        # 合并同方向的连续小段
        segs = []
        for a, b in zip(arm[:-1], arm[1:]):
            ang = blade_angle_deg(b[0] - a[0], b[1] - a[1])
            if segs and abs(segs[-1][2] - ang) < 1e-6:
                segs[-1][1] = b
            else:
                segs.append([a, b, ang])
        (y, z), rot = segs[0][0], segs[0][2]
        P(xr, y, z, rot)
        P(xc, y, z, rot)
        for k, (a, b, ang) in enumerate(segs):
            if k > 0 and ang != rot:
                if retract_between_turns:
                    P(xr, a[0], a[1], rot)
                    P(xr, a[0], a[1], ang)
                    P(xc, a[0], a[1], ang)
                else:
                    P(xc, a[0], a[1], ang)
                rot = ang
            P(xc, b[0], b[1], rot)
        P(xr, segs[-1][1][0], segs[-1][1][1], rot)
    st.points = pts
    st.cut_length = sum(polyline_length(p) for p in st.pieces)
    return pts


# ---------------------------------------------------------------------------
# 总流程
# ---------------------------------------------------------------------------
@dataclass
class SlotPlan:
    status: str
    corners_m: list = field(default_factory=list)
    # 切割路径：正交折线（直角坐标解耦机械臂每一刀只动一个轴，刀片 0° / 90°）
    cut_path: list = field(default_factory=list)
    # 仅供显示：线管中心线（拐角处按弯曲半径画圆弧，说明管子在直角槽内如何弯）
    conduit: list = field(default_factory=list)
    stations: list = field(default_factory=list)
    unreachable: list = field(default_factory=list)   # 机械臂够不到的子折线
    slot_depth: float = wr.SLOT_DEPTH / 100.0
    slot_width: float = wr.SLOT_WIDTH / 100.0
    planner: dict = field(default_factory=dict)

    @property
    def length(self):
        return polyline_length(self.cut_path) if self.cut_path else 0.0

    @property
    def unreachable_length(self):
        return sum(polyline_length(p) for p in self.unreachable)


def scene_grid_with_band(scene, band):
    """在开槽规范栅格上再把机械臂够不到的高度封掉."""
    g = scene.grid()
    lo, hi = int(math.ceil(band[0] / RES)), int(math.floor(band[1] / RES))
    g[:max(0, lo), :] = wr.BLOCKED
    g[hi + 1:, :] = wr.BLOCKED
    return g


def plan_task(scene, wall: WallFrame, robot: RobotModel, planner,
              restrict_to_reach=True, retract_between_turns=True):
    """scene: wall_rules.WallScene（厘米栅格），planner: scr_planner_py.OrientedAStar."""
    band = robot.reach_band
    grid = scene_grid_with_band(scene, band) if restrict_to_reach else scene.grid()
    res = planner.plan(grid, scene.start, scene.goal, **scene.params())
    plan = SlotPlan(status=res['status'],
                    planner=dict(turns=res['turns'], cost=res['cost'],
                                 expansions=res['expansions'], time_ms=res['time_ms']))
    if not res['ok']:
        return plan
    plan.corners_m = corners_to_m(res['corners'])
    plan.cut_path = [tuple(p) for p in plan.corners_m]
    plan.conduit = conduit_polyline(plan.corners_m, wr.BEND_RADIUS / 100.0)

    reach = clip_polyline(plan.cut_path, band[0], band[1], axis=1)
    plan.unreachable = _complement(plan.cut_path, band)
    intervals = [(min(a[0], b[0]), max(a[0], b[0]))
                 for piece in reach for a, b in zip(piece[:-1], piece[1:])]
    for k, (lo, hi) in enumerate(cover_stations(intervals, robot.window_width)):
        # 窗口 [lo, hi) 对应机械臂 y ∈ (y_max - margin - width, y_max - margin]
        u_c = lo + (robot.y_range[1] - robot.margin)
        st = Station(k, lo, hi, u_c, wall.station_pose(u_c, robot.standoff))
        st.pieces = [q for piece in reach for q in clip_polyline(piece, lo - EPS, hi - EPS, axis=0)]
        if st.pieces:
            station_points(st, wall, robot, plan.slot_depth, retract_between_turns)
            plan.stations.append(st)
    for k, st in enumerate(plan.stations):
        st.index = k
    return plan


def _densify(pts, step):
    out = [pts[0]]
    for a, b in zip(pts[:-1], pts[1:]):
        n = max(1, int(math.ceil(math.dist(a, b) / step)))
        out += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
                for k in range(1, n + 1)]
    return out


def _complement(pts, band):
    below = clip_polyline(pts, -1e9, band[0], axis=1)
    above = clip_polyline(pts, band[1], 1e9, axis=1)
    return [p for p in below + above if polyline_length(p) > 1e-6]


# ---------------------------------------------------------------------------
# 配置读取 / 结果导出
# ---------------------------------------------------------------------------
def scene_from_config(cfg):
    """墙面任务配置（米）→ (WallScene（厘米栅格）, WallFrame).

    cfg 示例见 config/wall_task_room3_south.yaml。
    """
    w = cfg['wall']
    frame = WallFrame(tuple(w['left_end']), tuple(w['right_end']), w.get('frame_id', 'map'))
    cm = lambda m: int(round(m * 100))  # noqa: E731
    obstacles = [wr.Rect(cm(o['u']), cm(o['v']), max(1, cm(o['w'])), max(1, cm(o['h'])),
                         o['kind'], o.get('label', o['kind']))
                 for o in cfg.get('obstacles', [])]
    t = cfg['task']
    scene = wr.WallScene(cm(frame.length), cm(w['height']), bool(w.get('load_bearing', False)),
                         obstacles, start=(cm(t['start'][0]), cm(t['start'][1])),
                         goal=(cm(t['goal'][0]), cm(t['goal'][1])),
                         start_label=t.get('start_label', '起点'),
                         goal_label=t.get('goal_label', '终点'))
    return scene, frame


def plan_to_dict(plan: SlotPlan, wall: WallFrame, robot: RobotModel, duration=2.0):
    """导出为 YAML 友好的 dict；每个 station 的 points 与 navigation_cutting_task 格式相同."""
    r3 = lambda v: round(float(v), 4)  # noqa: E731
    return dict(
        status=plan.status,
        frame_id=wall.frame_id,
        wall=dict(left_end=[r3(c) for c in wall.p0], right_end=[r3(c) for c in wall.to_map(wall.length)],
                  length=r3(wall.length)),
        slot=dict(depth=r3(plan.slot_depth), width=r3(plan.slot_width),
                  bend_radius=r3(wr.BEND_RADIUS / 100.0), length=r3(plan.length),
                  unreachable_length=r3(plan.unreachable_length),
                  corners=[[r3(u), r3(v)] for u, v in plan.corners_m],
                  cut_path=[[r3(u), r3(v)] for u, v in plan.cut_path],
                  conduit_display=[[r3(u), r3(v)] for u, v in plan.conduit]),
        planner=plan.planner,
        robot=dict(standoff=robot.standoff, reach_band=[r3(b) for b in robot.reach_band],
                   window_width=r3(robot.window_width), x_cut=r3(robot.x_cut(plan.slot_depth)),
                   x_retract=robot.x_retract),
        stations=[dict(index=s.index, u_c=r3(s.u_c), window=[r3(s.u_lo), r3(s.u_hi)],
                       pose=dict(x=r3(s.pose[0]), y=r3(s.pose[1]), yaw=r3(s.pose[2])),
                       cut_length=r3(s.cut_length),
                       points=s.points, motion=dict(duration=duration, return_home=False))
                  for s in plan.stations])
