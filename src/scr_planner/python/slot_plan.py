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
# 总流程：多条槽 → 停靠站划分 → 访问顺序 → 时间预测
# ---------------------------------------------------------------------------
@dataclass
class TimeModel:
    """执行时间模型（仿真时间，秒）：每站 t_settle + 横移距离 / v_strafe + 点数 × t_point.

    默认值为当前默认执行路径（行为树任务层；精对位为 AprilTag 标记地图 + 激光 EKF + 横移
    比例估计，横移前机械臂回到观测位姿）的标定结果，即 TIME_MODELS['align_v3']。此前执行
    路径下的标定保留为 TIME_MODELS['align_v1'] / ['align_v2']，用于复现此前的报告。标定数据与
    方法见 calibrate_time_model.py 与 docs/experiments/plan_validation。仿真或执行代码改动后
    应重标定。
    """
    t_settle: float = 4.70      # 每站：横移启停 + 到位 + 测残差 + 观测位姿 → 首个切割点
    v_strafe: float = 0.0971    # 沿墙横移的等效速度（m/s）
    t_point: float = 2.030      # 每个机械臂点（motion.duration = 2.0 s + 通信开销）

    def move(self, d):
        return self.t_settle + abs(d) / self.v_strafe

    @classmethod
    def preset(cls, name):
        """按名称取标定预设（见 TIME_MODELS）."""
        if name not in TIME_MODELS:
            raise ValueError(f'unknown time model {name!r}; choose from {sorted(TIME_MODELS)}')
        return TIME_MODELS[name]


# 时间模型标定预设（Gazebo 仿真配置 v2）
TIME_MODELS = {
    # 第一版精对位（色块 + PBVS，站间横移用激光朝向平滑），行为树改造之前的执行路径：
    # slot_execution/runs/dp、dp_repeat、per_slot、per_slot_repeat 共 54 站，整体最小二乘，
    # 每站 RMSE 0.10 s
    'align_v1': TimeModel(t_settle=4.40, v_strafe=0.0945, t_point=2.034),
    # 精对位 v2（单个 AprilTag + 激光 EKF），行为树改造之前的执行路径：
    # slot_execution/runs/fine_align_dp、fine_align_dp_repeat、fine_align_dp_nofold 共 33 站，
    # 按阶段拟合（横移阶段回归 + 固定开销均值 + 切割总时间 / 总点数），每站 RMSE 0.68 s
    'align_v2': TimeModel(t_settle=2.83, v_strafe=0.0984, t_point=2.049),
    # 当前默认执行路径（行为树 + 标记地图 + 横移比例估计 + 横移前回观测位姿）：
    # along_wall_accuracy/runs/default_s1、default_s2、scale_tags_0.6_s1、scale_tags_0.6_s2
    # 共 44 站，按阶段拟合且切割阶段用回归（method phases-reg），每站 RMSE 0.22 s，
    # 逐次运行留一交叉验证 0.24 s
    'align_v3': TimeModel(),
}


@dataclass
class Slot:
    label: str
    status: str
    corners_m: list = field(default_factory=list)
    cut_path: list = field(default_factory=list)     # 正交折线，切割只沿它进行
    conduit: list = field(default_factory=list)      # 仅供显示：线管在直角槽内按 R 弯过的中心线
    planner: dict = field(default_factory=dict)
    circuit: str = 'strong'                          # 强电 / 弱电（weak）


@dataclass
class SlotPlan:
    status: str
    slots: list = field(default_factory=list)
    stations: list = field(default_factory=list)     # 按执行顺序排列
    unreachable: list = field(default_factory=list)  # 机械臂够不到的子折线
    slot_depth: float = wr.SLOT_DEPTH / 100.0
    slot_width: float = wr.SLOT_WIDTH / 100.0
    partition: str = ''
    order: str = ''
    routes: str = 'rule'
    start_u: float = 0.0
    prediction: dict = field(default_factory=dict)

    @property
    def cut_paths(self):
        return [s.cut_path for s in self.slots if s.cut_path]

    @property
    def conduits(self):
        return [s.conduit for s in self.slots if s.conduit]

    @property
    def length(self):
        return sum(polyline_length(p) for p in self.cut_paths)

    @property
    def unreachable_length(self):
        return sum(polyline_length(p) for p in self.unreachable)


def scene_grid_with_band(scene, band, circuit='strong'):
    """在开槽规范栅格上再把机械臂够不到的高度封掉."""
    g = scene.grid(circuit=circuit)
    lo, hi = int(math.ceil(band[0] / RES)), int(math.floor(band[1] / RES))
    g[:max(0, lo), :] = wr.BLOCKED
    g[hi + 1:, :] = wr.BLOCKED
    return g


def scene_tasks(scene):
    return scene.tasks or [(scene.start, scene.goal, scene.start_label, scene.goal_label)]


def route_grid(scene, k, others, robot: RobotModel, restrict_to_reach=True):
    """第 k 条槽的规划栅格：others 中已定的槽（同类间距 wr.SLOT_SPACING，强弱电之间
    wr.WEAK_CURRENT_SPACING）与其他槽两端的线盒都作为障碍."""
    circuit = scene.circuit_of(k)
    extra = [o for s_ in others if s_.cut_path
             for o in _slot_obstacles(s_, s_.circuit == circuit)]
    extra += [b for j, boxes in enumerate(scene.task_boxes) if j != k for b in boxes]
    base = scene.obstacles
    scene.obstacles = base + extra
    try:
        return (scene_grid_with_band(scene, robot.reach_band, circuit) if restrict_to_reach
                else scene.grid(circuit=circuit))
    finally:
        scene.obstacles = base


def make_slot(scene, k, res):
    _, _, sl, gl = scene_tasks(scene)[k]
    slot = Slot(f'{sl}→{gl}', res['status'], circuit=scene.circuit_of(k),
                planner=dict(turns=res['turns'], cost=res['cost'],
                             expansions=res['expansions'], time_ms=res['time_ms']))
    if res['ok']:
        slot.corners_m = corners_to_m(res['corners'])
        slot.cut_path = [tuple(p) for p in slot.corners_m]
        slot.conduit = conduit_polyline(slot.corners_m, wr.BEND_RADIUS / 100.0)
    return slot


def plan_routes(scene, robot: RobotModel, planner, restrict_to_reach=True):
    """逐条规划槽线；已规划的槽作为后续槽的障碍（同类线管分槽，间距 wr.SLOT_SPACING）."""
    slots = []
    for k, (start, goal, _, _) in enumerate(scene_tasks(scene)):
        grid = route_grid(scene, k, slots, robot, restrict_to_reach)
        slots.append(make_slot(scene, k, planner.plan(grid, start, goal, **scene.params())))
    return slots


def _slot_obstacles(slot, same_circuit=True):
    out = []
    for (u0, v0), (u1, v1) in zip(slot.cut_path[:-1], slot.cut_path[1:]):
        x0, x1 = sorted((round(u0 / RES - 0.5), round(u1 / RES - 0.5)))
        y0, y1 = sorted((round(v0 / RES - 0.5), round(v1 / RES - 0.5)))
        out.append(wr.Rect(x0, y0, x1 - x0 + 1, y1 - y0 + 1, 'slot' if same_circuit else 'slot_x',
                           '已规划槽'))
    return out


def count_points(pieces):
    """与 station_points 相同的点数：每段子折线 退刀/下刀 2 点 + 每小段 1 点 + 每次转向 3 点 + 退刀 1 点."""
    n = 0
    for piece in pieces:
        segs = 0
        prev = None
        for a, b in zip(piece[:-1], piece[1:]):
            d = 'h' if abs(b[1] - a[1]) < 1e-12 else 'v'
            if d != prev:
                segs += 1
            prev = d
        n += 3 + segs + 3 * (segs - 1)
    return n


# 机械臂横向行程是闭区间：窗口 [a, y) 允许 y 比 a + 窗宽多出这一点，使恰在 a + 窗宽 处的
# 竖切（候选位置 v + 1e-6）与左侧的槽线归同一站，而不是为它单独多停一站
REACH_TOL = 2e-6

# 截窗口时折线端部短于此长度的小段是 EPS 偏移留下的数值残片（例如拐点恰在窗口边界上时，
# 相邻横段在右侧窗口里留下 1e-7 m 的一截），它不需要切，却会让该站多出一组退刀 / 下刀 /
# 转刀（4 个点，约 8 s）。设为 0 即恢复不剔除残片的旧行为（仅用于对比实验）。
SLIVER = 1e-6


def _trim_slivers(piece):
    while len(piece) > 2 and math.dist(piece[0], piece[1]) < SLIVER:
        piece = piece[1:]
    while len(piece) > 2 and math.dist(piece[-2], piece[-1]) < SLIVER:
        piece = piece[:-1]
    return piece


def _clip_all(pieces, lo, hi):
    out = []
    for piece in pieces:
        for q in clip_polyline(piece, lo - EPS, hi - EPS, axis=0):
            q = _trim_slivers(q) if SLIVER > 0 else q
            if polyline_length(q) > 1e-6:
                out.append(q)
    return out


def partition_greedy(pieces, robot: RobotModel):
    intervals = [(min(a[0], b[0]), max(a[0], b[0]))
                 for piece in pieces for a, b in zip(piece[:-1], piece[1:])]
    return cover_stations(intervals, robot.window_width)


def sweep_travel(r_star, l_star, start_u):
    """站中心最小值不大于 r_star、最大值不小于 l_star 时，从 start_u 出发的最短扫掠距离
    （见 place_centres）."""
    if l_star <= r_star:
        return abs(start_u - min(max(start_u, l_star), r_star))
    return min(abs(start_u - r_star), abs(start_u - l_star)) + (l_star - r_star)


def partition_dp(pieces, robot: RobotModel, tm: TimeModel, step=0.005, start_u=None):
    """动态规划划分停靠站窗口，最小化 Σ(每站固定开销 + 点数 × 每点时间) + 横移时间.

    状态 x = “u < x 的槽线已切完”。下一站从第一个未覆盖点 a 开始，结束于候选位置
    y ∈ (a, a + 窗宽]；该站代价只取决于 [a, y) 内的槽线。候选位置为 step 网格加上所有
    折线顶点的 u，保证能把窗口边界对齐到拐角。
    给出 start_u 时还计入横移：按 place_centres 摆放站位后，扫掠距离只取决于第一个
    窗口的左端（恒为 umin）和最后一个窗口的右端 x，所以在终止状态上加 sweep_travel / v
    即得到时间模型下的精确最优划分。start_u 为 None 时不计横移（旧行为）。
    """
    w = robot.window_width
    verts = sorted({round(p[0], 9) for piece in pieces for p in piece})
    if not verts:
        return []
    umin, umax = verts[0], verts[-1]
    cand = set(verts)
    n = int(math.ceil((umax + w - umin) / step)) + 1
    cand.update(round(umin + k * step, 9) for k in range(n))
    cand.update(round(v + 1e-6, 9) for v in verts)      # 恰好包住位于 v 的竖切
    cand = sorted(cand)
    intervals = sorted((min(a[0], b[0]), max(a[0], b[0]))
                       for piece in pieces for a, b in zip(piece[:-1], piece[1:]))

    def first_uncovered(x):
        """u >= x 处第一个尚未切的槽线位置（与 _clip_all 的半开边界一致）."""
        best = None
        for s0, e0 in intervals:
            if s0 == e0:                       # 竖切：恰在边界上的归下一站
                u = s0 if s0 >= x - EPS else None
            else:                              # 横切：只剩零长度端点的不算
                u = max(s0, x) if e0 > x + EPS else None
            if u is not None and (best is None or u < best):
                best = u
        return best

    INF = float('inf')
    dp = {umin: (0.0, None)}
    order = [umin]
    done_cost, done_x = INF, None
    i = 0
    while i < len(order):
        x = order[i]
        i += 1
        cost_x = dp[x][0]
        a = first_uncovered(x)
        if a is None:
            if start_u is not None:
                off = robot.y_range[1] - robot.margin
                cost_x += sweep_travel(umin + off, x - w + off, start_u) / tm.v_strafe
            if cost_x < done_cost - 1e-9:
                done_cost, done_x = cost_x, x
            continue
        for y in cand:
            if y <= a + 1e-12:
                continue
            if y > a + w + REACH_TOL:
                break
            cut = _clip_all(pieces, a, y)
            if not cut:
                continue
            c = cost_x + tm.t_settle + tm.t_point * count_points(cut)
            if y not in dp or c < dp[y][0] - 1e-9:
                if y not in dp:
                    order.append(y)
                dp[y] = (c, (x, a))
        order.sort()
    # 回溯
    wins = []
    x = done_x
    while dp[x][1] is not None:
        px, a = dp[x][1]
        wins.append((a, x))
        x = px
    return sorted(wins)


def build_stations(windows, pieces, wall, robot, depth, retract_between_turns=True):
    stations = []
    for lo, hi in windows:
        u_c = lo + (robot.y_range[1] - robot.margin)    # 窗口左端对应机械臂 y 上限
        st = Station(len(stations), lo, hi, u_c, wall.station_pose(u_c, robot.standoff))
        st.pieces = _clip_all(pieces, lo, hi)
        if st.pieces:
            station_points(st, wall, robot, depth, retract_between_turns)
            stations.append(st)
    return stations


def place_centres(stations, start_u, wall, robot, depth, retract_between_turns=True):
    """在不改变各站切割窗口的前提下移动站位，使一维扫掠的横移距离最短.

    站 i 的切割窗口为 [a_i, y_i)（y_i - a_i ≤ 窗宽 w），机械臂可达 [lo, lo + w)，只要
    lo ∈ [y_i - w, a_i] 即可，对应站中心 c_i ∈ [l_i, r_i] = [y_i - w + off, a_i + off]。
    扫掠距离 = min(|s - c_min|, |s - c_max|) + (c_max - c_min)，随 c_min 减小、c_max 增大
    单调不减，而 c_max ≥ L* = max l_i、c_min ≤ R* = min r_i。取 c_i = clamp(t, l_i, r_i)
    （t ∈ [R*, L*]）即同时达到 c_min = R*、c_max = L*；若 L* ≤ R*，全部站取同一点
    clamp(s, L*, R*)。因此结果是给定窗口下的最短横移。
    """
    if not stations:
        return
    off = robot.y_range[1] - robot.margin
    w = robot.window_width
    rng = [(min(st.u_hi - w, st.u_lo) + off, st.u_lo + off) for st in stations]
    l_star, r_star = max(a for a, _ in rng), min(b for _, b in rng)
    if l_star <= r_star:
        t = min(max(start_u, l_star), r_star)
    else:
        t = min(max(start_u, r_star), l_star)
    for st, (lo, hi) in zip(stations, rng):
        c = min(max(t, lo), hi)
        if abs(c - st.u_c) > 1e-12:
            st.u_c = c
            st.pose = wall.station_pose(c, robot.standoff)
            station_points(st, wall, robot, depth, retract_between_turns)


def order_stations(stations, start_u, method='sweep'):
    """一维上从 start_u 出发访问全部站：先到较近的一端再扫到另一端（旅行距离最短）."""
    stations = sorted(stations, key=lambda s: s.u_c)
    if method == 'sweep' and stations:
        lo, hi = stations[0].u_c, stations[-1].u_c
        if abs(start_u - hi) < abs(start_u - lo):
            stations = stations[::-1]
    return stations


def predict_time(stations, start_u, tm: TimeModel):
    travel, u = 0.0, start_u
    t_move = t_cut = 0.0
    n_pts = 0
    for st in stations:
        d = abs(st.u_c - u)
        travel += d
        t_move += tm.move(d)
        t_cut += tm.t_point * len(st.points)
        n_pts += len(st.points)
        u = st.u_c
    return dict(stations=len(stations), points=n_pts, travel_m=round(travel, 4),
                t_move=round(t_move, 1), t_cut=round(t_cut, 1), t_total=round(t_move + t_cut, 1))


def plan_task(scene, wall: WallFrame, robot: RobotModel, planner,
              restrict_to_reach=True, retract_between_turns=True,
              partition='dp', order='sweep', start_u=None, tm=None, routes='rule'):
    """完整离线规划.

    routes:    'rule'（逐条槽按开槽规范代价 J 最优，见 OrientedAStar）/
               'joint'（槽线与停靠站联合规划，最小化预测执行时间，见 task_plan.py）
    partition: 'dp'（全部槽合并后动态规划）/ 'greedy'（全部槽合并后贪心最少站）/
               'per_slot'（每条槽各自贪心划分，站不共享 —— 基线）
    order:     'sweep'（从 start_u 出发，最近端先、单向扫过）/
               'slot_by_slot'（逐条槽、每条从左到右 —— 基线）
    """
    tm = tm or TimeModel()
    if routes == 'joint':
        import task_plan
        return task_plan.plan_task_joint(scene, wall, robot, planner, tm=tm, start_u=start_u,
                                         restrict_to_reach=restrict_to_reach,
                                         retract_between_turns=retract_between_turns,
                                         partition=partition, order=order)
    if routes != 'rule':
        raise ValueError(f'unknown routes {routes!r}')
    slots = plan_routes(scene, robot, planner, restrict_to_reach)
    return assemble_plan(slots, wall, robot, partition=partition, order=order, start_u=start_u,
                         tm=tm, retract_between_turns=retract_between_turns)


def assemble_plan(slots, wall: WallFrame, robot: RobotModel, partition='dp', order='sweep',
                  start_u=None, tm=None, retract_between_turns=True):
    """给定各条槽的折线：停靠站划分 → 访问顺序 → 机械臂点 → 时间预测."""
    tm = tm or TimeModel()
    band = robot.reach_band
    ok = [s for s in slots if s.status == 'success']
    status = 'success' if len(ok) == len(slots) else \
        next(s.status for s in slots if s.status != 'success')
    plan = SlotPlan(status=status, slots=slots, partition=partition, order=order)
    if not ok:
        return plan
    reach = [clip_polyline(s.cut_path, band[0], band[1], axis=1) for s in ok]
    plan.unreachable = [q for s in ok for q in _complement(s.cut_path, band)]
    us = [p[0] for r in reach for q in r for p in q]
    plan.start_u = start_u if start_u is not None else (min(us) if us else 0.0)
    if partition == 'per_slot':
        groups = [build_stations(partition_greedy(r, robot), r, wall, robot, plan.slot_depth,
                                 retract_between_turns) for r in reach]
    else:
        pieces = [q for r in reach for q in r]
        wins = partition_dp(pieces, robot, tm, start_u=plan.start_u) if partition == 'dp' \
            else partition_greedy(pieces, robot)
        groups = [build_stations(wins, pieces, wall, robot, plan.slot_depth,
                                 retract_between_turns)]
    if order == 'slot_by_slot':
        stations = [st for g in groups for st in sorted(g, key=lambda s: s.u_c)]
    else:
        stations = [st for g in groups for st in g]
        place_centres(stations, plan.start_u, wall, robot, plan.slot_depth, retract_between_turns)
        stations = order_stations(stations, plan.start_u, 'sweep')
    for k, st in enumerate(stations):
        st.index = k
    plan.stations = stations
    plan.prediction = predict_time(stations, plan.start_u, tm)
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

    cfg 示例见 config/wall_task_room3_south.yaml。每项任务可选 circuit（strong / weak，
    强弱电槽之间保持 wr.WEAK_CURRENT_SPACING）与 start_box / goal_box（{u, v, w, h}，
    两端线盒，作为其他槽的障碍）。
    """
    w = cfg['wall']
    frame = WallFrame(tuple(w['left_end']), tuple(w['right_end']), w.get('frame_id', 'map'))
    cm = lambda m: int(round(m * 100))  # noqa: E731

    def rect(o, kind=None, label=None):
        return wr.Rect(cm(o['u']), cm(o['v']), max(1, cm(o['w'])), max(1, cm(o['h'])),
                       kind or o['kind'], label or o.get('label', o.get('kind', '')))

    obstacles = [rect(o) for o in cfg.get('obstacles', [])]
    tasks = cfg.get('tasks') or [cfg['task']]
    tl = [((cm(t['start'][0]), cm(t['start'][1])), (cm(t['goal'][0]), cm(t['goal'][1])),
           t.get('start_label', '起点'), t.get('goal_label', '终点')) for t in tasks]
    scene = wr.WallScene(cm(frame.length), cm(w['height']), bool(w.get('load_bearing', False)),
                         obstacles, start=tl[0][0], goal=tl[0][1], start_label=tl[0][2],
                         goal_label=tl[0][3], tasks=tl)
    scene.circuits = [t.get('circuit', 'strong') for t in tasks]
    if any('start_box' in t or 'goal_box' in t for t in tasks):
        scene.task_boxes = [tuple(rect(t[k], 'box', t.get(k[:-4] + '_label', ''))
                                  for k in ('start_box', 'goal_box') if k in t) for t in tasks]
    return scene, frame


def plan_to_dict(plan: SlotPlan, wall: WallFrame, robot: RobotModel, duration=2.0,
                 scene=None, marker_u=None):
    """导出为 YAML 友好的 dict（执行器只读这个文件，不再重新规划）.

    stations 按执行顺序排列；每站 points 与 navigation_cutting_task 的格式相同。
    """
    r3 = lambda v: round(float(v), 4)  # noqa: E731
    pl = lambda pts: [[r3(u), r3(v)] for u, v in pts]  # noqa: E731
    d = dict(
        status=plan.status,
        frame_id=wall.frame_id,
        wall=dict(left_end=[r3(c) for c in wall.p0],
                  right_end=[r3(c) for c in wall.to_map(wall.length)], length=r3(wall.length),
                  load_bearing=bool(scene.load_bearing) if scene else None),
        slot_spec=dict(depth=r3(plan.slot_depth), width=r3(plan.slot_width),
                       bend_radius=r3(wr.BEND_RADIUS / 100.0)),
        routes=plan.routes,
        slots=[dict(label=s.label, status=s.status, circuit=s.circuit, planner=s.planner,
                    length=r3(polyline_length(s.cut_path)) if s.cut_path else 0.0,
                    cut_path=pl(s.cut_path), conduit_display=pl(s.conduit))
               for s in plan.slots],
        length=r3(plan.length), unreachable_length=r3(plan.unreachable_length),
        unreachable=[pl(p) for p in plan.unreachable],
        partition=plan.partition, order=plan.order, start_u=r3(plan.start_u),
        prediction=plan.prediction,
        robot=dict(standoff=robot.standoff, reach_band=[r3(b) for b in robot.reach_band],
                   window_width=r3(robot.window_width), x_cut=r3(robot.x_cut(plan.slot_depth)),
                   x_retract=robot.x_retract, tool_offset=robot.tool_offset),
        stations=[dict(index=s.index, u_c=r3(s.u_c), window=[r3(s.u_lo), r3(s.u_hi)],
                       pose=dict(x=r3(s.pose[0]), y=r3(s.pose[1]), yaw=r3(s.pose[2])),
                       cut_length=r3(s.cut_length), pieces=[pl(p) for p in s.pieces],
                       points=s.points, motion=dict(duration=duration, return_home=False))
                  for s in plan.stations])
    if scene is not None:
        d['obstacles'] = [dict(kind=o.kind, label=o.label, u=o.x / 100, v=o.y / 100,
                               w=o.w / 100, h=o.h / 100) for o in scene.obstacles]
    if marker_u is not None:
        d['marker_u'] = r3(marker_u)
    return d


def plan_from_dict(d):
    """读回 plan_to_dict 的结果（执行器用）→ (SlotPlan, WallFrame, obstacles)."""
    wall = WallFrame(tuple(d['wall']['left_end']), tuple(d['wall']['right_end']),
                     d.get('frame_id', 'map'))
    plan = SlotPlan(status=d['status'], partition=d.get('partition', ''),
                    order=d.get('order', ''), routes=d.get('routes', 'rule'),
                    start_u=d.get('start_u', 0.0),
                    prediction=d.get('prediction', {}))
    plan.slot_depth = d['slot_spec']['depth']
    plan.slot_width = d['slot_spec']['width']
    plan.slots = [Slot(s['label'], s['status'], cut_path=[tuple(p) for p in s['cut_path']],
                       conduit=[tuple(p) for p in s['conduit_display']], planner=s['planner'],
                       circuit=s.get('circuit', 'strong'))
                  for s in d['slots']]
    plan.unreachable = [[tuple(p) for p in q] for q in d.get('unreachable', [])]
    for s in d['stations']:
        st = Station(s['index'], s['window'][0], s['window'][1], s['u_c'],
                     (s['pose']['x'], s['pose']['y'], s['pose']['yaw']))
        st.pieces = [[tuple(p) for p in q] for q in s['pieces']]
        st.points = s['points']
        st.cut_length = s['cut_length']
        plan.stations.append(st)
    obstacles = [wr.Rect(round(o['u'] * 100), round(o['v'] * 100), round(o['w'] * 100),
                         round(o['h'] * 100), o['kind'], o['label'])
                 for o in d.get('obstacles', [])]
    return plan, wall, obstacles
