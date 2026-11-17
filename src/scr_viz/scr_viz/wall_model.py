# -*- coding: utf-8 -*-
"""
墙面坐标系、墙面任务 / 规划结果 YAML 的读取（纯 Python，不依赖 ROS）.

墙面坐标与 scr_planner 一致（见 scr_planner/python/slot_plan.py 的 WallFrame）：

- u：沿墙水平向右（面对墙面时），从 ``left_end`` 起算，米；
- v：离地高度，米；
- n：墙面法向，指向房间（机器人一侧）；
- map 点 = left_end + u * t + off * n，z = v。

规范边距与 scr_planner/python/wall_rules.py 保持一致（那边单位是厘米）。
单元测试会在源码树可用时核对两边的数值。
"""
import math
from dataclasses import dataclass, field

import yaml

# ---------------------------------------------------------------------------
# 规范边距（米），与 wall_rules.py 中的常量一一对应
# ---------------------------------------------------------------------------
OPENING_MARGIN = 0.20       # 门窗洞口两侧
BEAM_MARGIN = 0.50          # 梁 / 柱及其左右
WEAK_CURRENT_SPACING = 0.30  # 强弱电平行间距
BOX_MARGIN = 0.05           # 与其他线盒的净距
EDGE_MARGIN = 0.10          # 与墙边 / 阴阳角的净距
SLOT_SPACING = 0.05         # 两条同类槽之间的净距

KIND_MARGIN = {
    'door': OPENING_MARGIN, 'window': OPENING_MARGIN, 'beam': BEAM_MARGIN,
    'column': BEAM_MARGIN, 'box': BOX_MARGIN, 'weak_line': WEAK_CURRENT_SPACING,
    'pier': 0.0, 'slot': SLOT_SPACING,
}

# 机器人默认参数（与 slot_plan.RobotModel 一致；规划结果 YAML 的 robot 字段优先）
BASE_HEIGHT = 0.1025        # base_link 离地高度
TOOL_OFFSET = 0.1594        # 刀尖 = link4 原点沿 base_link x 前伸
STANDOFF = 0.70             # 对位时 base_link 中心到墙面
REACH_BAND = (0.2693, 0.7993)
# base_link 碰撞盒（URDF）：0.7467 × 0.5662，中心 x 偏 -0.0003
FOOTPRINT = (0.7467, 0.5662)


def margin_of(kind):
    """障碍物种类 → 规范边距（米）；未知种类返回 0."""
    return KIND_MARGIN.get(kind, 0.0)


# ---------------------------------------------------------------------------
# 墙面坐标系
# ---------------------------------------------------------------------------
@dataclass
class Wall:
    """一面墙在 map 中的位姿与尺寸."""

    left_end: tuple
    right_end: tuple
    height: float = 2.0
    frame_id: str = 'map'
    load_bearing: bool = False

    def __post_init__(self):
        x0, y0 = (float(c) for c in self.left_end)
        x1, y1 = (float(c) for c in self.right_end)
        self.length = math.hypot(x1 - x0, y1 - y0)
        if self.length <= 0.0:
            raise ValueError('墙面两端点重合')
        self.t = ((x1 - x0) / self.length, (y1 - y0) / self.length)
        self.n = (self.t[1], -self.t[0])
        self.p0 = (x0, y0)

    @property
    def yaw(self):
        """墙面 u 轴（t 方向）在 map 中的朝向角."""
        return math.atan2(self.t[1], self.t[0])

    @property
    def facing_yaw(self):
        """正对墙面（朝 -n）的朝向角：停靠站 base_link 的 yaw."""
        return math.atan2(-self.n[1], -self.n[0])

    def to_map(self, u, v=0.0, off=0.0):
        """墙面点 (u, v) 沿法向偏移 off（> 0 进入房间）→ map (x, y, z)."""
        return (self.p0[0] + u * self.t[0] + off * self.n[0],
                self.p0[1] + u * self.t[1] + off * self.n[1],
                float(v))

    def from_map(self, x, y, z=0.0):
        """把 map 点转成 (u, v, off)：off > 0 在房间一侧，< 0 进入墙体."""
        dx, dy = x - self.p0[0], y - self.p0[1]
        return (dx * self.t[0] + dy * self.t[1], float(z), dx * self.n[0] + dy * self.n[1])


# ---------------------------------------------------------------------------
# 坐标变换：停靠站 / 机械臂点 / 刀尖
# ---------------------------------------------------------------------------
def base_to_map(pose, x, y, z=0.0):
    """base_link 中的点 → map；pose = (x, y, yaw)，z 原样返回（相对 base_link）."""
    px, py, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return (px + c * x - s * y, py + s * x + c * y, z)


def arm_point_to_tip_map(pose, point, tool_offset=TOOL_OFFSET, base_height=BASE_HEIGHT):
    """
    规划结果里的机械臂点（link4 原点，base_link 坐标）→ 刀尖在 map 中的位置.

    刀尖 = link4 原点沿 base_link x 方向前伸 tool_offset；z 加上 base_link 离地高度。
    """
    x, y, z = base_to_map(pose, float(point['x']) + tool_offset, float(point['y']),
                          float(point['z']))
    return (x, y, z + base_height)


def tip_from_link_poses(base_xy, base_yaw, link4_xyz, tool_offset=TOOL_OFFSET):
    """Gazebo link 真值 → 刀尖 map 位置（与 execute_slot_plan 的 _links_cb 一致）."""
    hx, hy = math.cos(base_yaw), math.sin(base_yaw)
    return (link4_xyz[0] + tool_offset * hx, link4_xyz[1] + tool_offset * hy, link4_xyz[2])


def yaw_from_quaternion(x, y, z, w):
    """四元数 → 绕 z 轴的 yaw."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw):
    """绕 z 轴 yaw → 四元数 (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_from_rpy(roll, pitch, yaw):
    """欧拉角（ZYX 顺序：先 yaw 再 pitch 再 roll）→ 四元数 (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


# ---------------------------------------------------------------------------
# YAML 读取
# ---------------------------------------------------------------------------
@dataclass
class Obstacle:
    """墙面已有物体（u, v 为左下角，w, h 为宽高，米）."""

    kind: str
    u: float
    v: float
    w: float
    h: float
    label: str = ''

    def grown(self, m):
        """各边外扩 m 后的 (u, v, w, h)."""
        return (self.u - m, self.v - m, self.w + 2 * m, self.h + 2 * m)


@dataclass
class TaskEndpoint:
    """开槽任务的线盒（插座 / 开关等）中心点."""

    u: float
    v: float
    label: str
    slot_index: int
    role: str          # start / goal


@dataclass
class Station:
    """一个停靠站（按执行顺序）."""

    index: int
    u_c: float
    window: tuple
    pose: tuple        # (x, y, yaw)
    pieces: list = field(default_factory=list)
    points: list = field(default_factory=list)
    cut_length: float = 0.0


@dataclass
class Slot:
    """一条槽."""

    label: str
    status: str
    cut_path: list
    conduit: list
    length: float = 0.0


@dataclass
class Scene:
    """可视化需要的全部内容：墙面 + 任务 + （可选）规划结果."""

    wall: Wall
    obstacles: list = field(default_factory=list)
    endpoints: list = field(default_factory=list)
    slots: list = field(default_factory=list)
    stations: list = field(default_factory=list)
    unreachable: list = field(default_factory=list)
    slot_width: float = 0.035
    slot_depth: float = 0.035
    reach_band: tuple = REACH_BAND
    standoff: float = STANDOFF
    tool_offset: float = TOOL_OFFSET
    base_height: float = BASE_HEIGHT
    start_u: float = None
    status: str = ''
    prediction: dict = field(default_factory=dict)

    def start_pose(self):
        """执行起点（定位标记正前方）的 base_link 位姿；无规划结果时为 None."""
        if self.start_u is None:
            return None
        x, y, _ = self.wall.to_map(self.start_u, 0.0, self.standoff)
        return (x, y, self.wall.facing_yaw)


def _pairs(pts):
    return [(float(a), float(b)) for a, b in pts]


def obstacles_from(lst):
    """YAML obstacles 列表 → [Obstacle]."""
    return [Obstacle(o['kind'], float(o['u']), float(o['v']), float(o['w']), float(o['h']),
                     o.get('label', o['kind'])) for o in (lst or [])]


def endpoints_from(tasks):
    """YAML tasks 列表 → 每条槽的起点 / 终点线盒."""
    out = []
    for k, t in enumerate(tasks or []):
        out.append(TaskEndpoint(float(t['start'][0]), float(t['start'][1]),
                                t.get('start_label', '起点'), k, 'start'))
        out.append(TaskEndpoint(float(t['goal'][0]), float(t['goal'][1]),
                                t.get('goal_label', '终点'), k, 'goal'))
    return out


def scene_from_dicts(task=None, plan=None):
    """
    墙面任务 dict 和 / 或规划结果 dict → Scene.

    两者都给时：墙高、任务线盒取自任务；槽线、停靠站、机器人参数取自规划结果；
    障碍物优先取任务（规划结果里的障碍物是任务的副本，取整到厘米）。
    """
    if task is None and plan is None:
        raise ValueError('至少需要墙面任务或规划结果之一')
    src = task if task is not None else plan
    w = src['wall']
    height = float((task or {}).get('wall', {}).get('height', 2.0))
    frame = (task or {}).get('wall', {}).get('frame_id') or (plan or {}).get('frame_id') or 'map'
    wall = Wall(tuple(w['left_end']), tuple(w['right_end']), height, frame,
                bool(w.get('load_bearing') or False))
    scene = Scene(wall)
    obs = (task or {}).get('obstacles')
    if obs is None and plan is not None:
        obs = plan.get('obstacles')
    scene.obstacles = obstacles_from(obs)
    if task is not None:
        scene.endpoints = endpoints_from(task.get('tasks') or
                                         ([task['task']] if 'task' in task else []))
    if plan is not None:
        scene.status = plan.get('status', '')
        spec = plan.get('slot_spec', {})
        scene.slot_width = float(spec.get('width', scene.slot_width))
        scene.slot_depth = float(spec.get('depth', scene.slot_depth))
        rb = plan.get('robot', {})
        scene.reach_band = tuple(float(b) for b in rb.get('reach_band', REACH_BAND))
        scene.standoff = float(rb.get('standoff', STANDOFF))
        scene.tool_offset = float(rb.get('tool_offset', TOOL_OFFSET))
        scene.start_u = plan.get('start_u', plan.get('marker_u'))
        scene.prediction = plan.get('prediction') or {}
        scene.slots = [Slot(s.get('label', ''), s.get('status', ''), _pairs(s.get('cut_path', [])),
                            _pairs(s.get('conduit_display', [])), float(s.get('length', 0.0)))
                       for s in plan.get('slots', [])]
        scene.unreachable = [_pairs(p) for p in plan.get('unreachable', [])]
        for s in plan.get('stations', []):
            p = s['pose']
            scene.stations.append(Station(
                int(s['index']), float(s['u_c']), tuple(float(c) for c in s['window']),
                (float(p['x']), float(p['y']), float(p['yaw'])),
                [_pairs(q) for q in s.get('pieces', [])], list(s.get('points', [])),
                float(s.get('cut_length', 0.0))))
    return scene


def load_yaml(path):
    """读 UTF-8 YAML 文件；path 为空返回 None."""
    if not path:
        return None
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def load_scene(task_file=None, plan_file=None):
    """从文件读 Scene（任一可为空）."""
    return scene_from_dicts(load_yaml(task_file), load_yaml(plan_file))
