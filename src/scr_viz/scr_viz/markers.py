# -*- coding: utf-8 -*-
"""
Scene → 与 ROS 无关的 Marker 描述（MarkerSpec）.

节点用 ros_adapter.to_marker_array 转成 visualization_msgs/MarkerArray；
离线预览（preview.py）直接画这些描述，两者看到的是同一份几何。
"""
import colorsys
import math
from dataclasses import dataclass, field

from scr_viz import wall_model as wm

# 障碍物颜色（与 scr_planner/python/slot_ros.py 一致）
KIND_RGBA = {
    'door': (0.56, 0.60, 0.69, 0.9), 'window': (0.62, 0.77, 0.91, 0.9),
    'beam': (0.5, 0.5, 0.5, 0.9), 'column': (0.5, 0.5, 0.5, 0.9),
    'box': (0.88, 0.40, 0.40, 0.95), 'weak_line': (0.42, 0.66, 0.31, 0.95),
    'pier': (0.71, 0.65, 0.84, 0.9),
}
DEFAULT_RGBA = (1.0, 0.0, 1.0, 0.9)
SLOT_RGBA = (0.85, 0.10, 0.10, 1.0)
CONDUIT_RGBA = (1.0, 1.0, 1.0, 0.95)
UNREACHABLE_RGBA = (1.0, 0.6, 0.0, 1.0)
WALL_RGBA = (0.75, 0.72, 0.70, 0.25)
REACH_RGBA = (0.2, 0.7, 0.3, 0.18)
ENDPOINT_EXISTING_RGBA = (0.95, 0.95, 0.95, 1.0)
ENDPOINT_NEW_RGBA = (0.20, 0.60, 1.00, 1.0)
ORDER_RGBA = (1.0, 0.85, 0.1, 0.9)
PLANNED_TIP_RGBA = (0.6, 0.3, 0.9, 0.9)
TRACE_CUT_RGBA = (1.0, 0.15, 0.15, 1.0)
TRACE_MOVE_RGBA = (0.1, 0.8, 1.0, 1.0)
TEXT_RGBA = (1.0, 1.0, 1.0, 1.0)

# 各层离墙面的距离（m），避免 z-fighting
OFF_WALL = 0.001
OFF_REACH = 0.002
OFF_WINDOW = 0.003
OFF_OBSTACLE = 0.004
OFF_SLOT = 0.006
OFF_CONDUIT = 0.008


@dataclass
class MarkerSpec:
    """
    一个 Marker 的几何与外观（坐标都在 frame_id 下）.

    kind: line_strip / line_list / cube / sphere / cylinder / arrow / text / sphere_list
    """

    ns: str
    id: int
    kind: str
    points: list = field(default_factory=list)
    position: tuple = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    scale: tuple = (0.01, 0.01, 0.01)
    color: tuple = (1.0, 1.0, 1.0, 1.0)
    colors: list = field(default_factory=list)
    text: str = ''


def station_rgba(k, alpha=1.0):
    """第 k 个停靠站的颜色（色相循环）."""
    r, g, b = colorsys.hsv_to_rgb((k * 0.13) % 1.0, 0.75, 0.95)
    return (r, g, b, alpha)


def wall_rect(wall, u, v, w, h, off):
    """墙面矩形（左下角 u, v；宽 w 高 h）→ 闭合折线的 map 点."""
    return [wall.to_map(a, b, off) for a, b in
            ((u, v), (u + w, v), (u + w, v + h), (u, v + h), (u, v))]


def wall_panel(ns, mid, wall, u, v, w, h, off, rgba, thickness=0.002):
    """贴墙的薄板（CUBE）：中心在 (u + w/2, v + h/2)，离墙 off."""
    return MarkerSpec(ns, mid, 'cube', position=wall.to_map(u + w / 2, v + h / 2, off),
                      yaw=wall.yaw, scale=(abs(w), thickness, abs(h)), color=rgba)


def wall_polyline(ns, mid, wall, pts, off, rgba, width):
    """墙面折线 [(u, v)] → LINE_STRIP."""
    return MarkerSpec(ns, mid, 'line_strip', points=[wall.to_map(u, v, off) for u, v in pts],
                      scale=(width, 0.0, 0.0), color=rgba)


def ascii_label(text, fallback):
    """在 RViz 中文字只保证 ASCII 能显示：含非 ASCII 字符时用 fallback."""
    return text if text and all(ord(c) < 128 for c in text) else fallback


def is_existing(label):
    """任务线盒是否“已有”（否则是新装 / 待开）."""
    return '已有' in label or label.lower().startswith('existing')


def wall_markers(scene):
    """墙体轮廓、墙边净距、机械臂可达高度带."""
    w = scene.wall
    e = wm.EDGE_MARGIN
    lo, hi = scene.reach_band
    return [
        wall_panel('wall', 0, w, 0.0, 0.0, w.length, w.height, OFF_WALL, WALL_RGBA),
        MarkerSpec('wall', 1, 'line_strip', points=wall_rect(w, 0, 0, w.length, w.height,
                                                             OFF_WALL),
                   scale=(0.01, 0, 0), color=(0.9, 0.9, 0.9, 0.8)),
        MarkerSpec('edge_margin', 0, 'line_strip',
                   points=wall_rect(w, e, e, w.length - 2 * e, w.height - 2 * e, OFF_WALL),
                   scale=(0.004, 0, 0), color=(0.9, 0.9, 0.9, 0.4)),
        wall_panel('reach_band', 0, w, 0.0, lo, w.length, hi - lo, OFF_REACH, REACH_RGBA),
    ]


def obstacle_markers(scene, ascii_text=True):
    """墙面障碍物（实体）、规范边距（外扩轮廓）与标签."""
    out = []
    w = scene.wall
    for k, o in enumerate(scene.obstacles):
        rgba = KIND_RGBA.get(o.kind, DEFAULT_RGBA)
        out.append(wall_panel('obstacles', k, w, o.u, o.v, o.w, o.h, OFF_OBSTACLE, rgba, 0.004))
        m = wm.margin_of(o.kind)
        gu, gv, gw, gh = o.grown(m)
        out.append(MarkerSpec('margins', k, 'line_strip',
                              points=wall_rect(w, gu, gv, gw, gh, OFF_OBSTACLE),
                              scale=(0.004, 0, 0), color=rgba[:3] + (0.6,)))
        label = o.label if not ascii_text else ascii_label(o.label, f'{o.kind}{k + 1}')
        out.append(MarkerSpec('obstacle_labels', k, 'text',
                              position=w.to_map(o.u + o.w / 2, gv + gh + 0.04, 0.05),
                              scale=(0, 0, 0.05), color=TEXT_RGBA, text=label))
    return out


def endpoint_markers(scene, ascii_text=True, size=0.086):
    """开槽任务的线盒：已有（白）/ 新装（蓝），带标签."""
    out = []
    w = scene.wall
    for k, e in enumerate(scene.endpoints):
        rgba = ENDPOINT_EXISTING_RGBA if is_existing(e.label) else ENDPOINT_NEW_RGBA
        out.append(wall_panel('endpoints', k, w, e.u - size / 2, e.v - size / 2, size, size,
                              OFF_OBSTACLE, rgba, 0.006))
        fb = f'S{e.slot_index + 1}-{"A" if e.role == "start" else "B"}'
        label = e.label if not ascii_text else ascii_label(e.label, fb)
        out.append(MarkerSpec('endpoint_labels', k, 'text',
                              position=w.to_map(e.u, e.v + size / 2 + 0.04, 0.05),
                              scale=(0, 0, 0.045), color=TEXT_RGBA, text=label))
    return out


def slot_markers(scene):
    """槽线（直角折线，线宽 = 槽宽）、管线中心线、够不到的段."""
    out = []
    w = scene.wall
    for k, s in enumerate(scene.slots):
        if len(s.cut_path) >= 2:
            out.append(wall_polyline('slots', k, w, s.cut_path, OFF_SLOT, SLOT_RGBA,
                                     scene.slot_width))
        if len(s.conduit) >= 2:
            out.append(wall_polyline('conduits', k, w, s.conduit, OFF_CONDUIT, CONDUIT_RGBA,
                                     0.006))
    for k, p in enumerate(scene.unreachable):
        if len(p) >= 2:
            out.append(wall_polyline('unreachable', k, w, p, OFF_CONDUIT, UNREACHABLE_RGBA,
                                     0.012))
    return out


def footprint_points(pose, size=wm.FOOTPRINT, z=0.02):
    """base_link 位姿 → 底盘轮廓闭合折线（map）."""
    lx, ly = size[0] / 2, size[1] / 2
    corners = [(lx, ly), (-lx, ly), (-lx, -ly), (lx, -ly), (lx, ly)]
    return [wm.base_to_map(pose, x, y, z) for x, y in corners]


def station_markers(scene, show_footprints=True):
    """停靠站：位置箭头、序号、访问顺序连线、每站机械臂可达窗口、每站切割段."""
    out = []
    w = scene.wall
    lo, hi = scene.reach_band
    order = []
    start = scene.start_pose()
    if start is not None:
        order.append((start[0], start[1], 0.03))
        out.append(MarkerSpec('start', 0, 'cylinder', position=(start[0], start[1], 0.01),
                              scale=(0.08, 0.08, 0.02), color=(1.0, 1.0, 0.0, 0.9)))
        out.append(MarkerSpec('start', 1, 'text', position=(start[0], start[1], 0.35),
                              scale=(0, 0, 0.08), color=(1.0, 1.0, 0.0, 1.0), text='start'))
    for k, st in enumerate(scene.stations):
        rgba = station_rgba(k)
        x, y, yaw = st.pose
        order.append((x, y, 0.03))
        out.append(MarkerSpec('station_arrows', k, 'arrow', position=(x, y, 0.03), yaw=yaw,
                              scale=(0.25, 0.03, 0.03), color=rgba))
        out.append(MarkerSpec('station_ids', k, 'text', position=(x, y, 0.15 + 0.12 * (k % 2)),
                              scale=(0, 0, 0.09), color=rgba, text=str(k + 1)))
        if show_footprints:
            out.append(MarkerSpec('station_footprints', k, 'line_strip',
                                  points=footprint_points(st.pose),
                                  scale=(0.004, 0, 0), color=rgba[:3] + (0.35,)))
        u0, u1 = st.window
        out.append(wall_panel('station_windows', k, w, u0, lo, u1 - u0, hi - lo, OFF_WINDOW,
                              rgba[:3] + (0.25,)))
        out.append(MarkerSpec('station_window_ids', k, 'text',
                              position=w.to_map((u0 + u1) / 2, lo - 0.04, 0.03),
                              scale=(0, 0, 0.035), color=rgba, text=str(k + 1)))
        for j, piece in enumerate(st.pieces):
            if len(piece) >= 2:
                out.append(wall_polyline('station_pieces', 100 * k + j, w, piece,
                                         OFF_SLOT + 0.001, rgba, 0.008))
    if len(order) >= 2:
        out.append(MarkerSpec('station_order', 0, 'line_strip', points=order,
                              scale=(0.012, 0, 0), color=ORDER_RGBA))
    return out


def planned_tip_markers(scene):
    """每站机械臂点 → 计划刀尖轨迹（map），用于和实际刀尖轨迹对比."""
    out = []
    for k, st in enumerate(scene.stations):
        pts = [wm.arm_point_to_tip_map(st.pose, p, scene.tool_offset, scene.base_height)
               for p in st.points]
        if len(pts) >= 2:
            out.append(MarkerSpec('planned_tip_path', k, 'line_strip', points=pts,
                                  scale=(0.003, 0, 0), color=PLANNED_TIP_RGBA))
    return out


def scene_markers(scene, ascii_text=True, show_footprints=True, show_planned_tip=True):
    """完整的静态 Marker 列表."""
    out = wall_markers(scene) + obstacle_markers(scene, ascii_text) + \
        endpoint_markers(scene, ascii_text) + slot_markers(scene) + \
        station_markers(scene, show_footprints)
    if show_planned_tip:
        out += planned_tip_markers(scene)
    return out


# ---------------------------------------------------------------------------
# 实际刀尖轨迹
# ---------------------------------------------------------------------------
class TipTrace:
    """实际刀尖轨迹：按最小间距抽稀、限长，按离墙距离着色（贴墙 = 切割）."""

    def __init__(self, wall=None, min_step=0.002, max_points=20000, contact_threshold=0.02):
        self.wall = wall
        self.min_step = min_step
        self.max_points = max_points
        self.contact_threshold = contact_threshold
        self.points = []

    def clear(self):
        """清空轨迹."""
        self.points = []

    def add(self, p):
        """加入一个 map 点；与上一点距离 < min_step 时丢弃。返回是否加入."""
        p = (float(p[0]), float(p[1]), float(p[2]))
        if self.points and math.dist(self.points[-1], p) < self.min_step:
            return False
        self.points.append(p)
        if len(self.points) > self.max_points:
            del self.points[:len(self.points) - self.max_points]
        return True

    def in_contact(self, p):
        """刀尖离墙面 < contact_threshold（或已进入墙体）."""
        if self.wall is None:
            return False
        return self.wall.from_map(p[0], p[1], p[2])[2] < self.contact_threshold

    def markers(self):
        """轨迹 LINE_STRIP（逐点着色）+ 当前刀尖球."""
        if not self.points:
            return []
        colors = [TRACE_CUT_RGBA if self.in_contact(p) else TRACE_MOVE_RGBA
                  for p in self.points]
        out = [MarkerSpec('tool_tip', 1, 'sphere', position=self.points[-1],
                          scale=(0.025, 0.025, 0.025), color=colors[-1])]
        if len(self.points) >= 2:
            out.insert(0, MarkerSpec('tool_trace', 0, 'line_strip', points=list(self.points),
                                     scale=(0.004, 0, 0), color=TRACE_MOVE_RGBA, colors=colors))
        return out
