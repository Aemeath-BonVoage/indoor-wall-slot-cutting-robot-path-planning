# -*- coding: utf-8 -*-
"""开槽规划的 ROS 辅助：RViz MarkerArray、包内路径."""
import os

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

KIND_RGBA = {
    'door': (0.56, 0.60, 0.69, 0.9), 'window': (0.62, 0.77, 0.91, 0.9),
    'beam': (0.5, 0.5, 0.5, 0.9), 'column': (0.5, 0.5, 0.5, 0.9),
    'box': (0.88, 0.40, 0.40, 0.95), 'weak_line': (0.42, 0.66, 0.31, 0.95),
    'pier': (0.71, 0.65, 0.84, 0.9),
}


def package_python_dir():
    """源码树或安装树中 python/ 目录（供节点 import 规划库）."""
    try:
        from ament_index_python.packages import get_package_share_directory
        d = os.path.join(get_package_share_directory('scr_planner'), 'python')
        if os.path.isdir(d):
            return d
    except Exception:  # noqa: BLE001
        pass
    return os.path.dirname(os.path.abspath(__file__))


def _pt(wall, u, v, off=0.0):
    p = wall.to_map(u, off)
    return Point(x=float(p[0]), y=float(p[1]), z=float(v))


def _rect(wall, u, v, w, h, off, ns, mid, rgba, frame, stamp):
    m = Marker(ns=ns, id=mid, type=Marker.LINE_STRIP, action=Marker.ADD)
    m.header.frame_id, m.header.stamp = frame, stamp
    m.scale.x = 0.006
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    m.points = [_pt(wall, a, b, off) for a, b in
                ((u, v), (u + w, v), (u + w, v + h), (u, v + h), (u, v))]
    m.pose.orientation.w = 1.0
    return m


def build_markers(plan, scene, wall, robot, stamp, executed=None):
    """plan: slot_plan.SlotPlan；executed: 可选的实际刀尖轨迹 [(u, v)]."""
    frame = wall.frame_id
    out = MarkerArray()
    clear = Marker(action=Marker.DELETEALL)
    clear.header.frame_id = frame
    out.markers.append(clear)
    mid = 0
    for o in scene.obstacles:
        mid += 1
        out.markers.append(_rect(wall, o.x / 100, o.y / 100, o.w / 100, o.h / 100, 0.003,
                                 'obstacles', mid, KIND_RGBA.get(o.kind, (1, 0, 1, 1)), frame,
                                 stamp))
        g = o.grown(scene.margin_of(o))
        mid += 1
        rgba = KIND_RGBA.get(o.kind, (1, 0, 1, 1))[:3] + (0.35,)
        out.markers.append(_rect(wall, g.x / 100, g.y / 100, g.w / 100, g.h / 100, 0.003,
                                 'margins', mid, rgba, frame, stamp))
    lo, hi = robot.reach_band
    out.markers.append(_rect(wall, 0.0, lo, wall.length, hi - lo, 0.002, 'reach_band', 1,
                             (0.2, 0.7, 0.3, 0.5), frame, stamp))

    def strip(ns, pts, rgba, width, mid_):
        m = Marker(ns=ns, id=mid_, type=Marker.LINE_STRIP, action=Marker.ADD)
        m.header.frame_id, m.header.stamp = frame, stamp
        m.scale.x = width
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.points = [_pt(wall, u, v, 0.004) for u, v in pts]
        m.pose.orientation.w = 1.0
        return m

    for k, cp in enumerate(plan.cut_paths):
        out.markers.append(strip('slot', cp, (0.85, 0.1, 0.1, 1.0), plan.slot_width, k))
    for k, cd in enumerate(plan.conduits):
        out.markers.append(strip('conduit_centerline', cd, (1.0, 1.0, 1.0, 0.9), 0.004, k))
    for k, piece in enumerate(plan.unreachable):
        out.markers.append(strip('unreachable', piece, (1.0, 0.6, 0.0, 1.0), 0.01, k))
    if executed:
        out.markers.append(strip('executed', executed, (0.1, 0.3, 0.9, 1.0), 0.008, 1))
    for st in plan.stations:
        c = (0.2, 0.4, 0.9, 0.8) if st.index % 2 == 0 else (0.9, 0.6, 0.1, 0.8)
        out.markers.append(_rect(wall, st.u_lo, lo, st.u_hi - st.u_lo, hi - lo, 0.005,
                                 'station_windows', st.index, c, frame, stamp))
        a = Marker(ns='station_poses', id=st.index, type=Marker.ARROW, action=Marker.ADD)
        a.header.frame_id, a.header.stamp = frame, stamp
        a.pose.position.x, a.pose.position.y, a.pose.position.z = st.pose[0], st.pose[1], 0.05
        import math
        a.pose.orientation.z, a.pose.orientation.w = math.sin(st.pose[2] / 2), math.cos(st.pose[2] / 2)
        a.scale.x, a.scale.y, a.scale.z = 0.25, 0.03, 0.03
        a.color.r, a.color.g, a.color.b, a.color.a = c
        out.markers.append(a)
        t = Marker(ns='station_labels', id=st.index, type=Marker.TEXT_VIEW_FACING,
                   action=Marker.ADD)
        t.header.frame_id, t.header.stamp = frame, stamp
        p = wall.to_map(st.u_c, 0.25)
        t.pose.position.x, t.pose.position.y, t.pose.position.z = float(p[0]), float(p[1]), 0.15
        t.pose.orientation.w = 1.0
        t.scale.z = 0.06
        t.color.r = t.color.g = t.color.b = t.color.a = 1.0
        t.text = f'S{st.index}'
        out.markers.append(t)
    return out
