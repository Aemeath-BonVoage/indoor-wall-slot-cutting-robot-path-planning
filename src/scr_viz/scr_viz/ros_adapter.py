# -*- coding: utf-8 -*-
"""MarkerSpec → visualization_msgs/Marker（唯一依赖 ROS 消息的转换层）."""
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from scr_viz.wall_model import quaternion_from_yaw

_TYPES = {
    'line_strip': Marker.LINE_STRIP, 'line_list': Marker.LINE_LIST, 'cube': Marker.CUBE,
    'sphere': Marker.SPHERE, 'cylinder': Marker.CYLINDER, 'arrow': Marker.ARROW,
    'text': Marker.TEXT_VIEW_FACING, 'sphere_list': Marker.SPHERE_LIST,
}


def _rgba(c):
    return ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=float(c[3]))


def to_marker(spec, frame_id, stamp):
    """一个 MarkerSpec → Marker."""
    m = Marker(ns=spec.ns, id=int(spec.id), type=_TYPES[spec.kind], action=Marker.ADD)
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(c) for c in spec.position)
    q = quaternion_from_yaw(spec.yaw)
    m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
    m.scale.x, m.scale.y, m.scale.z = (float(c) for c in spec.scale)
    m.color = _rgba(spec.color)
    m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in spec.points]
    m.colors = [_rgba(c) for c in spec.colors]
    m.text = spec.text
    return m


def to_marker_array(specs, frame_id, stamp, clear_first=True):
    """把 MarkerSpec 列表转成 MarkerArray；clear_first 时先发 DELETEALL（重新加载规划时清旧的）."""
    arr = MarkerArray()
    if clear_first:
        clear = Marker(action=Marker.DELETEALL)
        clear.header.frame_id = frame_id
        clear.header.stamp = stamp
        arr.markers.append(clear)
    arr.markers.extend(to_marker(s, frame_id, stamp) for s in specs)
    return arr
