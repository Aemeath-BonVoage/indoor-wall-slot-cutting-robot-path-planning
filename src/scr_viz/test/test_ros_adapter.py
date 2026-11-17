# -*- coding: utf-8 -*-
"""ros_adapter：MarkerSpec → visualization_msgs（需要 ROS 环境，否则跳过）."""
import math

import pytest

pytest.importorskip('visualization_msgs')

from builtin_interfaces.msg import Time  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402

from scr_viz import markers as mk  # noqa: E402
from scr_viz import wall_model as wm  # noqa: E402
from scr_viz.ros_adapter import to_marker_array  # noqa: E402


def test_marker_array(task_file, plan_file):
    scene = wm.load_scene(task_file, plan_file)
    specs = mk.scene_markers(scene)
    arr = to_marker_array(specs, 'map', Time(sec=1))
    assert arr.markers[0].action == Marker.DELETEALL
    assert len(arr.markers) == len(specs) + 1
    m = [m for m in arr.markers if m.ns == 'station_arrows'][0]
    assert m.type == Marker.ARROW and m.header.frame_id == 'map'
    assert 2 * math.atan2(m.pose.orientation.z, m.pose.orientation.w) == \
        pytest.approx(scene.stations[0].pose[2])
    slot = [m for m in arr.markers if m.ns == 'slots'][0]
    assert slot.type == Marker.LINE_STRIP and len(slot.points) == len(scene.slots[0].cut_path)


def test_trace_colors(task_file):
    scene = wm.load_scene(task_file, None)
    tr = mk.TipTrace(scene.wall)
    tr.add((8.0, -3.5, 0.5))
    tr.add((8.1, -3.92, 0.5))
    arr = to_marker_array(tr.markers(), 'map', Time(), clear_first=False)
    line = [m for m in arr.markers if m.ns == 'tool_trace'][0]
    assert len(line.colors) == 2 and line.colors[1].r == pytest.approx(mk.TRACE_CUT_RGBA[0])
