# -*- coding: utf-8 -*-
"""markers：Scene → MarkerSpec."""
import math

import pytest

from scr_viz import markers as mk
from scr_viz import wall_model as wm


@pytest.fixture
def scene(task_file, plan_file):
    return wm.load_scene(task_file, plan_file)


def _by_ns(specs):
    out = {}
    for s in specs:
        out.setdefault(s.ns, []).append(s)
    return out


def test_ids_unique_and_finite(scene):
    specs = mk.scene_markers(scene)
    keys = [(s.ns, s.id) for s in specs]
    assert len(keys) == len(set(keys))
    for s in specs:
        for p in list(s.points) + [s.position]:
            assert all(math.isfinite(c) for c in p)
        assert len(s.color) == 4 and all(0 <= c <= 1 for c in s.color)


def test_expected_layers(scene):
    ns = _by_ns(mk.scene_markers(scene))
    assert len(ns['slots']) == 2 and len(ns['conduits']) == 2
    assert len(ns['obstacles']) == len(ns['margins']) == 1
    assert len(ns['endpoints']) == 4
    assert len(ns['station_arrows']) == len(ns['station_windows']) == 11
    assert [s.text for s in ns['station_ids']] == [str(k) for k in range(1, 12)]
    # 访问顺序：起点 + 11 站
    assert len(ns['station_order'][0].points) == 12
    assert len(ns['planned_tip_path']) == 11


def test_slot_width_and_on_wall(scene):
    ns = _by_ns(mk.scene_markers(scene))
    for s in ns['slots']:
        assert s.scale[0] == pytest.approx(scene.slot_width)
        for p in s.points:
            u, v, off = scene.wall.from_map(*p)
            assert off == pytest.approx(mk.OFF_SLOT)
            assert 0 <= u <= scene.wall.length and 0 <= v <= scene.wall.height
    # 第一条槽的折线端点 = 规划结果的 cut_path
    first = [c for p in ns['slots'][0].points for c in scene.wall.from_map(*p)[:2]]
    assert first == pytest.approx([c for p in scene.slots[0].cut_path for c in p])


def test_margin_outline(scene):
    ns = _by_ns(mk.scene_markers(scene))
    o = scene.obstacles[0]
    uv = [scene.wall.from_map(*p)[:2] for p in ns['margins'][0].points]
    us, vs = [a for a, _ in uv], [b for _, b in uv]
    m = wm.margin_of(o.kind)
    assert min(us) == pytest.approx(o.u - m) and max(us) == pytest.approx(o.u + o.w + m)
    assert min(vs) == pytest.approx(o.v - m) and max(vs) == pytest.approx(o.v + o.h + m)


def test_wall_panel_orientation(scene):
    p = mk.wall_panel('x', 0, scene.wall, 1.0, 0.2, 0.5, 0.3, 0.01, (1, 1, 1, 1))
    assert p.yaw == pytest.approx(scene.wall.yaw)
    assert p.scale == pytest.approx((0.5, 0.002, 0.3))
    assert scene.wall.from_map(*p.position) == pytest.approx((1.25, 0.35, 0.01))


def test_station_windows_cover_reach_band(scene):
    ns = _by_ns(mk.scene_markers(scene))
    lo, hi = scene.reach_band
    for st, m in zip(scene.stations, ns['station_windows']):
        u, v, _ = scene.wall.from_map(*m.position)
        assert u == pytest.approx(sum(st.window) / 2)
        assert m.scale[0] == pytest.approx(st.window[1] - st.window[0])
        assert m.scale[2] == pytest.approx(hi - lo)


def test_ascii_labels(scene):
    specs = mk.scene_markers(scene, ascii_text=True)
    for s in specs:
        if s.kind == 'text':
            assert all(ord(c) < 128 for c in s.text)
    texts = [s.text for s in mk.scene_markers(scene, ascii_text=False)
             if s.ns == 'endpoint_labels']
    assert '已有插座A' in texts
    assert mk.ascii_label('abc', 'x') == 'abc' and mk.ascii_label('插座', 'x') == 'x'


def test_task_only_scene(task_file):
    s = wm.load_scene(task_file, None)
    ns = _by_ns(mk.scene_markers(s))
    assert 'slots' not in ns and 'station_order' not in ns
    assert len(ns['endpoints']) == 4


def test_endpoint_colors(scene):
    ns = _by_ns(mk.scene_markers(scene))
    cols = [m.color for m in ns['endpoints']]
    assert cols[0] == mk.ENDPOINT_EXISTING_RGBA and cols[1] == mk.ENDPOINT_NEW_RGBA


def test_footprint_points():
    pts = mk.footprint_points((1.0, 2.0, 0.0), (0.4, 0.2))
    assert pts[0] == pytest.approx((1.2, 2.1, 0.02)) and pts[0] == pts[-1]


def test_tip_trace(scene):
    tr = mk.TipTrace(scene.wall, min_step=0.01, max_points=5, contact_threshold=0.02)
    assert tr.markers() == []
    assert tr.add(scene.wall.to_map(1.0, 0.5, 0.3))
    assert not tr.add(scene.wall.to_map(1.001, 0.5, 0.3))     # 太近，丢弃
    for k in range(10):
        tr.add(scene.wall.to_map(1.0 + 0.02 * k, 0.5, 0.005 if k > 5 else 0.3))
    assert len(tr.points) == 5
    ms = tr.markers()
    line = [m for m in ms if m.ns == 'tool_trace'][0]
    assert len(line.colors) == len(line.points) == 5
    assert line.colors[-1] == mk.TRACE_CUT_RGBA
    assert [m for m in ms if m.ns == 'tool_tip'][0].position == tr.points[-1]
    tr.clear()
    assert tr.points == []
