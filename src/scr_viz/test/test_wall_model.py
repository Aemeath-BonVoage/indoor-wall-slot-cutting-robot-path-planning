# -*- coding: utf-8 -*-
"""wall_model：墙面坐标、坐标变换、YAML 读取."""
import math
import os
import sys

import pytest

from scr_viz import wall_model as wm

SOUTH = ((9.925, -3.925), (5.075, -3.925))


def test_south_wall_frame():
    w = wm.Wall(*SOUTH)
    assert w.length == pytest.approx(4.85)
    assert w.t == pytest.approx((-1.0, 0.0))
    assert w.n == pytest.approx((0.0, 1.0))           # 指向房间（+y）
    assert w.yaw == pytest.approx(math.pi)
    assert w.facing_yaw == pytest.approx(-math.pi / 2)  # 正对南墙朝 -y


def test_marker_position_matches_world():
    # 任务里的黄色定位标记（左下角 1.48, 0.575，5 cm）中心 = world 中 (8.42, -3.925, 0.6)
    w = wm.Wall(*SOUTH)
    x, y, z = w.to_map(1.48 + 0.025, 0.575 + 0.025)
    assert (x, y, z) == pytest.approx((8.42, -3.925, 0.6))


def test_to_map_from_map_roundtrip():
    w = wm.Wall((1.0, 2.0), (4.0, 6.0), 2.5)
    for u, v, off in ((0, 0, 0), (2.5, 1.2, 0.3), (5.0, 0.4, -0.05)):
        assert w.from_map(*w.to_map(u, v, off)) == pytest.approx((u, v, off))


def test_degenerate_wall():
    with pytest.raises(ValueError):
        wm.Wall((1, 1), (1, 1))


def test_quaternions():
    for yaw in (-2.5, -math.pi / 2, 0.0, 0.7, 3.0):
        q = wm.quaternion_from_yaw(yaw)
        assert wm.yaw_from_quaternion(*q) == pytest.approx(yaw)
        assert wm.quaternion_from_rpy(0, 0, yaw) == pytest.approx(q)
    q = wm.quaternion_from_rpy(0.3, -0.2, 1.1)
    assert sum(c * c for c in q) == pytest.approx(1.0)


def test_base_to_map():
    # 朝 -y 的底盘：base x → map -y，base y（左）→ map +x
    assert wm.base_to_map((1.0, 2.0, -math.pi / 2), 0.5, 0.1) == pytest.approx((1.1, 1.5, 0.0))


def test_tip_from_link_poses():
    tip = wm.tip_from_link_poses((0, 0), math.pi / 2, (0.2, 0.5, 0.4), tool_offset=0.1)
    assert tip == pytest.approx((0.2, 0.6, 0.4))


def test_scene_from_task_only(task_file):
    s = wm.load_scene(task_file, None)
    assert s.wall.height == 2.0
    assert len(s.obstacles) == 1 and s.obstacles[0].kind == 'box'
    assert [e.role for e in s.endpoints] == ['start', 'goal', 'start', 'goal']
    assert s.endpoints[1].label == '新插座A'
    assert s.stations == [] and s.start_pose() is None


def test_scene_from_plan_only(plan_file):
    s = wm.load_scene(None, plan_file)
    assert s.wall.length == pytest.approx(4.85)
    assert len(s.slots) == 2 and len(s.stations) == 11
    assert s.obstacles and s.endpoints == []
    assert s.slot_width == pytest.approx(0.035)


def test_scene_requires_input():
    with pytest.raises(ValueError):
        wm.scene_from_dicts(None, None)


def test_station_poses_consistent(task_file, plan_file):
    """规划结果里的停靠站位姿 = 墙面 u_c 处离墙 standoff、正对墙面."""
    s = wm.load_scene(task_file, plan_file)
    for st in s.stations:
        x, y, _ = s.wall.to_map(st.u_c, 0.0, s.standoff)
        assert st.pose[:2] == pytest.approx((x, y), abs=2e-4)
        assert st.pose[2] == pytest.approx(s.wall.facing_yaw, abs=1e-3)
        # 切割窗口在机械臂横向行程内（y ∈ [-0.06, 0.08]，y = -(u - u_c)）
        assert st.u_c - 0.08 <= st.window[0] < st.window[1] <= st.u_c + 0.06 + 1e-3


def test_planned_tip_lands_on_slot(task_file, plan_file):
    """切割位置的机械臂点换算到 map 后：刀尖进入墙面 slot_depth，且落在本站窗口内、槽线上."""
    s = wm.load_scene(task_file, plan_file)
    x_cut = max(p['x'] for st in s.stations for p in st.points)
    n_cut = 0
    for st in s.stations:
        piece_pts = [p for piece in st.pieces for p in piece]
        for p in st.points:
            tip = wm.arm_point_to_tip_map(st.pose, p, s.tool_offset, s.base_height)
            u, v, off = s.wall.from_map(*tip)
            assert st.window[0] - 1e-3 <= u <= st.window[1] + 1e-3
            if abs(p['x'] - x_cut) < 1e-6:
                n_cut += 1
                assert off == pytest.approx(-s.slot_depth, abs=1e-3)
                # 每个切割点都在本站某段子折线的某个顶点 / 边上
                assert any(_on_polyline((u, v), piece) for piece in st.pieces), (u, v, piece_pts)
    assert n_cut > 0


def _on_polyline(q, pts, tol=2e-3):
    for (a, b), (c, d) in zip(pts[:-1], pts[1:]):
        if (min(a, c) - tol <= q[0] <= max(a, c) + tol and
                min(b, d) - tol <= q[1] <= max(b, d) + tol):
            return True
    return False


def test_start_pose(plan_file):
    s = wm.load_scene(None, plan_file)
    x, y, yaw = s.start_pose()
    assert (x, y) == pytest.approx((8.42, -3.225), abs=1e-3)   # 定位标记正前方 standoff


def test_margins_match_wall_rules():
    """边距常量与 scr_planner/python/wall_rules.py（厘米）一致；源码树不在时跳过."""
    d = os.path.join(os.path.dirname(__file__), '..', '..', 'scr_planner', 'python')
    if not os.path.exists(os.path.join(d, 'wall_rules.py')):
        pytest.skip('scr_planner 源码不在旁边')
    pytest.importorskip('numpy')
    sys.path.insert(0, d)
    try:
        import wall_rules as wr
    finally:
        sys.path.remove(d)
    scene = wr.WallScene(100, 100, False)
    for kind, m in wm.KIND_MARGIN.items():
        assert m * 100 == pytest.approx(scene.margin_of(wr.Rect(0, 0, 1, 1, kind)))
    assert wm.EDGE_MARGIN * 100 == pytest.approx(wr.EDGE_MARGIN)
