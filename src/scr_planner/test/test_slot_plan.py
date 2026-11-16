# -*- coding: utf-8 -*-
"""Tests for slot_plan: robot constants, geometry, station covering, arm points."""
import math
import os
import random
import re
import sys

import numpy as np
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, '..'))
SRC = os.path.abspath(os.path.join(PKG, '..'))
sys.path.insert(0, os.path.join(PKG, 'python'))

import slot_plan as sp  # noqa: E402
import wall_rules as wr  # noqa: E402
from scr_planner_py import OrientedAStar  # noqa: E402

ROBOT = sp.RobotModel()


@pytest.fixture(scope='module')
def planner():
    return OrientedAStar()


# --- robot constants must match the code / URDF they were taken from --------
def _read(path):
    with open(os.path.join(SRC, path), encoding='utf-8') as f:
        return f.read()


def test_arm_constants_match_cartesian_cutting():
    src = _read('scr_arm_controller/scr_arm_controller/cartesian_cutting.py')
    for name in ('x0', 'y0', 'z0'):
        val = float(re.search(rf'self\.{name} = ([-\d.]+)', src).group(1))
        assert getattr(ROBOT, name) == pytest.approx(val)
    for j in ('joint1', 'joint2', 'joint3', 'joint4'):
        lo, hi = map(float, re.search(rf"'{j}': \(([-\d.]+), ([-\d.]+)\)", src).groups())
        assert getattr(ROBOT, 'j' + j[-1]) == pytest.approx((lo, hi))


def test_tool_offset_standoff_and_base_height():
    vs = _read('scr_arm_controller/scr_arm_controller/visual_servo_ros.py')
    assert float(re.search(r"offset=([\d.]+)", vs).group(1)) == pytest.approx(ROBOT.tool_offset)
    task = yaml.safe_load(_read('scr_arm_controller/config/task_at_target.yaml'))
    assert task['precise_positioning']['distance_from_marker'] == pytest.approx(ROBOT.standoff)
    urdf = _read('scr_description/urdf/scr_robot_gazebo.urdf')
    z = float(re.search(r'joint_lf_link.*?<origin xyz="[-\d.]+ [-\d.]+ ([-\d.]+)"', urdf,
                        re.S).group(1))
    r = float(re.search(r'<cylinder radius="([\d.]+)" length="0.0602"', urdf).group(1))
    assert -z + r == pytest.approx(ROBOT.base_height, abs=1e-3)


def test_cut_depth_within_arm_travel():
    xc = ROBOT.x_cut(wr.SLOT_DEPTH / 100.0)
    lo, hi = ROBOT.x_range
    assert lo <= xc <= hi
    # tool tip penetrates exactly the slot depth at standoff
    assert xc + ROBOT.tool_offset - ROBOT.standoff == pytest.approx(0.035)
    assert ROBOT.x_retract + ROBOT.tool_offset < ROBOT.standoff  # retracted tip is off the wall


# --- geometry -----------------------------------------------------------------
def test_wall_frame_and_arm_point_round_trip():
    wall = sp.WallFrame((9.925, -3.925), (5.075, -3.925))
    assert wall.length == pytest.approx(4.85)
    assert np.allclose(wall.n, (0, 1))                   # towards the room
    x, y, yaw = wall.station_pose(1.505, 0.70)
    assert (x, y) == pytest.approx((8.42, -3.225))       # in front of the marker
    assert yaw == pytest.approx(-math.pi / 2)            # facing the wall
    # base_link left axis is +x here, i.e. -t: arm y grows as u decreases
    ay, az = wall.arm_point(1.505 + 0.03, 0.6, 1.505, ROBOT)
    assert ay == pytest.approx(-0.03) and az == pytest.approx(0.6 - ROBOT.base_height)
    left = np.array([-math.sin(yaw), math.cos(yaw)])
    p = wall.to_map(1.535)
    assert float((p - np.array([x, y])) @ left) == pytest.approx(ay)


def test_blade_angle_matches_existing_convention():
    assert sp.blade_angle_deg(0, 1) == 0 and sp.blade_angle_deg(0, -1) == 0
    assert sp.blade_angle_deg(1, 0) == 90 and sp.blade_angle_deg(-1, 0) == 90
    assert sp.blade_angle_deg(1, 1) == pytest.approx(45)
    assert sp.blade_angle_deg(-1, -1) == pytest.approx(45)
    assert sp.blade_angle_deg(1, -1) == pytest.approx(-45)


def test_conduit_arc_is_tangent_and_shorter():
    corners = [(0.0, 0.0), (0.5, 0.0), (0.5, 0.4)]
    poly = sp.conduit_polyline(corners, 0.12)
    assert poly[0] == corners[0] and poly[-1] == corners[-1]
    assert (0.38, 0.0) == pytest.approx(poly[1])         # tangent point on first leg
    assert (0.5, 0.12) == pytest.approx(poly[-2])        # tangent point on second leg
    ctr = np.array([0.38, 0.12])
    for p in poly[1:-1]:
        assert np.linalg.norm(np.array(p) - ctr) == pytest.approx(0.12)
    assert sp.polyline_length(poly) < 0.9


def test_clip_polyline_partitions_length():
    rng = random.Random(3)
    for _ in range(200):
        pts = [(rng.uniform(0, 2), rng.uniform(0, 2)) for _ in range(rng.randrange(2, 7))]
        lo, hi = sorted((rng.uniform(-0.2, 2.2), rng.uniform(-0.2, 2.2)))
        axis = rng.randrange(2)
        inside = sp.clip_polyline(pts, lo, hi, axis)
        below = sp.clip_polyline(pts, -1e9, lo, axis)
        above = sp.clip_polyline(pts, hi, 1e9, axis)
        total = sum(sp.polyline_length(p) for p in inside + below + above)
        assert total == pytest.approx(sp.polyline_length(pts), abs=1e-9)
        for piece in inside:
            assert all(lo - 1e-9 <= p[axis] <= hi + 1e-9 for p in piece)


# --- station covering is minimal and complete ---------------------------------
def test_cover_stations_complete_and_minimal():
    rng = random.Random(11)
    for _ in range(300):
        ivs = []
        for _ in range(rng.randrange(1, 6)):
            s = rng.uniform(0, 3)
            ivs.append((s, s) if rng.random() < 0.3 else (s, s + rng.uniform(0.01, 0.6)))
        w = rng.choice((0.12, 0.25, 0.5))
        wins = sp.cover_stations(ivs, w)
        samples = [s + (e - s) * k / 200 for s, e in ivs for k in range(201)]
        for x in samples:  # every point in exactly one half-open window
            assert sum(lo <= x < hi for lo, hi in wins) == 1
        # optimality certificate: window left edges lie on the covered set and are
        # pairwise >= w apart, so no window of width w can contain two of them and
        # any cover needs at least len(wins) windows
        starts = [lo for lo, _ in wins]
        assert all(any(s - 1e-12 <= x <= e + 1e-12 for s, e in ivs) for x in starts)
        assert all(b - a >= w - 1e-9 for a, b in zip(starts[:-1], starts[1:]))


# --- whole pipeline -------------------------------------------------------------
def _check_plan(plan, scene, wall):
    band = ROBOT.reach_band
    xc = ROBOT.x_cut(plan.slot_depth)
    for st in plan.stations:
        assert st.u_hi - st.u_lo == pytest.approx(ROBOT.window_width)
        for p in st.points:
            assert ROBOT.reachable(p['x'], p['y'], p['z'], p['rotation'])
            assert p['x'] in (pytest.approx(xc), pytest.approx(ROBOT.x_retract))
        assert st.points[0]['x'] == pytest.approx(ROBOT.x_retract)
        assert st.points[-1]['x'] == pytest.approx(ROBOT.x_retract)
        # cutting points map back onto the conduit inside this station's window
        for p in st.points:
            u, v = st.u_c - p['y'], p['z'] + ROBOT.base_height
            assert st.u_lo - 1e-4 <= u <= st.u_hi + 1e-4
            assert band[0] - 1e-4 <= v <= band[1] + 1e-4
            d = min(_dist_to_polyline((u, v), plan.conduit), 1.0)
            assert d < 1e-4
    # every reachable conduit point is cut by exactly one station
    for piece in sp.clip_polyline(plan.conduit, band[0], band[1], axis=1):
        for q in sp._densify(piece, 0.003)[1:-1]:
            n = sum(any(_dist_to_polyline(q, pc) < 1e-6 for pc in st.pieces)
                    for st in plan.stations)
            assert n >= 1
            owners = [st for st in plan.stations if st.u_lo <= q[0] < st.u_hi]
            assert len(owners) == 1


def _dist_to_polyline(p, pts):
    p = np.asarray(p)
    best = math.inf
    for a, b in zip(pts[:-1], pts[1:]):
        a, b = np.asarray(a), np.asarray(b)
        t = np.clip(np.dot(p - a, b - a) / max(np.dot(b - a, b - a), 1e-18), 0, 1)
        best = min(best, float(np.linalg.norm(p - (a + t * (b - a)))))
    return best


def test_example_config_plans_and_is_executable(planner):
    cfg = yaml.safe_load(open(os.path.join(PKG, 'config', 'wall_task_room3_south.yaml')))
    scene, wall = sp.scene_from_config(cfg)
    plan = sp.plan_task(scene, wall, ROBOT, planner)
    assert plan.status == 'success'
    assert plan.unreachable_length == 0.0
    cells = wr.densify([(round(u / sp.RES - 0.5), round(v / sp.RES - 0.5))
                        for u, v in plan.corners_m])
    assert sum(scene.violations(cells).values()) == 0
    # horizontal extent 1.10 m with 0.12 m windows needs at least 10 stations
    extent = max(u for u, _ in plan.conduit) - min(u for u, _ in plan.conduit)
    assert len(plan.stations) == math.ceil(extent / ROBOT.window_width - 1e-9)
    _check_plan(plan, scene, wall)
    d = sp.plan_to_dict(plan, wall, ROBOT)
    yaml.safe_dump(d, allow_unicode=True)  # serialisable
    assert all(set(p) == {'x', 'y', 'z', 'rotation'} for s in d['stations'] for p in s['points'])


def test_random_walls_full_pipeline(planner):
    rng = random.Random(5)
    wall = sp.WallFrame((10.0, 0.0), (5.0, 0.0))
    done = 0
    while done < 15:
        scene = wr.random_scene(rng, load_bearing=False)
        if scene is None:
            continue
        # put the task inside the arm's reach band
        scene.start = (scene.start[0], 34)
        scene.goal = (scene.goal[0], 34)
        plan = sp.plan_task(scene, wall, ROBOT, planner)
        if plan.status != 'success':
            continue
        done += 1
        _check_plan(plan, scene, wall)


def test_unrestricted_plan_reports_unreachable_part(planner):
    rng = random.Random(8)
    wall = sp.WallFrame((10.0, 0.0), (5.0, 0.0))
    while True:
        scene = wr.random_scene(rng, load_bearing=False)  # tasks reach up to 2.2 m
        if scene is None or scene.goal[1] < 100:
            continue
        plan = sp.plan_task(scene, wall, ROBOT, planner, restrict_to_reach=False)
        if plan.status == 'success':
            break
    assert plan.unreachable_length > 0
    total = plan.unreachable_length + sum(st.cut_length for st in plan.stations)
    assert total == pytest.approx(plan.length, abs=1e-6)
    _check_plan(plan, scene, wall)
