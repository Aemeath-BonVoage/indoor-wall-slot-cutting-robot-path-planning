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
        for x in samples:  # every point in exactly one window
            assert sum(lo - sp.EPS <= x < hi - sp.EPS for lo, hi in wins) == 1
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
        assert st.u_hi - st.u_lo <= ROBOT.window_width + sp.REACH_TOL
        for p in st.points:
            assert ROBOT.reachable(p['x'], p['y'], p['z'], p['rotation'])
            assert p['x'] in (pytest.approx(xc), pytest.approx(ROBOT.x_retract))
        assert st.points[0]['x'] == pytest.approx(ROBOT.x_retract)
        assert st.points[-1]['x'] == pytest.approx(ROBOT.x_retract)
        # single-axis cuts: blade 0 / 90 deg, each move at cutting depth changes only y or z
        for a, b in zip(st.points[:-1], st.points[1:]):
            assert b['rotation'] in (0.0, 90.0)
            if a['x'] == pytest.approx(xc) and b['x'] == pytest.approx(xc):
                moved_y = abs(b['y'] - a['y']) > 1e-9
                moved_z = abs(b['z'] - a['z']) > 1e-9
                assert not (moved_y and moved_z), 'cut moves two axes at once'
                assert a['rotation'] == b['rotation']
                assert b['rotation'] == (90.0 if moved_y else 0.0) or not (moved_y or moved_z)
        # cutting points map back onto the cut path inside this station's window
        for p in st.points:
            u, v = st.u_c - p['y'], p['z'] + ROBOT.base_height
            assert st.u_lo - 1e-4 <= u <= st.u_hi + 1e-4
            assert band[0] - 1e-4 <= v <= band[1] + 1e-4
            d = min(_dist_to_polyline((u, v), cp) for cp in plan.cut_paths)
            assert d < 1e-4
    # every reachable point of the cut path is cut by exactly one station
    for piece in [q for cp in plan.cut_paths for q in sp.clip_polyline(cp, band[0], band[1], 1)]:
        for q in sp._densify(piece, 0.003)[1:-1]:
            if any(abs(q[0] - b) < 1e-5 for st in plan.stations for b in (st.u_lo, st.u_hi)):
                continue      # 恰在窗口边界上的采样点会同时贴着两侧的子折线
            n = sum(any(_dist_to_polyline(q, pc) < 1e-6 for pc in st.pieces)
                    for st in plan.stations)
            assert n == 1, f'point {q} cut by {n} stations'


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
    assert len(scene.tasks) == 2
    plans = {}
    for part, order in (('per_slot', 'slot_by_slot'), ('greedy', 'sweep'), ('dp', 'sweep')):
        plan = sp.plan_task(scene, wall, ROBOT, planner, partition=part, order=order,
                            start_u=1.505)
        assert plan.status == 'success'
        assert plan.unreachable_length == 0.0
        for slot in plan.slots:
            cells = wr.densify([(round(u / sp.RES - 0.5), round(v / sp.RES - 0.5))
                                for u, v in slot.corners_m])
            assert sum(scene.violations(cells).values()) == 0
        _check_plan(plan, scene, wall)
        plans[part] = plan
    # stations shared between the two slots, DP never worse than greedy under the model
    assert len(plans['greedy'].stations) < len(plans['per_slot'].stations)
    assert plans['dp'].prediction['t_total'] <= plans['greedy'].prediction['t_total'] + 1e-6
    assert plans['dp'].prediction['t_total'] < plans['per_slot'].prediction['t_total']
    # the two slots keep the 5 cm spacing (no crossing)
    a, b = plans['dp'].slots
    assert min(_dist_to_polyline(p, b.cut_path) for p in sp._densify(a.cut_path, 0.005)) >= \
        wr.SLOT_SPACING / 100 - 0.011


def test_plan_file_round_trip(planner):
    cfg = yaml.safe_load(open(os.path.join(PKG, 'config', 'wall_task_room3_south.yaml')))
    scene, wall = sp.scene_from_config(cfg)
    plan = sp.plan_task(scene, wall, ROBOT, planner, start_u=1.505)
    d = yaml.safe_load(yaml.safe_dump(sp.plan_to_dict(plan, wall, ROBOT, scene=scene,
                                                      marker_u=1.505), allow_unicode=True))
    plan2, wall2, obstacles = sp.plan_from_dict(d)
    assert np.allclose(wall2.p0, wall.p0) and wall2.length == pytest.approx(wall.length)
    assert [s.index for s in plan2.stations] == list(range(len(plan.stations)))
    for s1, s2 in zip(plan.stations, plan2.stations):
        assert s2.points == s1.points
        assert s2.u_c == pytest.approx(s1.u_c, abs=1e-4)
    assert len(obstacles) == len(scene.obstacles)
    assert plan2.length == pytest.approx(plan.length, abs=1e-3)


def _main_task_module():
    """navigation_cutting_task（执行主路径）的纯函数；需要 ROS 环境（colcon test 中有）."""
    sys.path.insert(0, os.path.join(SRC, 'scr_arm_controller'))
    pytest.importorskip('rclpy')
    from scr_arm_controller import navigation_cutting_task as nct
    return nct


def test_main_task_executes_plan_file(planner):
    """plan_wall_slots.py 的输出能被主任务直接读取：位姿、切割点、空切、补偿一致."""
    nct = _main_task_module()
    cfg = yaml.safe_load(open(os.path.join(PKG, 'config', 'wall_task_room3_south.yaml')))
    scene, wall = sp.scene_from_config(cfg)
    plan = sp.plan_task(scene, wall, ROBOT, planner, start_u=1.505)
    d = yaml.safe_load(yaml.safe_dump(sp.plan_to_dict(plan, wall, ROBOT, scene=scene,
                                                      marker_u=1.505), allow_unicode=True))
    assert nct.is_slot_plan(d)
    single = yaml.safe_load(_read('scr_arm_controller/config/task_at_target.yaml'))
    assert not nct.is_slot_plan(single)
    x_rng, y_rng = ROBOT.x_range, ROBOT.y_range
    gap = 0.01
    for st in d['stations']:
        assert nct.plan_wall_pose(d, st['u_c'], ROBOT.standoff) == pytest.approx(
            (st['pose']['x'], st['pose']['y'], st['pose']['yaw']), abs=2e-4)
        pts, is_cut = nct.station_arm_points(st, d)
        assert [p[:3] for p in pts] == [(p['x'], p['y'], p['z']) for p in st['points']]
        assert sum(is_cut) >= 2 and not is_cut[0] and not is_cut[-1]
        dry, _ = nct.station_arm_points(st, d, dry_run_gap=gap)
        for (x, y, z, rot), cut in zip(dry, is_cut):
            # 空切：刀尖停在规划墙面前 gap 处
            if cut:
                assert x + ROBOT.tool_offset == pytest.approx(ROBOT.standoff - gap)
            assert ROBOT.reachable(x, y, z, math.degrees(rot))
        same, n = nct.compensate_points(dry, is_cut, 0.0, 0.0, 0.0, ROBOT.tool_offset,
                                        x_rng, y_rng)
        assert same == dry and n == 0


def test_main_task_compensation_cancels_station_error():
    """到站误差 (横向 e_l, 离墙 e_n, 朝向 psi) 下，补偿后的刀尖仍落在名义墙面位置上."""
    nct = _main_task_module()
    x_rng, y_rng = ROBOT.x_range, ROBOT.y_range
    pts = [(0.52, 0.0, 0.5, 0.0), (0.56, 0.02, 0.5, 0.0), (0.56, -0.03, 0.4, 0.0)]
    is_cut = [False, True, True]

    def tip_on_wall(p, e_l, e_n, psi):
        # 底盘：墙面系中 u 向右、离墙 D + e_n、朝向偏 psi（逆时针）；base y 向左
        fx, fy = p[0] + ROBOT.tool_offset, p[1]
        c, s = math.cos(psi), math.sin(psi)
        # 面墙时 base x = -n，base y = -t；逆时针 psi 让刀尖向右（+u）偏 fx·sin(psi)
        u = e_l - (s * fx + c * fy)
        depth = (ROBOT.standoff + e_n) - (c * fx - s * fy)
        return u, depth

    for e_l, e_n, psi in ((0.004, -0.003, math.radians(0.6)), (-0.008, 0.002, -0.01)):
        comp, n = nct.compensate_points(pts, is_cut, e_l, e_n, psi, ROBOT.tool_offset,
                                        x_rng, y_rng)
        assert n == 0
        for p0, p1, cut in zip(pts, comp, is_cut):
            u0, d0 = tip_on_wall(p0, 0.0, 0.0, 0.0)
            u1, d1 = tip_on_wall(p1, e_l, e_n, psi)
            assert u1 == pytest.approx(u0, abs=3e-4)       # 横向：小角度近似误差 < 0.3 mm
            if cut:
                assert d1 == pytest.approx(d0, abs=1e-3)
    # 超出 Y 行程的补偿被钳制并计数
    _, n = nct.compensate_points(pts, is_cut, 0.2, 0.0, 0.0, ROBOT.tool_offset, x_rng, y_rng)
    assert n == len(pts)


def test_dp_partition_never_worse_and_complete(planner):
    rng = random.Random(21)
    wall = sp.WallFrame((10.0, 0.0), (5.0, 0.0))
    tm = sp.TimeModel()
    done = 0
    while done < 12:
        scene = wr.random_scene(rng, load_bearing=False, band=(27, 79), n_tasks=2)
        if scene is None:
            continue
        g = sp.plan_task(scene, wall, ROBOT, planner, partition='greedy', tm=tm)
        if g.status != 'success':
            continue
        d = sp.plan_task(scene, wall, ROBOT, planner, partition='dp', tm=tm)
        model = lambda p: len(p.stations) * tm.t_settle + tm.t_point * sum(  # noqa: E731
            len(s.points) for s in p.stations)
        assert model(d) <= model(g) + 1e-6
        _check_plan(d, scene, wall)
        done += 1


def test_sweep_order_minimises_travel():
    import itertools
    rng = random.Random(4)
    for _ in range(100):
        sts = [sp.Station(k, 0, 0, rng.uniform(0, 3), (0, 0, 0)) for k in range(rng.randrange(1, 7))]
        start = rng.uniform(-0.5, 3.5)
        order = sp.order_stations(sts, start)

        def travel(seq):
            u, t = start, 0.0
            for s_ in seq:
                t, u = t + abs(s_.u_c - u), s_.u_c
            return t
        best = min(travel(p) for p in itertools.permutations(sts))
        assert travel(order) == pytest.approx(best)


def test_in_band_tasks_are_inside_reach():
    rng = random.Random(9)
    band = tuple(b * 100 for b in ROBOT.reach_band)
    n = 0
    while n < 30:
        lb = n % 3 == 0
        scene = wr.random_scene(rng, load_bearing=lb, band=band, n_tasks=1 + n % 3)
        if scene is None:
            continue
        n += 1
        for (sx, sy), (gx, gy), *_ in scene.tasks:
            assert band[0] + 6 <= sy <= band[1] - 6 and band[0] + 6 <= gy <= band[1] - 6
            if lb:
                assert sx == gx      # load-bearing: vertical only


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
        scene.tasks = []
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
    assert scene.tasks == [] or len(scene.tasks) == 1
    assert total == pytest.approx(plan.length, abs=1e-6)
    _check_plan(plan, scene, wall)


# --- time model presets ---------------------------------------------------------
def test_time_model_presets():
    v1 = sp.TimeModel.preset('align_v1')
    assert (v1.t_settle, v1.v_strafe, v1.t_point) == (4.40, 0.0945, 2.034)
    assert sp.TimeModel() == sp.TimeModel.preset('align_v2')
    with pytest.raises(ValueError):
        sp.TimeModel.preset('nonexistent')


RUNS = os.path.join(SRC, '..', 'docs', 'experiments', 'slot_execution', 'runs')


@pytest.mark.skipif(not os.path.isdir(RUNS), reason='experiment data not in this tree')
def test_time_model_presets_reproduce_calibration():
    """两个预设分别由各自的标定数据与方法复现（calibrate_time_model.py）."""
    sys.path.insert(0, os.path.join(PKG, 'scripts'))
    import calibrate_time_model as cal
    for name, runs, method in (
            ('align_v1', ('dp', 'dp_repeat', 'per_slot', 'per_slot_repeat'), 'joint'),
            ('align_v2', ('fine_align_dp', 'fine_align_dp_repeat', 'fine_align_dp_nofold'),
             'phases')):
        a = cal.arrays(cal.load([os.path.join(RUNS, r) for r in runs]))
        t_settle, v, t_point = cal.FIT[method](a)
        tm = sp.TimeModel.preset(name)
        assert t_settle == pytest.approx(tm.t_settle, abs=0.005)
        assert v == pytest.approx(tm.v_strafe, abs=5e-5)
        assert t_point == pytest.approx(tm.t_point, abs=5e-4)
