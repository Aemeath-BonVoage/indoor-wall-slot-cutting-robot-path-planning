# -*- coding: utf-8 -*-
"""Tests for task-level joint planning: wall generator, window accounting, joint planner,
lower bound, Lee baseline, and plan-file compatibility with the executor."""
import itertools
import math
import os
import random
import sys

import numpy as np
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(PKG, 'python'))
sys.path.insert(0, os.path.join(PKG, 'scripts'))
sys.path.insert(0, HERE)

import slot_plan as sp  # noqa: E402
import task_plan as tp  # noqa: E402
import wall_gen as wg  # noqa: E402
import wall_rules as wr  # noqa: E402
from scr_planner_py import OrientedAStar  # noqa: E402
from test_slot_plan import _check_plan  # noqa: E402

ROBOT = sp.RobotModel()
# 联合规划的性质在逐点执行的时间模型（每多一个小段约 8 s）下检验；连续轨迹的时间模型
# （每多一个小段约 1.7 s，另有切割长度项）见文末的专门测试
TM = sp.TimeModel.preset('align_v3')
TM_TRAJ = sp.TimeModel.preset('traj_v1')
WALL = sp.WallFrame((0.0, 0.0), (6.0, 0.0))


@pytest.fixture(scope='module')
def planner():
    return OrientedAStar()


@pytest.fixture(scope='module')
def scenes(planner):
    """Generated walls (each task routable on its own) with a random marker position."""
    out = []
    for seed in range(40):
        scene, meta = wg.residential_wall(seed, routable=tp.standalone_routable(planner, ROBOT))
        su = random.Random(seed).uniform(0.3, scene.width / 100 - 0.3)
        out.append((scene, meta, su))
    return out


def _key(scene):
    return (scene.width, scene.load_bearing, [tuple(vars(o).values()) for o in scene.obstacles],
            scene.tasks, scene.circuits)


# --- wall generator -------------------------------------------------------------
def test_generator_band_matches_robot():
    assert wg.BAND_CM == pytest.approx(tuple(b * 100 for b in ROBOT.reach_band), abs=1e-6)


def test_generator_is_reproducible_and_varied():
    a = [wg.residential_wall(s)[0] for s in range(15)]
    b = [wg.residential_wall(s)[0] for s in range(15)]
    assert [_key(x) for x in a] == [_key(x) for x in b]
    assert len({_key(x)[0] for x in a}) > 3                  # different wall widths
    metas = [wg.residential_wall(s)[1] for s in range(120)]
    assert {m['n_slots'] for m in metas} == {1, 2, 3, 4}
    lb = sum(m['load_bearing'] for m in metas) / len(metas)
    assert 0.15 < lb < 0.45


def test_generator_layout_rules():
    lo, hi = wg.BAND_CM
    for seed in range(80):
        scene, meta = wg.residential_wall(seed)
        assert 1 <= len(scene.tasks) <= 4 and len(scene.tasks) == meta['n_slots']
        assert len(scene.circuits) == len(scene.tasks) == len(scene.task_boxes)
        ends = []
        for k, ((sx, sy), (gx, gy), _, _) in enumerate(scene.tasks):
            assert lo <= sy <= hi and lo <= gy <= hi            # both ends inside the reach band
            assert sy == wg.SOCKET_H                            # fed from a socket-height box
            if scene.load_bearing:
                assert sx == gx                                 # vertical only
            ends += [(sx, sy, scene.circuit_of(k)), (gx, gy, scene.circuit_of(k))]
        for (x0, y0, c0), (x1, y1, c1) in itertools.combinations(ends, 2):
            if c0 != c1:                                        # strong / weak separation
                assert max(abs(x0 - x1), abs(y0 - y1)) >= wg.WEAK_SEP
        # out-of-band fixtures exist on some walls: switches at 1.3 m, boxes high up
        labels = {o.label for o in scene.obstacles}
        assert labels <= {'梁', '柱', '门', '窗', '飘窗', '窄墙垛', '开关', '配电箱', '接线盒',
                          '弱电箱', '弱电线槽', '已有插座', '已有弱电口'}


def test_generated_scene_config_round_trip():
    scene, _ = wg.residential_wall(3)
    cfg = yaml.safe_load(yaml.safe_dump(wg.scene_to_config(scene), allow_unicode=True))
    back, frame = sp.scene_from_config(cfg)
    assert frame.length == pytest.approx(scene.width / 100)
    assert back.tasks == scene.tasks and back.circuits == scene.circuits
    assert [vars(b) for bs in back.task_boxes for b in bs] == \
        [dict(vars(b), kind='box') for bs in scene.task_boxes for b in bs]
    assert [(o.x, o.y, o.w, o.h, o.kind) for o in back.obstacles] == \
        [(o.x, o.y, o.w, o.h, o.kind) for o in scene.obstacles]


def test_circuit_aware_margins():
    scene = wr.WallScene(200, 100, False, [wr.Rect(100, 0, 2, 100, 'weak_line', '弱电线槽')])
    strong = scene.grid(top_preference=False, circuit='strong') >= 253
    weak = scene.grid(top_preference=False, circuit='weak') >= 253
    assert strong[50, 100 - wr.WEAK_CURRENT_SPACING] and not weak[50, 100 - wr.SLOT_SPACING - 1]


# --- window accounting: search surrogate == plan evaluation ---------------------
def test_subsegments_match_clipped_points():
    """On a lattice of full-width windows, the per-route sub-segment count used by the
    search equals the arm points of the actual station pieces / 4."""
    rng = random.Random(1)
    wc = tp.window_cells(ROBOT)
    for _ in range(300):
        x, y = rng.randrange(100, 200), rng.randrange(30, 70)
        corners = [(x, y)]
        horiz = rng.random() < 0.5
        for _ in range(rng.randrange(1, 6)):
            if horiz:
                x += rng.choice((-1, 1)) * rng.randrange(1, 60)
            else:
                y += rng.choice((-1, 1)) * rng.randrange(1, 30)
            corners.append((x, y))
            horiz = not horiz
        phi = rng.randrange(wc)
        slot = sp.Slot('t', 'success', cut_path=sp.corners_to_m(corners))
        win = tp.windows_used(corners, phi, wc)
        pts = 0
        for k in win:
            lo = (phi + k * wc + 0.5) * sp.RES
            pieces = sp._clip_all([slot.cut_path], lo, lo + ROBOT.window_width)
            pts += sp.count_points(pieces)
        assert pts == 4 * tp.subsegments(corners, phi, wc)
        # windows not in `win` receive nothing
        for k in (min(win) - 1, max(win) + 1):
            lo = (phi + k * wc + 0.5) * sp.RES
            assert not sp._clip_all([slot.cut_path], lo, lo + ROBOT.window_width)


def test_cover_count_is_minimal():
    rng = random.Random(2)
    w = ROBOT.window_width
    for _ in range(200):
        iv = []
        for _ in range(rng.randrange(1, 5)):
            a = rng.randrange(0, 300) / 100
            iv.append((a, a if rng.random() < 0.3 else a + rng.randrange(1, 120) / 100))
        n = tp._cover_count(iv, w)
        # brute force on a 1 mm grid of window starts: greedy windows from the left
        pts = sorted({round(a + k * 0.001, 3) for a, b in iv
                      for k in range(int(round((b - a) / 0.001)) + (1 if a == b else 0))})
        m, cov = 0, -1.0
        for p in pts:
            if p > cov + 1e-9:
                m, cov = m + 1, p + w - 0.001 + 1e-9
        assert n == m


def test_sweep_travel_and_centre_placement_are_optimal():
    rng = random.Random(3)
    for _ in range(200):
        sts = []
        for k in range(rng.randrange(1, 5)):
            a = rng.uniform(0, 2)
            sts.append(sp.Station(k, a, a + rng.uniform(0.0, ROBOT.window_width), 0.0, (0, 0, 0)))
        s = rng.uniform(-0.5, 2.5)
        off = ROBOT.y_range[1] - ROBOT.margin
        rngs = [(min(st.u_hi - ROBOT.window_width, st.u_lo) + off, st.u_lo + off) for st in sts]
        sp.place_centres(sts, s, WALL, ROBOT, 0.035)
        for st, (lo, hi) in zip(sts, rngs):
            assert lo - 1e-12 <= st.u_c <= hi + 1e-12
        order = sp.order_stations(sts, s)
        cs = [x.u_c for x in order]
        got = sum(abs(b - a) for a, b in zip([s] + cs[:-1], cs))
        # brute force: centres on a fine grid inside their ranges, best visiting order
        grids = [np.linspace(lo, hi, 7) for lo, hi in rngs]
        best = math.inf
        for cs in itertools.product(*grids):
            lo_c, hi_c = min(cs), max(cs)
            best = min(best, min(abs(s - lo_c), abs(s - hi_c)) + hi_c - lo_c)
        assert got <= best + 1e-9


# --- joint planner ------------------------------------------------------------------
def test_joint_never_worse_compliant_and_above_lower_bound(planner, scenes):
    better = 0
    for scene, meta, su in scenes:
        cur = sp.plan_task(scene, WALL, ROBOT, planner, start_u=su, tm=TM)
        st = {}
        slots, plan = tp.joint_routes(scene, WALL, ROBOT, planner, TM, su, stats=st)
        lb = tp.lower_bound(scene, ROBOT, planner, TM, su)
        assert lb is not None                    # every task is routable on its own
        if cur.status == 'success':
            assert plan is not None and plan.status == 'success'
            assert plan.prediction['t_total'] <= cur.prediction['t_total'] + 1e-9
            assert lb['t_total'] <= cur.prediction['t_total'] + 0.05
            better += plan.prediction['t_total'] < cur.prediction['t_total'] - 0.5
        if plan is None:
            continue
        assert lb['t_total'] <= plan.prediction['t_total'] + 0.05
        assert sum(tp.plan_violations(scene, slots, ROBOT).values()) == 0
        assert plan.stations and plan.unreachable_length == 0.0
        _check_plan(plan, scene, WALL)
    assert better >= 5                           # the joint search matters on some walls


def test_load_bearing_walls_have_nothing_to_choose(planner, scenes):
    for scene, meta, su in scenes:
        if not meta['load_bearing']:
            continue
        cur = sp.plan_task(scene, WALL, ROBOT, planner, start_u=su, tm=TM)
        _, plan = tp.joint_routes(scene, WALL, ROBOT, planner, TM, su)
        assert plan.prediction['t_total'] == pytest.approx(cur.prediction['t_total'])


def test_corrected_evaluation_never_worse_than_legacy(planner, scenes):
    import task_planning_benchmark as tb
    for scene, meta, su in scenes[:20]:
        rule = sp.plan_routes(scene, ROBOT, planner)
        if any(s.status != 'success' for s in rule):
            continue
        tb.WALL = WALL
        old = tb.legacy_plan(rule, ROBOT, su, TM)
        new = sp.assemble_plan(rule, WALL, ROBOT, 'dp', 'sweep', su, TM)
        assert tb.exact_time(new, TM) <= tb.exact_time(old, TM) + 1e-6


def test_lattice_search_aligns_jog_with_window_boundary(planner):
    """One slot, socket to socket at the same height: with the window lattice anchored
    at the start box, the station-aware route puts the jog on a window boundary and
    needs fewer arm points than the rule route."""
    scene = wr.WallScene(400, 280, False, [], tasks=[((100, 30), (177, 55), 'a', 'b')])
    scene.start, scene.goal = (100, 30), (177, 55)
    wc = tp.window_cells(ROBOT)
    s = tp.plan_slot_on_lattice(scene, 0, [], ROBOT, planner, TM, phi=100 % wc)
    corners = tp.path_cells(s)
    jogs = [a[0] for a, b in zip(corners[:-1], corners[1:]) if a[0] == b[0]]
    assert jogs and all((x - 100) % wc == 0 for x in jogs)
    _, plan = tp.joint_routes(scene, WALL, ROBOT, planner, TM, 1.0)
    cur = sp.plan_task(scene, WALL, ROBOT, planner, start_u=1.0, tm=TM)
    assert plan.prediction['points'] < cur.prediction['points']


# --- classic baseline -----------------------------------------------------------------
def test_lee_router_matches_astar_without_segment_rules(planner, scenes):
    n = 0
    for scene, meta, su in scenes[:12]:
        for k, (s, g, _, _) in enumerate(sp.scene_tasks(scene)):
            grid = sp.route_grid(scene, k, [], ROBOT)
            p = scene.params(rules=False)
            p.update({kk: v for kk, v in scene.params().items()
                      if kk in ('weight_x', 'cost_weight', 'allow_x')})
            a = planner.plan(grid, s, g, **p)
            lee = tp.lee_route(grid, s, g, **p)
            assert a['ok'] and lee['ok']
            assert lee['cost'] == pytest.approx(a['cost'], abs=1e-6)
            n += 1
    assert n >= 15


# --- integration: plan file stays executable -----------------------------------------
def test_plan_wall_slots_joint_round_trip(planner, tmp_path):
    import plan_wall_slots
    cfg = os.path.join(PKG, 'config', 'wall_task_room3_south.yaml')
    d_rule, _ = plan_wall_slots.plan_file(cfg)
    d, plan = plan_wall_slots.plan_file(cfg, routes='joint')
    assert d['status'] == 'success' and d['routes'] == 'joint'
    assert d['prediction']['t_total'] <= d_rule['prediction']['t_total']
    assert set(d) == set(d_rule)                             # same file layout
    for key in ('robot', 'wall', 'slot_spec', 'marker_u'):
        assert d[key] == d_rule[key]
    path = tmp_path / 'plan.yaml'
    path.write_text(yaml.safe_dump(d, allow_unicode=True, sort_keys=False), encoding='utf-8')
    back = yaml.safe_load(path.read_text(encoding='utf-8'))
    plan2, wall2, _ = sp.plan_from_dict(back)
    assert len(plan2.stations) == len(plan.stations)
    assert [s.points for s in plan2.stations] == [s.points for s in plan.stations]


def test_joint_plan_file_is_executable_by_main_task(planner):
    pytest.importorskip('rclpy')
    from scr_arm_controller import navigation_cutting_task as nct
    import plan_wall_slots
    d, _ = plan_wall_slots.plan_file(os.path.join(PKG, 'config', 'wall_task_room3_south.yaml'),
                                     routes='joint')
    d = yaml.safe_load(yaml.safe_dump(d, allow_unicode=True))
    x_rng = (ROBOT.x_range[0] - 1e-6, ROBOT.x_range[1] + 1e-6)
    for st in d['stations']:
        pts, is_cut = nct.station_arm_points(st, d)
        assert len(pts) == len(st['points']) and any(is_cut)
        for (x, y, z, rot), cut in zip(pts, is_cut):
            assert x_rng[0] <= x <= x_rng[1]
            assert ROBOT.reachable(x, y, z, math.degrees(rot))
        dry, _ = nct.station_arm_points(st, d, dry_run_gap=0.01)
        assert all(abs(p[0] - (d['robot']['standoff'] - 0.01 - d['robot']['tool_offset'])) < 1e-9
                   for p, c in zip(dry, is_cut) if c)


# --- continuous-trajectory time model (cut length term) --------------------------------
def test_time_params_charge_cut_length():
    scene = wr.WallScene(400, 280, False, [], tasks=[((100, 30), (177, 55), 'a', 'b')])
    p2p, traj = tp.time_params(scene, TM), tp.time_params(scene, TM_TRAJ)
    cell = sp.RES / TM_TRAJ.v_feed
    rule = scene.params()
    assert p2p['weight_y'] == pytest.approx(tp.EPS_RULE * rule.get('weight_y', 1.0))
    assert traj['weight_y'] == pytest.approx(cell + tp.EPS_RULE * rule.get('weight_y', 1.0))
    assert traj['weight_x'] == pytest.approx(cell + tp.EPS_RULE * rule.get('weight_x', 1.0))
    assert traj['turn_penalty'] == pytest.approx(
        4 * TM_TRAJ.t_point + tp.EPS_RULE * rule['turn_penalty'])


def test_joint_and_lower_bound_with_cut_length_term(planner, scenes):
    """连续轨迹的时间模型下：下界 ≤ 联合规划 ≤ 规则路线；下界含最短槽长的切割时间."""
    import dataclasses
    no_feed = dataclasses.replace(TM_TRAJ, v_feed=math.inf)
    checked = 0
    for scene, meta, su in scenes[:16]:
        cur = sp.plan_task(scene, WALL, ROBOT, planner, start_u=su, tm=TM_TRAJ)
        slots, plan = tp.joint_routes(scene, WALL, ROBOT, planner, TM_TRAJ, su)
        lb = tp.lower_bound(scene, ROBOT, planner, TM_TRAJ, su)
        lb0 = tp.lower_bound(scene, ROBOT, planner, no_feed, su)
        assert lb is not None and lb0 is not None
        # 每条槽的长度不小于两端的曼哈顿距离
        manhattan = sum(abs(g[0] - s[0]) + abs(g[1] - s[1])
                        for s, g, _, _ in sp.scene_tasks(scene)) * sp.RES
        assert lb['t_total'] >= lb0['t_total'] + manhattan / TM_TRAJ.v_feed - 1e-6
        assert (lb['stations'], lb['points']) == (lb0['stations'], lb0['points'])
        if plan is None:
            continue
        assert lb['t_total'] <= plan.prediction['t_total'] + 0.05
        assert sum(tp.plan_violations(scene, slots, ROBOT).values()) == 0
        if cur.status == 'success':
            assert plan.prediction['t_total'] <= cur.prediction['t_total'] + 1e-9
            # 预测时间含切割长度项
            length = sum(st.cut_length for st in cur.stations)
            assert cur.prediction['t_cut'] == pytest.approx(
                TM_TRAJ.t_point * cur.prediction['points'] + length / TM_TRAJ.v_feed, abs=0.06)
        checked += 1
    assert checked >= 10
