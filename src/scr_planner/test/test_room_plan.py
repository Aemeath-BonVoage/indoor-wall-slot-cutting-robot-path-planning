# -*- coding: utf-8 -*-
"""无解墙面的替代方案（plan_alternatives）与房间级规划（room_plan / room_gen）的测试."""
import itertools
import math
import os
import random
import sys

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'python'))
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))

import plan_alternatives as pa  # noqa: E402
import room_gen as rg  # noqa: E402
import room_plan as rp  # noqa: E402
import slot_plan as sp  # noqa: E402
import task_plan as tp  # noqa: E402
import wall_gen as wg  # noqa: E402
from scr_planner_py import OrientedAStar  # noqa: E402

ROOM3 = os.path.join(HERE, '..', 'config', 'room_task_room3.yaml')
WALL = sp.WallFrame((0.0, 0.0), (6.0, 0.0))


@pytest.fixture(scope='module')
def planner():
    return OrientedAStar()


@pytest.fixture(scope='module')
def robot():
    return sp.RobotModel()


def wall(seed, planner, robot):
    return wg.residential_wall(seed, routable=tp.standalone_routable(planner, robot))[0]


# ---------------------------------------------------------------------------
# 墙面生成器的新参数
# ---------------------------------------------------------------------------
def test_wall_generator_unchanged_and_width():
    a, _ = wg.residential_wall(1121)
    b, _ = wg.residential_wall(1121, width=None, accept=None)
    assert (a.width, a.tasks, [(o.kind, o.x, o.y, o.w, o.h) for o in a.obstacles]) == \
        (b.width, b.tasks, [(o.kind, o.x, o.y, o.w, o.h) for o in b.obstacles])
    c, meta = wg.residential_wall(5, width=430, accept=lambda s: len(s.tasks) >= 1)
    assert c.width == 430
    assert all(len(f) == 4 for f in meta['feeds'])


# ---------------------------------------------------------------------------
# 诊断与替代方案
# ---------------------------------------------------------------------------
def test_routable_matches_permutation_search(planner, robot):
    """深度优先的顺序搜索与逐个排列尝试的结论一致."""
    for seed in (1000, 1003, 1022, 1040, 1121):
        scene = wall(seed, planner, robot)
        K = len(scene.tasks)
        ok = False
        for order in itertools.permutations(range(K)):
            slots = [None] * K
            for k in order:
                grid = sp.route_grid(scene, k, [s for s in slots if s], robot)
                res = planner.plan(grid, scene.tasks[k][0], scene.tasks[k][1], **scene.params())
                if not res['ok']:
                    break
                slots[k] = sp.make_slot(scene, k, res)
            else:
                ok = True
                break
        assert (pa.routable(scene, robot, planner) is not None) == ok, seed


def test_diagnose_infeasible_wall(planner, robot):
    scene = wall(1022, planner, robot)
    d = pa.diagnose(scene, robot, planner)
    assert not d['feasible']
    assert d['min_remove'] == 1 and len(d['core']) == 2
    assert d['cause'] == 'staircase_in_band' and 'band' in d['relax']
    # 诊断中放松过的规则常量必须恢复
    import wall_rules as wr
    assert (wr.H_RUN_MAX, wr.BEND_RADIUS, wr.SLOT_SPACING, wr.WEAK_CURRENT_SPACING) == \
        (50, 12.0, 5, 30)


def test_alternatives_are_compliant_and_ranked(planner, robot):
    scene = wall(1022, planner, robot)
    alts = pa.alternatives(scene, robot, planner)
    assert alts and [a.key for a in alts] == sorted(a.key for a in alts)
    tiers = {pa.TIERS[a.tier] for a in alts}
    assert 'refeed' in tiers and 'floor' in tiers
    assert pa.TIERS[alts[0].tier] == 'refeed'            # 改动最小的一级排在最前
    for tier in tiers:
        a = next(x for x in alts if pa.TIERS[x.tier] == tier)
        plan = pa.finalize(a, WALL, robot, planner)
        assert plan.status == 'success'
        assert sum(pa.check(a, scene, robot).values()) == 0, tier
    # 原任务未改动的槽仍在方案里；引线点改动只换了起点
    a = alts[0]
    k = a.changes[0].slot
    assert a.scene.tasks[k][1] == scene.tasks[k][1]
    assert a.scene.tasks[k][0] != scene.tasks[k][0]


def test_floor_feed_quantities(planner, robot):
    """走地：插座高度（30 cm）的线盒整段人工开 26 cm；高处线盒的带内部分由机器人开."""
    scene = wall(1022, planner, robot)
    a = next(x for x in pa.alternatives(scene, robot, planner, tiers=('floor',)))
    ch = a.changes[0]
    start, goal = scene.tasks[ch.slot][:2]
    assert ch.conduit_m == pytest.approx(abs(goal[0] - start[0]) / 100)
    lo = math.ceil(robot.reach_band[0] / sp.RES)
    manual = robot_m = 0.0
    for x, y in (start, goal):
        if y - lo >= pa.MIN_ROBOT_STUB:
            manual, robot_m = manual + lo / 100, robot_m + (y - lo) / 100
        else:
            manual += (y - wg.BOX // 2) / 100
    assert ch.manual_m == pytest.approx(manual) and ch.robot_m == pytest.approx(robot_m)
    plan = pa.finalize(a, WALL, robot, planner)
    fixed = [s for s in plan.slots if s.planner.get('fixed')]
    assert len(fixed) == len(a.fixed)
    for s in fixed:                                       # 固定竖槽是一条竖直线
        assert len(s.cut_path) == 2 and s.cut_path[0][0] == s.cut_path[1][0]
    assert a.installer_cost == pytest.approx(pa.COST_MANUAL_PER_M * ch.manual_m +
                                             pa.COST_CONDUIT_PER_M * ch.conduit_m)


def test_stub_checker_detects_conflict(planner, robot):
    """独立检查器能发现压在引线竖槽上的槽线."""
    scene = wall(1022, planner, robot)
    a = next(x for x in pa.alternatives(scene, robot, planner, tiers=('floor',)))
    x, y0, y1, circ, k = a.stubs[0]
    bad = sp.Slot('bad', 'success', circuit=circ, planner={})
    bad.cut_path = sp.corners_to_m([(x - 20, max(y1 - 3, 1)), (x + 20, max(y1 - 3, 1))])
    assert pa.stub_violations(a, scene, [bad])['stub_slot'] >= 1


def test_plan_with_fallback_feasible_wall_unchanged(planner, robot):
    scene = wall(1121, planner, robot)
    plan, alt, alts = pa.plan_with_fallback(scene, WALL, robot, planner, start_u=1.0)
    ref = tp.plan_task_joint(scene, WALL, robot, planner, start_u=1.0)
    assert not alt.changes and not alts
    assert plan.prediction['t_total'] <= ref.prediction['t_total'] + 1e-6


# ---------------------------------------------------------------------------
# 地面与转场
# ---------------------------------------------------------------------------
def test_floor_map_distances():
    sq = [(0, 0), (6, 0), (6, 6), (0, 6)]
    f = rp.FloorMap(sq)
    assert f.dist((1, 1), (5, 4)) == pytest.approx(5.0)
    assert not f.is_free((0.2, 3.0)) and f.is_free((0.5, 3.0))
    g = rp.FloorMap(sq, [(2.5, 0.0, 1.0, 4.0, 'x')])       # 从南墙伸出的隔断
    d = g.dist((1.0, 1.0), (5.0, 1.0))
    # 绕过隔断上端（外扩 0.45 m）的真实最短路约为 2 × hypot(1.05, 3.45) + 1.9
    true = 2 * math.hypot(1.05, 3.45) + 1.9
    assert true * 0.98 <= d <= true * 1.09
    h = rp.FloorMap(sq, [(2.5, 0.0, 1.0, 6.0, 'x')])       # 完全隔开
    assert math.isinf(h.dist((1.0, 1.0), (5.0, 1.0)))


def test_transit_model():
    t = rp.TransitModel()
    assert t.leg(4.17) == pytest.approx(3.07 + 10.0 + 15.0)


# ---------------------------------------------------------------------------
# 顺序问题
# ---------------------------------------------------------------------------
def random_instance(rng, n):
    service = [[rng.uniform(20, 200) for _ in range(2)] for _ in range(n)]
    start = [[rng.uniform(5, 40) for _ in range(2)] for _ in range(n)]
    trans = [[[[0.0 if i == j else rng.uniform(5, 40) for _ in range(2)] for j in range(n)]
              for _ in range(2)] for i in range(n)]
    return rp.Instance(service, start, trans)


def test_dp_matches_brute_force():
    rng = random.Random(0)
    for trial in range(120):
        n = 1 + trial % 6
        inst = random_instance(rng, n)
        t, seq = rp.solve_dp(inst)
        tb, _ = rp.solve_brute(inst)
        assert t == pytest.approx(tb)
        assert inst.cost(seq) == pytest.approx(t)
        assert sorted(i for i, _ in seq) == list(range(n))
        for m in rp.METHODS:
            tm, sm = rp.solve(inst, m)
            assert tm >= t - 1e-9
            assert inst.cost(sm) == pytest.approx(tm)
        assert rp.solve(inst, 'fixed_order_best_dir')[0] <= rp.solve(inst, 'fixed')[0] + 1e-9
        assert rp.solve(inst, 'best_order_fixed_dir')[0] <= rp.solve(inst, 'fixed')[0] + 1e-9


# ---------------------------------------------------------------------------
# 房间生成与规划
# ---------------------------------------------------------------------------
def test_room_generator_reproducible(planner, robot):
    cb = tp.standalone_routable(planner, robot)
    a, b = rg.residential_room(7, routable=cb), rg.residential_room(7, routable=cb)
    assert rp.room_to_config(a) == rp.room_to_config(b)
    for seed in range(6):
        room = rg.residential_room(seed, routable=cb)
        n = len(room.walls)
        for i, w in enumerate(room.walls):                 # 顺时针首尾相接
            nxt = room.walls[(i + 1) % n]
            assert math.dist(w.frame.to_map(w.frame.length), nxt.frame.to_map(0.0)) < 1e-9
            mid = w.frame.to_map(w.frame.length / 2, 0.6)  # 法向指向房间内
            assert rp.FloorMap(room.boundary, radius=0.1).is_free(mid)
            for t in w.scene.tasks:
                for p in t[:2]:
                    assert rg.CORNER_KEEP <= p[0] <= w.scene.width - rg.CORNER_KEEP
        assert sum(w.has_tasks for w in room.walls) >= 2
        assert rp.FloorMap(room.boundary, room.floor_obstacles).is_free(room.start_pose[:2])


def test_room_config_round_trip(planner, robot):
    room = rg.residential_room(3, routable=tp.standalone_routable(planner, robot))
    cfg = yaml.safe_load(yaml.safe_dump(rp.room_to_config(room), allow_unicode=True))
    back = rp.room_from_config(cfg)
    assert [w.scene.tasks for w in back.walls] == [w.scene.tasks for w in room.walls]
    assert [w.scene.load_bearing for w in back.walls] == [w.scene.load_bearing for w in room.walls]
    assert len(back.links) == len(room.links)


def test_plan_random_room(planner, robot):
    room = rg.residential_room(2, routable=tp.standalone_routable(planner, robot))
    plan = rp.plan_room(room, robot, planner)
    t = plan.totals
    assert t['t_total'] == pytest.approx(t['t_service'] + t['t_nav'] + t['t_align'], abs=0.05)
    assert t['t_total'] == pytest.approx(rp.solve_brute(plan.instance)[0])
    assert all(v['t_total'] >= t['t_total'] - 1e-9 for v in plan.methods.values())
    assert any(r.status == 'fallback' for r in plan.walls)          # 这个房间有一面无解的墙
    for r in plan.walls:
        for d, o in r.options.items():
            assert sum(r.violations(robot, d).values()) == 0
            cs = [st.u_c for st in o.plan.stations]
            assert cs == sorted(cs, reverse=(d == '-'))
            assert o.blocked == 0
            assert o.service == pytest.approx(o.plan.prediction['t_total'], abs=0.06)


def empty_room(ua, ub, vb=0.55, load_bearing=False):
    w = lambda name, a, b, lb=False: dict(  # noqa: E731
        name=name, wall=dict(left_end=a, right_end=b, height=2.8, load_bearing=lb), tasks=[],
        obstacles=[])
    return rp.room_from_config(dict(
        name='t', start_pose=[2, 2, 0],
        walls=[w('s', [4, 0], [0, 0], load_bearing), w('w', [0, 0], [0, 4]),
               w('n', [0, 4], [4, 4]), w('e', [4, 4], [4, 0])],
        links=[{'from': dict(wall='s', box=[ua, 0.34]), 'to': dict(wall='w', box=[ub, vb])}]))


def test_corner_link_keeps_conduit_continuous(planner, robot):
    plan = rp.plan_room(empty_room(3.3, 0.9), robot, planner)
    note = plan.link_notes[0]
    assert note['method'] == 'corner' and note['manual_m'] == pytest.approx(0.8)
    ends = []
    for r in plan.walls:
        if not r.options:
            continue
        slot = r.options['+'].plan.slots[0]
        cells = tp.path_cells(slot)
        assert cells[-1][0] == cells[-2][0]                # 竖直到达过线口
        assert abs(cells[-1][1] - cells[-2][1]) >= 2 * 12  # 过线口处的弯两侧直段足够长
        ends.append(cells[-1])
        assert sum(r.violations(robot).values()) == 0
    assert len(ends) == 2 and ends[0][1] == ends[1][1] == note['h_cm']      # 两面墙同一高度
    assert {ends[0][0], ends[1][0]} == {rp.PORT_OFFSET, 400 - 1 - rp.PORT_OFFSET}


def test_corner_link_falls_back_to_floor(planner, robot):
    # 承重墙禁止横槽；线盒离墙角太远时横槽长度与弯曲半径也排除墙角过线
    for room in (empty_room(3.3, 0.9, load_bearing=True), empty_room(3.0, 1.2)):
        plan = rp.plan_room(room, robot, planner)
        note = plan.link_notes[0]
        assert note['method'] == 'floor'
        assert note['conduit_m'] == pytest.approx(
            (400 - room.links[0].box_a[0] + room.links[0].box_b[0]) / 100)
        w = next(r for r in plan.walls if r.options)       # 只有高处线盒的引上段由机器人开
        cells = tp.path_cells(w.options['+'].plan.slots[0])
        assert cells[0][0] == cells[1][0] == room.links[0].box_b[0]
        assert sum(w.violations(robot).values()) == 0


def test_room3_example(planner, robot, tmp_path):
    import plan_room
    plan, (summary, files) = plan_room.plan_file(ROOM3)
    assert summary['status'] == 'success'
    assert [w['status'] for w in summary['walls']] == ['success'] * 3
    assert summary['links'][0]['method'] == 'floor'        # 东墙为承重墙
    assert set(files) == {'south', 'east', 'north'}
    best = summary['methods']['optimal']['t_total']
    assert all(v['t_total'] >= best for v in summary['methods'].values())
    for name, d in files.items():
        text = yaml.safe_dump(d, allow_unicode=True, sort_keys=False)
        assert '/home/' not in text
        p, frame, _ = sp.plan_from_dict(yaml.safe_load(text))     # 与单面墙规划文件同一格式
        assert p.status == 'success' and p.stations
        for st in p.stations:
            for pt in st.points:
                assert robot.reachable(pt['x'], pt['y'], pt['z'], pt['rotation'])
    # 南墙的任务与单面墙示例相同，另加跨墙角回路的引上段
    assert len(files['south']['slots']) == 3
    assert sum(r.violations(robot)[k] for r in plan.walls if r.options
               for k in r.violations(robot)) == 0
