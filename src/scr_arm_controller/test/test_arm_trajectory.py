"""机械臂切割轨迹时间参数化（arm_trajectory）的单元测试：限值、拐点、时长、结点."""
import math
import os
import re

import numpy as np
import pytest

import yaml

from scr_arm_controller.arm_trajectory import (
    ARM_MOTION_PARAMS, AxisLimit, limits_from_params, LINEAR, MotionLimits, path_limit,
    plan_trajectory, rest_to_rest, rest_to_rest_time, segment_kinds, segment_time)

# 一个带拐点的停靠站（关节空间：j1 = Z、j2 = Y、j3 = X、j4 = 刀片角）：
# 换位 → 下刀 → 横切 → 退刀 → 转刀 → 下刀 → 竖切 → 退刀
START = (-0.15, 0.0, 0.02, 0.0)
RETRACT, DEPTH = 0.0168, 0.0062
WAYPOINTS = [
    (-0.2643, -0.07, RETRACT, math.pi / 2),
    (-0.2643, -0.07, DEPTH, math.pi / 2),
    (-0.2643, 0.02, DEPTH, math.pi / 2),
    (-0.2643, 0.02, RETRACT, math.pi / 2),
    (-0.2643, 0.02, RETRACT, 0.0),
    (-0.2643, 0.02, DEPTH, 0.0),
    (-0.0243, 0.02, DEPTH, 0.0),
    (-0.0243, 0.02, RETRACT, 0.0),
]
IS_CUT = [False, True, True, False, False, True, True, False]
KINDS = ['rapid', 'plunge', 'cut', 'retract', 'rapid', 'plunge', 'cut', 'retract']


def profile_states(phases, n=400):
    """按阶段解析积分，返回密集采样的 (t, s, v, a)."""
    out, t, s, v = [], 0.0, 0.0, 0.0
    for dt, a0, j in phases:
        for d in np.linspace(0.0, dt, max(2, int(n * dt) + 2)):
            out.append((t + d, s + v * d + a0 * d * d / 2 + j * d ** 3 / 6,
                        v + a0 * d + j * d * d / 2, a0 + j * d))
        s += v * dt + a0 * dt * dt / 2 + j * dt ** 3 / 6
        v += a0 * dt + j * dt * dt / 2
        t += dt
    return np.array(out)


@pytest.mark.parametrize('jerk', [0.0, 10.0, 200.0])
@pytest.mark.parametrize('length', [1e-4, 0.003, 0.0106, 0.05, 0.12, 0.55, 2.0])
@pytest.mark.parametrize('v,a', [(0.05, 0.5), (0.25, 1.0), (0.25, 4.0), (2.5, 12.0)])
def test_rest_to_rest_reaches_length_within_limits(length, v, a, jerk):
    lim = AxisLimit(v, a, jerk)
    phases = rest_to_rest(length, lim)
    st = profile_states(phases)
    tol = 1e-9
    assert st[-1, 1] == pytest.approx(length, rel=1e-9, abs=1e-12)
    assert st[-1, 2] == pytest.approx(0.0, abs=1e-9)
    assert st[:, 2].max() <= v + tol and st[:, 2].min() >= -tol       # 不超速、不倒退
    assert np.abs(st[:, 3]).max() <= a + tol
    if jerk > 0.0:
        assert st[-1, 3] == pytest.approx(0.0, abs=1e-9)             # 末端加速度回零
        assert all(abs(j) <= jerk + tol for _, _, j in phases)
        # 加速度连续：每个阶段的起点加速度等于上一阶段的终点加速度
        acc = 0.0
        for dt, a0, j in phases:
            assert a0 == pytest.approx(acc, abs=1e-9)
            acc = a0 + j * dt


def test_rest_to_rest_time_closed_form():
    """行程足够长时：T = L / v + v / a + a / j（梯形没有最后一项）."""
    s = AxisLimit(0.05, 0.5, 10.0)
    assert rest_to_rest_time(0.24, s) == pytest.approx(0.24 / 0.05 + 0.05 / 0.5 + 0.5 / 10.0)
    assert len(rest_to_rest(0.24, s)) == 7
    t = AxisLimit(0.05, 0.5)
    assert rest_to_rest_time(0.24, t) == pytest.approx(0.24 / 0.05 + 0.05 / 0.5)
    assert len(rest_to_rest(0.24, t)) == 3
    # 短行程达不到 v：梯形退化为三角形 T = 2·sqrt(L / a)
    assert rest_to_rest_time(1e-3, t) == pytest.approx(2 * math.sqrt(1e-3 / 0.5))
    assert rest_to_rest(0.0, s) == []


def test_rest_to_rest_time_is_monotone():
    """时长随行程单调增加，随任一限值放宽单调不增."""
    lim = AxisLimit(0.25, 1.0, 20.0)
    ts = [rest_to_rest_time(x, lim) for x in np.linspace(1e-4, 0.6, 300)]
    assert all(b > a for a, b in zip(ts[:-1], ts[1:]))
    for length in (0.005, 0.05, 0.5):
        base = rest_to_rest_time(length, lim)
        assert rest_to_rest_time(length, AxisLimit(0.5, 1.0, 20.0)) <= base + 1e-12
        assert rest_to_rest_time(length, AxisLimit(0.25, 2.0, 20.0)) <= base + 1e-12
        assert rest_to_rest_time(length, AxisLimit(0.25, 1.0, 40.0)) <= base + 1e-12
        assert rest_to_rest_time(length, AxisLimit(0.25, 1.0)) <= base + 1e-12   # 梯形


def test_segment_kinds():
    assert segment_kinds(IS_CUT) == KINDS
    assert segment_kinds([True, False], start_at_depth=True) == ['cut', 'retract']


def test_limits_reject_invalid_values():
    with pytest.raises(ValueError):
        MotionLimits(feed=0.0)
    with pytest.raises(ValueError):
        MotionLimits(jerk_time=-0.1)
    with pytest.raises(ValueError):
        MotionLimits(rapid_accel_lateral=-1.0)
    with pytest.raises(ValueError):
        plan_trajectory(START, WAYPOINTS, IS_CUT[:-1])


@pytest.mark.parametrize('lim', [MotionLimits(), MotionLimits(jerk_time=0.0),
                                 MotionLimits(feed=0.2, rapid_accel=4.0, dwell=0.1)])
def test_trajectory_hits_waypoints_and_stops(lim):
    traj = plan_trajectory(START, WAYPOINTS, IS_CUT, lim)
    assert [s.kind for s in traj.segments] == KINDS
    assert traj.segments[0].t0 == 0.0
    for seg, wp, prev in zip(traj.segments, WAYPOINTS, [START] + WAYPOINTS[:-1]):
        assert seg.q0 == pytest.approx(prev) and seg.q1 == pytest.approx(wp)
        # 段末（含下刀后的 dwell）精确停在目标点
        for t in (seg.t0 + seg.motion_time, seg.t1):
            q, qd, qdd = traj.sample(t)
            assert q == pytest.approx(wp, abs=1e-12)
            assert qd == pytest.approx([0.0] * 4, abs=1e-9)
        dwell = lim.dwell if seg.kind == 'plunge' else 0.0      # 只在下刀到位后等待
        assert seg.t1 - seg.t0 == pytest.approx(
            segment_time(seg.kind, seg.q0, seg.q1, lim) + dwell)
    # 段首尾相接，总时长 = 各段之和
    for a, b in zip(traj.segments[:-1], traj.segments[1:]):
        assert b.t0 == pytest.approx(a.t1)
    assert traj.duration == pytest.approx(sum(s.t1 - s.t0 for s in traj.segments))


@pytest.mark.parametrize('lim', [MotionLimits(), MotionLimits(jerk_time=0.0),
                                 MotionLimits(feed=0.02, plunge=0.01, rapid_speed=0.4)])
def test_trajectory_respects_limits(lim):
    traj = plan_trajectory(START, WAYPOINTS, IS_CUT, lim)
    tol = 1e-9
    for seg in traj.segments:
        moving = [i for i in range(4) if abs(seg.q1[i] - seg.q0[i]) > 1e-9]
        if seg.kind in ('cut', 'plunge', 'retract'):
            assert len(moving) == 1                 # 切割、下刀、退刀只动一个轴
        for tau in np.linspace(0.0, seg.motion_time, 200):
            q, qd, qdd = seg.state(tau)
            for i in range(4):
                ax = lim.axis(i)
                assert abs(qd[i]) <= ax.v + tol and abs(qdd[i]) <= ax.a + tol
                if i not in moving:
                    assert q[i] == seg.q0[i] and qd[i] == 0.0
            tip = math.sqrt(sum(qd[i] ** 2 for i in LINEAR))
            tip_acc = math.sqrt(sum(qdd[i] ** 2 for i in LINEAR))
            if seg.kind == 'cut':
                assert tip <= lim.feed + tol and tip_acc <= lim.feed_accel + tol
            elif seg.kind == 'plunge':
                assert tip <= lim.plunge + tol and tip_acc <= lim.feed_accel + tol
    # 切割段在行程够长时确实以进给速度匀速走
    cut = [s for s in traj.segments if s.kind == 'cut'][1]
    assert math.sqrt(sum(v * v for v in cut.state(cut.motion_time / 2)[1])) == \
        pytest.approx(lim.feed)


def test_multi_axis_move_is_synchronised():
    """多轴同动：各轴同时起停，由最慢的轴决定时长，关节空间走直线."""
    lim = MotionLimits()
    q0, q1 = (0.0, 0.0, 0.0, 0.0), (0.2, -0.05, 0.0, math.pi / 2)
    seg = plan_trajectory(q0, [q1], [False], lim).segments[0]
    single = [segment_time('rapid', q0, tuple(q1[k] if k == i else 0.0 for k in range(4)), lim)
              for i in range(4)]
    assert seg.motion_time == pytest.approx(max(single))
    pl = path_limit('rapid', q0, q1, lim)
    assert pl.v == pytest.approx(min(0.25 / 0.2, 0.25 / 0.05, 2.5 / (math.pi / 2)))
    for tau in np.linspace(0.0, seg.motion_time, 50):
        q, _, _ = seg.state(tau)
        s = q[0] / q1[0]
        assert q == pytest.approx([s * v for v in q1], abs=1e-12)


def test_lateral_axis_has_its_own_acceleration_limit():
    """沿墙轴（joint2）的加速度可以单独限制；其他轴、沿其他轴的段不受影响."""
    base, soft = MotionLimits(), MotionLimits(rapid_accel_lateral=0.25)
    assert soft.axis(1).a == 0.25 and soft.axis(0).a == base.axis(0).a == 1.0
    assert soft.axis(1).j == pytest.approx(0.25 / soft.jerk_time)
    q0, q1 = (0.0, 0.0, 0.0, 0.0), (0.0, 0.12, 0.0, 0.0)
    seg = plan_trajectory(q0, [q1], [False], soft).segments[0]
    assert seg.motion_time > plan_trajectory(q0, [q1], [False], base).segments[0].motion_time
    a, b = (plan_trajectory(START, WAYPOINTS, IS_CUT, lim) for lim in (base, soft))
    for sa, sb in zip(a.segments, b.segments):
        if abs(sa.q1[1] - sa.q0[1]) < 1e-9:
            assert sb.motion_time == pytest.approx(sa.motion_time)
        else:
            assert sb.motion_time >= sa.motion_time - 1e-12
        # 沿墙轴在任何段里（空行程或横切）都不超过它自己的加速度上限
        for tau in np.linspace(0.0, sb.motion_time, 200):
            assert abs(sb.state(tau)[2][1]) <= 0.25 + 1e-9
    # 上限不低于切割加速度时，切割段与原来相同
    c = plan_trajectory(START, WAYPOINTS, IS_CUT, MotionLimits(rapid_accel_lateral=0.5))
    for sa, sc in zip(a.segments, c.segments):
        if sa.kind in ('cut', 'plunge'):
            assert sc.motion_time == pytest.approx(sa.motion_time)


def test_parameter_defaults_are_consistent():
    """ROS 参数默认值、MotionLimits 默认值与 config/arm_motion.yaml 三者一致."""
    assert limits_from_params(ARM_MOTION_PARAMS) == MotionLimits()
    assert limits_from_params(dict(ARM_MOTION_PARAMS, rapid_accel_lateral=0.5)) == \
        MotionLimits(rapid_accel_lateral=0.5)
    path = os.path.join(os.path.dirname(__file__), '..', 'config', 'arm_motion.yaml')
    with open(path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)['/**']['ros__parameters']['arm_motion']
    assert cfg == ARM_MOTION_PARAMS


def test_repeated_point_takes_no_time():
    wps = [WAYPOINTS[0], WAYPOINTS[0], WAYPOINTS[1]]
    traj = plan_trajectory(START, wps, [False, False, True])
    assert len(traj.segments) == 3
    assert traj.segments[1].t1 == traj.segments[1].t0
    ts = [k[0] for k in traj.knots()]
    assert all(b > a for a, b in zip(ts[:-1], ts[1:]))
    assert traj.sample(traj.duration)[0] == pytest.approx(WAYPOINTS[1])


def _hermite(a, b, t):
    """与控制器相同的样条：两端给位置 + 速度（三次）或再加加速度（五次）."""
    (ta, qa, va, aa), (tb, qb, vb, ab) = a, b
    T, d = tb - ta, t - ta
    out = []
    for i in range(4):
        if aa is None:
            c0, c1 = qa[i], va[i]
            c2 = (3 * (qb[i] - qa[i]) - (2 * va[i] + vb[i]) * T) / T ** 2
            c3 = (-2 * (qb[i] - qa[i]) + (va[i] + vb[i]) * T) / T ** 3
            out.append(c0 + c1 * d + c2 * d ** 2 + c3 * d ** 3)
            continue
        c0, c1, c2 = qa[i], va[i], aa[i] / 2
        A = np.array([[T ** 3, T ** 4, T ** 5], [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                      [6 * T, 12 * T ** 2, 20 * T ** 3]])
        rhs = np.array([qb[i] - c0 - c1 * T - c2 * T * T, vb[i] - c1 - 2 * c2 * T,
                        ab[i] - 2 * c2])
        c3, c4, c5 = np.linalg.solve(A, rhs)
        out.append(c0 + c1 * d + c2 * d ** 2 + c3 * d ** 3 + c4 * d ** 4 + c5 * d ** 5)
    return out


@pytest.mark.parametrize('lim', [MotionLimits(), MotionLimits(jerk_time=0.0),
                                 MotionLimits(dwell=0.2)])
def test_knots_reproduce_profile_under_spline_interpolation(lim):
    """结点之间按控制器的样条插值，与解析曲线一致（不需要加密采样）."""
    traj = plan_trajectory(START, WAYPOINTS, IS_CUT, lim)
    knots = traj.knots()
    assert knots[0][0] == 0.0 and knots[0][1] == pytest.approx(START)
    assert knots[-1][0] == pytest.approx(traj.duration)
    assert knots[-1][1] == pytest.approx(WAYPOINTS[-1], abs=0.0)
    assert all((k[3] is None) == (lim.jerk_time == 0.0) for k in knots)
    assert len(knots) <= 7 * len(WAYPOINTS) + 1 + (KINDS.count('plunge') if lim.dwell else 0)
    worst = 0.0
    for a, b in zip(knots[:-1], knots[1:]):
        assert b[0] - a[0] >= 1e-6
        for t in np.linspace(a[0], b[0], 9):
            q = _hermite(a, b, t)
            worst = max(worst, max(abs(x - y) for x, y in zip(q, traj.sample(t)[0])))
    assert worst < 1e-9
    # 每个目标点都是一个速度为零的结点
    for seg in traj.segments:
        k = min(knots, key=lambda kn: abs(kn[0] - (seg.t0 + seg.motion_time)))
        assert k[1] == pytest.approx(seg.q1, abs=0.0) and k[2] == [0.0] * 4


def test_summary_and_cut_time_formula():
    """切割段时长 = 长度 / 进给 + 每段的加减速开销；汇总的路径长度与点列一致."""
    lim = MotionLimits()
    traj = plan_trajectory(START, WAYPOINTS, IS_CUT, lim)
    n, t, length = traj.summary()['cut']
    assert n == 2 and length == pytest.approx(0.09 + 0.24)
    ramp = lim.feed / lim.feed_accel + lim.jerk_time
    assert t == pytest.approx(length / lim.feed + n * ramp)
    assert traj.summary()['plunge'][2] == pytest.approx(2 * (RETRACT - DEPTH))


def test_default_limits_below_urdf_limits():
    """默认限值不超过 URDF 中各关节的速度上限."""
    urdf = os.path.join(os.path.dirname(__file__), '..', '..', 'scr_description', 'urdf',
                        'scr_robot_gazebo.urdf')
    if not os.path.exists(urdf):
        pytest.skip('URDF not available next to this package')
    text = open(urdf, encoding='utf-8').read()
    lim = MotionLimits()
    for i in range(4):
        m = re.search(rf'<joint name="joint{i + 1}".*?<limit[^>]*velocity="([\d.]+)"', text,
                      re.S)
        assert m and lim.axis(i).v < float(m.group(1))
    assert lim.feed <= lim.rapid_speed and lim.plunge <= lim.rapid_speed


def test_feed_ramp_time_limits_cut_and_plunge_acceleration():
    lim = MotionLimits(feed=0.005, plunge=0.002, feed_ramp_time=2.0)
    assert lim.path('cut').a == pytest.approx(0.0025)
    assert lim.path('plunge').a == pytest.approx(0.001)
    assert MotionLimits(feed=0.005).path('cut').a == MotionLimits.feed_accel
    # 空行程不受影响；进给很快时仍以 feed_accel 为准
    assert lim.axis(0).a == MotionLimits.rapid_accel
    assert MotionLimits(feed=2.0, feed_ramp_time=1.0).path('cut').a == 0.5
    with pytest.raises(ValueError):
        MotionLimits(feed_ramp_time=-1.0)
    # 一段 100 mm 的切割：加减速各约 2 s，总时间 ≈ 长度 / 进给 + 建立时间
    t = rest_to_rest_time(0.1, lim.path('cut'))
    assert t == pytest.approx(0.1 / 0.005 + 2.0, abs=0.1)


def test_cut_dwell_waits_before_retracting():
    base = plan_trajectory(START, WAYPOINTS, IS_CUT, MotionLimits(dwell=0.0))
    held = plan_trajectory(START, WAYPOINTS, IS_CUT, MotionLimits(dwell=0.0, cut_dwell=1.0))
    cuts = [s for s in held.segments if s.kind == 'cut']
    assert cuts and all(s.dwell == 1.0 for s in cuts)
    assert all(s.dwell == 0.0 for s in held.segments if s.kind != 'cut')
    assert held.duration == pytest.approx(base.duration + len(cuts))
    # 等待期间机械臂停在切完的位置
    q, qd, _ = held.sample(cuts[0].t1 - 0.5)
    assert q == pytest.approx(list(cuts[0].q1)) and qd == pytest.approx([0.0] * 4)
