"""切削状态时间表单元测试：切深折算、接触点拆分、预计力旋量与切割力模型的一致性."""
import math

import pytest

from scr_arm_controller import arm_trajectory as at
from scr_arm_controller import cut_schedule as cs
from scr_arm_controller import cutting_force as cf

X_RETRACT, DEPTH = 0.52, 0.035
X_REAL = 0.70 + DEPTH - cs.TOOL_OFFSET          # 真实切割时切深处的伸缩轴位置
X_DRY = 0.70 - 0.010 - cs.TOOL_OFFSET           # 空切：刀尖停在墙前 10 mm


def joints(x, y, z, rot):
    return [z - cs.ARM_Z0, -(y - cs.ARM_Y0), -(x - cs.ARM_X0), rot]


def station(x_cut, rot=math.pi / 2, dy=-0.05, dz=0.0):
    """一刀：退刀位置 → 下刀 → 切 → 退刀；返回 (点列, 切深标注)."""
    y0, z0 = 0.05, 0.40
    pts = [(X_RETRACT, y0, z0, rot), (x_cut, y0, z0, rot),
           (x_cut, y0 + dy, z0 + dz, rot), (X_RETRACT, y0 + dy, z0 + dz, rot)]
    return pts, [False, True, True, False]


def schedule(pts, is_cut, lim, **kw):
    traj = at.plan_trajectory(joints(*pts[0]), [joints(*p) for p in pts], is_cut, lim)
    return cs.CutSchedule(traj, slot_depth=DEPTH, **kw)


def test_tool_position_inverts_the_arm_kinematics():
    assert cs.tool_position(joints(0.55, 0.03, 0.4, 0.0)) == pytest.approx((0.55, 0.03, 0.4))


def test_real_cut_engages_over_the_last_slot_depth_of_the_plunge():
    pts, is_cut = station(X_REAL)
    sch = schedule(pts, is_cut, at.MotionLimits(feed=0.005, plunge=0.002))
    assert sch.travel_ratio == pytest.approx(1.0)
    plunge = next(s for s in sch.traj.segments if s.kind == 'plunge')
    # 下刀行程 55.6 mm，前 20.6 mm 在墙外
    assert sch.state(plunge.t0 + 1.0).depth == 0.0
    mid = sch.state(plunge.t0 + plunge.motion_time - 5.0)
    assert 0.0 < mid.depth < DEPTH
    assert mid.plunge == pytest.approx(0.002, rel=1e-6)
    assert sch.state(plunge.t1).depth == pytest.approx(DEPTH)


def test_dry_run_maps_the_whole_plunge_onto_the_slot_depth():
    pts, is_cut = station(X_DRY)
    ratio = cs.travel_ratio(pts, is_cut, DEPTH)
    assert ratio == pytest.approx((X_DRY - X_RETRACT) / DEPTH)
    sch = schedule(pts, is_cut, at.MotionLimits(feed=0.005, plunge=0.002 * ratio))
    assert sch.travel_ratio == pytest.approx(ratio)
    plunge = next(s for s in sch.traj.segments if s.kind == 'plunge')
    assert sch.state(0.0).depth == 0.0
    mid = sch.state(plunge.t0 + 0.5 * plunge.motion_time)
    assert mid.depth == pytest.approx(0.5 * DEPTH, rel=1e-3)
    assert mid.plunge == pytest.approx(0.002, rel=1e-3)      # 折算后的切深变化率
    assert plunge.motion_time == pytest.approx(DEPTH / 0.002, rel=0.05)


def test_no_engagement_outside_the_trajectory_or_without_cuts():
    pts, is_cut = station(X_DRY)
    sch = schedule(pts, is_cut, at.MotionLimits())
    assert sch.state(-1.0).depth == 0.0
    assert sch.state(sch.duration + 1.0).depth == 0.0
    assert sch.wrench(sch.duration + 1.0).planar == (0.0, 0.0, 0.0)
    free = schedule(pts, [False] * 4, at.MotionLimits())
    assert free.x_deep is None and free.state(1.0).depth == 0.0


@pytest.mark.parametrize('dy, sign', [(-0.05, 1), (0.05, -1)])
def test_expected_wrench_matches_the_force_model_for_a_horizontal_cut(dy, sign):
    pts, is_cut = station(X_DRY, dy=dy)
    sch = schedule(pts, is_cut, at.MotionLimits(feed=0.005, plunge=0.002))
    cut = next(s for s in sch.traj.segments if s.kind == 'cut')
    t = cut.t0 + 0.5 * cut.motion_time
    st = sch.state(t)
    assert st.depth == pytest.approx(DEPTH)
    assert st.orientation == 'horizontal' and st.feed_sign == sign
    assert abs(st.feed) == pytest.approx(0.005, rel=1e-6)
    y_mid = 0.05 + 0.5 * dy
    ref = cf.cut_wrench(cf.BladeParams(), cf.Platform(tool_y=y_mid), DEPTH, 0.005,
                        'horizontal', sign)
    assert sch.wrench(t).planar == pytest.approx(ref.planar, rel=1e-6, abs=1e-9)
    # 沿墙向右（−y）切：进给反力向左（+y）；偏航力矩以它为主
    assert sch.wrench(t).fy * sign > 0.0


def test_vertical_cut_gives_no_lateral_force():
    pts, is_cut = station(X_DRY, rot=0.0, dy=0.0, dz=0.1)
    sch = schedule(pts, is_cut, at.MotionLimits(feed=0.005, plunge=0.002))
    cut = next(s for s in sch.traj.segments if s.kind == 'cut')
    w = sch.wrench(cut.t0 + 0.5 * cut.motion_time)
    assert sch.state(cut.t0 + 1.0).orientation == 'vertical'
    assert w.fy == pytest.approx(0.0, abs=1e-9)
    assert w.fx < 0.0                                   # 把底盘推离墙面
    assert w.mz == pytest.approx(-0.05 * w.fx, rel=1e-6)    # 只有横向偏置产生的力矩


def test_feed_ramp_builds_the_force_gradually():
    pts, is_cut = station(X_DRY)
    step = schedule(pts, is_cut, at.MotionLimits(feed=0.005, plunge=0.002))
    ramp = schedule(pts, is_cut, at.MotionLimits(feed=0.005, plunge=0.002, feed_ramp_time=2.0))
    for sch, early in ((step, True), (ramp, False)):
        cut = next(s for s in sch.traj.segments if s.kind == 'cut')
        w = abs(sch.wrench(cut.t0 + 0.3).fy)
        full = abs(sch.wrench(cut.t0 + 0.5 * cut.motion_time).fy)
        assert (w > 0.9 * full) == early
    # 稳态进给时的力不变，变的只是建立的快慢
    assert full == pytest.approx(14.84, abs=0.01)


def test_split_plunges_inserts_a_stop_at_contact_only_when_needed():
    pts, is_cut = station(X_REAL)
    p2, c2 = cs.split_plunges(pts, is_cut, DEPTH, margin=0.002)
    assert len(p2) == len(pts) + 1 and c2 == [False, False, True, True, False]
    assert p2[1][0] == pytest.approx(X_REAL - DEPTH - 0.002)
    assert p2[1][1:] == pts[1][1:]
    # 空切：退刀位置已经在接触点之后，不插入
    pts, is_cut = station(X_DRY)
    assert cs.split_plunges(pts, is_cut, DEPTH) == ([tuple(p) for p in pts], is_cut)
    # 拆分后切入从静止开始：下刀段的起点就是接触点，切深从 0 连续增加
    sch = schedule(p2, c2, at.MotionLimits(feed=0.005, plunge=0.002, feed_ramp_time=2.0))
    plunge = next(s for s in sch.traj.segments if s.kind == 'plunge')
    assert sch.state(plunge.t0).depth == pytest.approx(0.0, abs=1e-6)
    assert sch.wrench(plunge.t0 + 0.1).fx > -2.0


def test_rapid_moves_never_engage_even_beyond_the_contact_position():
    # 机械臂从伸出位置（比空切的最深位置还靠前）回到退刀位置：这一段不算切入
    pts, is_cut = station(X_DRY)
    start = joints(X_DRY + 0.006, 0.0, 0.5, math.pi / 2)
    traj = at.plan_trajectory(start, [joints(*p) for p in pts], is_cut,
                              at.MotionLimits(feed=0.005, plunge=0.002))
    sch = cs.CutSchedule(traj, slot_depth=DEPTH)
    first = traj.segments[0]
    assert first.kind == 'rapid' and first.motion_time > 0.0
    for k in range(11):
        t = first.t0 + first.motion_time * k / 10.0 - 1e-6
        assert sch.state(t).depth == 0.0
        assert sch.wrench(t).planar == (0.0, 0.0, 0.0)
