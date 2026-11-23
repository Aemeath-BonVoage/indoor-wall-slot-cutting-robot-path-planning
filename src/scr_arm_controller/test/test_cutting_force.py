"""切割力模型单元测试：参考数值、力的方向、静力学裕度、切削过程的时间历程."""
import math

import pytest

from scr_arm_controller import cutting_force as cf


def forces(f):
    return (f.tangential, f.normal, f.feed, f.wall, f.couple, f.power)


def test_reference_values():
    # 与 scr_description/test/test_cutting_force_model.cpp 相同的参考数值
    b = cf.BladeParams()
    assert b.cutting_speed == pytest.approx(73.042029, abs=1e-5)
    # 参考工况：切削功率 = u_0·b·a·v_f = 0.4e9 × 0.005 × 0.035 × 0.005 = 350 W
    assert forces(cf.tool_forces(b, 0.035, 0.005)) == pytest.approx(
        (4.791762, 19.167047, -14.84215, 13.040216, 0.359382, 350.0), abs=1e-5)
    assert forces(cf.tool_forces(b, 0.035, 0.002, 0.0, 1.3)) == pytest.approx(
        (3.133167, 12.532668, -9.704768, 8.526546, 0.234988, 228.852884), abs=1e-5)
    assert forces(cf.tool_forces(b, 0.020, 0.010, 0.001)) == pytest.approx(
        (7.214409, 28.857634, -15.029967, 24.894929, 0.541081, 526.955042), abs=1e-5)
    assert forces(cf.tool_forces(b, 0.010, 0.0, 0.002)) == pytest.approx(
        (4.261472, 17.04589, -4.261472, 17.04589, 0.31961, 311.266595), abs=1e-5)
    down = cf.BladeParams(up_cut=False)
    assert forces(cf.tool_forces(down, 0.035, 0.005)) == pytest.approx(
        (4.791762, 19.167047, -6.959443, 18.490615, -0.359382, 350.0), abs=1e-5)


def test_no_engagement_no_force():
    b = cf.BladeParams()
    assert cf.tool_forces(b, 0.0, 0.005).normal == 0.0
    assert cf.tool_forces(b, 0.035, 0.0).normal == 0.0
    assert cf.tool_forces(b, 0.035, 0.0, -0.01).normal == 0.0


def test_power_limited_feed():
    for name in cf.MATERIALS:
        b = cf.BladeParams.for_material(name)
        v = cf.max_feed_rate(b, cf.SLOT_DEPTH)
        power = cf.tool_forces(b, cf.SLOT_DEPTH, v).power
        assert power == pytest.approx(b.spindle_power, rel=1e-6)
    # 比切削能越高，功率允许的进给越慢
    v = [cf.max_feed_rate(cf.BladeParams.for_material(n), cf.SLOT_DEPTH)
         for n in ('aerated_block', 'brick', 'concrete')]
    assert v[0] > v[1] > v[2]


def test_base_wrench_horizontal_and_vertical():
    b, pf = cf.BladeParams(), cf.Platform()
    h = cf.cut_wrench(b, pf, 0.035, 0.005, 'horizontal', 1)
    assert (h.fx, h.fy, h.fz, h.mx, h.my, h.mz) == pytest.approx(
        (-13.04022, 14.84215, 0.0, -8.16318, -7.17212, 10.1552), abs=1e-4)
    # 偏航力矩 = 力臂（刀片中心）× 横向力 + 主轴反力偶
    arm = pf.standoff + 0.035 - b.radius
    f = cf.tool_forces(b, 0.035, 0.005)
    assert h.mz == pytest.approx(arm * h.fy + f.couple)
    # 反向进给：横向力与偏航力矩反号，推离墙面的力不变
    hb = cf.cut_wrench(b, pf, 0.035, 0.005, 'horizontal', -1)
    assert (hb.fx, hb.fy, hb.mz) == pytest.approx((h.fx, -h.fy, -h.mz))
    v = cf.cut_wrench(b, pf, 0.035, 0.005, 'vertical', 1)
    assert (v.fx, v.fy, v.fz, v.mz) == pytest.approx((-13.04022, 0.0, -14.84215, 0.0), abs=1e-4)
    # 竖槽：刀具横向偏置时，推离墙面的力产生偏航力矩
    off = cf.Platform(tool_y=0.06)
    vo = cf.cut_wrench(b, off, 0.035, 0.005, 'vertical', 1)
    assert vo.mz == pytest.approx(-0.06 * vo.fx)


def test_wheel_loads_and_friction():
    pf = cf.Platform()
    rest = cf.wheel_loads(cf.Wrench(), pf)
    assert sum(rest) == pytest.approx(pf.mass * cf.G)
    assert rest[0] > rest[2]            # 质心略靠前
    h = cf.cut_wrench(cf.BladeParams(), pf, 0.035, 0.005, 'horizontal', 1)
    m = cf.static_margins(h, pf)
    assert m.loads == pytest.approx([79.543, 63.012, 86.59, 70.059], abs=2e-3)
    assert m.mu_mecanum == pytest.approx(0.1663, abs=1e-4)
    assert m.mu_locked == pytest.approx(0.154, abs=1e-4)
    assert m.min_load_ratio == pytest.approx(0.8424, abs=1e-4)
    # 辊子轴向力与地面需要提供的合力平衡，且都不超过 μ·N
    f, mu = cf.roller_forces(h, pf)
    lx, ly = pf.half_length, pf.half_width
    pos = ((lx, ly), (lx, -ly), (-lx, ly), (-lx, -ly))
    assert sum(fi * r[0] for fi, r in zip(f, cf._ROLLER)) == pytest.approx(-h.fx)
    assert sum(fi * r[1] for fi, r in zip(f, cf._ROLLER)) == pytest.approx(-h.fy)
    assert sum(fi * (p[0] * r[1] - p[1] * r[0])
               for fi, r, p in zip(f, cf._ROLLER, pos)) == pytest.approx(-h.mz)
    assert max(abs(fi) / ni for fi, ni in zip(f, m.loads)) == pytest.approx(mu)


def test_pure_force_capacity_of_mecanum_base():
    # 纯横向力：四个轮各承担 F / (4·cos 45°)，可承受的最大力为 0.707·μ·M·g
    pf = cf.Platform(com_x=0.0)
    f = 50.0
    _, mu = cf.roller_forces(cf.Wrench(fy=f), pf)
    assert mu == pytest.approx(f / (math.sqrt(0.5) * pf.mass * cf.G))


def test_tipping_detected():
    pf = cf.Platform(tool_z=0.81)
    b = cf.BladeParams.for_material('concrete')
    m = cf.static_margins(cf.cut_wrench(b, pf, 0.035, 0.010, 'horizontal', 1), pf)
    assert m.min_load_ratio < 0.0 and math.isinf(m.mu_mecanum)


def test_cut_profile_phases():
    p = cf.CutProfile(kind='cut', depth=0.035, plunge_rate=0.002, feed_rate=0.005,
                      length=0.05, t_start=1.0)
    assert p.t_feed == pytest.approx(18.5) and p.t_end == pytest.approx(28.5)
    assert p.state(0.5) == (0.0, 0.0, 0.0, 0.0)
    depth, vf, vp, dist = p.state(10.0)
    assert (depth, vf, vp, dist) == pytest.approx((0.018, 0.0, 0.002, 0.0))
    depth, vf, vp, dist = p.state(20.5)
    assert (depth, vf, vp, dist) == pytest.approx((0.035, 0.005, 0.0, 0.010))
    assert p.state(29.0) == (0.0, 0.0, 0.0, 0.0)
    ramp = cf.CutProfile(kind='ramp', ramp_time=4.0, t_start=1.0)
    assert [ramp.force_scale(t) for t in (0.0, 3.0, 5.0, 9.0)] == pytest.approx(
        [0.0, 0.5, 1.0, 1.0])


def test_process_is_seeded_and_nominal_is_predictable():
    b, pf = cf.BladeParams(), cf.Platform()
    prof = cf.CutProfile(kind='step', feed_rate=0.005, length=0.05)
    ts = [1.0 + 0.02 * k for k in range(400)]
    a = [cf.CuttingProcess(b, pf, prof, seed=5).wrench(t).mz for t in ts[:1]]
    p1 = cf.CuttingProcess(b, pf, prof, seed=5)
    p2 = cf.CuttingProcess(b, pf, prof, seed=5)
    p3 = cf.CuttingProcess(b, pf, prof, seed=6)
    m1, m2, m3 = ([p.wrench(t).mz for t in ts] for p in (p1, p2, p3))
    assert m1 == m2 and m1 != m3 and a[0] == m1[0]
    # 不带种子：等于名义值；带种子：围绕名义值波动
    quiet = cf.CuttingProcess(b, pf, prof)
    nom = [quiet.nominal(t).mz for t in ts]
    assert [quiet.wrench(t).mz for t in ts] == pytest.approx(nom)
    ratio = [x / n for x, n in zip(m1, nom)]
    assert 0.5 < sum(ratio) / len(ratio) < 1.5 and max(ratio) - min(ratio) > 0.05


def test_inhomogeneity_statistics():
    g = cf.Inhomogeneity(cov=0.25, length=0.015, seed=7)
    xs = [g.advance(0.003) for _ in range(100000)]
    mean = sum(xs) / len(xs)
    sd = math.sqrt(sum((x - mean) ** 2 for x in xs) / len(xs))
    assert mean == pytest.approx(1.0, abs=0.01) and sd == pytest.approx(0.25, abs=0.01)
    assert cf.Inhomogeneity(cov=0.0).advance(0.01) == 1.0
