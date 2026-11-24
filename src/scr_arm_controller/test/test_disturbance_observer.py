"""定点保持控制律单元测试：模型离散化、观测器极点、稳态与抗饱和、前馈、卡尔曼形式."""
import math

import numpy as np
import pytest

from scr_arm_controller import disturbance_observer as do

T, K = 0.02, 20.0


def simulate(ctrl, d, n=600, k_true=K, delay=1, noise=None, measure_every=1, d_ff=0.0):
    """单轴闭环：输入端扰动 d(k)，测量延迟 delay 拍；返回 (偏差序列, 指令序列)."""
    m = do.AxisModel(k_true, T)
    x = np.zeros(2)
    ys = [0.0] * (delay + 1)
    es, us = [], []
    for k in range(n):
        dk = d(k) if callable(d) else d
        y = ys[0] + (noise[k] if noise is not None else 0.0)
        if measure_every > 1 and k % measure_every:
            y = None
        ff = d_ff(k) if callable(d_ff) else d_ff
        u = ctrl.update(y, ff) if ff else ctrl.update(y)
        x = m.A @ x + m.B * (u + dk)
        ys = ys[1:] + [x[0]]
        es.append(x[0])
        us.append(u)
    return np.array(es), np.array(us)


def make(mode, **kw):
    m = do.AxisModel(K, T)
    if mode == 'p':
        return do.AxisP(5.0, kw.get('u_max', math.inf))
    if mode == 'pi':
        return do.AxisPID(*do.pi_triple_pole(K), 0.0, T, **kw)
    if mode == 'pid':
        return do.AxisPID(*do.pid_pole_placement(K, 0.4 * K), T, **kw)
    return do.AxisDOB(m, 5.0, 0.0, gain=do.bandwidth_gain(m, 40.0), **kw)


def test_zoh_model_matches_continuous_integration():
    m = do.AxisModel(K, T)
    x = np.array([0.01, -0.02])
    u, d, h = 0.3, 0.1, 1e-5
    e, v = x
    for _ in range(int(round(T / h))):
        e, v = e + h * v, v + h * K * (u + d - v)
    assert m.A @ x + m.B * (u + d) == pytest.approx([e, v], abs=5e-5)
    assert m.Aa @ np.array([x[0], x[1], d]) + m.Ba * u == pytest.approx([e, v, d], abs=5e-5)


def test_observer_gain_places_poles():
    m = do.AxisModel(K, T)
    for poles in ([0.5, 0.5, 0.5], [0.6, 0.4 + 0.2j, 0.4 - 0.2j]):
        L = do.observer_gain(m, poles)
        eig = np.linalg.eigvals((np.eye(3) - np.outer(L, m.C)) @ m.Aa)
        assert np.poly(eig) == pytest.approx(np.real(np.poly(poles)), abs=1e-4)
    L = do.bandwidth_gain(m, 40.0)
    eig = np.linalg.eigvals((np.eye(3) - np.outer(L, m.C)) @ m.Aa)
    assert np.abs(eig) == pytest.approx([math.exp(-40.0 * T)] * 3, abs=5e-3)


def test_tuning_rules_give_the_designed_poles():
    kp, ki = do.pi_triple_pole(K)
    assert np.roots([1.0, K, K * kp, K * ki]) == pytest.approx([-K / 3] * 3, abs=1e-3)
    kp, ki, kd = do.pid_pole_placement(K, 8.0, 0.8)
    roots = sorted(np.roots([1.0, K * (1 + kd), K * kp, K * ki]), key=lambda z: z.imag)
    assert roots[1] == pytest.approx(-8.0)
    assert abs(roots[0]) == pytest.approx(8.0) and -roots[0].real / 8.0 == pytest.approx(0.8)


def test_steady_state_under_constant_disturbance():
    d = 0.05
    e, _ = simulate(make('p'), d)
    assert e[-1] == pytest.approx(d / 5.0, rel=1e-3)        # e_ss = d / k_p
    for mode in ('pi', 'pid', 'dob'):
        e, u = simulate(make(mode), d)
        assert abs(e[-1]) < 1e-6 and u[-1] == pytest.approx(-d, abs=1e-5), mode


def test_observer_estimates_the_disturbance_force():
    hold = do.PlanarHold('dob', dt=T, limits=(1.0, 1.0, 10.0))
    wrench = (12.0, -20.0, 5.0)
    d = hold.feedforward(wrench)
    assert d == pytest.approx((12.0 / (30.5 * K), -20.0 / (30.5 * K), 5.0 / (0.92 * K)))
    plants = [do.AxisModel(K, T) for _ in range(3)]
    xs = [np.zeros(2) for _ in range(3)]
    prev = (0.0, 0.0, 0.0)
    for _ in range(500):
        u = hold.update(prev)
        prev = tuple(x[0] for x in xs)
        xs = [m.A @ x + m.B * (ui + di) for m, x, ui, di in zip(plants, xs, u, d)]
    assert hold.disturbance_estimate() == pytest.approx(wrench, rel=1e-3)
    assert max(abs(x[0]) for x in xs) < 1e-6


@pytest.mark.parametrize('mode', ['pi', 'pid', 'dob'])
def test_state_space_matches_the_implementation(mode):
    ctrl = make(mode)
    A, B, C, D = do.state_space(ctrl)
    rng = np.random.default_rng(3)
    ys = rng.normal(0.0, 1e-3, 80)
    ctrl.reset()
    ctrl.set_state(np.zeros(len(A)))
    x = np.zeros(len(A))
    for y in ys:
        assert ctrl.update(y) == pytest.approx(float(C @ x + D * y), abs=1e-12)
        x = A @ x + B * y


@pytest.mark.parametrize('mode', ['p', 'pi', 'pid', 'dob'])
@pytest.mark.parametrize('ratio', [0.5, 1.0, 2.0])
def test_stable_with_delay_and_gain_mismatch(mode, ratio):
    e, _ = simulate(make(mode), lambda k: 0.05 if k >= 10 else 0.0, n=1500, k_true=K * ratio)
    assert np.all(np.isfinite(e))
    # 比例保持收敛到 d / k_p，其余收敛到 0
    assert e[-1] == pytest.approx(0.05 / 5.0 if mode == 'p' else 0.0, abs=1e-5)
    assert np.ptp(e[-200:]) < 1e-5


def test_dob_rejects_a_step_faster_than_pid_and_pi():
    peak = {m: np.abs(simulate(make(m), 0.05)[0]).max() for m in ('p', 'pi', 'pid', 'dob')}
    assert peak['dob'] < peak['pid'] < peak['pi'] < peak['p'] * 1.01


def test_anti_windup():
    # 扰动超过限幅 8 s 后消失：无抗饱和的 PI 积分累积，释放后回到 1 mm 内明显更慢
    def d(k):
        return 0.3 if 50 <= k < 450 else 0.0

    def recovery(ctrl):
        e, u = simulate(ctrl, d, n=3000)
        assert np.abs(u).max() <= 0.2 + 1e-12
        over = np.where(np.abs(e[450:]) > 1e-3)[0]
        return (over[-1] + 1) * T if len(over) else 0.0
    t_none = recovery(make('pi', u_max=0.2, anti_windup=False))
    t_aw = recovery(make('pi', u_max=0.2))
    t_dob = recovery(make('dob', u_max=0.2))
    assert t_aw < 0.5 * t_none and t_dob < 0.5 * t_none
    # 限幅期间观测器仍给出正确的扰动估计
    ctrl = make('dob', u_max=0.2)
    simulate(ctrl, 0.3, n=400)
    assert ctrl.disturbance == pytest.approx(0.3, rel=1e-3)


def test_feedforward_cancels_a_known_disturbance():
    def d(k):
        return 0.05 if k >= 50 else 0.0
    plain = np.abs(simulate(make('dob'), d)[0]).max()
    ctrl = make('dob')
    exact = np.abs(simulate(ctrl, d, d_ff=d)[0]).max()
    assert exact < 1e-9 and abs(ctrl.disturbance) < 1e-9
    # 前馈低估 30 %：观测器只需补上剩余部分，瞬态按比例减小
    ctrl = make('dob')
    part = np.abs(simulate(ctrl, d, d_ff=lambda k: 0.7 * d(k))[0]).max()
    assert part == pytest.approx(0.3 * plain, rel=0.05)
    assert ctrl.disturbance == pytest.approx(0.015, rel=1e-3)


def test_kalman_form_with_slow_noisy_measurements():
    m = do.AxisModel(50.0, T)
    sigma = 1.3e-3
    ctrl = do.AxisDOB(m, 2.0, 0.0, kalman=((0.0, (2e-4) ** 2, (3.5e-4) ** 2), sigma ** 2))
    rng = np.random.default_rng(1)
    noise = rng.normal(0.0, sigma, 3000)
    e, _ = simulate(ctrl, 0.003, n=3000, k_true=50.0, noise=noise, measure_every=5)
    assert abs(np.mean(e[-2000:])) < 3e-4            # 无稳态偏差（P 保持时为 d / k_p = 1.5 mm）
    assert np.std(e[-2000:]) < 0.6 * sigma           # 偏差的抖动小于单帧测量噪声
    assert ctrl.disturbance == pytest.approx(0.003, abs=1e-3)
    # 协方差保持对称正定
    assert np.allclose(ctrl.obs.P, ctrl.obs.P.T) and np.all(np.linalg.eigvalsh(ctrl.obs.P) > 0)


def test_lqr_lqi_and_kalman_designs():
    m = do.AxisModel(K, T)
    kp, kd = do.lqr_gains(m, 25.0, 1.0)
    assert kp > 0 and kd > 0
    assert np.abs(np.linalg.eigvals(m.A - np.outer(m.B, [kp, kd]))).max() < 1.0
    # 连续时间极限：k_p → sqrt(q / r)
    assert kp == pytest.approx(5.0, rel=0.06)
    kp, ki, kd = do.lqi_gains(m, 2500.0, 100.0, 1.0)
    assert min(kp, ki, kd) > 0
    L = do.kalman_gain(m, 2.0, 1.0)
    assert np.abs(np.linalg.eigvals((np.eye(3) - np.outer(L, m.C)) @ m.Aa)).max() < 1.0
    # 扰动噪声越大（越不信任模型），增益越高
    assert do.kalman_gain(m, 8.0, 1.0)[2] > L[2] > do.kalman_gain(m, 0.5, 1.0)[2]


def test_planar_hold_p_mode_is_the_existing_law():
    hold = do.PlanarHold('p')
    dev = (0.004, -0.03, 0.2)
    assert hold.update(dev) == pytest.approx((-0.02, 0.1, -0.5))     # −5·e、−6·e 并限幅
    assert hold.disturbance_estimate() == (0.0, 0.0, 0.0)


def test_planar_hold_modes_and_errors():
    for mode in do.MODES:
        hold = do.PlanarHold(mode)
        out = hold.update((1e-3, -1e-3, 1e-3))
        assert len(out) == 3 and all(math.isfinite(v) for v in out)
        hold.reset()
    with pytest.raises(ValueError):
        do.PlanarHold('mpc')
    with pytest.raises(ValueError):
        do.DisturbanceObserver(do.AxisModel(K, T))


def test_period_guard_accepts_the_design_rate_with_jitter():
    guard = do.PeriodGuard(0.02)
    t = 0.0
    for k in range(200):
        t += 0.02 + (0.004 if k % 7 == 0 else -0.001)
        assert guard.update(t)
    # 个别丢帧（间隔翻倍）不触发切换
    for _ in range(3):
        t += 0.02
        guard.update(t)
    t += 0.04
    assert guard.update(t)


def test_period_guard_rejects_a_slow_stream_and_recovers():
    guard = do.PeriodGuard(0.02)
    t = 0.0
    oks = []
    for _ in range(12):
        t += 0.1
        oks.append(guard.update(t))
    assert oks[:3] == [True, True, True]      # 间隔不足 3 个时按可用处理
    assert not any(oks[3:])
    assert abs(guard.median - 0.1) < 1e-9
    for _ in range(12):
        t += 0.02
        guard.update(t)
    assert guard.ok
