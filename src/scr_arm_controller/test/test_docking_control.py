"""docking_control 单元测试：平方根整形律、离散闭环收敛、坐标变换、收敛判据."""
import math

import numpy as np
import pytest

from scr_arm_controller.docking_control import (ConvergenceMonitor, WallAlignController,
                                                rate_limit, sqrt_controller)


def test_sqrt_controller_shape():
    k, a = 2.0, 0.15
    e0 = a / k ** 2
    assert sqrt_controller(0.0, k, a) == 0.0
    assert sqrt_controller(1e-4, k, a) == pytest.approx(k * 1e-4)
    # 线性区与平方根区在 e0 处连续且斜率连续
    lo, hi = sqrt_controller(e0 - 1e-7, k, a), sqrt_controller(e0 + 1e-7, k, a)
    assert lo == pytest.approx(hi, abs=1e-6)
    slope = (sqrt_controller(e0 + 1e-5, k, a) - sqrt_controller(e0 - 1e-5, k, a)) / 2e-5
    assert slope == pytest.approx(k, rel=1e-3)
    # 奇函数、单调、扇区 0 < v/e ≤ k
    es = np.linspace(1e-4, 1.0, 200)
    vs = np.array([sqrt_controller(e, k, a) for e in es])
    assert np.all(np.diff(vs) > 0)
    assert np.all(vs / es <= k + 1e-9)
    assert sqrt_controller(-0.3, k, a) == pytest.approx(-sqrt_controller(0.3, k, a))
    # 远处满足恒减速度制动：v² ≈ 2a(e − e0/2)
    assert sqrt_controller(0.5, k, a) ** 2 == pytest.approx(2 * a * (0.5 - e0 / 2))


def test_rate_limit():
    assert rate_limit(1.0, 0.0, 0.1) == pytest.approx(0.1)
    assert rate_limit(-1.0, 0.0, 0.1) == pytest.approx(-0.1)
    assert rate_limit(0.05, 0.0, 0.1) == pytest.approx(0.05)


def simulate(start, T=0.05, tau=0.05, steps=400):
    """墙面系积分器 + 一阶速度滞后 + 一拍延迟（与 docs 中的分析模型一致）."""
    ctrl = WallAlignController(dt=T)
    X = np.array(start, dtype=float)
    v = np.zeros(3)
    pending = np.zeros(3)
    a = math.exp(-T / tau)
    hist = []
    for _ in range(steps):
        cmd = np.array(ctrl.command(X, (0.7, 0.0, 0.0)))
        v = a * v + (1 - a) * pending
        pending = cmd
        c, s = math.cos(X[2]), math.sin(X[2])
        vx, vy, w = v
        X = X + T * np.array([-(vx * c - vy * s), -(vx * s + vy * c), w])
        hist.append(X.copy())
    return np.array(hist)


@pytest.mark.parametrize('start', [(1.2, 0.3, math.radians(10)), (1.0, -0.25, -0.15),
                                   (0.705, 0.002, 0.0)])
def test_closed_loop_converges_without_overshoot(start):
    h = simulate(start)
    assert abs(h[-1, 0] - 0.7) < 1e-4 and abs(h[-1, 1]) < 1e-4 and abs(h[-1, 2]) < 1e-4
    # 离墙方向不越过目标（越过即刀具朝墙多走）
    assert np.min(h[:, 0]) > 0.7 - 1e-3


def test_command_is_rotated_into_base_frame():
    ctrl = WallAlignController(acc_lin=1e3, acc_ang=1e3)
    # 正对墙面、离墙偏远：只向前
    vx, vy, wz = ctrl.command((0.8, 0.0, 0.0), (0.7, 0.0, 0.0))
    assert vx > 0 and abs(vy) < 1e-12 and wz == 0.0
    # 偏右（u > 0）：向左（+y）
    ctrl.reset()
    vx, vy, wz = ctrl.command((0.7, 0.05, 0.0), (0.7, 0.0, 0.0))
    assert vy > 0 and abs(vx) < 1e-12
    # 左转 90°（ψ = π/2）时“向墙”在机体系中是 −y
    ctrl.reset()
    vx, vy, wz = ctrl.command((0.8, 0.0, math.pi / 2), (0.7, 0.0, math.pi / 2))
    assert vy < 0 and abs(vx) < 1e-12
    # 逆时针偏转 → 顺时针转回
    ctrl.reset()
    assert ctrl.command((0.7, 0.0, 0.1), (0.7, 0.0, 0.0))[2] < 0


def test_saturation_and_acceleration_limit():
    ctrl = WallAlignController(dt=0.05)
    speeds = []
    for _ in range(40):
        vx, vy, _ = ctrl.command((2.0, 1.0, 0.0), (0.7, 0.0, 0.0))
        speeds.append(math.hypot(vx, vy))
    assert max(speeds) <= ctrl.v_max + 1e-12
    assert np.all(np.diff(speeds) <= ctrl.acc_lin * ctrl.dt + 1e-12)


def test_convergence_monitor_requires_rest_and_persistence():
    m = ConvergenceMonitor(n_required=3)
    small = (0.001, -0.001, math.radians(0.1))
    assert not m.update(small, velocity=(0.01, 0.0, 0.0))       # 还在动
    assert not m.update(small)
    assert not m.update(small)
    assert m.update(small)
    m.reset()
    assert not m.update((0.003, 0.0, 0.0))                       # 超出容差
    sig = (0.003, 0.001, 0.001)                                  # 估计不确定
    assert not any(m.update(small, sigma=sig) for _ in range(5))
