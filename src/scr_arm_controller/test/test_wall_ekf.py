"""WallEKF 单元测试：过程模型、收敛与一致性、离群值门限、零位偏置、标记丢失."""
import math

import numpy as np
import pytest

from scr_arm_controller.wall_ekf import WallEKF

R_WALL = np.diag([1.3e-3 ** 2, math.radians(0.26) ** 2])
R_TAG = np.diag([3e-3 ** 2, 0.3e-3 ** 2])


def body_step(X, dx, dy, dpsi):
    """真值：底盘在机体系走 (dx, dy) 再转 dpsi（与 relative_pose 的定义一致）."""
    f = WallEKF()
    f.reset(*X)
    f.predict(dx, dy, dpsi)
    return f.x


def test_predict_matches_rigid_motion():
    # 正对墙面前进 0.1 m：d 减 0.1；左移 0.05 m：u 减 0.05（u 向右为正）
    assert np.allclose(body_step((0.7, 0.0, 0.0), 0.1, 0.0, 0.0), (0.6, 0.0, 0.0))
    assert np.allclose(body_step((0.7, 0.0, 0.0), 0.0, 0.05, 0.0), (0.7, -0.05, 0.0))
    # 偏转 90°（左转）时前进 = 向左（u 减小），d 不变
    assert np.allclose(body_step((0.7, 0.0, math.pi / 2), 0.1, 0.0, 0.0),
                       (0.7, -0.1, math.pi / 2))


def test_init_from_inverts_tag_model():
    X = np.array([0.82, -0.13, math.radians(7)])
    p = WallEKF.h_tag(X)
    d, u, psi, has_u = WallEKF.init_from((X[0], -X[2]), p)
    assert has_u
    assert (d, u, psi) == pytest.approx(tuple(X), abs=1e-12)


def test_tag_jacobian_matches_finite_difference():
    X = np.array([0.7, 0.05, math.radians(-4)])
    H = WallEKF.H_tag(X)
    eps = 1e-7
    for j in range(3):
        dX = np.zeros(3)
        dX[j] = eps
        num = (WallEKF.h_tag(X + dX) - WallEKF.h_tag(X - dX)) / (2 * eps)
        assert np.allclose(H[:, j], num, atol=1e-6)


def run_static(truth, n=200, offset=np.zeros(3), seed=0, tag=True):
    rng = np.random.default_rng(seed)
    f = WallEKF()
    f.offset = offset.copy()
    Xs = np.asarray(truth) + offset
    f.reset(truth[0] + 0.01, truth[1] + 0.05, truth[2] + 0.02, sigma=(0.02, 0.1, 0.05))
    nis = []
    for _ in range(n):
        f.predict(0.0, 0.0, 0.0)
        z = WallEKF.h_wall(Xs) + rng.multivariate_normal(np.zeros(2), R_WALL)
        f.update_wall(z[0], z[1], R_WALL)
        if tag:
            z = WallEKF.h_tag(Xs) + rng.multivariate_normal(np.zeros(2), R_TAG)
            f.update_tag(z, R_TAG)
            nis.append(f.last_nis['tag'])
    return f, nis


def test_static_convergence_and_consistency():
    truth = np.array([0.70, 0.004, math.radians(1.0)])
    f, nis = run_static(truth)
    err = f.x - truth
    sig = f.sigma()
    assert np.all(np.abs(err) < 4 * sig + 1e-9)
    assert abs(err[1]) < 0.5e-3 and abs(err[0]) < 1e-3
    assert np.mean(nis[50:]) == pytest.approx(2.0, abs=0.8)          # NIS 均值 ≈ 自由度


def test_outlier_is_rejected():
    truth = np.array([0.70, 0.0, 0.0])
    f, _ = run_static(truth, n=50)
    x0 = f.x.copy()
    ok, nis = f.update_tag(WallEKF.h_tag(truth) + np.array([0.0, 0.03]), R_TAG)   # 3 cm 误检
    assert not ok and nis > f.gate
    assert np.allclose(f.x, x0)
    ok, _ = f.update_wall(0.60, 0.0, R_WALL)                               # 拟合到障碍物
    assert not ok


def test_offset_is_removed_inside_measurement_model():
    truth = np.array([0.70, 0.0, 0.0])
    c = np.array([0.002, 0.010, math.radians(1.0)])   # 1° 相机偏角 ≈ 10 mm 横向
    f, _ = run_static(truth, offset=c)
    assert np.allclose(f.x, truth, atol=(1e-3, 0.5e-3, math.radians(0.1)))
    f0, _ = run_static(truth)                        # 不扣除时估计带偏
    f0.offset = np.zeros(3)
    g, _ = run_static(truth + c)
    assert g.x[1] == pytest.approx(0.010, abs=0.5e-3)


def test_dropout_dead_reckoning():
    """标记丢失时 u 由里程计外推，σ_u 随运动增长."""
    truth = np.array([0.70, 0.02, 0.0])
    f, _ = run_static(truth, n=100)
    s0 = f.sigma()[1]
    for _ in range(20):                               # 横移 20 × 1 mm（向左 → u 减小）
        f.predict(0.0, 0.001, 0.0)
        f.update_wall(0.70, 0.0, R_WALL)
    assert f.x[1] == pytest.approx(0.0, abs=1e-3)
    assert f.sigma()[1] > s0


def test_relative_pose_composes_with_ekf_prediction():
    """里程计相对位姿 → EKF 预测，与直接由真值位姿算出的墙面状态一致（大角度、大位移）."""
    pytest.importorskip('nav_msgs')
    from scr_arm_controller.wall_estimator_ros import relative_pose

    def wall_state(pose):
        # 墙面 y = 0、法向指向 −y；标记在 x = 0；正对墙面时 yaw = −π/2
        x, y, yaw = pose
        return np.array([y, -x, math.atan2(math.sin(yaw + math.pi / 2),
                                           math.cos(yaw + math.pi / 2))])

    a = (0.3, 1.2, -math.pi / 2 + 0.2)
    b = (-0.8, 2.5, math.pi / 3)
    f = WallEKF()
    f.reset(*wall_state(a))
    f.predict(*relative_pose(a, b))
    assert np.allclose(f.x, wall_state(b), atol=1e-12)


# ----------------------------------------------------------------------
# 多标记地图与横移比例修正
# ----------------------------------------------------------------------
def test_tag_model_with_map_offset_is_a_shift_of_u():
    X = np.array([0.7, 0.62, math.radians(3)])
    shifted = X - np.array([0.0, 0.6, 0.0])
    assert np.allclose(WallEKF.h_tag(X, 0.6), WallEKF.h_tag(shifted))
    H = WallEKF.H_tag(X, 0.6)
    eps = 1e-7
    for j in range(3):
        dX = np.zeros(3)
        dX[j] = eps
        num = (WallEKF.h_tag(X + dX, 0.6) - WallEKF.h_tag(X - dX, 0.6)) / (2 * eps)
        assert np.allclose(H[:, j], num, atol=1e-6)
    d, u, psi, _ = WallEKF.init_from((X[0], -X[2]), WallEKF.h_tag(X, 0.6), u_tag=0.6)
    assert (d, u, psi) == pytest.approx(tuple(X), abs=1e-12)


def test_mapped_tag_corrects_along_wall_position():
    """在 u = 0.62 处看到 u = 0.6 的标记：先验偏 2 cm 的 u 被拉回真值."""
    rng = np.random.default_rng(1)
    truth = np.array([0.70, 0.62, 0.0])
    f = WallEKF()
    f.reset(0.70, 0.64, 0.0, sigma=(0.005, 0.03, 0.01))
    for _ in range(60):
        f.predict(0.0, 0.0, 0.0)
        f.update_wall(*(WallEKF.h_wall(truth) + rng.multivariate_normal(np.zeros(2), R_WALL)),
                      R_WALL)
        z = WallEKF.h_tag(truth, 0.6) + rng.multivariate_normal(np.zeros(2), R_TAG)
        f.update_tag(z, R_TAG, u_tag=0.6)
    assert f.x[1] == pytest.approx(0.62, abs=0.5e-3)
    assert f.stats['tag'][1] == 0


def test_map_uncertainty_keeps_a_misplaced_tag_from_being_rejected():
    """标记实际偏 4 mm：不计地图误差时被当成离群值拒收，计入 σ_map = 2 mm 后接受."""
    truth = np.array([0.70, 0.30, 0.0])
    z = WallEKF.h_tag(truth, 0.6 + 0.004)        # 实际粘贴位置 0.604，地图记 0.6
    results = {}
    for sigma_map in (0.0, 0.002):
        f, _ = run_static(np.array([0.70, 0.30, 0.0]), n=80)
        ok, nis = f.update_tag(z, R_TAG, u_tag=0.6, sigma_map=sigma_map)
        results[sigma_map] = ok
    assert results == {0.0: False, 0.002: True}


def strafe(f, truth, du, lateral_slip, tag_window, steps, rng, tags=(0.0,)):
    """真值横移 du（沿墙，向右为正）；编码器高估横移 lateral_slip；|u − u_i| < tag_window 时看到标记 i."""
    step = du / steps
    for _ in range(steps):
        truth[1] += step
        dy_odom = -step * (1.0 + lateral_slip)   # 正对墙面：向右 = 机体 −y
        f.predict(0.0, dy_odom, 0.0)
        f.update_wall(*(WallEKF.h_wall(truth) + rng.multivariate_normal(np.zeros(2), R_WALL)),
                      R_WALL)
        for ut in tags:
            if abs(truth[1] - ut) < tag_window:
                z = WallEKF.h_tag(truth, ut) + rng.multivariate_normal(np.zeros(2), R_TAG)
                f.update_tag(z, R_TAG, u_tag=ut)


def far_station_error(estimate_scale, slip=0.03, seed=2):
    """标记在 u = 0，视野 ±0.4 m；从 −0.25 横移到 +0.9 m，返回终点的估计误差与滤波器."""
    rng = np.random.default_rng(seed)
    truth = np.array([0.70, -0.25, 0.0])
    f = WallEKF(estimate_scale=estimate_scale)
    f.reset(*truth, sigma=(0.002, 0.002, 0.002))
    strafe(f, truth, 1.15, slip, 0.4, 230, rng)
    return f.x[1] - truth[1], f


def test_scale_state_removes_dead_reckoning_drift_beyond_the_tag():
    e_plain, _ = far_station_error(False)
    e_scale, f = far_station_error(True)
    # 不估计 s：出视野后的 0.5 m 按 3 % 累积（停得偏近 → 估计偏大）
    assert e_plain == pytest.approx(0.5 * 0.03, rel=0.25)
    assert abs(e_scale) < 0.2 * abs(e_plain)
    s, sig_s = f.scale
    assert s == pytest.approx(1.0 / 1.03 - 1.0, abs=3 * sig_s + 1e-3)
    assert sig_s < 0.01


def test_scale_is_not_observable_from_a_round_trip():
    """往返同一位置：去程、回程的比例误差相互抵消，s 没有被约束（σ_s 仍接近先验）."""
    rng = np.random.default_rng(4)
    truth = np.array([0.70, 0.0, 0.0])
    f = WallEKF(estimate_scale=True, scale_sigma=0.05)
    f.reset(*truth, sigma=(0.002, 0.002, 0.002))
    for _ in range(20):                                    # 静止看标记
        f.predict(0.0, 0.0, 0.0)
        f.update_tag(WallEKF.h_tag(truth) + rng.multivariate_normal(np.zeros(2), R_TAG), R_TAG)
    strafe(f, truth, 0.8, 0.03, 0.0, 80, rng)             # 出视野（tag_window = 0）
    strafe(f, truth, -0.8, 0.03, 0.0, 80, rng)
    for _ in range(20):                                    # 回到原处再看标记
        f.predict(0.0, 0.0, 0.0)
        f.update_tag(WallEKF.h_tag(truth) + rng.multivariate_normal(np.zeros(2), R_TAG), R_TAG)
    assert f.scale[1] > 0.8 * 0.05


def test_scale_survives_a_new_alignment():
    _, f = far_station_error(True)
    s, sig = f.scale
    f.clear()
    assert not f.initialized and f.scale == (pytest.approx(s), pytest.approx(sig))
    f.reset(0.7, 0.0, 0.0)
    assert f.x[3] == pytest.approx(s) and math.sqrt(f.P[3, 3]) == pytest.approx(sig)
    assert f.sigma().shape == (3,)


def test_known_scale_corrects_the_prediction():
    f = WallEKF(estimate_scale=True, scale_init=-0.03, scale_sigma=1e-6)
    f.reset(0.7, 0.0, 0.0)
    f.predict(0.0, -0.1, 0.0)                     # 里程计向右 0.1 m
    assert f.x[1] == pytest.approx(0.097)
    assert f.correct_increment(-0.1) == pytest.approx(-0.097)
