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
