"""ibvs 纯算法单元测试：交互矩阵数值校验、闭环收敛、退化情形."""
import math

import numpy as np
import pytest

from scr_arm_controller import ibvs

K = (381.4, 381.4, 319.5, 239.5)
P_C = np.array([0.148668, 0.0214, 0.398])     # 相机光心在 base_link 中（joint1 = −0.15）
H = 0.4976                                     # 方块中心高度（base_link 系）
D = 0.70
SIZE = 0.05


def corners_base(pose, size=SIZE, h=H):
    """墙面 x_w = 0，方块中心 (0, 0, h)；机器人位姿 (x, y, θ) → base_link 中的四角点."""
    cw = ibvs.marker_corners_base([0.0, 0.0, h], [1.0, 0.0], size)
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    rx, ry = cw[:, 0] - x, cw[:, 1] - y
    return np.column_stack([c * rx + s * ry, -s * rx + c * ry, cw[:, 2]])


def wall_of(pose):
    """机器人位姿下 base_link 中的墙面 dict（与 visual_servo.fit_wall 同格式，无噪声）."""
    x, _, th = pose
    n = (math.cos(-th), math.sin(-th))
    return {'normal': n, 'distance': -x, 'yaw_error': -th}


def move(pose, u, dt):
    """全向底盘运动学积分（小步长）."""
    x, y, th = pose
    for _ in range(10):
        h = dt / 10
        x += (u[0] * math.cos(th) - u[1] * math.sin(th)) * h
        y += (u[0] * math.sin(th) + u[1] * math.cos(th)) * h
        th += u[2] * h
    return np.array([x, y, th])


def observe(pose, size=SIZE, r_bo=ibvs.R_BO, p_c=P_C):
    px, z = ibvs.project_points(corners_base(pose, size), p_c, r_bo, K)
    return px, z


# ------------------------------------------------------------------ 交互矩阵
@pytest.mark.parametrize('pose,tilt', [
    ((-0.70, 0.0, 0.0), (0, 0, 0)),
    ((-1.20, 0.15, math.radians(8)), (0, 0, 0)),
    ((-0.95, -0.25, math.radians(-12)), (0, 0, 0)),
    ((-1.00, 0.10, math.radians(5)), (math.radians(3), math.radians(-4), math.radians(2))),
])
def test_feature_jacobian_matches_finite_difference(pose, tilt):
    """解析 J = L·W 与对底盘位姿做有限差分得到的像点速度一致."""
    r_bo = ibvs.rot_rpy(*tilt) @ ibvs.R_BO
    px, z = observe(pose, r_bo=r_bo)
    xy = ibvs.pixels_to_normalized(px, K)
    j = ibvs.feature_jacobian(xy, z, P_C, r_bo)
    eps = 1e-6
    j_fd = np.zeros_like(j)
    for i in range(3):
        u = np.zeros(3)
        u[i] = 1.0
        p2, _ = observe(move(pose, u, eps), r_bo=r_bo)
        p1, _ = observe(move(pose, -u, eps), r_bo=r_bo)
        d = ibvs.pixels_to_normalized(p2, K) - ibvs.pixels_to_normalized(p1, K)
        j_fd[:, i] = d.reshape(-1) / (2 * eps)
    assert np.allclose(j, j_fd, atol=1e-6 * np.abs(j).max() + 1e-8)


def test_camera_twist_nominal_structure():
    """标称安装：v_x = −(vy + wz·p_cx)，v_z = vx − wz·p_cy，ω_y = −wz，其余为 0."""
    w = ibvs.base_to_camera_twist(P_C)
    vx, vy, wz = 0.3, -0.2, 0.5
    tw = w @ [vx, vy, wz]
    assert tw == pytest.approx([-(vy + wz * P_C[0]), 0.0, vx - wz * P_C[1], 0.0, -wz, 0.0])


def test_point_interaction_pure_rotation_independent_of_depth():
    """纯旋转引起的像点运动与深度无关（交互矩阵后三列不含 Z）."""
    a = ibvs.point_interaction(0.1, -0.2, 0.5)
    b = ibvs.point_interaction(0.1, -0.2, 5.0)
    assert a[:, 3:] == pytest.approx(b[:, 3:])
    assert not np.allclose(a[:, :3], b[:, :3])


# ------------------------------------------------------------------ 几何工具
def test_order_corners_is_permutation_invariant():
    px, _ = observe((-1.0, 0.1, math.radians(6)))
    rng = np.random.default_rng(0)
    for _ in range(10):
        assert ibvs.order_corners(px[rng.permutation(4)]) == pytest.approx(px)


def test_polygon_centroid_of_square():
    sq = np.array([[0, 0], [2, 0], [2, 2], [0, 2]], dtype=float)
    assert ibvs.polygon_centroid(sq) == pytest.approx([1.0, 1.0])


def test_depth_estimates():
    pose = (-1.1, 0.05, math.radians(4))
    px, z = observe(pose)
    xy = ibvs.pixels_to_normalized(px, K)
    assert ibvs.depth_from_wall(xy, wall_of(pose), P_C, ibvs.R_BO) == pytest.approx(z, rel=1e-9)
    assert ibvs.depth_from_size(xy, SIZE) == pytest.approx(z.mean(), rel=0.02)


def test_saturate_keeps_direction():
    u = ibvs.saturate([0.4, 0.3, 0.9], max_lin=0.2, max_ang=0.3)
    assert math.hypot(u[0], u[1]) <= 0.2 + 1e-12 and abs(u[2]) <= 0.3 + 1e-12
    assert np.cross(u, [0.4, 0.3, 0.9]) == pytest.approx(np.zeros(3), abs=1e-12)


# ------------------------------------------------------------------ 闭环
def make_ibvs(depth_mode='desired', size=SIZE):
    ctl = ibvs.IbvsController(P_C, K, size=size, depth_mode=depth_mode, smoothing=0.0)
    px, z = observe((-D, 0.0, 0.0), size=size)
    ctl.set_reference(px, z)
    return ctl


@pytest.mark.parametrize('depth_mode', ['desired', 'laser', 'size', 'mean'])
@pytest.mark.parametrize('start', [(-1.4, 0.25, math.radians(9)),
                                   (-1.05, -0.28, math.radians(-10)),
                                   (-1.2, 0.0, 0.0)])
def test_ibvs_closed_loop_converges(depth_mode, start):
    """无噪声运动学闭环：各深度方式都收敛到期望位姿（1 mm / 0.05°）."""
    ctl = make_ibvs(depth_mode)
    pose = np.array(start)
    for _ in range(600):
        px, _ = observe(pose)
        u, info = ctl.compute(px, wall_of(pose))
        pose = move(pose, u, 0.1)
    assert -pose[0] == pytest.approx(D, abs=1e-3)
    assert pose[1] == pytest.approx(0.0, abs=1e-3)
    assert pose[2] == pytest.approx(0.0, abs=math.radians(0.05))
    assert info['err_rms_px'] < 0.01
    assert ibvs.ibvs_converged(info)


def test_hybrid_closed_loop_converges():
    ctl = ibvs.HybridController(P_C, K, desired_distance=D, smoothing=0.0)
    px, _ = observe((-D, 0.0, 0.0))
    ctl.set_reference(ibvs.pixels_to_normalized(ibvs.polygon_centroid(px), K)[0, 0])
    pose = np.array([-1.4, -0.25, math.radians(10)])
    for _ in range(300):
        px, _ = observe(pose)
        u, info = ctl.compute(ibvs.polygon_centroid(px), wall_of(pose))
        pose = move(pose, u, 0.1)
    assert pose == pytest.approx([-D, 0.0, 0.0], abs=1e-3)
    assert ibvs.hybrid_converged(info)


def test_hybrid_feedforward_keeps_marker_still_while_turning():
    """横向已对准、只有朝向误差时，转向引起的质心像面运动被前馈抵消."""
    ctl = ibvs.HybridController(P_C, K, desired_distance=D, smoothing=0.0)
    pose = np.array([-D, 0.0, math.radians(6)])
    px, _ = observe(pose)
    xg0 = ibvs.pixels_to_normalized(ibvs.polygon_centroid(px), K)[0, 0]
    ctl.set_reference(xg0)                         # 当前质心即期望值 → e_x = 0
    u, _ = ctl.compute(ibvs.polygon_centroid(px), wall_of(pose))
    assert abs(u[2]) > 0.05                        # 确实在转
    px2, _ = observe(move(pose, u, 0.01))
    xg1 = ibvs.pixels_to_normalized(ibvs.polygon_centroid(px2), K)[0, 0]
    # 不加前馈时质心会移动约 K[0]·|ωz|·dt ≈ 0.5 px；前馈后应小于其 5 %
    assert abs(xg1 - xg0) * K[0] < 0.05 * K[0] * abs(u[2]) * 0.01


def test_ibvs_with_wrong_extrinsics_still_reaches_taught_pose():
    """示教期望：控制器用的外参有误差，只要 s* 在真实相机下示教，仍收敛到示教位姿."""
    r_true = ibvs.rot_rpy(0.0, 0.0, math.radians(1.5)) @ ibvs.R_BO
    p_true = P_C + np.array([0.005, -0.008, 0.0])
    ctl = ibvs.IbvsController(P_C, K, smoothing=0.0)           # 标称外参
    px_star, z_star = observe((-D, 0.0, 0.0), r_bo=r_true, p_c=p_true)
    ctl.set_reference(px_star, z_star)
    pose = np.array([-1.3, 0.2, math.radians(-8)])
    for _ in range(800):
        px, _ = observe(pose, r_bo=r_true, p_c=p_true)
        u, _ = ctl.compute(px)
        pose = move(pose, u, 0.1)
    assert pose == pytest.approx([-D, 0.0, 0.0], abs=1e-3)


# ------------------------------------------------------------------ 退化情形
def test_weak_direction_is_orbit_about_marker():
    """
    5 cm 方块：期望位姿处 J 条件数很大，最弱方向是绕方块竖直轴的“公转”.

    右奇异向量 (vx, vy, wz) 中 vy / (−wz) 即瞬时转动中心到底盘中心的距离，应约等于离墙距离。
    """
    px, z = observe((-D, 0.0, 0.0))
    j = ibvs.feature_jacobian(ibvs.pixels_to_normalized(px, K), z, P_C)
    _, sv, vt = np.linalg.svd(j)
    assert sv[0] / sv[-1] > 100
    weak = vt[-1]
    assert abs(weak[0]) < 0.01
    assert weak[1] / -weak[2] == pytest.approx(D, rel=0.05)


def test_bigger_marker_improves_conditioning():
    def cond(size):
        px, z = observe((-D, 0.0, 0.0), size=size)
        sv = np.linalg.svd(ibvs.feature_jacobian(ibvs.pixels_to_normalized(px, K), z, P_C),
                           compute_uv=False)
        return sv[0] / sv[-1]
    assert cond(0.05) > 3 * cond(0.20)


def test_single_point_jacobian_is_rank_deficient_but_command_finite():
    """只有一个点（2 个特征）无法约束 3 个自由度：rank 2，伪逆仍给出有限的最小范数解."""
    px, z = observe((-1.0, 0.1, 0.05))
    xy = ibvs.pixels_to_normalized(px[:1], K)
    j = ibvs.feature_jacobian(xy, z[:1], P_C)
    assert np.linalg.matrix_rank(j) == 2
    u = -np.linalg.pinv(j) @ np.array([0.01, -0.02])
    assert np.all(np.isfinite(u))


def test_depth_from_wall_parallel_ray_is_nan():
    wall = {'normal': (0.0, 1.0), 'distance': 1.0}       # 墙面与光轴平行
    z = ibvs.depth_from_wall([[0.0, 0.0]], wall, P_C, ibvs.R_BO)
    assert np.isnan(z[0])


def test_ibvs_requires_reference():
    ctl = ibvs.IbvsController(P_C, K)
    px, _ = observe((-1.0, 0.0, 0.0))
    with pytest.raises(RuntimeError):
        ctl.compute(px)


# ------------------------------------------------------------------ 检测
def test_detect_marker_corners_on_synthetic_image():
    import cv2
    img = np.zeros((480, 640, 3), np.uint8)
    quad = np.array([[300, 200], [342, 204], [340, 246], [298, 242]], np.int32)
    cv2.fillPoly(img, [quad], (0, 255, 255))           # BGR 黄色
    c = ibvs.detect_marker_corners(img)
    assert c is not None
    assert c == pytest.approx(quad.astype(float), abs=1.5)


def test_detect_marker_corners_none_on_empty():
    assert ibvs.detect_marker_corners(np.zeros((480, 640, 3), np.uint8)) is None
