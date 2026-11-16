"""visual_servo 纯算法单元测试（合成数据，已知真值）."""
import math

import numpy as np
import pytest

from scr_arm_controller import visual_servo as vs


def synth_scan(wall_dist, wall_yaw, noise=0.0, n=360, seed=0):
    """机器人在原点，墙面法向与 x 轴夹角 wall_yaw、垂直距离 wall_dist，生成 360 线激光."""
    rng = np.random.default_rng(seed)
    angle_min, inc = -math.pi, 2 * math.pi / n
    a = angle_min + inc * np.arange(n)
    cos_rel = np.cos(a - wall_yaw)
    with np.errstate(divide='ignore'):
        r = np.where(cos_rel > 1e-3, wall_dist / cos_rel, np.inf)
    r = r + rng.normal(0, noise, n)
    return r.tolist(), angle_min, inc


@pytest.mark.parametrize('dist,yaw',
                         [(0.7, 0.0), (1.0, math.radians(8)), (0.9, math.radians(-15))])
def test_fit_wall_recovers_plane(dist, yaw):
    r, amin, inc = synth_scan(dist, yaw, noise=0.01)
    w = vs.fit_wall(r, amin, inc)
    assert w is not None
    assert w['distance'] == pytest.approx(dist, abs=0.01)
    assert w['yaw_error'] == pytest.approx(yaw, abs=math.radians(0.5))


def test_fit_wall_rejects_outliers():
    r, amin, inc = synth_scan(0.8, 0.0, noise=0.005)
    r = list(r)
    r[180] = 0.3                     # 正前方一个障碍点
    r[183] = 0.35
    w = vs.fit_wall(r, amin, inc)
    assert w['distance'] == pytest.approx(0.8, abs=0.01)


def test_fit_wall_none_when_no_wall():
    w = vs.fit_wall([float('inf')] * 360, -math.pi, 2 * math.pi / 360)
    assert w is None


def test_detect_marker_center():
    img = np.zeros((480, 640, 3), np.uint8)
    img[200:230, 300:330] = (0, 255, 255)      # BGR 黄色
    det = vs.detect_marker(img)
    assert det is not None
    assert det['u'] == pytest.approx(314.5, abs=1.0)
    assert det['v'] == pytest.approx(214.5, abs=1.0)


def test_detect_marker_none_on_empty():
    assert vs.detect_marker(np.zeros((480, 640, 3), np.uint8)) is None


def test_ray_wall_intersection():
    # 光心在 (0, 0, 0.5)，墙面 x = 1.0，射线指向 (1, 0.1, 0.1) 方向
    d = np.array([1.0, 0.1, 0.1])
    p = vs.intersect_ray_wall(np.array([0.0, 0.0, 0.5]), d / np.linalg.norm(d), (1.0, 0.0), 1.0)
    assert p == pytest.approx([1.0, 0.1, 0.6])


def test_pixel_to_ray_center_is_optical_axis():
    assert vs.pixel_to_ray(320, 240, 381.4, 381.4, 320, 240) == pytest.approx([0, 0, 1])


def test_pbvs_command_directions():
    wall = {'yaw_error': math.radians(5), 'distance': 1.0, 'normal': (1.0, 0.0)}
    err = vs.pbvs_error((1.0, 0.2, 0.6), wall, desired_distance=0.7)
    assert err['normal'] == pytest.approx(0.3)
    assert err['lateral'] == pytest.approx(0.2)
    vx, vy, wz = vs.pbvs_command(err)
    assert vx > 0 and vy > 0 and wz > 0          # 前进、向左、逆时针
    assert math.hypot(vx, vy) <= 0.2 + 1e-9


def test_pbvs_closed_loop_converges():
    """简化运动学闭环：全向底盘按指令运动，误差应收敛到 1 mm 以内."""
    x, y, th = -0.4, 0.15, math.radians(10)     # 机器人在世界系中的位姿
    wall_x, marker = 1.0, np.array([1.0, 0.0])   # 墙面 x = 1.0，方块位于 (1.0, 0)
    dt = 0.1
    for _ in range(400):
        # 世界 → 机器人坐标系下的墙面与方块
        n_b = np.array([math.cos(-th), math.sin(-th)])
        dist = wall_x - x
        wall = {'yaw_error': math.atan2(n_b[1], n_b[0]), 'distance': dist, 'normal': tuple(n_b)}
        rel = marker - np.array([x, y])
        m_b = np.array([rel[0] * math.cos(th) + rel[1] * math.sin(th),
                        -rel[0] * math.sin(th) + rel[1] * math.cos(th)])
        err = vs.pbvs_error(m_b, wall, desired_distance=0.7)
        vx, vy, wz = vs.pbvs_command(err, pos_deadband=0.0005, ang_deadband=math.radians(0.05))
        x += (vx * math.cos(th) - vy * math.sin(th)) * dt
        y += (vx * math.sin(th) + vy * math.cos(th)) * dt
        th += wz * dt
    assert x == pytest.approx(0.3, abs=0.002)
    assert y == pytest.approx(0.0, abs=0.002)
    assert th == pytest.approx(0.0, abs=math.radians(0.2))
