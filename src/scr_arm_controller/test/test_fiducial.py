"""fiducial（AprilTag 检测与 PnP）单元测试：合成透视图像，已知真值."""
import math

import cv2
import numpy as np
import pytest

from scr_arm_controller import fiducial as fd

FX = 320 / math.tan(math.radians(40))
K = np.array([[FX, 0, 319.5], [0, FX, 239.5], [0, 0, 1.0]])
SIZE = 0.10


def render(tag_id, t, yaw_deg=0.0, noise=2.0, seed=0, ss=4):
    """标记（黑框边长 SIZE）在光学系 t 处、绕竖直轴转 yaw，4 倍超采样渲染 640×480 灰度图."""
    tag, side = fd.render_tag(tag_id, cell_px=64)
    yaw = math.radians(yaw_deg)
    R0 = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)   # 标记正对相机
    Ry = np.array([[math.cos(yaw), 0, math.sin(yaw)], [0, 1, 0],
                   [-math.sin(yaw), 0, math.cos(yaw)]])
    R = Ry @ R0
    scale = SIZE / side
    c0 = tag.shape[0] / 2.0
    A = np.array([[scale, 0, (0.5 - c0) * scale], [0, -scale, (c0 - 0.5) * scale], [0, 0, 1]])
    Ks = K.copy()
    Ks[:2] *= ss
    Ks[0, 2] = K[0, 2] * ss + (ss - 1) / 2
    Ks[1, 2] = K[1, 2] * ss + (ss - 1) / 2
    H = Ks @ np.column_stack([R[:, 0], R[:, 1], t]) @ A
    img = cv2.warpPerspective(tag, H, (640 * ss, 480 * ss), flags=cv2.INTER_LINEAR,
                              borderValue=200)
    img = cv2.resize(img, (640, 480), interpolation=cv2.INTER_AREA).astype(float)
    img += np.random.default_rng(seed).normal(0, noise, img.shape)
    s = SIZE / 2
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]])
    pr = (K @ (R @ obj.T + np.asarray(t)[:, None])).T
    return np.clip(img, 0, 255).astype(np.uint8), pr[:, :2] / pr[:, 2:]


@pytest.mark.parametrize('t,yaw', [((0.0, -0.19, 0.55), 0.0), ((0.12, -0.19, 0.8), 8.0),
                                   ((-0.2, -0.15, 1.2), -10.0)])
def test_detect_and_pose(t, yaw):
    img, truth = render(0, np.array(t), yaw)
    det = fd.make_detector(refine='apriltag')
    tags = fd.detect_apriltags(img, det, tag_id=0)
    assert len(tags) == 1
    c = tags[0]['corners']
    assert np.max(np.linalg.norm(c - truth, axis=1)) < 0.35          # 亚像素角点
    R, tv = fd.tag_pose(c, K, SIZE)
    assert abs(tv[0] - t[0]) < 1.5e-3                                 # 横向 < 1.5 mm
    assert abs(tv[2] - t[2]) < 0.02 * t[2]                            # 深度 < 2 %


def test_edge_refinement_improves_corners():
    errs = {True: [], False: []}
    det = fd.make_detector(refine='apriltag')
    for k in range(6):
        img, truth = render(0, np.array([0.05 * (k - 3), -0.18, 0.6 + 0.1 * k]), 3 * k - 8,
                            seed=k)
        for er in (True, False):
            tags = fd.detect_apriltags(img, det, 0, edge_refine=er)
            if tags:
                errs[er].append(np.max(np.linalg.norm(tags[0]['corners'] - truth, axis=1)))
    assert len(errs[True]) >= 5
    assert np.mean(errs[True]) < np.mean(errs[False])


def test_wrong_id_is_filtered():
    img, _ = render(3, np.array([0.0, -0.1, 0.6]))
    det = fd.make_detector()
    assert fd.detect_apriltags(img, det, tag_id=0) == []
    assert [d['id'] for d in fd.detect_apriltags(img, det)] == [3]


def test_no_tag_in_blank_image():
    det = fd.make_detector()
    assert fd.detect_apriltags(np.full((480, 640), 128, np.uint8), det) == []


def test_covariance_is_anisotropic_along_line_of_sight():
    t = np.array([0.0, -0.2, 0.55])
    C = fd.tag_position_covariance(t, FX, SIZE)
    los = t / np.linalg.norm(t)
    perp = np.cross(los, [1.0, 0, 0])
    perp /= np.linalg.norm(perp)
    assert los @ C @ los == pytest.approx(5e-3 ** 2, rel=1e-6)       # 深度下限 5 mm
    assert perp @ C @ perp < (0.5e-3) ** 2                            # 横向亚毫米
    assert np.allclose(C, C.T)
