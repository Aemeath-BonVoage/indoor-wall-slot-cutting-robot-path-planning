"""
AprilTag（tag36h11）定位标记的检测与位姿估计——纯算法部分（不依赖 ROS）.

检测用 OpenCV 的 ArUco 模块（自带 AprilTag 字典 DICT_APRILTAG_36h11），与 apriltag_ros
使用同一个标记族，换成该节点时只需改输入接口。流程：
1. 自适应阈值 → 四边形候选 → 按 tag36h11 码本解码（得到 ID，可纠正少量位错误）；
2. 角点亚像素细化（cornerSubPix，在黑白边缘的梯度上迭代）；
3. IPPE_SQUARE 平面方块 PnP（Collins & Bartoli 2014）求标记中心在光学系中的位置。

与色块质心相比的区别：
- 有 ID：画面中的其他黄色物体、反光不会被当成标记；
- 部分遮挡时要么解码失败（不输出），要么四个角点完整（输出无偏）；色块被遮挡一部分时
  质心会静默地偏移，下游无法察觉；
- 四个角点提供尺度，能单独估计深度；朝向（绕竖直轴）对小标记仍然很弱
  （见 docs/experiments/ibvs_vs_pbvs），因此只取标记中心位置，朝向交给激光。
"""
import math

import numpy as np

TAG_FAMILIES = {
    'tag36h11': 'DICT_APRILTAG_36h11',
    'tag25h9': 'DICT_APRILTAG_25h9',
    'tag16h5': 'DICT_APRILTAG_16h5',
}


def _aruco():
    import cv2
    return cv2, cv2.aruco


def get_dictionary(family='tag36h11'):
    """返回 OpenCV 的预定义 AprilTag 字典."""
    _, aruco = _aruco()
    return aruco.getPredefinedDictionary(getattr(aruco, TAG_FAMILIES[family]))


def render_tag(tag_id, family='tag36h11', cell_px=32, margin_cells=1):
    """
    生成标记图像（灰度 uint8）.

    tag36h11 的黑色外框加数据区共 8 × 8 格；外面再留 margin_cells 格白边（检测四边形
    需要黑框外有对比度）。返回图像以及黑色方框在图像中所占的像素边长。
    """
    cv2, aruco = _aruco()
    d = get_dictionary(family)
    cells = d.markerSize + 2                       # 数据区 + 黑框
    side = cells * cell_px
    if hasattr(aruco, 'generateImageMarker'):      # OpenCV ≥ 4.7
        img = aruco.generateImageMarker(d, tag_id, side, borderBits=1)
    else:
        img = aruco.drawMarker(d, tag_id, side, borderBits=1)
    m = margin_cells * cell_px
    img = cv2.copyMakeBorder(img, m, m, m, m, cv2.BORDER_CONSTANT, value=255)
    return img, side


def make_detector(family='tag36h11', refine='subpix'):
    """构造检测器（兼容 OpenCV 4.5 旧接口与 4.7+ 的 ArucoDetector）."""
    _, aruco = _aruco()
    if hasattr(aruco, 'DetectorParameters_create'):
        params = aruco.DetectorParameters_create()
    else:
        params = aruco.DetectorParameters()
    params.cornerRefinementMethod = {
        'none': aruco.CORNER_REFINE_NONE,
        'subpix': aruco.CORNER_REFINE_SUBPIX,
        'apriltag': aruco.CORNER_REFINE_APRILTAG,
    }[refine]
    # 标记外圈白边比墙面亮时，白边外沿本身也是一个四边形候选，且与黑框四角只差 1 格；
    # 默认的 minMarkerDistanceRate = 0.05 会把两者当作重复、只保留外面的白边（解码必然失败）
    params.minMarkerDistanceRate = 0.01
    params.cornerRefinementWinSize = 3
    params.cornerRefinementMaxIterations = 50
    params.cornerRefinementMinAccuracy = 0.01
    dictionary = get_dictionary(family)
    if hasattr(aruco, 'ArucoDetector'):
        det = aruco.ArucoDetector(dictionary, params)
        return lambda gray: det.detectMarkers(gray)
    return lambda gray: aruco.detectMarkers(gray, dictionary, parameters=params)


def detect_apriltags(image, detector, tag_id=None, edge_refine=True):
    """
    检测图像中的标记.

    image：BGR 或灰度图；detector：make_detector() 的返回值；tag_id：只保留该 ID（None = 全部）；
    edge_refine：是否用 refine_corners_edges 做边缘直线求交细化。
    返回列表，每项 dict(id, corners (4×2，按标记自身的 左上 / 右上 / 右下 / 左下 顺序), center)。
    """
    cv2, _ = _aruco()
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector(gray)
    out = []
    if ids is None:
        return out
    for c, i in zip(corners, ids.ravel()):
        if tag_id is not None and int(i) != int(tag_id):
            continue
        c = np.asarray(c, dtype=float).reshape(4, 2)
        if edge_refine:
            c = refine_corners_edges(gray, c)
        out.append({'id': int(i), 'corners': c, 'center': _diagonal_intersection(c)})
    return out


def _bilinear(gray, xs, ys):
    """双线性插值取灰度（xs, ys 为同形数组，像素中心为整数坐标）."""
    h, w = gray.shape
    xs = np.clip(xs, 0, w - 1.001)
    ys = np.clip(ys, 0, h - 1.001)
    x0, y0 = np.floor(xs).astype(int), np.floor(ys).astype(int)
    fx, fy = xs - x0, ys - y0
    g = gray.astype(float)
    return ((1 - fx) * (1 - fy) * g[y0, x0] + fx * (1 - fy) * g[y0, x0 + 1]
            + (1 - fx) * fy * g[y0 + 1, x0] + fx * fy * g[y0 + 1, x0 + 1])


def refine_corners_edges(gray, corners, samples=12, step=0.25):
    """
    基于边缘的亚像素角点细化（与 AprilTag 库 refine_edges 同一思路）.

    黑色外框四条边各取若干采样点，沿法向取灰度剖面，用梯度加权质心求亚像素边缘位置，
    再对每条边做总体最小二乘直线拟合，相邻两条直线求交得到角点。
    cornerSubPix 针对的是棋盘格“鞍点”，用在黑方块外角（L 形角）上会系统性地向内 / 外偏，
    合成图像上实测深度偏差约 5 mm；直线求交没有这个偏差，且利用了整条边的信息。
    采样窗口取半个格宽（外框宽度 = 1 格），避免碰到内部的数据格。
    """
    c = np.asarray(corners, dtype=float).reshape(4, 2)
    centroid = c.mean(axis=0)
    cell = np.mean(np.linalg.norm(np.roll(c, -1, axis=0) - c, axis=1)) / 8.0
    r = float(np.clip(0.45 * cell, 1.0, 4.0))
    offs = np.arange(-r, r + 1e-9, step)
    mids = (offs[:-1] + offs[1:]) / 2
    lines = []
    for i in range(4):
        a, b = c[i], c[(i + 1) % 4]
        L = np.linalg.norm(b - a)
        d = (b - a) / L
        n = np.array([-d[1], d[0]])
        if n @ ((a + b) / 2 - centroid) < 0:      # 法向朝外（黑 → 白）
            n = -n
        p = a + np.outer(np.linspace(0.15, 0.85, samples) * L, d)       # samples × 2
        prof = _bilinear(gray, p[:, :1] + offs * n[0], p[:, 1:] + offs * n[1])
        g = np.clip(np.diff(prof, axis=1), 0.0, None)                    # 黑 → 白为正
        w = g.sum(axis=1)
        good = w > 1e-6
        if good.sum() < 3:
            return c
        s = (g[good] * mids).sum(axis=1) / w[good]
        pts = p[good] + s[:, None] * n
        m = pts.mean(axis=0)
        _, _, vt = np.linalg.svd(pts - m)
        nn = vt[-1]
        lines.append((nn, float(nn @ m)))
    out = []
    for i in range(4):
        (n1, d1), (n2, d2) = lines[i - 1], lines[i]   # 角点 i 位于第 i-1 与第 i 条边之间
        A = np.array([n1, n2])
        if abs(np.linalg.det(A)) < 1e-9:
            return c
        out.append(np.linalg.solve(A, [d1, d2]))
    out = np.array(out)
    if np.max(np.linalg.norm(out - c, axis=1)) > 2.0:   # 细化跑偏则保留原角点
        return c
    return out


def _diagonal_intersection(c):
    """四边形对角线交点 = 标记中心的透视投影（比四角平均更准确）."""
    p1, p2, p3, p4 = c
    d1, d2 = p3 - p1, p4 - p2
    a = np.array([d1, -d2]).T
    if abs(np.linalg.det(a)) < 1e-9:
        return c.mean(axis=0)
    s, _ = np.linalg.solve(a, p2 - p1)
    return p1 + s * d1


def tag_pose(corners, K, tag_size, dist=None):
    """
    平面方块 PnP（SOLVEPNP_IPPE_SQUARE）：返回 (R, t)，标记系 → 光学系.

    标记系：原点在中心，x 向右、y 向上（看着标记时），z 指向观察者。t 即标记中心在光学系中
    的位置。K 为 3×3 内参矩阵。
    """
    cv2, _ = _aruco()
    s = tag_size / 2.0
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=float)
    dist = np.zeros(5) if dist is None else np.asarray(dist, dtype=float)
    ok, rvec, tvec = cv2.solvePnP(obj, np.asarray(corners, dtype=float).reshape(4, 1, 2),
                                  np.asarray(K, dtype=float), dist,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.ravel()


def tag_position_covariance(t_cam, fx, tag_size, sigma_px=0.2, lateral_floor=3e-4,
                            depth_floor=5e-3):
    """
    标记中心位置测量的协方差（光学系，3×3）.

    一阶误差传播：横向 / 竖向 σ ≈ σ_px·Z/f（中心点的方位角误差）；深度来自边长
    Z = f·L/l，σ_Z ≈ Z²/(f·L)·σ_px·√2/2（四条边平均）。
    lateral_floor / depth_floor 为模型误差下限：深度依赖标记打印尺寸与焦距标定，
    1 % 的尺度误差在 0.55 m 处就是 5.5 mm，故深度下限取 5 mm，使法向距离主要由激光决定，
    标记主要提供沿墙位置（与 docs/experiments/ibvs_vs_pbvs 的可观测性分析一致）。
    """
    z = max(float(t_cam[2]), 1e-3)
    s_lat = max(sigma_px * z / fx, lateral_floor)
    s_dep = max(sigma_px * z * z / (fx * tag_size) * math.sqrt(0.5), depth_floor)
    # 视线方向为深度，其余两个方向为横向；按视线方向旋转
    los = np.asarray(t_cam, dtype=float) / np.linalg.norm(t_cam)
    return s_lat ** 2 * (np.eye(3) - np.outer(los, los)) + s_dep ** 2 * np.outer(los, los)
