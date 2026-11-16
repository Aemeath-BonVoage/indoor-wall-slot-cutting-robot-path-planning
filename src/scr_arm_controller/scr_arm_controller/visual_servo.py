"""相机 + 激光融合的基于位置的视觉伺服（PBVS）——纯算法部分.

不依赖 ROS，便于单元测试。坐标约定：
- base_link：x 前、y 左、z 上
- 光学坐标系（camera_optical_link）：z 前、x 右、y 下

思路：
1. 激光雷达拟合前方墙面直线 → 墙面法向（相对机器人朝向的偏角）与垂直距离；
2. 相机 HSV 检测方块像素中心 → 由内参反投影成射线 → 与墙面平面求交 → 方块三维位置；
3. 由「朝向偏角 / 法向距离误差 / 沿墙横向误差」三项构成 PBVS 误差，比例控制输出全向速度。
"""
import math

import numpy as np


def detect_marker(bgr, hsv_lower=(20, 100, 100), hsv_upper=(30, 255, 255), min_area=30):
    """HSV 阈值检测最大黄色色块，返回像素中心与面积；未检测到返回 None."""
    import cv2

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(hsv_lower), np.array(hsv_upper))
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < min_area:
        return None
    m = cv2.moments(c)
    if m['m00'] <= 0:
        return None
    return {'u': m['m10'] / m['m00'], 'v': m['m01'] / m['m00'], 'area': float(area)}


def fit_wall(ranges, angle_min, angle_increment, half_fov=math.radians(35.0),
             max_range=4.0, min_points=10, inlier_tol=0.03, iterations=3):
    """在雷达正前方 ±half_fov 内拟合墙面直线.

    直线表示为 n·p = d（n 为指向墙面的单位法向，d > 0 为垂直距离）。
    采用总体最小二乘 + 迭代剔除离群点。

    返回 dict(yaw_error, distance, normal, inliers, rms)，失败返回 None。
    yaw_error：机器人需要转过的角度（逆时针为正），使 x 轴正对墙面法向。
    """
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_increment * np.arange(len(r))
    a = np.arctan2(np.sin(a), np.cos(a))
    keep = np.isfinite(r) & (r > 0.05) & (r < max_range) & (np.abs(a) <= half_fov)
    if keep.sum() < min_points:
        return None
    pts = np.stack([r[keep] * np.cos(a[keep]), r[keep] * np.sin(a[keep])], axis=1)

    mask = np.ones(len(pts), dtype=bool)
    n = d = None
    for _ in range(iterations):
        p = pts[mask]
        if len(p) < min_points:
            return None
        c = p.mean(axis=0)
        _, _, vt = np.linalg.svd(p - c)
        n = vt[-1]                      # 最小奇异值方向 = 直线法向
        d = float(n @ c)
        if d < 0:                       # 让法向指向墙面（远离原点）
            n, d = -n, -d
        resid = np.abs(pts @ n - d)
        mask = resid < max(inlier_tol, 3.0 * np.std(resid[mask]))
    p = pts[mask]
    rms = float(np.sqrt(np.mean((p @ n - d) ** 2)))
    return {
        'yaw_error': float(math.atan2(n[1], n[0])),
        'distance': d,
        'normal': (float(n[0]), float(n[1])),
        'inliers': int(mask.sum()),
        'rms': rms,
    }


def pixel_to_ray(u, v, fx, fy, cx, cy):
    """像素 → 光学坐标系下的单位射线方向."""
    ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
    return ray / np.linalg.norm(ray)


def intersect_ray_wall(origin, direction, normal_xy, distance):
    """射线（base_link 中的起点与方向）与竖直墙面 n·p = d 求交，返回交点或 None."""
    n = np.array([normal_xy[0], normal_xy[1], 0.0])
    denom = float(n @ direction)
    if abs(denom) < 1e-6:
        return None
    t = (distance - float(n @ origin)) / denom
    if t <= 0:
        return None
    return origin + t * direction


def pbvs_error(marker_base, wall, desired_distance):
    """计算 PBVS 误差（均在 base_link 中）.

    返回 dict:
      yaw     朝向误差（使 x 轴对准墙面法向）
      normal  沿墙面法向的距离误差（>0 表示需要向墙靠近）
      lateral 沿墙面切向、方块相对机器人中心的偏移（>0 表示方块在左侧）
      n, t    墙面法向 / 切向单位向量（base_link 平面内）
    """
    n = np.array(wall['normal'])
    t = np.array([-n[1], n[0]])         # 法向逆时针转 90°：沿墙向左
    return {
        'yaw': wall['yaw_error'],
        'normal': wall['distance'] - desired_distance,
        'lateral': float(t @ np.asarray(marker_base[:2])),
        'n': n,
        't': t,
    }


def pbvs_command(err, kp_lin=0.8, kp_ang=1.5, max_lin=0.2, max_ang=0.3,
                 min_lin=0.01, min_ang=0.02, pos_deadband=0.0, ang_deadband=0.0):
    """由 PBVS 误差计算全向底盘速度 (vx, vy, wz)，base_link 坐标系."""
    v = err['normal'] * err['n'] + err['lateral'] * err['t']
    dist = float(np.linalg.norm(v))
    vx = vy = 0.0
    if dist > pos_deadband and dist > 1e-9:
        speed = min(max(kp_lin * dist, min_lin), max_lin)
        vx, vy = (speed * v / dist).tolist()
    wz = 0.0
    if abs(err['yaw']) > ang_deadband:
        wz = math.copysign(min(max(kp_ang * abs(err['yaw']), min_ang), max_ang), err['yaw'])
    return vx, vy, wz
