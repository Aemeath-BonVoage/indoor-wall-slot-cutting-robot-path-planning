"""
全向底盘的基于图像的视觉伺服（IBVS）与 2.5D 混合伺服——纯算法部分.

不依赖 ROS，与 visual_servo.py（PBVS）共用坐标约定：
- base_link：x 前、y 左、z 上；底盘只有 (vx, vy, wz) 三个自由度
- 光学坐标系：z 前、x 右、y 下；相机刚性安装在升降滑台上，对位过程中相对 base_link 固定

一、交互矩阵（雅可比）推导
  归一化像点 x = X/Z, y = Y/Z（光学系）。相机在光学系中的速度旋量 (v, ω) 下，静止点满足
  经典点特征交互矩阵（Chaumette & Hutchinson 2006）::

      ẋ = -v_x/Z + x v_z/Z + x y ω_x - (1 + x²) ω_y + y ω_z
      ẏ = -v_y/Z + y v_z/Z + (1 + y²) ω_x - x y ω_y - x ω_z

  底盘速度 u_b = (vx, vy, wz) 到相机旋量：相机原点在 base_link 中位置 p_c、姿态 R_bo
  （光学系 → base_link），则 v_o = R_boᵀ (v_b + ω_b × p_c)，ω_o = R_boᵀ ω_b，
  其中 v_b = (vx, vy, 0)、ω_b = (0, 0, wz)。写成 6×3 矩阵 W，特征雅可比 J = L·W（2N×3）。
  标称安装（光轴水平朝前）时 W 只有三个非零行：v_x = -(vy + wz·p_cx)，v_z = vx - wz·p_cy，
  ω_y = -wz。即：前进只改变深度，横移只平移像面，转向 = 绕竖直轴旋转 + 相机横向平移。

二、控制律
  IBVS：u_b = -λ Ĵ⁺ (s - s*)，Ĵ 中的深度 Ẑ 可选：
    'desired'  Ẑ = Z*（期望位姿处深度，常数雅可比，最常用）
    'laser'    由激光墙面与像点射线求交得当前每个角点深度
    'size'     由方块已知边长与像面边长估计（单一深度）
    'mean'     Ĵ = (J(s, Ẑ_laser) + J(s*, Z*)) / 2（Malis 2004 的均值雅可比）
  平移 / 角速度超限时整体等比缩放，保持像面误差下降方向不变。

  2.5D 混合（分区）：朝向与离墙距离由激光墙面直接闭环（与 PBVS 相同），横向由方块质心
  的像面坐标 x_g 闭环，并用质心交互矩阵行把转向 / 前进引起的像点运动前馈抵消掉::

      ẋ_g = J_g · u_b = -λ (x_g - x_g*)  ⇒  解出沿墙切向速度 v_t

三、为什么 3 自由度下纯 IBVS 的朝向难以观测
  5 cm 方块在 0.55 m 处仅约 35 px。底盘偏航 δ 与横移 δ·Z 在像面上几乎产生相同的平移，
  二者只能靠透视畸变（左右边长差 ∝ 边长·sin δ / Z）区分：1° 偏航只带来约 0.05 px 的边长差。
  因而 J 有一个近奇异方向（绕方块竖直轴“公转”），像素噪声在该方向被放大，
  这是 docs/experiments/ibvs_vs_pbvs 中 IBVS 朝向精度差、混合方案引入激光朝向的原因。
"""
import math

import numpy as np

# 光学系 → base_link 的标称旋转：光学 z → base x，光学 x → base −y，光学 y → base −z
R_BO = np.array([[0.0, 0.0, 1.0],
                 [-1.0, 0.0, 0.0],
                 [0.0, -1.0, 0.0]])


def rot_rpy(roll=0.0, pitch=0.0, yaw=0.0):
    """绕固定轴 x、y、z 依次旋转（R = Rz·Ry·Rx），用于构造安装误差."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


# ---------------------------------------------------------------- 几何 / 投影
def project_points(points_base, p_c, r_bo, k):
    """
    base_link 中的三维点 → 像素坐标.

    k = (fx, fy, cx, cy)。返回 (px, depth)：px 为 N×2 像素，depth 为光学系 Z（N,）。
    """
    pts = np.atleast_2d(np.asarray(points_base, dtype=float))
    p_o = (pts - np.asarray(p_c, dtype=float)) @ np.asarray(r_bo, dtype=float)
    z = p_o[:, 2]
    fx, fy, cx, cy = k
    with np.errstate(divide='ignore', invalid='ignore'):
        px = np.stack([fx * p_o[:, 0] / z + cx, fy * p_o[:, 1] / z + cy], axis=1)
    return px, z


def pixels_to_normalized(px, k):
    """像素 → 归一化像平面坐标 (x, y)."""
    fx, fy, cx, cy = k
    px = np.atleast_2d(np.asarray(px, dtype=float))
    return np.stack([(px[:, 0] - cx) / fx, (px[:, 1] - cy) / fy], axis=1)


def marker_corners_base(center, normal_xy, size):
    """
    竖直墙面上正方形方块的四个角点（base_link 中）.

    normal_xy 为指向墙内的墙面法向。顺序按图像中的 左上、右上、右下、左下
    （相机面对墙时，沿墙向左 = 图像左侧，z 向上 = 图像上方）。
    """
    c = np.asarray(center, dtype=float)
    n = np.asarray(normal_xy, dtype=float)
    t = np.array([-n[1], n[0], 0.0])            # 沿墙向左
    up = np.array([0.0, 0.0, 1.0])
    h = 0.5 * size
    return np.array([c + h * t + h * up, c - h * t + h * up,
                     c - h * t - h * up, c + h * t - h * up])


def order_corners(px):
    """把任意顺序的四个角点排成 左上、右上、右下、左下（图像坐标，v 向下）."""
    p = np.asarray(px, dtype=float).reshape(4, 2)
    c = p.mean(axis=0)
    ang = np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0])     # 图像系下 −π..π
    p = p[np.argsort(ang)]                               # 从左上方向开始顺时针
    start = int(np.argmin(p[:, 0] + p[:, 1]))            # u + v 最小者为左上
    return np.roll(p, -start, axis=0)


def polygon_centroid(px):
    """多边形（像面四边形）的面积质心，等价于二值色块的一阶矩质心."""
    p = np.asarray(px, dtype=float)
    x, y = p[:, 0], p[:, 1]
    xn, yn = np.roll(x, -1), np.roll(y, -1)
    cross = x * yn - xn * y
    a = 0.5 * cross.sum()
    if abs(a) < 1e-12:
        return p.mean(axis=0)
    return np.array([((x + xn) * cross).sum(), ((y + yn) * cross).sum()]) / (6.0 * a)


def depth_from_wall(xy, wall, p_c, r_bo):
    """
    像点射线与激光墙面求交，返回每个点的光学系深度 Z（无交点处为 nan）.

    wall 为 visual_servo.fit_wall 风格的 dict（base_link 中的 normal、distance）。
    """
    xy = np.atleast_2d(np.asarray(xy, dtype=float))
    n = np.array([wall['normal'][0], wall['normal'][1], 0.0])
    rays_o = np.column_stack([xy, np.ones(len(xy))])      # Z = 1 的光学系方向
    rays_b = rays_o @ np.asarray(r_bo, dtype=float).T
    denom = rays_b @ n
    with np.errstate(divide='ignore', invalid='ignore'):
        z = (wall['distance'] - float(n @ np.asarray(p_c, dtype=float))) / denom
    z[~((z > 0) & np.isfinite(z) & (np.abs(denom) > 1e-9))] = np.nan
    return z


def depth_from_size(xy, size):
    """由方块已知边长与像面四条边的平均长度估计深度（弱透视近似）."""
    p = np.asarray(xy, dtype=float).reshape(4, 2)
    side = np.linalg.norm(p - np.roll(p, -1, axis=0), axis=1).mean()
    return size / side if side > 1e-9 else float('nan')


# ---------------------------------------------------------------- 交互矩阵
def point_interaction(x, y, z):
    """单个归一化像点的 2×6 交互矩阵，对应光学系旋量 (vx, vy, vz, ωx, ωy, ωz)."""
    return np.array([
        [-1.0 / z, 0.0, x / z, x * y, -(1.0 + x * x), y],
        [0.0, -1.0 / z, y / z, 1.0 + y * y, -x * y, -x],
    ])


def base_to_camera_twist(p_c, r_bo=R_BO):
    """6×3 矩阵 W：底盘速度 (vx, vy, wz) → 相机在光学系中的旋量 (v, ω)."""
    r_ob = np.asarray(r_bo, dtype=float).T
    px, py, _ = np.asarray(p_c, dtype=float)
    # v_b + ω_b × p_c，ω_b = (0, 0, wz)
    lin_b = np.array([[1.0, 0.0, -py],
                      [0.0, 1.0, px],
                      [0.0, 0.0, 0.0]])
    ang_b = np.array([[0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0],
                      [0.0, 0.0, 1.0]])
    return np.vstack([r_ob @ lin_b, r_ob @ ang_b])


def feature_jacobian(xy, z, p_c, r_bo=R_BO):
    """
    点特征对底盘速度的雅可比 J（2N×3），ṡ = J·(vx, vy, wz).

    xy 为 N×2 归一化坐标，z 为每个点的深度（标量或 N,）。
    """
    xy = np.atleast_2d(np.asarray(xy, dtype=float))
    z = np.broadcast_to(np.asarray(z, dtype=float), (len(xy),))
    lmat = np.vstack([point_interaction(x, y, zz) for (x, y), zz in zip(xy, z)])
    return lmat @ base_to_camera_twist(p_c, r_bo)


def saturate(u, max_lin=0.2, max_ang=0.3):
    """整体等比缩放 (vx, vy, wz)，使平移速度与角速度都不超限（保持方向）."""
    u = np.asarray(u, dtype=float)
    lin = math.hypot(u[0], u[1])
    scale = 1.0
    if lin > max_lin:
        scale = min(scale, max_lin / lin)
    if abs(u[2]) > max_ang:
        scale = min(scale, max_ang / abs(u[2]))
    return u * scale


# ---------------------------------------------------------------- IBVS 控制器
class IbvsController:
    """
    以方块四角点为特征的 IBVS（3 自由度全向底盘）.

    参数
      p_c, r_bo   相机外参（标称值；与真实安装的偏差只影响收敛过程，不影响收敛点）
      k           相机内参 (fx, fy, cx, cy)
      size        方块边长（米），仅 depth_mode='size' 使用
      depth_mode  'desired' / 'laser' / 'size' / 'mean'，见模块说明
      gain        λ；max_lin / max_ang 速度上限
      smoothing   特征的指数滑动平均系数（与 PBVS 的测量滤波一致）
    期望特征 s* 用 set_reference() 给出：可来自标称模型投影，也可来自示教（在期望位姿实拍）。
    """

    def __init__(self, p_c, k, r_bo=R_BO, size=0.05, depth_mode='desired', gain=0.8,
                 max_lin=0.2, max_ang=0.3, smoothing=0.5):
        self.p_c = np.asarray(p_c, dtype=float)
        self.r_bo = np.asarray(r_bo, dtype=float)
        self.k = tuple(k)
        self.size = size
        self.depth_mode = depth_mode
        self.gain = gain
        self.max_lin = max_lin
        self.max_ang = max_ang
        self.alpha = smoothing
        self.s_star = None
        self.z_star = None
        self._filtered = None

    def set_reference(self, px_star, z_star):
        """设置期望角点（像素，左上/右上/右下/左下）与期望深度（标量或每点）."""
        self.s_star = pixels_to_normalized(px_star, self.k)
        self.z_star = np.broadcast_to(np.asarray(z_star, dtype=float), (4,)).copy()

    def reset_filter(self):
        """清空特征滤波状态（每次对位开始时调用）."""
        self._filtered = None

    def _depth(self, xy, wall):
        if self.depth_mode in ('laser', 'mean') and wall is not None:
            z = depth_from_wall(xy, wall, self.p_c, self.r_bo)
            if np.all(np.isfinite(z)):
                return z
        if self.depth_mode == 'size':
            z = depth_from_size(xy, self.size)
            if math.isfinite(z):
                return np.full(4, z)
        return self.z_star

    def jacobian(self, xy, wall=None):
        """按所选深度方式构造当前使用的 Ĵ（8×3）."""
        if self.depth_mode == 'desired':
            return feature_jacobian(self.s_star, self.z_star, self.p_c, self.r_bo)
        j = feature_jacobian(xy, self._depth(xy, wall), self.p_c, self.r_bo)
        if self.depth_mode == 'mean':
            j = 0.5 * (j + feature_jacobian(self.s_star, self.z_star, self.p_c, self.r_bo))
        return j

    def compute(self, px, wall=None):
        """
        由当前四角点像素计算底盘速度.

        返回 (u, info)：u = (vx, vy, wz)；info 含像面误差 RMS（像素）、
        线性化位姿误差估计 pose_err = Ĵ⁺e（米, 米, 弧度；即还需走的位移的相反数）、Ĵ 的条件数。
        """
        if self.s_star is None:
            raise RuntimeError('未设置期望特征 s*')
        xy = order_corners(pixels_to_normalized(px, self.k))
        if self.alpha > 0 and self._filtered is not None:
            xy = self.alpha * self._filtered + (1 - self.alpha) * xy
        self._filtered = xy
        e = (xy - self.s_star).reshape(-1)
        j = self.jacobian(xy, wall)
        j_pinv = np.linalg.pinv(j)
        pose_err = j_pinv @ e
        u = saturate(-self.gain * pose_err, self.max_lin, self.max_ang)
        fx, fy = self.k[0], self.k[1]
        e_px = (xy - self.s_star) * np.array([fx, fy])
        sv = np.linalg.svd(j, compute_uv=False)
        return u, {
            'err_rms_px': float(np.sqrt(np.mean(e_px ** 2))),
            'pose_err': pose_err,
            'cond': float(sv[0] / sv[-1]),
        }


def ibvs_converged(info, pos_tol=0.01, ang_tol=math.radians(1.0)):
    """IBVS 的停止判据：线性化位姿误差 Ĵ⁺e 落入与 PBVS 相同的位置 / 角度容差."""
    d = info['pose_err']
    return math.hypot(d[0], d[1]) < pos_tol and abs(d[2]) < ang_tol


# ---------------------------------------------------------------- 2.5D 混合
class HybridController:
    """
    2.5D 分区伺服：激光管朝向与离墙距离，图像质心管沿墙横向.

    与 PBVS 的区别：横向误差不经过“射线–墙面求交 + 相机外参”换算成米，而是直接在像面上
    以 x_g → x_g* 闭环（x_g* 可示教），并前馈抵消转向 / 前进引起的质心像面运动，
    使方块在对位过程中尽量停留在期望像面位置附近（不易出视野）。
    """

    def __init__(self, p_c, k, r_bo=R_BO, desired_distance=0.70, kp_lin=0.8, kp_ang=1.5,
                 gain=0.8, max_lin=0.2, max_ang=0.3, smoothing=0.5):
        self.p_c = np.asarray(p_c, dtype=float)
        self.r_bo = np.asarray(r_bo, dtype=float)
        self.k = tuple(k)
        self.desired_distance = desired_distance
        self.kp_lin, self.kp_ang, self.gain = kp_lin, kp_ang, gain
        self.max_lin, self.max_ang = max_lin, max_ang
        self.alpha = smoothing
        self.xg_star = None
        self.yaw_offset = 0.0
        self.dist_offset = 0.0
        self._filtered = None

    def set_reference(self, xg_star, yaw_offset=0.0, dist_offset=0.0):
        """期望质心归一化横坐标；可选的激光朝向 / 距离零偏（示教时记录）."""
        self.xg_star = float(xg_star)
        self.yaw_offset = float(yaw_offset)
        self.dist_offset = float(dist_offset)

    def reset_filter(self):
        """清空测量滤波状态."""
        self._filtered = None

    def measure(self, centroid_px, wall):
        """返回滤波后的 (朝向误差, 离墙距离, x_g, y_g)."""
        xg, yg = pixels_to_normalized(centroid_px, self.k)[0]
        raw = np.array([wall['yaw_error'] - self.yaw_offset,
                        wall['distance'] - self.dist_offset, xg, yg])
        if self.alpha > 0 and self._filtered is not None:
            raw = self.alpha * self._filtered + (1 - self.alpha) * raw
        self._filtered = raw
        return raw

    def compute(self, centroid_px, wall):
        """
        由方块质心像素与激光墙面计算底盘速度.

        返回 (u, info)：info 含 yaw / normal / lateral 三项误差（lateral 为像面误差乘质心深度
        换算的米数，仅用于停止判据与日志）。
        """
        if self.xg_star is None:
            raise RuntimeError('未设置期望质心 x_g*')
        yaw, dist, xg, yg = self.measure(centroid_px, wall)
        n = np.array(wall['normal'])
        t = np.array([-n[1], n[0]])
        wall_f = {'normal': wall['normal'], 'distance': dist}
        zc = depth_from_wall([[xg, yg]], wall_f, self.p_c, self.r_bo)[0]
        if not math.isfinite(zc):
            zc = self.desired_distance - float(self.p_c[0])
        e_n = dist - self.desired_distance
        e_x = xg - self.xg_star

        wz = float(np.clip(self.kp_ang * yaw, -self.max_ang, self.max_ang))
        v_n = float(np.clip(self.kp_lin * e_n, -self.max_lin, self.max_lin))
        jg = feature_jacobian([[xg, yg]], zc, self.p_c, self.r_bo)[0]   # 质心 x 行
        jn, jt = float(jg[:2] @ n), float(jg[:2] @ t)
        # 速度预算优先分给横向（保持方块在像面期望位置 = 保视野），剩余给法向；
        # 横向本身超限时同比降低转速，使转向引起的像面运动仍能被完全抵消
        v_t = 0.0
        if abs(jt) > 1e-9:
            cap = 0.7 * self.max_lin                         # 给前馈留 30 % 余量
            v_t0 = float(np.clip(-self.gain * e_x / jt, -cap, cap))
            ff = -(jn * v_n + jg[2] * wz) / jt              # 前馈：抵消转向 / 前进的像面运动
            room = self.max_lin - abs(v_t0)
            if abs(ff) > room:                              # 前馈放不下：转向与前进同比放慢
                k = room / abs(ff)
                wz, v_n, ff = wz * k, v_n * k, ff * k
            v_t = v_t0 + ff
        lim = math.sqrt(max(self.max_lin ** 2 - v_t ** 2, 0.0))
        if abs(v_n) > lim:                                  # 法向只用剩余预算，同步修正前馈
            v_n_new = math.copysign(lim, v_n)
            if abs(jt) > 1e-9:
                v_t -= jn * (v_n_new - v_n) / jt
            v_n = v_n_new
        vxy = v_n * n + v_t * t
        lateral = -e_x * zc        # x 右为正 → 方块在期望位置右侧时需向右（−t）移动
        return np.array([vxy[0], vxy[1], wz]), {
            'yaw': float(yaw), 'normal': float(e_n), 'lateral': float(lateral),
            'depth': float(zc),
        }


def hybrid_converged(info, pos_tol=0.01, ang_tol=math.radians(1.0)):
    """混合方案的停止判据：与 PBVS 相同的位置 / 角度容差."""
    return math.hypot(info['normal'], info['lateral']) < pos_tol and abs(info['yaw']) < ang_tol


# ---------------------------------------------------------------- 检测
def detect_marker_corners(bgr, hsv_lower=(20, 100, 100), hsv_upper=(30, 255, 255),
                          min_area=30, refine=True):
    """
    HSV 阈值 + 轮廓多边形近似检测方块四角点.

    返回 4×2 像素数组（左上、右上、右下、左下）；未检测到或不是四边形时返回 None。
    refine=True 时用 cornerSubPix 做亚像素细化（在灰度化的掩码上）。
    """
    import cv2

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(hsv_lower), np.array(hsv_upper))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < min_area:
        return None
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.04 * peri, True)
    if len(approx) != 4:
        return None
    pts = approx.reshape(4, 2).astype(np.float32)
    if refine:
        blur = cv2.GaussianBlur(mask, (5, 5), 0)
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
        pts = cv2.cornerSubPix(blur, pts.reshape(-1, 1, 2), (3, 3), (-1, -1), crit)
    return order_corners(pts.reshape(4, 2))
