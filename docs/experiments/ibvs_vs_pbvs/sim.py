"""
离线运动学仿真器：PBVS / IBVS / 2.5D 混合视觉伺服对比（不需要 ROS 与 Gazebo）.

世界：墙面为平面 x_w = 0（法向 +x 指向墙内），方块中心位于 (0, 0, h)，机器人在 x < 0 一侧。
期望位姿：(−D, 0, 0)，即离墙 D、正对方块（与 navigation_cutting_task 的 visual 模式一致）。

传感器（“真值”参数可与控制器使用的标称参数不同，以模拟标定误差）：
- 相机：针孔模型，640×480，四角点像素 + 高斯噪声；任一角点出画面或随机丢帧则本帧无检测；
- 激光：360 线，0.98 m 高处水平扫描（只看到墙面），每束高斯测距噪声，复用 visual_servo.fit_wall。
底盘：理想全向运动学，10 Hz 控制、零阶保持（每周期 10 个子步积分）。
"""
from dataclasses import dataclass, field, replace
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'src', 'scr_arm_controller'))

from scr_arm_controller import ibvs  # noqa: E402
from scr_arm_controller import visual_servo as vs  # noqa: E402

# ------------------------------------------------------------------ 标称参数
K_NOM = (381.4, 381.4, 319.5, 239.5)
IMG_W, IMG_H = 640, 480
# 相机光心在 base_link 中：link1 原点 (0.038668, −0.0001, 0.048 + joint1) + (0.11, 0.0215, 0.5)，
# 视觉伺服观测位姿 joint1 = −0.15
P_C_NOM = np.array([0.148668, 0.0214, 0.398])
MARKER_H = 0.6 - 0.1024        # 方块中心离地 0.6 m，base_link 离地 0.1024 m
MARKER_SIZE = 0.05
D_DES = 0.70
TOOL_FRONT = 0.676             # 机械臂伸缩轴缩回时刀具前端离底盘中心（对位时）
DT = 0.1
SUBSTEPS = 10
MAX_STEPS = 500                # 50 s
LOST_LIMIT = 20                # 连续 2 s 无测量判失败
REQUIRED = 5                   # 连续 5 个周期满足判据才停止
POS_TOL = 0.01
ANG_TOL = math.radians(1.0)
SCAN_N = 360
SCAN_AMIN = -math.pi
SCAN_INC = 2 * math.pi / SCAN_N


@dataclass
class Scenario:
    """一次实验条件：噪声、标定误差、参考来源."""

    pixel_noise: float = 0.0       # 每个角点像素噪声 σ（px）
    range_noise: float = 0.0       # 每束激光测距噪声 σ（m）
    dropout: float = 0.0           # 每帧检测丢失概率
    cam_yaw: float = 0.0           # 相机安装误差（rad，绕 base_link 固定轴）
    cam_pitch: float = 0.0
    cam_roll: float = 0.0
    cam_dx: float = 0.0            # 相机位置误差（m）
    cam_dy: float = 0.0
    cam_dz: float = 0.0
    lidar_yaw: float = 0.0         # 激光安装偏航误差（rad）
    lidar_dx: float = 0.0          # 激光前后位置误差（m）
    fx_scale: float = 1.0          # 真实焦距 = 标称 × fx_scale（fx, fy 同比）
    cx_err: float = 0.0            # 真实主点 − 标称主点（px）
    cy_err: float = 0.0
    marker_dz: float = 0.0         # 运行时方块高度相对示教 / 标称的偏差（m）
    reference: str = 'model'       # 'model'：期望值由标称模型算；'taught'：在期望位姿示教
    teach_frames: int = 20
    depth_mode: str = 'desired'    # IBVS 雅可比深度
    marker_size: float = MARKER_SIZE
    extra: dict = field(default_factory=dict)


class World:
    """真值世界 + 传感器模型."""

    def __init__(self, sc: Scenario, rng):
        self.sc = sc
        self.rng = rng
        self.k_true = (K_NOM[0] * sc.fx_scale, K_NOM[1] * sc.fx_scale,
                       K_NOM[2] + sc.cx_err, K_NOM[3] + sc.cy_err)
        self.p_c_true = P_C_NOM + np.array([sc.cam_dx, sc.cam_dy, sc.cam_dz])
        self.r_bo_true = ibvs.rot_rpy(sc.cam_roll, sc.cam_pitch, sc.cam_yaw) @ ibvs.R_BO
        self.marker_h = MARKER_H + sc.marker_dz
        self.corners_w = ibvs.marker_corners_base([0.0, 0.0, self.marker_h], [1.0, 0.0],
                                                  sc.marker_size)

    @staticmethod
    def to_base(points_w, pose):
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        rx, ry = points_w[:, 0] - x, points_w[:, 1] - y
        return np.column_stack([c * rx + s * ry, -s * rx + c * ry, points_w[:, 2]])

    def true_corners(self, pose, corners_w=None):
        """无噪声角点像素与深度."""
        cw = self.corners_w if corners_w is None else corners_w
        return ibvs.project_points(self.to_base(cw, pose), self.p_c_true, self.r_bo_true,
                                   self.k_true)

    def fov_margin(self, pose):
        """四角点到画面边界的最小距离（px），在相机后方返回 −inf."""
        px, z = self.true_corners(pose)
        if np.any(z <= 0.02) or not np.all(np.isfinite(px)):
            return -math.inf
        return float(min(px[:, 0].min(), IMG_W - 1 - px[:, 0].max(),
                         px[:, 1].min(), IMG_H - 1 - px[:, 1].max()))

    def detect(self, pose, allow_dropout=True):
        """一帧角点检测；出视野 / 丢帧返回 None."""
        if allow_dropout and self.sc.dropout > 0 and self.rng.random() < self.sc.dropout:
            return None
        if self.fov_margin(pose) < 0:
            return None
        px, _ = self.true_corners(pose)
        if self.sc.pixel_noise > 0:
            px = px + self.rng.normal(0.0, self.sc.pixel_noise, px.shape)
        return px

    def scan(self, pose):
        """激光拟合墙面（激光系 = 控制器认为的 base_link），失败返回 None."""
        x, y, th = pose
        lx = x + self.sc.lidar_dx * math.cos(th)
        a = SCAN_AMIN + SCAN_INC * np.arange(SCAN_N)
        cphi = np.cos(th + self.sc.lidar_yaw + a)
        with np.errstate(divide='ignore'):
            r = np.where(cphi > 1e-3, (0.0 - lx) / cphi, np.inf)
        if self.sc.range_noise > 0:
            r = r + self.rng.normal(0.0, self.sc.range_noise, SCAN_N)
        return vs.fit_wall(r, SCAN_AMIN, SCAN_INC)


def truth_error(pose):
    """真值误差：法向（m）、横向（m）、偏航（rad）、刀具前端横向（m）."""
    x, y, th = pose
    return {'normal': -x - D_DES, 'lateral': y, 'yaw': th,
            'tip_lateral': y + TOOL_FRONT * math.sin(th)}


def nominal_reference(size=MARKER_SIZE):
    """标称模型下期望位姿的角点像素与深度（方块边长 size 视为已知）."""
    cw = ibvs.marker_corners_base([D_DES, 0.0, MARKER_H], [1.0, 0.0], size)
    return ibvs.project_points(cw, P_C_NOM, ibvs.R_BO, K_NOM)


DESIRED_POSE = np.array([-D_DES, 0.0, 0.0])


# ------------------------------------------------------------------ 方法
class PbvsMethod:
    """现有 PBVS（visual_servo.py），参数与 navigation_cutting_task 的 visual 模式一致."""

    name = 'PBVS'
    needs_scan = True

    def __init__(self, sc):
        self.sc = sc
        self.offset = np.zeros(3)
        self.filt = None

    def _raw(self, px, wall):
        u, v = ibvs.polygon_centroid(px)
        ray = ibvs.R_BO @ vs.pixel_to_ray(u, v, *K_NOM)
        marker = vs.intersect_ray_wall(P_C_NOM, ray, wall['normal'], wall['distance'])
        if marker is None:
            return None
        err = vs.pbvs_error(marker, wall, D_DES)
        return err, np.array([err['yaw'], err['normal'], err['lateral']])

    def teach(self, world):
        raws = []
        for _ in range(self.sc.teach_frames):
            px, wall = world.detect(DESIRED_POSE, False), world.scan(DESIRED_POSE)
            r = self._raw(px, wall)
            raws.append(r[1])
        self.offset = np.mean(raws, axis=0)

    def step(self, px, wall):
        if px is None or wall is None:
            return None, False, {}
        r = self._raw(px, wall)
        if r is None:
            return None, False, {}
        err, raw = r
        raw = raw - self.offset
        self.filt = raw if self.filt is None else 0.5 * self.filt + 0.5 * raw
        err['yaw'], err['normal'], err['lateral'] = self.filt.tolist()
        conv = math.hypot(err['normal'], err['lateral']) < POS_TOL and abs(err['yaw']) < ANG_TOL
        u = vs.pbvs_command(err, pos_deadband=POS_TOL * 0.5, ang_deadband=ANG_TOL * 0.5)
        return np.array(u), conv, {}


class IbvsMethod:
    """四角点 IBVS."""

    name = 'IBVS'

    def __init__(self, sc):
        self.sc = sc
        self.needs_scan = sc.depth_mode in ('laser', 'mean')
        self.ctl = ibvs.IbvsController(P_C_NOM, K_NOM, size=sc.marker_size,
                                       depth_mode=sc.depth_mode)
        px, z = nominal_reference(sc.marker_size)
        self.ctl.set_reference(px, z)

    def teach(self, world):
        pts = [world.detect(DESIRED_POSE, False) for _ in range(self.sc.teach_frames)]
        _, z = nominal_reference(self.sc.marker_size)
        self.ctl.set_reference(np.mean(pts, axis=0), z)

    def step(self, px, wall):
        if px is None:
            return None, False, {}
        u, info = self.ctl.compute(px, wall)
        return u, ibvs.ibvs_converged(info, POS_TOL, ANG_TOL), info


class HybridMethod:
    """2.5D 混合：激光朝向 / 距离 + 图像质心横向."""

    name = 'Hybrid'
    needs_scan = True

    def __init__(self, sc):
        self.sc = sc
        self.ctl = ibvs.HybridController(P_C_NOM, K_NOM, desired_distance=D_DES)
        px, _ = nominal_reference()
        xg = ibvs.pixels_to_normalized(ibvs.polygon_centroid(px), K_NOM)[0, 0]
        self.ctl.set_reference(xg)

    def teach(self, world):
        cs, yaws, ds = [], [], []
        for _ in range(self.sc.teach_frames):
            cs.append(ibvs.polygon_centroid(world.detect(DESIRED_POSE, False)))
            w = world.scan(DESIRED_POSE)
            yaws.append(w['yaw_error'])
            ds.append(w['distance'])
        xg = ibvs.pixels_to_normalized(np.mean(cs, axis=0), K_NOM)[0, 0]
        self.ctl.set_reference(xg, np.mean(yaws), np.mean(ds) - D_DES)

    def step(self, px, wall):
        if px is None or wall is None:
            return None, False, {}
        u, info = self.ctl.compute(ibvs.polygon_centroid(px), wall)
        return u, ibvs.hybrid_converged(info, POS_TOL, ANG_TOL), info


METHODS = {'PBVS': PbvsMethod, 'IBVS': IbvsMethod, 'Hybrid': HybridMethod}


# ------------------------------------------------------------------ 闭环
def integrate(pose, u, dt=DT, substeps=SUBSTEPS):
    """全向底盘运动学积分（base_link 速度，零阶保持）."""
    x, y, th = pose
    h = dt / substeps
    for _ in range(substeps):
        c, s = math.cos(th), math.sin(th)
        x += (u[0] * c - u[1] * s) * h
        y += (u[0] * s + u[1] * c) * h
        th += u[2] * h
    return np.array([x, y, th])


def run_trial(method, sc, start, seed, keep_traj=False, horizon=None):
    """
    单次对位试验，返回指标 dict（可选附带轨迹）.

    horizon=None：停止模式——按各方法自身判据连续 5 个周期满足即停（与 Gazebo 实验一致），
    误差取停止时的真值。horizon=秒数：稳态模式——不停止，运行固定时长，
    误差取最后 5 s 真值的 RMS，用来衡量与停止判据无关的精度下限 / 偏差。
    """
    rng = np.random.default_rng(seed)
    if sc.reference == 'taught':
        # 示教时方块在标称高度；运行时可偏 marker_dz
        teach_world = World(replace(sc, marker_dz=0.0, dropout=0.0), rng)
    world = World(sc, rng)
    m = METHODS[method](sc)
    if sc.reference == 'taught':
        m.teach(teach_world)
    pose = np.array(start, dtype=float)
    start_margin = world.fov_margin(pose)
    lost = conv = 0
    min_margin = math.inf
    min_clear = math.inf
    path = 0.0
    traj, centroid = [], []
    status = 'timeout'
    n_steps = MAX_STEPS if horizon is None else int(round(horizon / DT))
    steps = n_steps
    tail = []
    last_out = -1
    for k in range(n_steps):
        te = truth_error(pose)
        if math.hypot(te['normal'], te['lateral']) >= POS_TOL or abs(te['yaw']) >= ANG_TOL:
            last_out = k
        if horizon is not None and k >= n_steps - int(5.0 / DT):
            tail.append(pose.copy())
        margin = world.fov_margin(pose)
        min_margin = min(min_margin, margin)
        min_clear = min(min_clear, -pose[0] - TOOL_FRONT * math.cos(pose[2]))
        if keep_traj:
            traj.append(pose.copy())
            px_t, _ = world.true_corners(pose)
            centroid.append(ibvs.polygon_centroid(px_t) if margin > -math.inf else [np.nan] * 2)
        px = world.detect(pose)
        wall = world.scan(pose) if m.needs_scan else None
        u, ok, _ = m.step(px, wall)
        if u is None:
            lost += 1
            conv = 0
            if lost >= LOST_LIMIT:
                status = 'lost_fov' if margin < 0 else 'lost'
                steps = k + 1
                break
            continue                        # 测量丢失：原地停住
        lost = 0
        conv = conv + 1 if ok else 0
        if horizon is None and conv >= REQUIRED:
            status = 'converged'
            steps = k + 1
            break
        new = integrate(pose, u)
        path += math.hypot(new[0] - pose[0], new[1] - pose[1])
        pose = new
    e = truth_error(pose)
    if horizon is not None and status == 'timeout':
        status = 'horizon'
        if tail:
            es = [truth_error(p) for p in tail]
            e = {key: math.copysign(math.sqrt(np.mean([q[key] ** 2 for q in es])),
                                    np.mean([q[key] for q in es])) for key in es[0]}
    pos = math.hypot(e['normal'], e['lateral'])
    res = {
        'method': method, 'status': status, 'time': steps * DT,
        'settle_time': (last_out + 1) * DT,
        'pos_err': pos, 'normal': e['normal'], 'lateral': e['lateral'], 'yaw': e['yaw'],
        'tip_lateral': e['tip_lateral'],
        'success': (status in ('converged', 'horizon') and pos < POS_TOL
                    and abs(e['yaw']) < ANG_TOL),
        'min_fov_margin': min_margin, 'fov_drop': start_margin - min_margin,
        'min_clearance': min_clear, 'path': path,
        'fov_lost': min_margin < 0,
    }
    if keep_traj:
        res['traj'] = np.array(traj)
        res['centroid'] = np.array(centroid)
    return res


def random_starts(n, seed=42, dist=(1.0, 1.5), lateral=0.3, yaw_deg=10.0, margin=10.0):
    """
    与 Gazebo 对位实验相同分布的随机起点：离墙 dist、横向 ±lateral、朝向 ±yaw_deg.

    只保留起始时方块完整在画面内（离边界 ≥ margin px）的起点——对位前一步是扫描找方块，
    起始不可见的情形不属于伺服本身的问题。
    """
    rng = np.random.default_rng(seed)
    world = World(Scenario(), rng)
    out = []
    while len(out) < n:
        s = np.array([-rng.uniform(*dist), rng.uniform(-lateral, lateral),
                      math.radians(rng.uniform(-yaw_deg, yaw_deg))])
        if world.fov_margin(s) >= margin:
            out.append(s)
    return np.array(out)
