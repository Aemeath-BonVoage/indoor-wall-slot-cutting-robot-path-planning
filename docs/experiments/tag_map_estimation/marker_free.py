#!/usr/bin/env python3
"""
不依赖标记的沿墙参照：本机 2D 激光在工作距离下能给出的沿墙精度（定量估计，不是实现）.

  OPENBLAS_NUM_THREADS=1 nice -n 19 python3 marker_free.py       # 约 1 min

激光参数取自 scr_robot_gazebo.urdf：360 束 / 360°（角分辨率 1°），测距噪声 σ = 10 mm（高斯），
测距分辨率 0.01 m，量程 0.1–20 m；安装在 base_link 正上方（lidar_tf.yaml：平移 (0, 0, 0.98)）。
底盘离墙 d = 0.70 m（观测位姿；切割时约 0.55 m，结果相近）。Gazebo 中量化与加噪的先后
顺序未核实，两种顺序都算：
  noise→quantize：先加噪声再按 1 cm 取整（噪声远大于量化步长，取整近似无偏）；
  quantize→noise：先取整再加噪声（每个位姿有确定的取整误差，静止时不随帧平均）。

1. 墙角（与相邻墙面的内角）：在侧墙上选离墙角 0.1–3.0 m、与侧墙距离 < 5 cm 的点做总体最小
   二乘直线拟合，与正面墙的拟合直线求交得墙角沿墙位置。按离墙角的沿墙距离 L 计算：
   - 单帧标准差（底盘朝向每帧随机偏 ±0.5°，即“运动中”的抖动）；
   - 静止偏差：100 个随机静止位姿，每个位姿 30 帧平均后的误差分布（标准差、最大值）。
   墙体按 room.world 的 room3：墙角两侧墙面平直、无踢脚线、无遮挡（理想情形）。
2. 门洞 / 窗洞的边缘（深度不连续）：边缘位于相邻两束之间，单帧误差在束间距内均匀分布；
   束间距 = d·Δθ / cos²β（β 为边缘方位角）。解析计算。
输出 marker_free.csv。
"""
import csv
import math
import os

import numpy as np

from common import HERE

N_BEAMS = 360
DTHETA = 2 * math.pi / N_BEAMS
SIGMA_R = 0.010
RES = 0.01
D = 0.70
ROOM_DEPTH = 8.0


def scan(L, psi, rng, order, far=4.85):
    """
    底盘在 (0, D)（墙面 y = 0，侧墙 x = −L，另一侧墙 x = far − L），朝向 −y 偏 psi.

    返回点云（世界系，单位 m）与每束的真实落点所在墙面编号（0 正面，1 侧墙，2 其他）。
    """
    th = -math.pi / 2 + psi + (np.arange(N_BEAMS) - N_BEAMS / 2) * DTHETA
    c, s = np.cos(th), np.sin(th)
    big = 1e9
    r_front = np.where(s < -1e-9, -D / np.where(s < -1e-9, s, -1), big)
    r_side = np.where(c < -1e-9, -L / np.where(c < -1e-9, c, -1), big)
    r_other = np.where(c > 1e-9, (far - L) / np.where(c > 1e-9, c, 1), big)
    r_back = np.where(s > 1e-9, (ROOM_DEPTH - D) / np.where(s > 1e-9, s, 1), big)
    R = np.vstack([r_front, r_side, r_other, r_back])
    k = np.argmin(R, axis=0)
    r = R[k, np.arange(N_BEAMS)]
    if order == 'noise→quantize':
        rm = np.round((r + SIGMA_R * rng.standard_normal(N_BEAMS)) / RES) * RES
    else:
        rm = np.round(r / RES) * RES + SIGMA_R * rng.standard_normal(N_BEAMS)
    ok = (rm > 0.1) & (rm < 20.0)
    th_meas = -math.pi / 2 + (np.arange(N_BEAMS) - N_BEAMS / 2) * DTHETA   # 机体系的束角
    # 以“估计的朝向 = 0”把点放回世界系（朝向误差由正面墙拟合吸收，见 corner_estimate）
    x = rm * np.cos(th_meas)
    y = D + rm * np.sin(th_meas)
    return np.column_stack([x, y])[ok], k[ok]


def tls(p):
    c = p.mean(axis=0)
    _, _, vt = np.linalg.svd(p - c)
    n = vt[-1]
    return n, float(n @ c)


def corner_estimate(pts, L):
    """正面墙直线与侧墙直线求交：墙角在正面墙方向上相对底盘的位置（机体系），返回误差."""
    front = pts[(np.abs(pts[:, 1]) < 0.05) & (np.abs(pts[:, 0]) < 1.0)]
    side = pts[(np.abs(pts[:, 0] + L) < 0.05) & (pts[:, 1] > 0.1) & (pts[:, 1] < 3.0)]
    if len(front) < 10 or len(side) < 5:
        return math.nan, len(side)
    nf, cf = tls(front)
    ns, cs = tls(side)
    A = np.vstack([nf, ns])
    xy = np.linalg.solve(A, [cf, cs])
    # 沿正面墙方向的距离：把交点投影到正面墙方向（墙面方向 = 正面墙法向旋转 90°）
    t = np.array([-nf[1], nf[0]])
    # 底盘在正面墙上的垂足
    foot = np.array([0.0, D]) - (nf @ np.array([0.0, D]) - cf) * nf
    along = float(t @ (xy - foot))
    return abs(along) - L, len(side)


def corners(rng):
    rows = []
    for order in ('noise→quantize', 'quantize→noise'):
        for L in (0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.6):
            # 运动中：每帧朝向随机偏 ±0.5°（束相对墙面的位置随之改变）
            e = []
            npts = []
            for _ in range(300):
                pts, _ = scan(L, rng.uniform(-0.0087, 0.0087), rng, order)
                err, n = corner_estimate(pts, L)
                e.append(err)
                npts.append(n)
            e = np.array(e)
            # 静止：100 个随机位姿，每个 30 帧平均
            bias = []
            for _ in range(100):
                psi = rng.uniform(-0.0087, 0.0087)
                bias.append(np.nanmean([corner_estimate(scan(L, psi, rng, order)[0], L)[0]
                                        for _ in range(30)]))
            bias = np.array(bias)
            rows.append(dict(feature='corner', order=order, distance_m=L,
                             points=float(np.mean(npts)),
                             frame_sigma_mm=np.nanstd(e) * 1e3,
                             frame_mean_mm=np.nanmean(e) * 1e3,
                             static30_sigma_mm=np.nanstd(bias) * 1e3,
                             static30_max_mm=np.nanmax(np.abs(bias)) * 1e3))
            print(rows[-1])
    return rows


def edges():
    rows = []
    for e in (0.3, 0.6, 1.0, 1.5, 2.0):
        beta = math.atan2(e, D)
        spacing = D * DTHETA / math.cos(beta) ** 2
        rows.append(dict(feature='opening edge', order='analytic', distance_m=e, points=math.nan,
                         frame_sigma_mm=spacing / math.sqrt(12) * 1e3, frame_mean_mm=0.0,
                         static30_sigma_mm=spacing / math.sqrt(12) * 1e3,
                         static30_max_mm=spacing / 2 * 1e3))
        print(rows[-1])
    return rows


def main():
    rng = np.random.default_rng(7)
    rows = corners(rng) + edges()
    with open(os.path.join(HERE, 'marker_free.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        for r in rows:
            w.writerow({k: (f'{v:.4g}' if isinstance(v, float) else v) for k, v in r.items()})


if __name__ == '__main__':
    main()
