#!/usr/bin/env python3
"""
精对位控制律的离线分析（不需要 ROS / Gazebo）.

1. 线性区离散闭环极点随 k·T 的变化（含一拍计算延迟与底盘速度环一阶滞后）；
2. 蒙特卡洛闭环仿真：第一版（指数平滑 + 比例 / 最小速度 / 死区 + 1 cm / 1° 停止判据）
   与本版（EKF + 平方根整形饱和律 + “容差内且静止”收敛判据），以及本版在标记间歇遮挡下；
3. 单次试验的时间响应。

被控对象与 Gazebo 中一致的部分：全向底盘、速度环一阶滞后 τ = 1/20 s（scr_planar_move 的
k_lin = 20）、激光 10 Hz、相机 30 Hz、测量延迟 50 ms；噪声水平取 Gazebo 实测量级。
控制器、估计器、收敛判据直接调用 scr_arm_controller 中的同一份代码。

  python3 docs/experiments/fine_alignment/analysis.py [--trials 200] [--out docs/experiments/fine_alignment]

图使用 docs/figstyle 的统一样式（英文，PDF + PNG）。
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'src', 'scr_arm_controller'))
sys.path.insert(0, os.path.join(HERE, '..', '..'))

from scr_arm_controller.docking_control import ConvergenceMonitor, WallAlignController  # noqa
from scr_arm_controller.visual_servo import pbvs_command  # noqa: E402
from scr_arm_controller.wall_ekf import WallEKF  # noqa: E402

D = 0.70                 # 目标离墙距离
TAU = 0.05               # 底盘速度环时间常数（s）
DT_SIM = 0.002           # 物理积分步长
LATENCY = 0.05           # 测量延迟（s）
SIG_LIDAR = (1.3e-3, math.radians(0.26))     # 单帧激光墙面拟合：距离 / 角度
SIG_TAG = (3e-3, 0.3e-3)                      # 标记中心：深度 / 横向（base_link）
SIG_BLOB_LAT = 0.4e-3                         # 第一版色块沿墙位置


# ---------------------------------------------------------------------------
# 1. 线性区离散闭环极点
# ---------------------------------------------------------------------------
def closed_loop_poles(kT, T=0.05, tau=TAU):
    """
    单轴线性模型：e[k+1] = e[k] + T·v[k]；v[k+1] = a·v[k] + (1−a)·c[k−1]（ZOH 一阶滞后，
    指令晚一拍生效）；c[k] = −k·e[k]。状态 (e, v, c_prev)。
    """
    a = math.exp(-T / tau)
    k = kT / T
    A = np.array([[1.0, T, 0.0],
                  [0.0, a, 1.0 - a],
                  [-k, 0.0, 0.0]])
    return np.linalg.eigvals(A)


# ---------------------------------------------------------------------------
# 2. 闭环仿真
# ---------------------------------------------------------------------------
class Plant:
    """墙面系中的全向底盘：状态 (d, u, ψ)，base_link 速度带一阶滞后."""

    def __init__(self, d, u, psi):
        self.x = np.array([d, u, psi], dtype=float)
        self.v = np.zeros(3)
        self.t = 0.0
        self.odom = np.zeros(3)          # 里程计（任意固定系中的 x, y, yaw）
        self.hist = []                   # (t, 真值状态, 里程计)

    def step(self, cmd, dt):
        n = int(round(dt / DT_SIM))
        for _ in range(n):
            self.v += (np.asarray(cmd) - self.v) * (DT_SIM / TAU)
            vx, vy, w = self.v
            d, u, psi = self.x
            c, s = math.cos(psi), math.sin(psi)
            self.x += DT_SIM * np.array([-(vx * c - vy * s), -(vx * s + vy * c), w])
            co, so = math.cos(self.odom[2]), math.sin(self.odom[2])
            self.odom += DT_SIM * np.array([vx * co - vy * so, vx * so + vy * co, w])
            self.t += DT_SIM
            self.hist.append((self.t, self.x.copy(), self.odom.copy()))

    def at(self, t):
        if not self.hist:
            return self.t, self.x.copy(), self.odom.copy()
        i = max(0, min(len(self.hist) - 1, int(round(t / DT_SIM)) - 1))
        return self.hist[i]


def rel(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    ex, ey = b[0] - a[0], b[1] - a[1]
    return c * ex + s * ey, -s * ex + c * ey, math.atan2(math.sin(b[2] - a[2]),
                                                          math.cos(b[2] - a[2]))


def meas_lidar(x, rng):
    return x[0] + rng.normal(0, SIG_LIDAR[0]), -x[2] + rng.normal(0, SIG_LIDAR[1])


def meas_tag(x, rng):
    d, u, psi = x
    c, s = math.cos(psi), math.sin(psi)
    p = np.array([d * c + u * s, -d * s + u * c])
    return p + np.array([rng.normal(0, SIG_TAG[0]), rng.normal(0, SIG_TAG[1])])


def run_v1(start, rng, T=0.1, t_max=50.0):
    """第一版：10 Hz，测量取最新一帧（无延迟补偿）+ EMA 0.5，pbvs_command，1 cm / 1° × 5."""
    pl = Plant(*start)
    filt, conv, trace = None, 0, []
    while pl.t < t_max:
        _, xm, _ = pl.at(pl.t - LATENCY)
        d_m, phi_m = meas_lidar(xm, rng)
        lat = xm[1] + rng.normal(0, SIG_BLOB_LAT)
        raw = np.array([phi_m, d_m - D, lat])
        filt = raw if filt is None else 0.5 * filt + 0.5 * raw
        yaw, normal, lateral = filt
        if math.hypot(normal, lateral) < 0.01 and abs(yaw) < math.radians(1.0):
            conv += 1
            if conv >= 5:
                break
        else:
            conv = 0
        n = np.array([math.cos(yaw), math.sin(yaw)])
        err = {'yaw': yaw, 'normal': normal, 'lateral': lateral, 'n': n,
               't': np.array([-n[1], n[0]])}
        cmd = pbvs_command(err, pos_deadband=0.005, ang_deadband=math.radians(0.5))
        trace.append((pl.t, *pl.x, *cmd))
        pl.step(cmd, T)
    pl.step((0.0, 0.0, 0.0), 0.5)       # 停车后滑行
    return pl, trace


def run_v2(start, rng, T=0.05, t_max=50.0, occlusion=None, kw=None):
    """本版：EKF（里程计预测 + 延迟补偿）+ 平方根整形饱和律 + 收敛判据."""
    kw = kw or {}
    pl = Plant(*start)
    ekf = WallEKF()
    ctrl = WallAlignController(dt=T, **kw)
    mon = ConvergenceMonitor()
    last_odom, next_lidar, next_cam, trace = None, 0.0, 0.0, []
    u_known = False
    while pl.t < t_max:
        _, _, odom_now = pl.at(pl.t)
        if ekf.initialized and last_odom is not None:
            ekf.predict(*rel(last_odom, odom_now))
        last_odom = odom_now.copy()
        # 本周期内到达的测量（按采集时间戳经里程计变换到当前时刻）
        while next_lidar <= pl.t - LATENCY:
            _, xm, om = pl.at(next_lidar)
            d_m, phi_m = meas_lidar(xm, rng)
            dx, dy, dyaw = rel(odom_now, om)
            n = np.array([math.cos(phi_m + dyaw), math.sin(phi_m + dyaw)])
            dist = d_m + n @ np.array([dx, dy])
            phi = math.atan2(n[1], n[0])
            if not ekf.initialized:
                ekf.reset(*WallEKF.init_from((dist, phi))[:3])
            else:
                ekf.update_wall(dist, phi, np.diag(np.square(SIG_LIDAR)))
            next_lidar += 0.1
        while next_cam <= pl.t - LATENCY:
            occluded = occlusion is not None and occlusion(next_cam)
            if ekf.initialized and not occluded:
                _, xm, om = pl.at(next_cam)
                p = meas_tag(xm, rng)
                dx, dy, dyaw = rel(odom_now, om)
                c, s = math.cos(dyaw), math.sin(dyaw)
                Rm = np.array([[c, -s], [s, c]])
                p = Rm @ p + np.array([dx, dy])
                if not u_known:
                    ekf.x[1] = WallEKF.init_from((ekf.x[0], -ekf.x[2]), p)[1]
                    ekf.P[1, 1] = 1e-4
                    u_known = True
                ekf.update_tag(p, Rm @ np.diag(np.square(SIG_TAG)) @ Rm.T)
            next_cam += 1.0 / 30
        cmd = (0.0, 0.0, 0.0)
        if ekf.initialized and u_known:
            errs = ctrl.errors(ekf.x, (D, 0.0, 0.0))
            if mon.update(errs, pl.v, ekf.sigma()):
                break
            cmd = ctrl.command(ekf.x, (D, 0.0, 0.0))
        trace.append((pl.t, *pl.x, *cmd))
        pl.step(cmd, T)
    pl.step((0.0, 0.0, 0.0), 0.5)
    return pl, trace


def final_errors(pl):
    d, u, psi = pl.x
    return (d - D) * 1000, u * 1000, math.degrees(psi), pl.t


def starts(n, seed=42):
    rng = np.random.default_rng(seed)
    return [(D + rng.uniform(0.3, 0.8), rng.uniform(-0.3, 0.3), math.radians(rng.uniform(-10, 10)))
            for _ in range(n)]


def summarize(rows):
    a = np.array(rows)
    pos = np.hypot(a[:, 0], a[:, 1])
    return {'pos_mean_mm': pos.mean(), 'pos_std_mm': pos.std(), 'pos_p95_mm': np.percentile(pos, 95),
            'pos_max_mm': pos.max(), 'lat_abs_mean_mm': np.abs(a[:, 1]).mean(),
            'norm_abs_mean_mm': np.abs(a[:, 0]).mean(), 'yaw_abs_mean_deg': np.abs(a[:, 2]).mean(),
            'time_mean_s': a[:, 3].mean()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trials', type=int, default=200)
    ap.add_argument('--out', default=HERE)
    args = ap.parse_args()

    import figstyle as fs

    # ---- 1. 极点
    kTs = np.linspace(0.01, 1.2, 240)
    rho = [max(abs(closed_loop_poles(x))) for x in kTs]
    fig, ax = fs.subplots(1, 2, width='double', height=2.4,
                          gridspec_kw=dict(width_ratios=[1.6, 1]))
    ax[0].plot(kTs, rho, color=fs.BLACK, lw=1.0)
    ax[0].axhline(1.0, **fs.line('reference', lw=0.6))
    ax[0].text(1.18, 1.008, 'stability limit |z| = 1', ha='right', va='bottom', fontsize=7)
    for kT, role, lab, ty in [(0.1, 'proposed', 'v2: k = 2 /s (kT = 0.1)', 1.06),
                              (0.04, 'baseline', 'v1: k = 0.8 /s (kT = 0.08 at 10 Hz)', 1.13)]:
        z = max(abs(closed_loop_poles(kT)))
        ax[0].plot(kT, z, **fs.marker(role, ms=4))
        ax[0].annotate(lab, (kT, z), xytext=(0.2, ty), textcoords='data', fontsize=7,
                       color=fs.color(role), va='center',
                       arrowprops=dict(arrowstyle='-', lw=0.5, color=fs.color(role),
                                       shrinkA=1, shrinkB=2, relpos=(0, 0.5)))
    ax[0].set_xlabel('Normalised gain k·T')
    ax[0].set_ylabel('Largest closed-loop pole magnitude |z|')
    th = np.linspace(0, 2 * np.pi, 200)
    ax[1].plot(np.cos(th), np.sin(th), color=fs.GREY, lw=0.6)
    for kT, role in [(0.1, 'proposed'), (0.25, 'alt'), (0.5, 'alt2'), (0.9, 'alt3')]:
        z = closed_loop_poles(kT)
        ax[1].plot(z.real, z.imag, ls='none', marker=fs.MARKER[role], ms=4, mfc='none',
                   mec=fs.color(role), mew=0.9, label=f'kT = {kT}')
    ax[1].set_aspect('equal', adjustable='datalim')
    ax[1].set_xlim(-1.15, 1.15)
    ax[1].set_ylim(-1.15, 1.15)
    ax[1].set_xlabel('Re z')
    ax[1].set_ylabel('Im z')
    ax[1].legend(loc='center left', bbox_to_anchor=(1.0, 0.5))
    fs.panel_labels(ax, ['Largest pole vs. gain', 'Closed-loop poles (z-plane)'])
    fs.save(fig, os.path.join(args.out, 'fig_poles.png'))
    pole_rows = [{'kT': kT, 'k_per_s': kT / 0.05,
                  'poles': ' '.join(f'{z:.3f}' for z in closed_loop_poles(kT)),
                  'max_abs': max(abs(closed_loop_poles(kT)))}
                 for kT in (0.05, 0.1, 0.25, 0.5, 0.75, 1.0)]

    # ---- 2. 蒙特卡洛
    rng_master = 7
    S = starts(args.trials)

    def occl(t):            # 每 3 s 遮挡 1.5 s
        return (t % 3.0) >= 1.5

    results = {}
    for name, fn in [('v1', lambda s, r: run_v1(s, r)), ('v2', lambda s, r: run_v2(s, r)),
                     ('v2_occlusion', lambda s, r: run_v2(s, r, occlusion=occl))]:
        rows = []
        for i, s in enumerate(S):
            pl, _ = fn(s, np.random.default_rng(rng_master + i))
            rows.append(final_errors(pl))
        results[name] = rows
    summary = [dict(case=k, n=len(v), **{kk: round(float(vv), 3) for kk, vv in summarize(v).items()})
               for k, v in results.items()]
    with open(os.path.join(args.out, 'sim_summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0].keys()))
        w.writeheader()
        w.writerows(summary)
    with open(os.path.join(args.out, 'sim_poles.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(pole_rows[0].keys()))
        w.writeheader()
        w.writerows(pole_rows)
    for s in summary:
        print(s)

    fig, ax = fs.subplots(1, 2, width='double', height=2.2)
    bins = np.linspace(0, 12, 49)
    for name, role, lab in [('v1', 'baseline', 'v1 (colour blob + PBVS)'),
                            ('v2', 'proposed', 'v2 (AprilTag + LiDAR EKF)'),
                            ('v2_occlusion', 'alt', 'v2, intermittent occlusion')]:
        a = np.array(results[name])
        kw = dict(histtype='step', lw=1.0, color=fs.color(role), ls=fs.LINESTYLE[role],
                  label=lab)
        ax[0].hist(np.hypot(a[:, 0], a[:, 1]), bins=bins, **kw)
        ax[1].hist(np.abs(a[:, 2]), bins=np.linspace(0, 1.2, 49), **kw)
    ax[0].set_xlabel(fs.label('Final position error', 'mm'))
    ax[1].set_xlabel(fs.label('Final heading error', '°'))
    ax[0].set_ylabel(f'Trials (n = {args.trials} per method)')
    ax[1].set_ylabel('Trials')
    ax[0].legend(loc='upper right')
    fs.panel_labels(ax)
    fs.save(fig, os.path.join(args.out, 'fig_sim_mc.png'))

    # ---- 3. 单次时间响应
    s = S[0]
    _, tr1 = run_v1(s, np.random.default_rng(1))
    _, tr2 = run_v2(s, np.random.default_rng(1))
    fig, ax = fs.subplots(1, 3, width='double', height=2.2)
    for tr, role, lab in [(tr1, 'baseline', 'v1 (stops inside the 1 cm tolerance)'),
                          (tr2, 'proposed', 'v2')]:
        a = np.array(tr)
        kw = fs.line(role, label=lab)

        def vis(v):          # 低于纵轴下限 0.05 的点不画（对数轴，避免路径延伸到坐标区外）
            return np.where(v >= 0.05, v, np.nan)
        ax[0].plot(a[:, 0], vis(np.abs(a[:, 1] - D) * 1000), **kw)
        ax[1].plot(a[:, 0], vis(np.abs(a[:, 2]) * 1000), **kw)
        ax[2].plot(a[:, 0], vis(np.hypot(a[:, 4], a[:, 5]) * 1000), **kw)
    ax[0].set_ylabel(fs.label('Distance error |d − d*|', 'mm'))
    ax[1].set_ylabel(fs.label('Along-wall error |u|', 'mm'))
    ax[2].set_ylabel(fs.label('Translational speed command', 'mm/s'))
    for a in ax:
        a.set_xlabel(fs.label('Time', 's'))
        a.set_yscale('log')
        fs.plain_log(a)
        a.set_ylim(bottom=0.05)
    fs.panel_labels(ax)
    h, lab = ax[0].get_legend_handles_labels()
    fig.legend(h, lab, loc='upper center', ncol=2, bbox_to_anchor=(0.5, 1.0))
    fs.fixed_layout(fig, left=0.42, right=0.06, bottom=0.42, top=0.36, wspace=0.62)
    fs.save(fig, os.path.join(args.out, 'fig_sim_response.png'))


if __name__ == '__main__':
    main()
