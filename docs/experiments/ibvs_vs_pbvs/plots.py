"""作图：由 trials_*.csv 与少量在线仿真生成 fig_*.png."""
import math
import os

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import sim  # noqa: E402
from sim import ibvs, Scenario  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
COLORS = {'PBVS': '#2a78d6', 'IBVS': '#eb6834', 'Hybrid': '#1baf7a'}
LABELS = {'PBVS': 'PBVS（现有）', 'IBVS': 'IBVS（四角点）', 'Hybrid': '2.5D 混合'}
SURFACE, INK, INK2, GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#e4e3df'
METHODS = ['PBVS', 'IBVS', 'Hybrid']
DEG = math.pi / 180

plt.rcParams.update({
    'font.family': ['Noto Sans CJK JP', 'DejaVu Sans'],
    'axes.unicode_minus': False,
    'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
    'axes.edgecolor': GRID, 'axes.labelcolor': INK2, 'xtick.color': INK2,
    'ytick.color': INK2, 'text.color': INK, 'axes.grid': True, 'grid.color': GRID,
    'grid.linewidth': 0.8, 'axes.spines.top': False, 'axes.spines.right': False,
    'lines.linewidth': 2.0, 'font.size': 10, 'axes.titlesize': 11,
    'legend.frameon': False,
})


def _save(fig, name):
    fig.savefig(os.path.join(HERE, name), dpi=130, bbox_inches='tight')
    plt.close(fig)


def _group(rows, exp):
    g = {}
    for r in rows:
        if r['exp'] == exp:
            g.setdefault((r['cond'], r['method']), []).append(r)
    return g


# ------------------------------------------------------------------ 轨迹
def fig_trajectories():
    starts = sim.random_starts(6, seed=7, dist=(0.8, 2.0), lateral=0.8, yaw_deg=35.0)
    sc = Scenario(pixel_noise=0.5, range_noise=0.01)
    fig, axes = plt.subplots(2, 3, figsize=(13, 8.2))
    for col, m in enumerate(METHODS):
        ax, axi = axes[0, col], axes[1, col]
        for i, s in enumerate(starts):
            r = sim.run_trial(m, sc, s, seed=1000 + i, keep_traj=True)
            tr, cen = r['traj'], r['centroid']
            lost = r['status'].startswith('lost')
            ls = ':' if lost else '-'
            ax.plot(tr[:, 1], -tr[:, 0], ls, color=COLORS[m], lw=1.6)
            ax.plot(tr[0, 1], -tr[0, 0], 'o', ms=5, color=INK2)
            for p in tr[::12]:
                ax.annotate('', xy=(p[1] + 0.06 * math.sin(p[2]), -p[0] - 0.06 * math.cos(p[2])),
                            xytext=(p[1], -p[0]),
                            arrowprops=dict(arrowstyle='-', color=COLORS[m], lw=0.8))
            if lost:
                ax.plot(tr[-1, 1], -tr[-1, 0], 'x', ms=9, mew=2, color=INK)
            axi.plot(cen[:, 0], cen[:, 1], ls, color=COLORS[m], lw=1.6)
            axi.plot(cen[0, 0], cen[0, 1], 'o', ms=5, color=INK2)
        ax.plot(0, sim.D_DES, '+', ms=14, mew=2, color=INK)
        ax.set_title(LABELS[m])
        ax.set_xlabel('沿墙横向 y (m)')
        ax.set_ylabel('离墙距离 (m)')
        ax.set_xlim(-0.9, 0.9)
        ax.set_ylim(0.6, 2.1)
        ax.invert_yaxis()
        ax.set_aspect('equal')
        px, _ = sim.nominal_reference()
        c = ibvs.polygon_centroid(px)
        axi.plot(*c, '+', ms=14, mew=2, color=INK)
        axi.add_patch(plt.Rectangle((0, 0), sim.IMG_W, sim.IMG_H, fill=False, ec=INK2, lw=1))
        axi.set_xlim(-20, sim.IMG_W + 20)
        axi.set_ylim(sim.IMG_H + 20, -20)
        axi.set_aspect('equal')
        axi.set_xlabel('u (px)')
        axi.set_ylabel('v (px)')
        axi.set_title('方块质心像面轨迹')
    fig.suptitle('6 个大范围起点（E4 分布：离墙 0.8–2 m、横向 ±0.8 m、朝向 ±35°），基准噪声。'
                 '上：俯视轨迹（短线为朝向，墙在上方）；虚线 + × = 方块出视野后停住失败',
                 fontsize=10, color=INK2)
    _save(fig, 'fig_trajectories.png')


# ------------------------------------------------------------------ 主对比
def fig_main(rows):
    from run_experiments import E1_CONDS
    g = _group(rows, 'E1')
    conds = [c for c, _ in E1_CONDS]
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.2))
    w = 0.26
    x = np.arange(len(conds))
    for k, m in enumerate(METHODS):
        pos = [np.array([r['pos_err'] for r in g[(f'{c}|stop', m)]]) * 1000 for c in conds]
        yaw = [np.abs([r['yaw'] for r in g[(f'{c}|stop', m)]]) / DEG for c in conds]
        tim = [np.array([r['time'] for r in g[(f'{c}|stop', m)]]) for c in conds]
        for ax, data in zip(axes, [pos, yaw, tim]):
            ax.bar(x + (k - 1) * w, [d.mean() for d in data], w * 0.92, color=COLORS[m],
                   label=LABELS[m], yerr=[d.std() for d in data], ecolor=INK2,
                   error_kw=dict(lw=1, capsize=2))
    axes[0].axhline(10, color=INK2, lw=1, ls='--')
    axes[1].axhline(1, color=INK2, lw=1, ls='--')
    axes[0].set_ylabel('停止时真值位置误差 (mm)')
    axes[1].set_ylabel('停止时真值偏航误差 (°)')
    axes[2].set_ylabel('停止时间 (s，超时记 50 s)')
    for ax in axes:
        ax.set_xticks(x, [c.replace(' ', '\n') for c in conds])
        ax.grid(axis='x', visible=False)
    axes[0].legend(loc='upper left')
    fig.suptitle('E1：Gazebo 同分布起点 × 100，停止模式（虚线 = 1 cm / 1° 容差）',
                 fontsize=11)
    _save(fig, 'fig_main.png')


# ------------------------------------------------------------------ 噪声扫描
def fig_noise(rows):
    from run_experiments import PIX_LEVELS, RANGE_LEVELS
    g = _group(rows, 'E2')
    fig, axes = plt.subplots(2, 2, figsize=(11, 7.5))
    for row, (kind, levels, xl) in enumerate([
            ('pix', PIX_LEVELS, '每个角点像素噪声 σ (px)，激光 σ = 1 cm'),
            ('range', RANGE_LEVELS, '每束激光测距噪声 σ (m)，像素 σ = 0.5 px')]):
        for m in METHODS:
            pos = [np.sqrt(np.mean(np.square([r['pos_err'] for r in g[(f'{kind}|{lv}', m)]])))
                   * 1000 for lv in levels]
            yaw = [np.sqrt(np.mean(np.square([r['yaw'] for r in g[(f'{kind}|{lv}', m)]]))) / DEG
                   for lv in levels]
            axes[row, 0].plot(levels, pos, 'o-', ms=6, color=COLORS[m], label=LABELS[m])
            axes[row, 1].plot(levels, yaw, 'o-', ms=6, color=COLORS[m], label=LABELS[m])
        axes[row, 0].set_ylabel('稳态位置误差 RMS (mm)')
        axes[row, 1].set_ylabel('稳态偏航误差 RMS (°)')
        for ax in axes[row]:
            ax.set_xlabel(xl)
            ax.set_yscale('symlog', linthresh=0.1)
        axes[row, 0].axhline(10, color=INK2, lw=1, ls='--')
        axes[row, 1].axhline(1, color=INK2, lw=1, ls='--')
    axes[0, 0].legend()
    fig.suptitle('E2：噪声扫描（稳态模式：不停止，30 s 末 5 s 的真值 RMS；50 个起点）', fontsize=11)
    fig.tight_layout()
    _save(fig, 'fig_noise.png')


# ------------------------------------------------------------------ 标定
def fig_calibration(rows):
    from run_experiments import CALIB
    g = _group(rows, 'E3')
    keys = ['cam_yaw', 'cam_dy', 'cam_pitch', 'lidar_yaw', 'fx_scale', 'cx_err']
    info = {k: (lab, lv) for k, lab, lv, _ in CALIB}
    fig, axes = plt.subplots(2, len(keys), figsize=(3.1 * len(keys), 6.4), sharey='row')
    for col, key in enumerate(keys):
        lab, levels = info[key]
        xs = sorted(levels)
        for m in METHODS:
            for ref, ls in [('model', '-'), ('taught', '--')]:
                pos, yaw = [], []
                for lv in xs:
                    rs = g[(f'{key}|{lv}|{ref}', m)]
                    pos.append(np.mean([r['pos_err'] for r in rs]) * 1000)
                    yaw.append(np.mean(np.abs([r['yaw'] for r in rs])) / DEG)
                kw = dict(color=COLORS[m], ls=ls, marker='o', ms=4, lw=1.6)
                axes[0, col].plot(xs, pos, label=f'{m} {"模型" if ref == "model" else "示教"}',
                                  **kw)
                axes[1, col].plot(xs, yaw, **kw)
        axes[0, col].set_title(lab)
        axes[1, col].set_xlabel(lab)
        axes[0, col].axhline(10, color=INK2, lw=1, ls=':')
        axes[1, col].axhline(1, color=INK2, lw=1, ls=':')
        axes[0, col].set_yscale('symlog', linthresh=1.0)
        axes[1, col].set_yscale('symlog', linthresh=0.1)
    axes[0, 0].set_ylabel('稳态位置误差 (mm)')
    axes[1, 0].set_ylabel('稳态偏航误差 (°)')
    axes[0, 0].legend(fontsize=8, ncol=1)
    fig.suptitle('E3：标定误差敏感性（无噪声，稳态偏差；实线 = 期望值由标称模型计算，'
                 '虚线 = 在期望位姿示教；纵轴对称对数；PBVS 的 ≈3.5 mm / 0.3° 底噪来自死区）', fontsize=11)
    fig.tight_layout()
    _save(fig, 'fig_calibration.png')


# ------------------------------------------------------------------ 可观性
def crlb(size, dist, sigma_px=1.0):
    """IBVS 四角点在给定位姿处对 (x, y, θ) 的线性化标准差（每 σ_px 像素噪声）."""
    cw = ibvs.marker_corners_base([dist, 0.0, sim.MARKER_H], [1.0, 0.0], size)
    px, z = ibvs.project_points(cw, sim.P_C_NOM, ibvs.R_BO, sim.K_NOM)
    j = ibvs.feature_jacobian(ibvs.pixels_to_normalized(px, sim.K_NOM), z, sim.P_C_NOM)
    j_px = j * sim.K_NOM[0]
    cov = np.linalg.inv(j_px.T @ j_px) * sigma_px ** 2
    sv = np.linalg.svd(j, compute_uv=False)
    return np.sqrt(np.diag(cov)), sv[0] / sv[-1]


def laser_sigma(range_noise=0.01, n=2000, seed=0):
    """激光拟合墙面在期望位姿处的朝向 / 距离标准差（蒙特卡洛）."""
    w = sim.World(Scenario(range_noise=range_noise), np.random.default_rng(seed))
    fits = [w.scan(sim.DESIRED_POSE) for _ in range(n)]
    return np.std([f['yaw_error'] for f in fits]), np.std([f['distance'] for f in fits])


def observability_table():
    """返回 Markdown 表：每 1 px / 1 cm 噪声下单帧测量的标准差."""
    ly, ld = laser_sigma()
    lines = ['| 方块边长 | J 条件数 | IBVS σ_法向 mm | IBVS σ_横向 mm | IBVS σ_偏航 ° |',
             '|---|---|---|---|---|']
    for s in [0.05, 0.10, 0.20, 0.30]:
        sd, cond = crlb(s, sim.D_DES)
        lines.append(f'| {s*100:.0f} cm | {cond:.0f} | {sd[0]*1000:.1f} | {sd[1]*1000:.1f} | '
                     f'{sd[2]/DEG:.2f} |')
    z = sim.D_DES - sim.P_C_NOM[0]
    lat = z * 0.5 / sim.K_NOM[0]
    lines += ['', f'对照（PBVS / 混合的单帧测量）：激光拟合 σ_偏航 = {ly/DEG:.3f}°、'
              f'σ_距离 = {ld*1000:.2f} mm（1 cm 测距噪声、360 线、±35° 扇区）；'
              f'质心横向 σ ≈ Z·(σ/2)/f = {lat*1000:.2f} mm / px（四角点平均，Z = {z:.3f} m）。']
    return '\n'.join(lines)


def fig_observability(rows):
    sizes = np.linspace(0.03, 0.40, 40)
    ly, _ = laser_sigma()
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.0))
    yaw = [crlb(s, sim.D_DES)[0][2] / DEG for s in sizes]
    lat = [crlb(s, sim.D_DES)[0][1] * 1000 for s in sizes]
    axes[0].plot(sizes * 100, yaw, color=COLORS['IBVS'], label='IBVS 四角点（1 px）')
    axes[0].axhline(ly / DEG, color=COLORS['Hybrid'], ls='--', lw=1.6,
                    label='激光拟合墙面（1 cm）')
    axes[0].set_yscale('log')
    axes[0].set_xlabel('方块边长 (cm)')
    axes[0].set_ylabel('单帧偏航标准差 (°)')
    axes[0].legend()
    axes[0].set_title('偏航可观性')
    axes[1].plot(sizes * 100, lat, color=COLORS['IBVS'], label='IBVS 底盘横向（1 px）')
    axes[1].axhline((sim.D_DES - sim.P_C_NOM[0]) * 0.5 / sim.K_NOM[0] * 1000,
                    color=COLORS['Hybrid'], ls='--', lw=1.6, label='质心横向（1 px，已知朝向）')
    axes[1].set_yscale('log')
    axes[1].set_xlabel('方块边长 (cm)')
    axes[1].set_ylabel('单帧横向标准差 (mm)')
    axes[1].legend()
    axes[1].set_title('横向可观性')
    g = _group(rows, 'E7')
    labels, xs = [], []
    for i, size in enumerate(['0.05', '0.1', '0.2']):
        for k, m in enumerate(METHODS):
            rs = g.get((f'{size}|基准噪声|steady', m))
            if not rs:
                continue
            v = np.sqrt(np.mean(np.square([r['yaw'] for r in rs]))) / DEG
            axes[2].bar(i + (k - 1) * 0.26, v, 0.24, color=COLORS[m],
                        label=LABELS[m] if i == 0 else None)
        labels.append(f'{float(size)*100:.0f} cm')
        xs.append(i)
    axes[2].set_xticks(xs, labels)
    axes[2].set_xlabel('方块边长（E7，基准噪声，稳态）')
    axes[2].set_ylabel('闭环稳态偏航 RMS (°)')
    axes[2].grid(axis='x', visible=False)
    axes[2].legend()
    axes[2].set_title('闭环结果')
    fig.tight_layout()
    _save(fig, 'fig_observability.png')


def make_all(rows):
    fig_trajectories()
    if any(r['exp'] == 'E1' for r in rows):
        fig_main(rows)
    if any(r['exp'] == 'E2' for r in rows):
        fig_noise(rows)
    if any(r['exp'] == 'E3' for r in rows):
        fig_calibration(rows)
    fig_observability(rows)
    with open(os.path.join(HERE, 'observability.md'), 'w') as f:
        f.write(observability_table() + '\n')

