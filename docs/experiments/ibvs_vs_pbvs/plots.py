"""作图：由 trials_*.csv 与少量在线仿真生成 fig_*.png / fig_*.pdf（docs/figstyle 统一样式，英文）."""
import math
import os
import sys

import numpy as np

import sim
from sim import ibvs, Scenario

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..'))
import figstyle as fs  # noqa: E402

# PBVS 为系统现用方法（proposed 色），IBVS 与 2.5D 混合为被比较的备选
ROLES = {'PBVS': 'proposed', 'IBVS': 'alt2', 'Hybrid': 'alt'}
COLORS = {m: fs.color(r) for m, r in ROLES.items()}
LABELS = {'PBVS': 'PBVS (current)', 'IBVS': 'IBVS (four corners)', 'Hybrid': '2.5D hybrid'}
INK2 = fs.GREY
METHODS = ['PBVS', 'IBVS', 'Hybrid']
DEG = math.pi / 180
COND_EN = {'无噪声': 'No noise', '基准 0.5px/1cm': 'Base noise\n0.5 px, 1 cm',
           '2px/2cm': '2 px, 2 cm', '5px/5cm': '5 px, 5 cm'}
CALIB_EN = {'cam_yaw': 'Camera yaw (°)', 'cam_pitch': 'Camera pitch (°)',
            'cam_roll': 'Camera roll (°)', 'cam_dy': 'Camera lateral offset (mm)',
            'cam_dx': 'Camera fore-aft offset (mm)', 'lidar_yaw': 'LiDAR yaw (°)',
            'lidar_dx': 'LiDAR fore-aft offset (mm)', 'fx_scale': 'Focal-length error (%)',
            'cx_err': 'Principal point cx error (px)', 'cy_err': 'Principal point cy error (px)'}


def _save(fig, name):
    fs.save(fig, os.path.join(HERE, name))


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
    fig, axes = fs.subplots(2, 3, width='double', height=4.0)
    any_lost = False
    for col, m in enumerate(METHODS):
        ax, axi = axes[0, col], axes[1, col]
        for i, s in enumerate(starts):
            r = sim.run_trial(m, sc, s, seed=1000 + i, keep_traj=True)
            tr, cen = r['traj'], r['centroid']
            lost = r['status'].startswith('lost')
            any_lost |= lost
            ls = (0, (1, 1)) if lost else '-'
            ax.plot(tr[:, 1], -tr[:, 0], ls=ls, color=COLORS[m], lw=0.9)
            ax.plot(tr[0, 1], -tr[0, 0], 'o', ms=3, color=INK2)
            for p in tr[::12]:
                ax.plot([p[1], p[1] + 0.06 * math.sin(p[2])],
                        [-p[0], -p[0] - 0.06 * math.cos(p[2])], color=COLORS[m], lw=0.5)
            if lost:
                ax.plot(tr[-1, 1], -tr[-1, 0], 'x', ms=5, mew=1.2, color=fs.BLACK)
            axi.plot(cen[:, 0], cen[:, 1], ls=ls, color=COLORS[m], lw=0.9)
            axi.plot(cen[0, 0], cen[0, 1], 'o', ms=3, color=INK2)
        ax.plot(0, sim.D_DES, '+', ms=8, mew=1.2, color=fs.BLACK)
        ax.set_xlabel(fs.label('Lateral position y', 'm'))
        ax.set_ylabel(fs.label('Distance from wall', 'm'))
        ax.set_xlim(-0.98, 0.98)     # 与下排像面图同一纵横比，两排面板等宽
        ax.set_ylim(2.1, 0.6)
        ax.set_aspect('equal', adjustable='box')
        px, _ = sim.nominal_reference()
        c = ibvs.polygon_centroid(px)
        axi.plot(*c, '+', ms=8, mew=1.2, color=fs.BLACK)
        axi.add_patch(fs.plt.Rectangle((0, 0), sim.IMG_W, sim.IMG_H, fill=False, ec=INK2, lw=0.6))
        axi.set_xlim(-20, sim.IMG_W + 20)
        axi.set_ylim(sim.IMG_H + 20, -20)
        axi.set_aspect('equal', adjustable='box')
        axi.set_xlabel(fs.label('Image u', 'px'))
        axi.set_ylabel(fs.label('Image v', 'px'))
        fs.panel_label(ax, 'abc'[col], f'{LABELS[m]}: top view')
        fs.panel_label(axi, 'def'[col], f'{m}: marker centroid in image')
    from matplotlib.lines import Line2D
    handles = [Line2D([], [], color=INK2, marker='o', ls='none', ms=3, label='Start'),
               Line2D([], [], color=fs.BLACK, marker='+', ls='none', ms=8, mew=1.2,
                      label='Goal'),
               Line2D([], [], color=INK2, lw=0.5, label='Heading (every 12th step)')]
    if any_lost:
        handles.append(Line2D([], [], color=fs.BLACK, ls=(0, (1, 1)), marker='x', ms=5,
                              label='Marker left the view (stopped, failed)'))
    fig.legend(handles=handles, loc='outside lower center', ncol=len(handles))
    _save(fig, 'fig_trajectories.png')


# ------------------------------------------------------------------ 主对比
def fig_main(rows):
    from run_experiments import E1_CONDS
    g = _group(rows, 'E1')
    conds = [c for c, _ in E1_CONDS]
    fig, axes = fs.subplots(1, 3, width='double', height=2.3)
    w = 0.26
    x = np.arange(len(conds))
    for k, m in enumerate(METHODS):
        pos = [np.array([r['pos_err'] for r in g[(f'{c}|stop', m)]]) * 1000 for c in conds]
        yaw = [np.abs([r['yaw'] for r in g[(f'{c}|stop', m)]]) / DEG for c in conds]
        tim = [np.array([r['time'] for r in g[(f'{c}|stop', m)]]) for c in conds]
        for ax, data in zip(axes, [pos, yaw, tim]):
            ax.bar(x + (k - 1) * w, [d.mean() for d in data], w * 0.92,
                   yerr=[d.std() for d in data], error_kw=dict(lw=0.6, capsize=1.5),
                   label=LABELS[m], **fs.bar(ROLES[m]))
    axes[0].axhline(10, **fs.line('reference', lw=0.7))
    axes[1].axhline(1, **fs.line('reference', lw=0.7))
    axes[0].set_ylabel(fs.label('Final position error', 'mm'))
    axes[1].set_ylabel(fs.label('Final yaw error', '°'))
    axes[2].set_ylabel(fs.label('Time to stop (timeout = 50 s)', 's'))
    for ax in axes:
        ax.set_xticks(x, [COND_EN.get(c, c) for c in conds], fontsize=7)
        fs.light_grid(ax, axis='y')
    fs.panel_labels(axes)
    h, lab = axes[0].get_legend_handles_labels()
    fig.legend(h, lab, loc='outside upper center', ncol=3)
    _save(fig, 'fig_main.png')


# ------------------------------------------------------------------ 噪声扫描
def fig_noise(rows):
    from run_experiments import PIX_LEVELS, RANGE_LEVELS
    g = _group(rows, 'E2')
    fig, axes = fs.subplots(2, 2, width='double', height=4.2)
    for row, (kind, levels, xl) in enumerate([
            ('pix', PIX_LEVELS, 'Corner pixel noise σ (px), LiDAR σ = 1 cm'),
            ('range', RANGE_LEVELS, 'LiDAR range noise σ (m), pixel σ = 0.5 px')]):
        for m in METHODS:
            pos = [np.sqrt(np.mean(np.square([r['pos_err'] for r in g[(f'{kind}|{lv}', m)]])))
                   * 1000 for lv in levels]
            yaw = [np.sqrt(np.mean(np.square([r['yaw'] for r in g[(f'{kind}|{lv}', m)]]))) / DEG
                   for lv in levels]
            axes[row, 0].plot(levels, pos, label=LABELS[m], **fs.line(ROLES[m], marker=True))
            axes[row, 1].plot(levels, yaw, label=LABELS[m], **fs.line(ROLES[m], marker=True))
        axes[row, 0].set_ylabel(fs.label('Steady-state position RMS', 'mm'))
        axes[row, 1].set_ylabel(fs.label('Steady-state yaw RMS', '°'))
        for ax in axes[row]:
            ax.set_xlabel(xl)
            ax.set_yscale('symlog', linthresh=0.1)
            fs.plain_ticks(ax)
            fs.light_grid(ax, axis='y')
        axes[row, 0].axhline(10, **fs.line('reference', lw=0.7))
        axes[row, 1].axhline(1, **fs.line('reference', lw=0.7))
    fs.panel_labels(axes.flat)
    h, lab = axes[0, 0].get_legend_handles_labels()
    fig.legend(h, lab, loc='outside upper center', ncol=3)
    _save(fig, 'fig_noise.png')


# ------------------------------------------------------------------ 标定
def fig_calibration(rows):
    from run_experiments import CALIB
    g = _group(rows, 'E3')
    keys = ['cam_yaw', 'cam_dy', 'cam_pitch', 'lidar_yaw', 'fx_scale', 'cx_err']
    info = {k: lv for k, _, lv, _ in CALIB}
    fig, axes = fs.subplots(2, len(keys), width='double', height=3.6, sharey='row')
    for col, key in enumerate(keys):
        levels = info[key]
        xs = sorted(levels)
        for m in METHODS:
            for ref, ls in [('model', '-'), ('taught', (0, (3, 1.5)))]:
                pos, yaw = [], []
                for lv in xs:
                    rs = g[(f'{key}|{lv}|{ref}', m)]
                    pos.append(np.mean([r['pos_err'] for r in rs]) * 1000)
                    yaw.append(np.mean(np.abs([r['yaw'] for r in rs])) / DEG)
                kw = dict(color=COLORS[m], ls=ls, marker=fs.MARKER[ROLES[m]], ms=2.6, lw=0.8,
                          mfc='white' if ref == 'taught' else COLORS[m])
                ref_en = 'nominal model' if ref == 'model' else 'taught at goal'
                axes[0, col].plot(xs, pos, label=f'{m}, {ref_en}', **kw)
                axes[1, col].plot(xs, yaw, **kw)
        axes[1, col].set_xlabel(CALIB_EN[key].replace(' error', '\nerror')
                                .replace(' offset', '\noffset'))
        axes[0, col].axhline(10, **fs.line('reference', lw=0.6))
        axes[1, col].axhline(1, **fs.line('reference', lw=0.6))
        axes[0, col].set_yscale('symlog', linthresh=1.0)
        axes[1, col].set_yscale('symlog', linthresh=0.1)
        fs.plain_ticks(axes[0, col])
        fs.plain_ticks(axes[1, col])
        fs.panel_label(axes[0, col], 'abcdef'[col])
        fs.panel_label(axes[1, col], 'ghijkl'[col])
    axes[0, 0].set_ylabel(fs.label('Steady-state position error', 'mm'))
    axes[1, 0].set_ylabel(fs.label('Steady-state yaw error', '°'))
    h, lab = axes[0, 0].get_legend_handles_labels()
    fig.legend(h, lab, loc='outside upper center', ncol=3)
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
    lines = ['| 标记边长 | J 条件数 | IBVS σ_法向 mm | IBVS σ_横向 mm | IBVS σ_偏航 ° |',
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
    fig, axes = fs.subplots(1, 3, width='double', height=2.4)
    yaw = [crlb(s, sim.D_DES)[0][2] / DEG for s in sizes]
    lat = [crlb(s, sim.D_DES)[0][1] * 1000 for s in sizes]
    axes[0].plot(sizes * 100, yaw, label='IBVS, four corners (1 px)', **fs.line('alt2'))
    axes[0].axhline(ly / DEG, label='LiDAR wall fit (1 cm)', **fs.line('alt'))
    axes[0].set_yscale('log')
    fs.plain_log(axes[0])
    axes[0].set_xlabel(fs.label('Marker side length', 'cm'))
    axes[0].set_ylabel(fs.label('Single-frame yaw std. dev.', '°'))
    axes[0].legend(loc='upper right')
    axes[1].plot(sizes * 100, lat, label='IBVS, base lateral (1 px)', **fs.line('alt2'))
    axes[1].axhline((sim.D_DES - sim.P_C_NOM[0]) * 0.5 / sim.K_NOM[0] * 1000,
                    label='Centroid lateral (1 px, known yaw)', **fs.line('alt'))
    axes[1].set_yscale('log')
    fs.plain_log(axes[1])
    axes[1].set_xlabel(fs.label('Marker side length', 'cm'))
    axes[1].set_ylabel(fs.label('Single-frame lateral std. dev.', 'mm'))
    axes[1].legend(loc='upper right')
    g = _group(rows, 'E7')
    labels, xs = [], []
    for i, size in enumerate(['0.05', '0.1', '0.2']):
        for k, m in enumerate(METHODS):
            rs = g.get((f'{size}|基准噪声|steady', m))
            if not rs:
                continue
            v = np.sqrt(np.mean(np.square([r['yaw'] for r in rs]))) / DEG
            axes[2].bar(i + (k - 1) * 0.26, v, 0.24, label=LABELS[m] if i == 0 else None,
                        **fs.bar(ROLES[m]))
        labels.append(f'{float(size)*100:.0f} cm')
        xs.append(i)
    axes[2].set_xticks(xs, labels)
    axes[2].set_xlabel('Marker side length')
    axes[2].set_ylabel(fs.label('Closed-loop steady-state yaw RMS', '°'))
    axes[2].legend(loc='upper left')
    axes[2].set_ylim(0, axes[2].get_ylim()[1] * 1.35)
    fs.panel_labels(axes, ['Yaw observability', 'Lateral observability',
                           'Closed loop, base noise'])
    fs.fixed_layout(fig, left=0.45, right=0.06, bottom=0.44, top=0.2, wspace=0.62)
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
