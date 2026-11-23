#!/usr/bin/env python3
"""
切割力扰动与抗扰定点保持的离线分析（不需要 ROS / Gazebo）.

1. 切割力模型：刀片受力、底盘力旋量随进给速度与材料的变化，一刀的时间历程；
2. 平台可行性：所需轮地摩擦系数、最小轮压（倾覆裕度）、主轴功率限制的进给速度；
3. 线性分析：各控制律的闭环极点、增益 / 相位裕度、灵敏度峰值、刀尖动态柔度，
   对速度环增益失配与延迟的鲁棒性，以及采样周期与速度环增益决定的结构性上限；
4. 闭环仿真（力控底盘，里程计反馈）：阶跃 / 斜坡 / 先下刀后进给 / 随机扰动，横槽与竖槽，
   三档进给速度；指标为刀尖偏移、底盘漂移、控制量与恢复时间；另含限幅与抗积分饱和；
5. 里程计看不到打滑的情形（轮地接触底盘）：里程计反馈与墙面相对反馈的对比。

控制律与切割力模型直接调用 scr_arm_controller 中的同一份代码。

  python3 docs/experiments/cutting_force/analysis.py [--seeds 8] [--out <dir>]

图使用 docs/figstyle 的样式（PDF + PNG）；全部随机数由固定种子生成，重复运行结果相同。
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', '..'))

import plant_sim as ps  # noqa: E402
from plant_sim import cf, do, K_IN, KP, M, IZ, PF, T  # noqa: E402

FEEDS = (0.002, 0.005, 0.010)
MATERIAL = 'brick'
CUT_LENGTH = 0.10
RAMP_TIME = 4.0
CTRL = ['P', 'P+FF', 'PI', 'PID', 'DOB', 'DOB+FF']
ROLE = {'P': 'baseline', 'P+FF': 'neutral', 'PI': 'alt2', 'PID': 'alt', 'DOB': 'proposed',
        'DOB+FF': 'alt3', 'LQG': 'alt4'}
LABEL = {'P': 'P (current hold)', 'P+FF': 'P + feedforward', 'PI': 'PI', 'PID': 'PID',
         'DOB': 'DOB', 'DOB+FF': 'DOB + feedforward', 'LQG': 'LQG'}


def write_csv(path, rows):
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def r(v, n=3):
    return round(float(v), n) if v == v else ''


# ---------------------------------------------------------------------------
# 线性分析工具
# ---------------------------------------------------------------------------
def tf(A, B, C, D, z):
    n = len(A)
    return (C @ np.linalg.solve(z * np.eye(n) - A, B) if n else 0.0) + D


def axis_controller(name, k_in=K_IN, kp=KP[2], dt=T, **kw):
    """线性分析用的单轴控制律（默认取偏航轴的增益）."""
    m = do.AxisModel(k_in, dt)
    if name == 'P':
        return do.AxisP(kp)
    if name == 'PI':
        return do.AxisPID(*do.pi_triple_pole(k_in), 0.0, dt)
    if name == 'PID':
        return do.AxisPID(*do.pid_pole_placement(k_in, kw.get('omega', 0.4 * k_in), 1.0), dt)
    if name == 'DOB':
        return do.AxisDOB(m, kp, 0.0, gain=do.bandwidth_gain(m, kw.get('omega_o', 40.0)),
                          delay_steps=kw.get('ctrl_delay', 1),
                          predict_ahead=kw.get('predict_ahead', True))
    if name == 'LQG':
        return do.AxisDOB(m, *do.lqr_gains(m, kp ** 2, 1.0),
                          gain=do.kalman_gain(m, kw.get('noise_ratio', 2.0), 1.0))
    raise ValueError(name)


class Loop:
    """单轴离散闭环：对象（零阶保持 + delay 拍延迟）与控制律的状态空间."""

    def __init__(self, ctrl, k_true=K_IN, delay=1, dt=T):
        self.dt, self.delay = dt, delay
        self.plant = do.AxisModel(k_true, dt)
        self.ctrl = do.state_space(ctrl)

    def response(self, w):
        """频率响应：返回 (回路传递函数 L, 输入端扰动 d → 偏差 e 的传递函数 G)."""
        A, B, C, D = self.ctrl
        L, G = [], []
        for wi in w:
            z = np.exp(1j * wi * self.dt)
            p0 = tf(self.plant.A, self.plant.B, np.array([1.0, 0.0]), 0.0, z)
            li = -tf(A, B, C, D, z) * p0 * z ** (-self.delay)
            L.append(li)
            G.append(p0 / (1.0 + li))
        return np.array(L), np.array(G)

    def poles(self):
        A, B, C, D = self.ctrl
        n, d = len(A), self.delay
        N = 2 + d + n
        Mx = np.zeros((N, N))
        Mx[:2, :2] = self.plant.A
        if d == 0:
            Mx[:2, 0] += self.plant.B * D
            Mx[:2, 2:] = np.outer(self.plant.B, C)
        else:
            Mx[:2, 2] = self.plant.B
            for i in range(d - 1):
                Mx[2 + i, 3 + i] = 1.0
            Mx[1 + d, 0] = D
            Mx[1 + d, 2 + d:] = C
        Mx[2 + d:, 0] = B
        Mx[2 + d:, 2 + d:] = A
        return np.linalg.eigvals(Mx)

    def margins(self, w):
        L, G = self.response(w)
        mag, ph = np.abs(L), np.unwrap(np.angle(L))
        gc = np.where((mag[:-1] > 1.0) & (mag[1:] <= 1.0))[0]
        pc = np.where((ph[:-1] > -np.pi) & (ph[1:] <= -np.pi))[0]
        pm = min((180.0 + np.degrees(ph[i]) for i in gc), default=float('nan'))
        wc = w[gc[0]] if len(gc) else float('nan')
        gm = min((1.0 / mag[i] for i in pc), default=float('inf'))
        return dict(wc=wc, pm=pm, gm=gm, ms=np.abs(1.0 / (1.0 + L)).max(),
                    g0=abs(G[0]), gpeak=np.abs(G).max(),
                    delay_margin=math.radians(pm) / wc if wc == wc else float('nan'))

    def step(self, n=500):
        """输入端单位阶跃扰动下的偏差响应（测量延迟一拍）."""
        A, B, C, D = self.ctrl
        x, xc = np.zeros(2), np.zeros(len(A))
        ys = [0.0] * (self.delay + 1)
        out = []
        for _ in range(n):
            y = ys[0]
            u = C @ xc + D * y
            xc = A @ xc + B * y
            x = self.plant.A @ x + self.plant.B * (u + 1.0)
            ys = ys[1:] + [x[0]]
            out.append(x[0])
        return np.array(out)


def tip_gain(g_yaw, g_lat=None):
    """输入端扰动 → 偏差的增益换算为横向切削力 → 刀尖沿墙偏移的柔度（mm/N）."""
    arm = PF.standoff + cf.SLOT_DEPTH - cf.BLADE_DIAMETER / 2      # 力臂：刀片中心
    tip = PF.tool_tip(cf.SLOT_DEPTH)[0]
    g_lat = g_yaw if g_lat is None else g_lat
    return 1e3 * (g_lat / (M * K_IN) + arm * tip * g_yaw / (IZ * K_IN))


# ---------------------------------------------------------------------------
# 1–2. 切割力模型与可行性
# ---------------------------------------------------------------------------
def force_and_feasibility(fs, out):
    feeds = np.linspace(0.0005, 0.012, 60)
    rows, feas = [], []
    fig, ax = fs.subplots(1, 3, width='double', height=2.3)
    roles = {'aerated_block': 'alt', 'brick': 'proposed', 'concrete': 'baseline'}
    names = {'aerated_block': 'aerated block', 'brick': 'fired brick', 'concrete': 'concrete'}
    for mat in cf.MATERIALS:
        blade = cf.BladeParams.for_material(mat)
        ft = [cf.tool_forces(blade, cf.SLOT_DEPTH, v) for v in feeds]
        ax[0].plot(feeds * 1e3, [f.normal for f in ft], label=names[mat],
                   **fs.line(roles[mat]))
        ax[0].plot(feeds * 1e3, [f.tangential for f in ft], lw=0.6,
                   **fs.line(roles[mat]))
        vmax = cf.max_feed_rate(blade, cf.SLOT_DEPTH)
        if vmax < feeds[-1]:
            fm = cf.tool_forces(blade, cf.SLOT_DEPTH, vmax)
            ax[0].plot(vmax * 1e3, fm.normal, **fs.marker(roles[mat], ms=4))
        for v in FEEDS + (vmax,):
            f = cf.tool_forces(blade, cf.SLOT_DEPTH, v)
            for orient in ('horizontal', 'vertical'):
                pf = cf.Platform(tool_y=0.0 if orient == 'horizontal' else 0.06)
                w = cf.base_wrench(f, pf, blade, cf.SLOT_DEPTH, orient, 1)
                mg = cf.static_margins(w, pf)
                rows.append(dict(
                    material=mat, feed_mm_s=r(v * 1e3, 2), power_limited=int(v == vmax),
                    orientation=orient, F_t_N=r(f.tangential, 2), F_n_N=r(f.normal, 2),
                    F_feed_N=r(f.feed, 2), F_wall_N=r(f.wall, 2), power_W=r(f.power, 0),
                    Fx_N=r(w.fx, 2), Fy_N=r(w.fy, 2), Fz_N=r(w.fz, 2), Mz_Nm=r(w.mz, 2),
                    mu_mecanum=r(mg.mu_mecanum), mu_locked=r(mg.mu_locked),
                    min_wheel_load_ratio=r(mg.min_load_ratio)))
    ax[0].set_xlabel(fs.label('Feed rate', 'mm/s'))
    ax[0].set_ylabel(fs.label('Force on the blade', 'N'))
    ax[0].set_ylim(0, 130)
    ax[0].legend(loc='upper left')
    fs.note(ax[0], 'thick: normal $F_n$\nthin: tangential $F_t$\nmarker: 820 W limit',
            x=0.98, y=0.56)
    # (b) 底盘力旋量（烧结砖，横槽）
    blade = cf.BladeParams.for_material(MATERIAL)
    w = [cf.cut_wrench(blade, PF, cf.SLOT_DEPTH, v, 'horizontal') for v in feeds]
    ax[1].plot(feeds * 1e3, [x.fy for x in w], label='lateral force $F_y$ (N)',
               **fs.line('proposed'))
    ax[1].plot(feeds * 1e3, [-x.fx for x in w], label='push-back force $-F_x$ (N)',
               **fs.line('alt'))
    ax[1].plot(feeds * 1e3, [x.mz for x in w], label='yaw moment $M_z$ (N·m)',
               **fs.line('baseline'))
    ax[1].axhline(IZ * ps.LIMITS['default']['alpha_max'], **fs.line('reference', lw=0.6))
    fs.note(ax[1], 'dotted: yaw limit of the\nsimulated base (9.2 N·m)', x=0.98, y=0.17)
    ax[1].set_xlabel(fs.label('Feed rate', 'mm/s'))
    ax[1].set_ylabel('Wrench on the base')
    ax[1].legend(loc='upper left')
    # (c) 一刀的时间历程
    prof = cf.CutProfile('horizontal', 1, feed_rate=0.005, length=CUT_LENGTH, kind='cut')
    proc = cf.CuttingProcess(blade, PF, prof, seed=3)
    t = np.arange(0.0, prof.t_end + 1.0, T)
    wr = [proc.wrench(x) for x in t]
    nom = [proc.nominal(x) for x in t]
    ax[2].plot(t, [x.mz for x in wr], label='$M_z$ (N·m)', **fs.line('baseline'))
    ax[2].plot(t, [x.fy for x in wr], label='$F_y$ (N)', **fs.line('proposed'))
    ax[2].plot(t, [-x.fx for x in wr], label='$-F_x$ (N)', **fs.line('alt'))
    ax[2].plot(t, [x.mz for x in nom], **fs.line('reference', lw=0.6))
    ax[2].set_xlabel(fs.label('Time', 's'))
    ax[2].set_ylabel('Wrench on the base')
    ax[2].legend(loc='upper left')
    fs.note(ax[2], 'plunge', x=0.25, y=0.08, ha='center')
    fs.note(ax[2], 'feed', x=0.78, y=0.08, ha='center')
    fs.panel_labels(ax, ['Blade forces (35 mm deep)', 'Base wrench, horizontal cut in brick',
                         'One cut at 5 mm/s (seed 3)'])
    fs.save(fig, os.path.join(out, 'fig_force_model.png'))
    write_csv(os.path.join(out, 'force_table.csv'), rows)

    # 可行性
    fig, ax = fs.subplots(1, 2, width='double', height=2.4)
    for mat in cf.MATERIALS:
        blade = cf.BladeParams.for_material(mat)
        vmax = cf.max_feed_rate(blade, cf.SLOT_DEPTH)
        v = np.append(feeds[feeds < vmax], vmax)
        for orient, lw in (('horizontal', 1.0), ('vertical', 0.6)):
            mu, nmin = [], []
            for vi in v:
                # 最不利的刀具位置：横槽取最高处，竖槽取横向最远处
                pf = cf.Platform(tool_z=0.81) if orient == 'horizontal' else \
                    cf.Platform(tool_y=0.08)
                mg = cf.static_margins(cf.cut_wrench(blade, pf, cf.SLOT_DEPTH, vi, orient), pf)
                mu.append(mg.mu_mecanum)
                nmin.append(mg.min_load_ratio)
            kw = fs.line(roles[mat], lw=lw)
            ax[0].plot(v * 1e3, mu, label=names[mat] if lw == 1.0 else None, **kw)
            ax[1].plot(v * 1e3, nmin, **kw)
            if vmax <= feeds[-1]:
                ax[0].plot(vmax * 1e3, mu[-1], **fs.marker(roles[mat], ms=4))
            feas.append(dict(material=mat, orientation=orient,
                             feed_power_limit_mm_s=r(vmax * 1e3, 2),
                             mu_required_at_limit=r(mu[-1]),
                             min_wheel_load_ratio_at_limit=r(nmin[-1])))
    ax[0].axhspan(0.3, 0.6, color=fs.LIGHT, lw=0)
    fs.note(ax[0], 'assumed friction range\n(dusty to clean floor)', x=0.98, y=0.62)
    ax[0].set_ylim(0, 1.0)
    ax[0].set_xlim(0, 12.5)
    ax[1].set_xlim(0, 12.5)
    ax[0].set_xlabel(fs.label('Feed rate', 'mm/s'))
    ax[0].set_ylabel('Required friction coefficient')
    ax[0].legend(loc='upper left')
    ax[1].axhline(0.0, **fs.line('reference', lw=0.6))
    ax[1].set_ylim(-0.1, 1.15)
    ax[1].set_xlabel(fs.label('Feed rate', 'mm/s'))
    ax[1].set_ylabel('Smallest wheel load / static load')
    fs.note(ax[1], 'thick: horizontal cut at 0.81 m\nthin: vertical cut', x=0.03, y=0.3,
            ha='left')
    fs.panel_labels(ax, ['Friction needed to hold the cut', 'Tipping margin'])
    fs.save(fig, os.path.join(out, 'fig_feasibility.png'))
    write_csv(os.path.join(out, 'feasibility.csv'), feas)
    return rows, feas


# ---------------------------------------------------------------------------
# 3. 线性分析
# ---------------------------------------------------------------------------
def linear_analysis(fs, out):
    w = np.logspace(-2, math.log10(math.pi / T), 3000)
    rows = []
    fig, ax = fs.subplots(1, 2, width='double', height=2.4)
    for name in ('P', 'PI', 'PID', 'DOB', 'LQG'):
        c = axis_controller(name)
        lp = Loop(c)
        mg = lp.margins(w)
        L, G = lp.response(w)
        p = lp.poles()
        st = lp.step(600)
        gains = {'P': f'kp={KP[2]:g}',
                 'PI': 'kp={:.2f} ki={:.2f}'.format(*do.pi_triple_pole(K_IN)),
                 'PID': 'kp={:.2f} ki={:.2f} kd={:.2f}'.format(
                     *do.pid_pole_placement(K_IN, 0.4 * K_IN, 1.0)),
                 'DOB': f'kp={KP[2]:g} omega_o=40',
                 'LQG': 'kp={:.2f} kd={:.2f} sigma_d/sigma_y=2'.format(
                     *do.lqr_gains(do.AxisModel(K_IN, T), KP[2] ** 2, 1.0))}[name]
        rows.append(dict(
            controller=name, gains=gains, max_pole_modulus=r(max(abs(p))),
            crossover_rad_s=r(mg['wc'], 2), phase_margin_deg=r(mg['pm'], 1),
            gain_margin=r(mg['gm'], 2), delay_margin_ms=r(1e3 * mg['delay_margin'], 0),
            sensitivity_peak=r(mg['ms'], 2),
            tip_compliance_lowfreq_mm_per_N=r(tip_gain(mg['g0']), 4),
            tip_compliance_peak_mm_per_N=r(tip_gain(mg['gpeak']), 3),
            step_peak_tip_mm_per_Nm=r(1e3 * PF.tool_tip(cf.SLOT_DEPTH)[0] * abs(st).max()
                                      / (IZ * K_IN), 3),
            step_settle_s='' if name == 'P' else r(
                (np.where(abs(st) > 0.02 * abs(st).max())[0][-1] + 1) * T, 2)))
        kw = fs.line(ROLE[name], label=LABEL[name])
        ax[0].loglog(w, tip_gain(np.abs(G)), **kw)
        ax[1].loglog(w, np.abs(1.0 / (1.0 + L)), **kw)
    ax[0].set_xlabel(fs.label('Frequency', 'rad/s'))
    ax[0].set_ylabel(fs.label('Tool-tip compliance', 'mm/N'))
    ax[0].legend(loc='lower center', ncol=2)
    ax[1].axhline(1.0, **fs.line('reference', lw=0.6))
    ax[1].set_xlabel(fs.label('Frequency', 'rad/s'))
    ax[1].set_ylabel('Sensitivity |S|')
    for a in ax:
        a.set_xlim(0.05, math.pi / T)
        fs.plain_log(a)
        fs.plain_log(a, 'x')
    ax[0].set_ylim(1e-3, 10)
    ax[1].set_ylim(1e-2, 5)
    fs.panel_labels(ax, ['Lateral force to tool-tip deviation', 'Sensitivity function'])
    fs.save(fig, os.path.join(out, 'fig_frequency.png'))
    write_csv(os.path.join(out, 'controllers.csv'), rows)

    # 设计折中、鲁棒性
    fig, ax = fs.subplots(1, 3, width='double', height=2.4)
    fam = [('PI', 'alt2', [dict(ki=k) for k in (4, 8, 14.8, 22, 30)]),
           ('PID', 'alt', [dict(omega=x) for x in (5, 6, 7, 8, 9, 10)]),
           ('DOB', 'proposed', [dict(omega_o=x) for x in (10, 20, 30, 40, 60, 80)]),
           ('LQG', 'alt4', [dict(noise_ratio=x) for x in (0.5, 1, 2, 4, 8, 16)])]
    trade = []
    for name, role, sweep in fam:
        xs, ys = [], []
        for kw in sweep:
            if name == 'PI':
                c = do.AxisPID(K_IN / 3.0, kw['ki'], 0.0, T)
            else:
                c = axis_controller(name, **kw)
            lp = Loop(c)
            mg = lp.margins(w)
            peak = abs(lp.step(600)).max()
            xs.append(mg['ms'])
            ys.append(1e3 * PF.tool_tip(cf.SLOT_DEPTH)[0] * peak / (IZ * K_IN))
            trade.append(dict(family=name, parameter=list(kw.values())[0],
                              sensitivity_peak=r(mg['ms'], 3), step_peak_tip_mm_per_Nm=r(ys[-1])))
        ax[0].plot(xs, ys, label=LABEL[name], **fs.line(role, marker=True))
    c = axis_controller('P')
    ax[0].plot(Loop(c).margins(w)['ms'],
               1e3 * PF.tool_tip(cf.SLOT_DEPTH)[0] * abs(Loop(c).step(600)).max() / (IZ * K_IN),
               label=LABEL['P'], **fs.marker('baseline', ms=4))
    ax[0].set_xlim(1.2, 2.6)
    ax[0].set_xlabel('Sensitivity peak $M_s$')
    ax[0].set_ylabel(fs.label('Peak tip deviation per N·m step', 'mm'))
    ax[0].legend(loc='upper right')
    ratios = np.geomspace(0.25, 4.0, 41)
    robust = []
    for name in ('P', 'PI', 'PID', 'DOB'):
        rho = [max(abs(Loop(axis_controller(name), k_true=K_IN * q).poles())) for q in ratios]
        ax[1].semilogx(ratios, rho, label=LABEL[name], **fs.line(ROLE[name]))
        for d in (0, 1, 2, 3, 4):
            robust.append(dict(controller=name, loop_delay_steps=d, max_pole_modulus=r(
                max(abs(Loop(axis_controller(name), delay=d).poles())))))
        for q in (0.25, 0.5, 2.0, 4.0):
            robust.append(dict(controller=name, k_in_ratio=q, max_pole_modulus=r(
                max(abs(Loop(axis_controller(name), k_true=K_IN * q).poles())))))
    ax[1].axhline(1.0, **fs.line('reference', lw=0.6))
    ax[1].set_xlabel('True / nominal velocity-loop gain')
    ax[1].set_ylabel('Largest closed-loop pole |z|')
    ax[1].set_ylim(0.75, 1.1)
    ax[1].set_xticks([0.25, 0.5, 1.0, 2.0, 4.0])
    fs.plain_ticks(ax[1], 'x')
    ax[1].legend(loc='upper right')
    # 结构性上限：采样周期与速度环增益
    struct = []
    for dt, role in ((0.02, 'baseline'), (0.005, 'alt'), (0.001, 'proposed')):
        ks = np.geomspace(10, 400, 14)
        pk = []
        for k in ks:
            wo = min(2.0 * k, 0.8 / dt)
            m = do.AxisModel(k, dt)
            c = do.AxisDOB(m, k / 4.0, 0.0, gain=do.bandwidth_gain(m, wo))
            lp = Loop(c, k_true=k, dt=dt)
            pk.append(1e3 * PF.tool_tip(cf.SLOT_DEPTH)[0] * abs(lp.step(int(8 / dt))).max()
                      / (IZ * k))
            struct.append(dict(period_ms=dt * 1e3, k_in=r(k, 1), observer_bandwidth=r(wo, 1),
                               step_peak_tip_mm_per_Nm=r(pk[-1], 4),
                               sensitivity_peak=r(lp.margins(
                                   np.logspace(-1, math.log10(math.pi / dt), 800))['ms'], 2)))
        ax[2].loglog(ks, pk, label=f'T = {dt * 1e3:g} ms', **fs.line(role))
    ax[2].axhline(0.1, **fs.line('reference', lw=0.6))
    fs.note(ax[2], '1 mm at 10 N·m', x=0.03, y=0.37, ha='left')
    ax[2].set_xlabel(fs.label('Velocity-loop gain $k_{in}$', '1/s'))
    ax[2].set_ylabel(fs.label('Peak tip deviation per N·m step', 'mm'))
    fs.plain_log(ax[2])
    fs.plain_log(ax[2], 'x')
    ax[2].legend(loc='upper right')
    fs.panel_labels(ax, ['Rejection vs. robustness', 'Velocity-loop gain mismatch',
                         'Limit set by sampling and inner loop'])
    fs.save(fig, os.path.join(out, 'fig_robustness.png'))
    write_csv(os.path.join(out, 'tradeoff.csv'), trade)
    write_csv(os.path.join(out, 'robustness.csv'),
              [dict(controller=x['controller'], loop_delay_steps=x.get('loop_delay_steps', ''),
                    k_in_ratio=x.get('k_in_ratio', ''), max_pole_modulus=x['max_pole_modulus'])
               for x in robust])
    write_csv(os.path.join(out, 'structural_limit.csv'), struct)
    return rows


# ---------------------------------------------------------------------------
# 4. 闭环仿真：力控底盘 + 里程计反馈
# ---------------------------------------------------------------------------
def run_force(name, orient, feed, kind, seed=None, limits='raised', material=MATERIAL,
              hold_kw=None):
    blade = cf.BladeParams.for_material(material)
    pf = cf.Platform(tool_y=0.0 if orient == 'horizontal' else 0.06)
    prof = cf.CutProfile(orient, 1, feed_rate=feed, length=CUT_LENGTH, kind=kind,
                         ramp_time=RAMP_TIME)
    proc = cf.CuttingProcess(blade, pf, prof, seed=seed)
    hold, ff = ps.make_hold(name, limits, **(hold_kw or {}))
    lim = ps.LIMITS[limits]
    plant = ps.ForcePlant(lim['a_max'], lim['alpha_max'], seed=1 if seed is None else seed)
    rec = ps.simulate(hold, proc, prof.t_end + 2.0, plant, ff)
    return rec, prof, ps.metrics(rec, prof, limits)


def closed_loop(fs, out, seeds):
    cases, summary = [], []
    stoch = {}
    for orient in ('horizontal', 'vertical'):
        for feed in FEEDS:
            for kind in ('step', 'ramp', 'cut'):
                for name in CTRL:
                    _, _, m = run_force(name, orient, feed, kind)
                    cases.append(dict(orientation=orient, feed_mm_s=feed * 1e3, disturbance=kind,
                                      controller=name, seed='',
                                      **{k: r(v) for k, v in m.items()}))
            for name in CTRL:
                ms = []
                for sd in range(1, seeds + 1):
                    _, _, m = run_force(name, orient, feed, 'ramp', seed=sd)
                    ms.append(m)
                    cases.append(dict(orientation=orient, feed_mm_s=feed * 1e3,
                                      disturbance='ramp+stochastic', controller=name, seed=sd,
                                      **{k: r(v) for k, v in m.items()}))
                peak = np.array([x['tip_peak_mm'] for x in ms])
                rms = np.array([x['tip_rms_mm'] for x in ms])
                stoch[(orient, feed, name)] = peak
                summary.append(dict(
                    orientation=orient, feed_mm_s=feed * 1e3, controller=name, n=seeds,
                    tip_peak_mean_mm=r(peak.mean()), tip_peak_std_mm=r(peak.std()),
                    tip_peak_max_mm=r(peak.max()), tip_rms_mean_mm=r(rms.mean()),
                    release_peak_mean_mm=r(np.mean([x['release_peak_mm'] for x in ms])),
                    base_peak_mean_mm=r(np.mean([x['base_peak_mm'] for x in ms])),
                    yaw_peak_mean_deg=r(np.mean([x['yaw_peak_deg'] for x in ms])),
                    cmd_ang_rms_rad_s=r(np.mean([x['cmd_ang_rms'] for x in ms])),
                    cmd_lin_rms_m_s=r(np.mean([x['cmd_lin_rms'] for x in ms]), 4)))
    write_csv(os.path.join(out, 'sim_cases.csv'), cases)
    write_csv(os.path.join(out, 'sim_summary.csv'), summary)

    # 时间响应：横槽、烧结砖、5 mm/s
    fig, ax = fs.subplots(1, 3, width='double', height=2.4)
    for j, kind in enumerate(('step', 'ramp')):
        for name in CTRL:
            rec, prof, _ = run_force(name, 'horizontal', 0.005, kind)
            tip = np.hypot(rec['tipx'], rec['tipy']) * 1e3
            ax[j].plot(rec['t'] - prof.t_start, np.where(tip >= 0.02, tip, np.nan),
                       label=LABEL[name], **fs.line(ROLE[name]))
        ax[j].set_yscale('log')
        fs.plain_log(ax[j])
        ax[j].set_ylim(0.02, 200)
        ax[j].set_xlim(-0.5, 12.0)
        ax[j].axhline(1.0, **fs.line('reference', lw=0.6))
        ax[j].set_xlabel(fs.label('Time since engagement', 's'))
        ax[j].set_ylabel(fs.label('Tool-tip deviation', 'mm'))
    for name in CTRL:
        rec, prof, _ = run_force(name, 'horizontal', 0.005, 'step')
        ax[2].plot(rec['t'] - prof.t_start, rec['uw'], label=LABEL[name],
                   **fs.line(ROLE[name]))
    ax[2].set_ylim(-0.9, 0.9)
    ax[2].legend(loc='upper right')
    ax[2].set_xlim(-0.2, 3.0)
    ax[2].set_xlabel(fs.label('Time since engagement', 's'))
    ax[2].set_ylabel(fs.label('Yaw-rate command', 'rad/s'))
    fs.panel_labels(ax, ['Step engagement', 'Force ramped over 4 s',
                         'Control effort (step)'])
    fs.save(fig, os.path.join(out, 'fig_step_response.png'))

    # 随机扰动下的刀尖偏移
    fig, ax = fs.subplots(1, 2, width='double', height=2.4, sharey=True)
    width = 0.14
    for j, orient in enumerate(('horizontal', 'vertical')):
        for i, name in enumerate(CTRL):
            mean = [stoch[(orient, f, name)].mean() for f in FEEDS]
            std = [stoch[(orient, f, name)].std() for f in FEEDS]
            x = np.arange(len(FEEDS)) + (i - 2.5) * width
            ax[j].bar(x, mean, width, yerr=std, label=LABEL[name], **fs.bar(ROLE[name]),
                      error_kw=dict(lw=0.6))
        ax[j].set_yscale('log')
        fs.plain_log(ax[j])
        ax[j].axhline(1.0, **fs.line('reference', lw=0.6))
        ax[j].set_xticks(np.arange(len(FEEDS)))
        ax[j].set_xticklabels([f'{f * 1e3:g}' for f in FEEDS])
        ax[j].set_xlabel(fs.label('Feed rate', 'mm/s'))
    ax[0].set_ylabel(fs.label('Peak tool-tip deviation', 'mm'))
    ax[0].set_ylim(0.01, 300)
    ax[1].legend(loc='upper left', ncol=2)
    fs.panel_labels(ax, ['Horizontal cut', 'Vertical cut (tool 60 mm off centre)'])
    fs.save(fig, os.path.join(out, 'fig_cases.png'))

    # 限幅与抗积分饱和（现有默认限幅 0.5 rad/s ≙ 9.2 N·m；横槽、烧结砖、阶跃切入 6 s 后退刀）
    fig, ax = fs.subplots(1, 2, width='double', height=2.3)
    sat = []
    variants = [('P', {}, 'baseline', 'P'), ('PI', dict(anti_windup=False), 'alt2',
                                             'PI, no anti-windup'),
                ('PI', {}, 'alt', 'PI, back-calculation'), ('DOB', {}, 'proposed', 'DOB')]
    for feed in (0.002, 0.004, 0.005):
        blade = cf.BladeParams.for_material(MATERIAL)
        mz = cf.cut_wrench(blade, PF, cf.SLOT_DEPTH, feed, 'horizontal').mz
        for name, kw, role, lab in variants:
            prof = cf.CutProfile('horizontal', 1, feed_rate=feed, length=feed * 6.0,
                                 kind='step')
            proc = cf.CuttingProcess(blade, PF, prof)
            hold, _ = ps.make_hold(name, 'default', **kw)
            lim = ps.LIMITS['default']
            rec = ps.simulate(hold, proc, prof.t_end + 6.0,
                              ps.ForcePlant(lim['a_max'], lim['alpha_max']))
            tip = np.hypot(rec['tipx'], rec['tipy']) * 1e3
            eng = (rec['t'] >= prof.t_start) & (rec['t'] <= prof.t_end)
            after = rec['t'] > prof.t_end
            back = np.where(tip[after] > 1.0)[0]
            sat.append(dict(feed_mm_s=feed * 1e3, yaw_moment_Nm=r(mz, 2), controller=lab,
                            tip_peak_engaged_mm=r(tip[eng].max()),
                            tip_at_release_mm=r(tip[after][0]),
                            tip_peak_after_release_mm=r(tip[after].max()),
                            saturated_frac=r(np.mean(np.abs(rec['uw'][eng]) >= 0.4995)),
                            return_within_1mm_after_release_s=r(
                                (back[-1] + 1) * T if len(back) else 0.0, 2)))
            if feed == 0.004:
                ax[0].plot(rec['t'] - prof.t_start, tip, label=lab, **fs.line(role))
                ax[1].plot(rec['t'] - prof.t_start, rec['uw'], **fs.line(role))
    ax[0].set_yscale('log')
    fs.plain_log(ax[0])
    ax[0].set_ylim(0.05, 500)
    ax[0].axhline(1.0, **fs.line('reference', lw=0.6))
    ax[0].set_xlabel(fs.label('Time since engagement', 's'))
    ax[0].set_ylabel(fs.label('Tool-tip deviation', 'mm'))
    h, lab = ax[0].get_legend_handles_labels()
    ax[1].legend(h, lab, loc='center left', bbox_to_anchor=(0.12, 0.62))
    ax[1].axhline(-0.5, **fs.line('reference', lw=0.6))
    ax[1].set_xlabel(fs.label('Time since engagement', 's'))
    ax[1].set_ylabel(fs.label('Yaw-rate command', 'rad/s'))
    fs.panel_labels(ax, ['8.6 N·m against a 9.2 N·m limit', 'Command and wind-up'])
    fs.save(fig, os.path.join(out, 'fig_saturation.png'))
    write_csv(os.path.join(out, 'saturation.csv'), sat)
    return summary


# ---------------------------------------------------------------------------
# 5. 里程计看不到打滑
# ---------------------------------------------------------------------------
def kalman_hold(use_tag):
    """卡尔曼形式的扰动观测器：对象取轮速环滞后 1/τ_w = 50 /s，扰动为底盘相对轮子的滑移速度."""
    q_lin = (0.0, (2e-4) ** 2, (3.5e-4) ** 2)
    q_ang = (0.0, (1e-4) ** 2, (3e-4) ** 2)
    kal = [(q_lin, ps.SIG_LIDAR[0] ** 2, 1e-8),
           (q_lin, ps.SIG_TAG ** 2 if use_tag else 1e-8, 1e-8),
           (q_ang, ps.SIG_LIDAR[1] ** 2, 1e-8)]
    return do.PlanarHold('dob', dt=T, kp=(2.0, 2.0, 2.0), k_in=(50.0, 50.0, 50.0),
                         limits=ps.LIMITS['default']['hold'], kalman=kal, delay_steps=1)


def run_slip(variant, feed, seed, orient='horizontal'):
    blade = cf.BladeParams.for_material(MATERIAL)
    pf = cf.Platform(tool_y=0.0 if orient == 'horizontal' else 0.06)
    prof = cf.CutProfile(orient, 1, feed_rate=feed, length=0.06, kind='ramp',
                         ramp_time=RAMP_TIME)
    proc = cf.CuttingProcess(blade, pf, prof, seed=seed)
    plant = ps.ContactPlant()
    lim = ps.LIMITS['default']['hold']
    use_tag = '+tag' in variant
    sensors = ps.WallSensors(100 + seed, use_tag, '+gyro' in variant)
    if variant == 'P, odometry':
        hold, dev = do.PlanarHold('p', dt=T, kp=KP, limits=lim), None
    elif variant == 'DOB, odometry':
        hold, dev = do.PlanarHold('dob', dt=T, kp=KP, limits=lim), None
    elif variant.startswith('P, wall EKF'):
        hold = do.PlanarHold('p', dt=T, kp=KP, limits=lim)
        dev = ps.wall_ekf_feedback(sensors)
    else:
        hold, dev = kalman_hold(use_tag), ps.kalman_dob_feedback(sensors)
    rec = ps.simulate(hold, proc, prof.t_end + 2.0, plant, dt=2.5e-4, deviation=dev)
    m = ps.metrics(rec, prof)
    m['odom_yaw_peak_deg'] = float(np.degrees(np.abs(rec['opsi']).max()))
    m['contact_saturated'] = int(plant.saturated)
    return rec, prof, m


def slip_cases(fs, out, seeds):
    seeds = min(seeds, 5)
    variants = ['P, odometry', 'DOB, odometry', 'P, wall EKF', 'P, wall EKF +tag',
                'DOB (Kalman), LiDAR', 'DOB (Kalman), LiDAR +tag',
                'DOB (Kalman), LiDAR +tag +gyro']
    roles = dict(zip(variants, ('baseline', 'neutral', 'alt2', 'alt', 'alt4', 'alt3',
                                'proposed')))
    rows, summary = [], []
    for feed in (0.002, 0.005):
        for v in variants:
            ms = []
            for sd in range(1, seeds + 1):
                _, _, m = run_slip(v, feed, sd)
                ms.append(m)
                rows.append(dict(feed_mm_s=feed * 1e3, variant=v, seed=sd,
                                 **{k: r(x) for k, x in m.items()}))
            summary.append(dict(
                feed_mm_s=feed * 1e3, variant=v, n=seeds,
                tip_peak_mean_mm=r(np.mean([x['tip_peak_mm'] for x in ms])),
                tip_peak_std_mm=r(np.std([x['tip_peak_mm'] for x in ms])),
                tip_rms_mean_mm=r(np.mean([x['tip_rms_mm'] for x in ms])),
                depth_peak_mean_mm=r(np.mean([x['depth_peak_mm'] for x in ms])),
                along_wall_peak_mean_mm=r(np.mean([x['along_wall_peak_mm'] for x in ms])),
                yaw_peak_mean_deg=r(np.mean([x['yaw_peak_deg'] for x in ms])),
                odom_yaw_peak_mean_deg=r(np.mean([x['odom_yaw_peak_deg'] for x in ms]))))
    write_csv(os.path.join(out, 'slip_cases.csv'), rows)
    write_csv(os.path.join(out, 'slip_summary.csv'), summary)

    fig, ax = fs.subplots(1, 3, width='double', height=2.4)
    for v in variants:
        rec, prof, _ = run_slip(v, 0.005, 1)
        t = rec['t'] - prof.t_start
        kw = fs.line(roles[v], label=v.replace(' +', ' + '))
        ax[0].plot(t, np.degrees(rec['psi']), **kw)
        ax[1].plot(t, -rec['tipy'] * 1e3, **kw)
        if v in ('P, odometry', 'DOB (Kalman), LiDAR +tag +gyro'):
            short = 'P on odometry' if v.startswith('P') else 'DOB on wall sensors'
            ax[2].plot(t, np.degrees(rec['psi']), label=f'true, {short}',
                       **fs.line(roles[v]))
            ax[2].plot(t, np.degrees(rec['opsi']), label=f'odometry, {short}',
                       **fs.line(roles[v], lw=0.6, marker=True, markevery=40))
    ax[0].set_ylabel(fs.label('True heading deviation', '°'))
    ax[1].set_ylabel(fs.label('Tool-tip along-wall deviation', 'mm'))
    ax[2].set_ylabel(fs.label('Heading', '°'))
    ax[0].set_ylim(-0.6, 3.0)
    ax[1].set_ylim(-70, 8)
    for a in ax:
        a.set_xlabel(fs.label('Time since engagement', 's'))
        a.set_xlim(-0.5, 13.5)
    h, lab = ax[0].get_legend_handles_labels()
    fig.legend(h, lab, loc='upper center', ncol=3, bbox_to_anchor=(0.36, 1.0))
    ax[2].set_ylim(-5.5, 12.5)
    ax[2].legend(loc='upper left')
    fs.panel_labels(ax, ['Heading', 'Along-wall tip deviation', 'What odometry reports'])
    fs.fixed_layout(fig, left=0.45, right=0.08, bottom=0.42, top=0.62, wspace=0.6)
    fs.save(fig, os.path.join(out, 'fig_slip.png'))
    return summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seeds', type=int, default=8)
    ap.add_argument('--out', default=HERE)
    ap.add_argument('--only', default='', help='force | linear | sim | slip')
    args = ap.parse_args()
    import figstyle as fs

    if args.only in ('', 'force'):
        _, feas = force_and_feasibility(fs, args.out)
        for x in feas:
            print(x)
    if args.only in ('', 'linear'):
        for x in linear_analysis(fs, args.out):
            print(x)
    if args.only in ('', 'sim'):
        for x in closed_loop(fs, args.out, args.seeds):
            print(x)
    if args.only in ('', 'slip'):
        for x in slip_cases(fs, args.out, args.seeds):
            print(x)


if __name__ == '__main__':
    main()
