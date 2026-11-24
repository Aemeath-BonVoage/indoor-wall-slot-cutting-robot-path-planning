#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""技术报告的新增图：系统架构图、结果汇总图（3 × 2）。其余图直接引用各实验报告中的原图.

  python3 docs/report/make_figures.py

只读取仓库中已提交的 CSV（docs/experiments/**），不需要 ROS / Gazebo。
输出：docs/report/fig_architecture.{png,pdf}、docs/report/fig_summary.{png,pdf}
（样式见 docs/figstyle，双栏宽度）
"""
import csv
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.join(HERE, '..', 'experiments')
sys.path.insert(0, os.path.join(HERE, '..'))
import figstyle as fs  # noqa: E402
import matplotlib.patches as mpatches  # noqa: E402
plt = fs.plt
INK, INK2, SURFACE = fs.BLACK, '#404040', 'white'
TXT = 7             # 方框正文字号（pt）；标题 7.5 pt


def load(path):
    with open(path, encoding='utf-8') as f:
        return list(csv.DictReader(f))


# ---------------------------------------------------------------------------
# 系统架构图
# ---------------------------------------------------------------------------
def box(ax, x, y, w, h, title, lines=(), fc='#ffffff', ec=INK2, title_size=7.5, dy=0.0235):
    ax.add_patch(mpatches.FancyBboxPatch((x, y), w, h,
                                         boxstyle='round,pad=0.004,rounding_size=0.008',
                                         fc=fc, ec=ec, lw=0.6))
    ax.text(x + w / 2, y + h - 0.012, title, ha='center', va='top', fontsize=title_size,
            color=INK, fontweight='bold')
    for i, s in enumerate(lines):
        ax.text(x + w / 2, y + h - 0.041 - i * dy, s, ha='center', va='top', fontsize=TXT,
                color=INK2)


def group(ax, x, y, w, h, label, fc, lx=None):
    ax.add_patch(mpatches.FancyBboxPatch((x, y), w, h,
                                         boxstyle='round,pad=0.004,rounding_size=0.01',
                                         fc=fc, ec='none'))
    ax.text(x + 0.006 if lx is None else lx, y + h - 0.008, label, ha='left', va='top',
            fontsize=TXT, color=INK, fontstyle='italic')


def _text_span(ax, t, pad=0.006):
    """文字在数据坐标中的竖直范围（含留白），用于把连线在标签处断开."""
    r = ax.figure.canvas.get_renderer()
    bb = t.get_window_extent(r).transformed(ax.transData.inverted())
    return bb.y0 - pad, bb.y1 + pad


def arrow(ax, p, q, text='', dashed=False, color=INK2, tx=None):
    """箭头 p → q；带标签的竖直箭头在标签处断开，标签不压线."""
    ls = (0, (3, 1.5)) if dashed else '-'
    head = dict(arrowstyle='-|>', color=color, lw=0.7, ls=ls, mutation_scale=6, shrinkA=1,
                shrinkB=1)
    if not text:
        ax.annotate('', xy=q, xytext=p, arrowprops=head)
        return
    mx, my = tx if tx else ((p[0] + q[0]) / 2, (p[1] + q[1]) / 2)
    t = ax.text(mx, my, text, fontsize=TXT, color=INK2, ha='center', va='center')
    lo, hi = _text_span(ax, t)
    up = q[1] > p[1]
    a, b = (lo, hi) if up else (hi, lo)
    ax.plot([p[0], p[0]], [p[1], a], color=color, lw=0.7, ls=ls)
    ax.annotate('', xy=q, xytext=(q[0], b), arrowprops=dict(head, shrinkA=0))


def vline(ax, pts, color=INK2, dashed=False):
    """折线（不带箭头），pts = [(x, y), ...]."""
    xs, ys = zip(*pts)
    ax.plot(xs, ys, color=color, lw=0.7, ls=(0, (3, 1.5)) if dashed else '-')


def vlabel(ax, x, y, text):
    """竖排标签；返回其竖直范围，调用方据此把竖线断开."""
    t = ax.text(x, y, text, rotation=90, fontsize=TXT, color=INK2, ha='center', va='center')
    return _text_span(ax, t)


def fig_architecture(out):
    # 坐标为图幅分数；方框带 0.004 的圆角外扩
    fig = fs.figure(width='double', height=5.9, layout='none')
    ax = fig.add_axes([0, 0, 1, 1])
    fig.canvas.draw()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis('off')
    C_PLAN, C_CLI, C_GT = '#EAF2FA', '#F2F2F2', '#F5F5F5'
    C_TASK, C_NAV, C_SIM, C_OUT = '#FBEFE6', '#E8F5EF', '#F6EFF7', '#FFF6E5'

    # ---- 左列：离线规划、命令行 / 记录器、真值 ----
    group(ax, 0.008, 0.62, 0.255, 0.372, 'Offline planning (no ROS): scr_planner', C_PLAN)
    box(ax, 0.02, 0.875, 0.231, 0.075, 'Wall task YAML', ['wall type / obstacles / boxes'])
    box(ax, 0.02, 0.742, 0.231, 0.1, 'plan_wall_slots',
        ['OrientedAStar: rule-constrained slots', 'joint planning, DP partition, time model'])
    box(ax, 0.02, 0.632, 0.231, 0.075, 'Plan YAML', ['slots / stations / arm points / time'],
        fc=C_OUT)
    arrow(ax, (0.135, 0.871), (0.135, 0.846))
    arrow(ax, (0.135, 0.738), (0.135, 0.711))

    group(ax, 0.008, 0.315, 0.255, 0.29, 'Command line / recorder', C_CLI)
    box(ax, 0.02, 0.465, 0.231, 0.1, 'navigation_cutting_task',
        ['task file → task action', 'prints feedback and station log'])
    box(ax, 0.02, 0.33, 0.231, 0.1, 'execute_slot_plan.py',
        ['client of the same action', '+ ground truth: tool tip, arrival'])
    arrow(ax, (0.235, 0.628), (0.235, 0.569))

    group(ax, 0.008, 0.012, 0.255, 0.285, 'Gazebo ground truth', C_GT)
    box(ax, 0.02, 0.026, 0.231, 0.225, 'Evaluation only',
        ['/ground_truth/odom', '/gazebo/model_states', '/gazebo/link_states',
         'never used for control', 'or estimation'], dy=0.03)
    arrow(ax, (0.135, 0.255), (0.135, 0.326), dashed=True)

    # 客户端 → 行为树执行器
    lo, hi = vlabel(ax, 0.278, 0.70, '/execute_slot_plan, /execute_cutting_task')
    vline(ax, [(0.255, 0.515), (0.278, 0.515), (0.278, lo)])
    vline(ax, [(0.278, hi), (0.278, 0.88)])
    vline(ax, [(0.255, 0.38), (0.278, 0.38), (0.278, 0.515)])
    arrow(ax, (0.278, 0.88), (0.299, 0.88))

    # ---- 中列：任务层 ----
    group(ax, 0.29, 0.45, 0.372, 0.544, 'Task layer: behaviour tree + skill actions', C_TASK)
    box(ax, 0.302, 0.81, 0.348, 0.135, 'task_executor (scr_task, BehaviorTree.CPP v3)',
        ['2 × BtActionServer (as in Nav2 bt_navigator)', 'slot_plan.xml / cutting_task.xml',
         'recovery: retry nav., re-align, re-cut'])
    arrow(ax, (0.476, 0.806), (0.476, 0.777))
    ax.text(0.488, 0.7915, 'skill actions', ha='left', va='center', fontsize=TXT, color=INK2)
    box(ax, 0.302, 0.462, 0.348, 0.311, 'task_skill_server (scr_arm_controller)',
        ['FineAlign · MoveToStation · CutStation', 'HoldBase · ScanMarker · RetractArm'])
    subs = [('Fine alignment', ['AprilTag + LiDAR', 'EKF (d, u, ψ, s)', 'saturated law',
                                'convergence test']),
            ('Strafe / hold', ['same wall EKF', 'multi-marker map', 'scale correction',
                               'hold: P or DOB']),
            ('Cutting', ['residual comp.', 'S-curve trajectory', 'per station',
                         'dry cut in sim.'])]
    for i, (t, ls) in enumerate(subs):
        box(ax, 0.311 + i * 0.1125, 0.474, 0.103, 0.16, t, ls, title_size=7.5)

    # ---- 右列：Nav2、定位 ----
    group(ax, 0.69, 0.62, 0.302, 0.372, 'Nav2 (scr_navigation)', C_NAV)
    box(ax, 0.702, 0.875, 0.135, 0.075, 'bt_navigator', ['NavigateToPose'])
    box(ax, 0.847, 0.875, 0.135, 0.075, 'amcl', ['map → odom'])
    box(ax, 0.702, 0.742, 0.135, 0.1, 'controller', ['MPPI (omni)', '1000 samples, 20 Hz'])
    box(ax, 0.847, 0.742, 0.135, 0.1, 'planner', ['Smac 2D'])
    ax.text(0.915, 0.672, 'DWB baseline:\nnav2_params_dwb.yaml', ha='center', va='center',
            fontsize=TXT, color=INK2)
    arrow(ax, (0.654, 0.912), (0.698, 0.912))

    group(ax, 0.69, 0.315, 0.302, 0.29, 'Localisation (robot_localization)', C_NAV, lx=0.79)
    box(ax, 0.702, 0.33, 0.28, 0.235, 'EKF (planar mode)',
        ['(vx, vy) of /wheel/odom + wz of /imu', '→ /odom and odom → base_link',
         'serves Nav2 and AMCL only;', 'skills close their local loops',
         'on /wheel/odom directly'], dy=0.03)
    arrow(ax, (0.76, 0.569), (0.76, 0.738), '/odom, TF', tx=(0.76, 0.655))

    # Nav2 控制器 → 底盘控制器（沿中列与右列之间的空隙）
    lo, hi = vlabel(ax, 0.676, 0.39, '/cmd_vel (navigation)')
    vline(ax, [(0.698, 0.79), (0.676, 0.79), (0.676, hi)])
    vline(ax, [(0.676, lo), (0.676, 0.225)])
    arrow(ax, (0.676, 0.225), (0.657, 0.225))

    # ---- 底部：ros2_control + Gazebo ----
    group(ax, 0.29, 0.012, 0.702, 0.285, 'ros2_control + Gazebo Classic', C_SIM, lx=0.70)
    box(ax, 0.302, 0.026, 0.09, 0.235, 'Sensors', ['2D LiDAR', '10 Hz', 'camera', '30 Hz'],
        dy=0.03)
    box(ax, 0.405, 0.185, 0.12, 0.076, 'arm_controller', ['joint trajectory'])
    box(ax, 0.405, 0.026, 0.12, 0.12, 'GazeboSystem', ['arm joints', 'position_pid'])
    box(ax, 0.537, 0.185, 0.12, 0.076, 'base_controller', ['cmd_vel ↔ wheels'])
    box(ax, 0.537, 0.026, 0.12, 0.12, 'ScrMecanumBase',
        ['force-driven plant', 'encoder errors', 'gyro → /imu'])
    box(ax, 0.702, 0.185, 0.28, 0.076, 'Earlier: base_control:=plugin',
        ['scr_planar_move force plugin, /odom = truth'], fc='#FBF8FC')
    box(ax, 0.702, 0.026, 0.28, 0.12, 'Other launch options',
        ['cutting_force:=true (cutting-force plugin)', 'plant:=wheels (wheel-driven, experimental)',
         'localization:=wheel (wheel odom only)', 'odom_error:=false (ideal encoders)'],
        fc='#FBF8FC', dy=0.0215)

    # 技能服务器 ↔ 传感器 / 机械臂 / 底盘
    arrow(ax, (0.347, 0.265), (0.347, 0.47), '/scan,\nimage', dashed=True, tx=(0.347, 0.37))
    arrow(ax, (0.445, 0.458), (0.445, 0.265), 'FollowJoint-\nTrajectory', tx=(0.445, 0.36))
    arrow(ax, (0.565, 0.458), (0.565, 0.265), '/cmd_vel', tx=(0.565, 0.41))
    arrow(ax, (0.63, 0.265), (0.63, 0.458), '/wheel/\nodom', dashed=True, tx=(0.63, 0.33))
    # 底盘 → EKF
    arrow(ax, (0.94, 0.30), (0.94, 0.326), dashed=True)
    ax.text(0.93, 0.312, '/wheel/odom, /imu', ha='right', va='center', fontsize=TXT,
            color=INK2)

    fs.save(fig, out)


# ---------------------------------------------------------------------------
# 结果汇总图
# ---------------------------------------------------------------------------
def fig_summary(out):
    fig, axs = fs.subplots(3, 2, width='double', height=7.8)

    # (a) 精对位：第一版 vs 第二版，各条件位置误差
    rows = load(os.path.join(EXP, 'alignment_robustness', 'summary.csv'))
    conds = [('nominal', 'Nominal'), ('odom_drift_10cm', 'Odom. drift\n10 cm'),
             ('light_cool', 'Cool\nlight'), ('occlusion_intermittent', 'Intermittent\nocclusion'),
             ('occlusion_static', 'Static\nocclusion'), ('camera_yaw_1deg', 'Camera\nyaw 1°')]
    S = {(r['condition'], r['pipeline']): r for r in rows}
    ax = axs[0, 0]
    x = np.arange(len(conds))
    pipes = (('v1', 'v1 (colour blob + PBVS)', 'baseline'),
             ('v2', 'v2 (AprilTag + LiDAR EKF)', 'proposed'),
             ('v2_taught', 'v2 + taught zero offset', 'alt2'))
    for j, (pipe, lab, role) in enumerate(pipes):
        for i, (cond, _) in enumerate(conds):
            r = S.get((cond, pipe))
            if r is None:
                continue
            present = [q for q, _, _ in pipes if (cond, q) in S]
            xi = x[i] + (present.index(pipe) - (len(present) - 1) / 2) * 0.27
            if int(r['success']) == 0:
                ax.text(xi, 0.3, 'all 8 failed', rotation=90, ha='center', va='bottom',
                        fontsize=7, color=fs.color(role))
                continue
            m, sd = float(r['pos_mean_mm']), float(r['pos_std_mm'])
            ax.bar(xi, m, 0.25, yerr=sd, error_kw=dict(lw=0.6, capsize=1.5), **fs.bar(role))
        ax.bar(0, 0, 0, label=lab, **fs.bar(role))
    ax.set_xticks(x)
    ax.set_xticklabels([c[1] for c in conds], fontsize=7)
    ax.set_ylabel(fs.label('Final position error', 'mm'))
    ax.set_ylim(0, 15)
    ax.legend(loc='upper left')
    fs.panel_label(ax, 'a', 'Fine alignment in Gazebo (n = 8 per condition)')

    # (b) 多站执行：每次运行的到站横向误差（均值 ± 站间标准差）
    ax = axs[0, 1]
    runs = [('dp', 'v1, run 1', 'baseline'), ('dp_repeat', 'v1, run 2', 'baseline'),
            ('per_slot', 'v1, per-slot plan 1', 'baseline'),
            ('per_slot_repeat', 'v1, per-slot plan 2', 'baseline'),
            ('dp_nav2_each_station', 'v1, Nav2 per station', 'baseline'),
            ('fine_align_dp', 'v2, run 1', 'proposed'),
            ('fine_align_dp_repeat', 'v2, run 2', 'proposed'),
            ('fine_align_dp_nofold', 'v2, origin fold-in off', 'neutral')]
    for i, (d, lab, role) in enumerate(runs):
        st = load(os.path.join(EXP, 'slot_execution', 'runs', d, 'stations.csv'))
        lat = np.array([float(r['lateral_mm']) for r in st])
        ax.errorbar(lat.mean(), i, xerr=lat.std(ddof=1), fmt=fs.MARKER[role],
                    color=fs.color(role), mfc='white' if role == 'baseline' else fs.color(role),
                    ms=3.5, elinewidth=0.7, capsize=2)
        ax.text(1.0, i, f'{lat.mean():+.2f}'.replace('-', '−'), ha='right', va='center',
                fontsize=7, transform=ax.get_yaxis_transform())
    ax.axvline(0, color=fs.GREY, lw=0.5)
    ax.set_yticks(range(len(runs)))
    ax.set_yticklabels([r[1] for r in runs], fontsize=7)
    ax.set_ylim(len(runs) - 0.5, -0.5)
    xl = ax.get_xlim()
    ax.set_xlim(xl[0], xl[1] + 0.25 * (xl[1] - xl[0]))     # 右侧留出均值一列
    ax.set_xlabel(fs.label('Along-wall arrival error, mean ± std. over stations', 'mm'))
    fs.panel_label(ax, 'b', 'Multi-station runs (value: mean)')

    # (c) 时间模型：当前默认执行路径下的样本外停靠站（plan_validation 的 5 次验证运行）
    ax = axs[1, 0]
    models = {'align_v2': (2.83, 0.0984, 2.049), 'align_v3': (4.70, 0.0971, 2.030)}
    st = [r for r in load(os.path.join(EXP, 'plan_validation', 'stations_all.csv'))
          if r['run'] != 'demo_joint']        # 演示运行另加了墙面实体与相机，单独列出
    y = np.array([float(r['station_s']) for r in st])
    dist = np.array([float(r['move_dist_m']) for r in st])
    n = np.array([float(r['points']) for r in st])
    lo, hi = 8, 42
    ax.plot([lo, hi], [lo, hi], **fs.line('reference', lw=0.6))
    for (name, (ts, v, tp)), role, lab in zip(models.items(), ('neutral', 'proposed'),
                                              ('align_v2 (earlier execution path)',
                                               'align_v3 (recalibrated)')):
        p = ts + dist / v + n * tp
        rmse = np.sqrt(np.mean((p - y) ** 2))
        ax.plot(y, p, ls='none', marker='o' if role == 'neutral' else '^', ms=3.2, mfc='none',
                mec=fs.color(role), mew=0.7, label=f'{lab}: RMSE {rmse:.2f} s')
    ax.set_xlim(lo, hi)
    ax.set_ylim(lo, hi)
    ax.set_xlabel(fs.label('Measured time per station (Gazebo)', 's'))
    ax.set_ylabel(fs.label('Predicted time per station', 's'))
    ax.legend(loc='upper left')
    ax.set_aspect('equal', adjustable='datalim')
    fs.panel_label(ax, 'c', f'Time model: {len(y)} held-out stations (5 runs)')

    # (d) 真实里程计下的沿墙精度：各配置每次运行的最大到站沿墙误差
    ax = axs[1, 1]
    cfgs = [('single_off', 'Single tag, odometry only', 'baseline'),
            ('single_on', 'Single tag, tag updates', 'neutral'),
            ('realign_0.3', 'Single tag + re-align 0.3 m', 'alt2'),
            ('tags_1.2', 'Tag map, 1.2 m spacing', 'alt'),
            ('tags_0.6', 'Tag map, 0.6 m spacing', 'alt'),
            ('tags_0.6_err3', 'Tag map 0.6 m, ±3 mm error', 'alt'),
            ('scale_single', 'Single tag + scale estimate', 'alt3'),
            ('scale_tags_0.6', 'Tag map 0.6 m + scale (default)', 'proposed'),
            ('default', 'Default, re-run', 'proposed')]
    runs = load(os.path.join(EXP, 'along_wall_accuracy', 'runs_summary.csv'))
    for i, (c, lab, role) in enumerate(cfgs):
        v = [float(r['lateral_max_abs_mm']) for r in runs if r['config'] == c]
        ax.plot(v, [i] * len(v), **fs.marker(role, ms=3.5))
        ax.text(1.0, i, f'{np.mean(v):.1f}', ha='right', va='center', fontsize=7,
                transform=ax.get_yaxis_transform())
    ax.set_xscale('log')
    ax.set_xlim(0.5, 80)
    ax.set_yticks(range(len(cfgs)))
    ax.set_yticklabels([c[1] for c in cfgs], fontsize=7)
    ax.set_ylim(len(cfgs) - 0.5, -0.5)
    ax.set_xlabel(fs.label('Max. along-wall arrival error, 11 stations', 'mm'))
    fs.panel_label(ax, 'd', 'Encoder odometry (value: mean of 2 runs)')

    # (e) 切割力：各定点保持方式的横槽刀尖偏移（每次运行）
    ax = axs[2, 0]
    runs = load(os.path.join(EXP, 'cutting_force', 'station_runs.csv'))
    cfgs = [('dry_f5.5', 'No cutting\nforce', 'neutral'), ('p_f5.5', 'P hold\n(earlier)', 'baseline'),
            ('dob_f5.5', 'Disturbance\nobserver', 'alt'),
            ('dobff_f5.5', 'Observer +\n70 % feed-fwd.', 'proposed')]
    for i, (c, lab, role) in enumerate(cfgs):
        rr = [r for r in runs if r['config'] == c]
        pk = [float(r['tip_peak_horizontal_mm']) for r in rr]
        rm = [float(r['tip_rms_horizontal_mm']) for r in rr]
        dx = np.linspace(-0.12, 0.12, len(rr)) if len(rr) > 1 else [0.0]
        ax.plot(i - 0.17 + np.asarray(dx) * 0.6, pk, ls='none', marker=fs.MARKER[role],
                ms=3.6, color=fs.color(role), mfc=fs.color(role))
        ax.plot(i + 0.17 + np.asarray(dx) * 0.6, rm, ls='none', marker=fs.MARKER[role],
                ms=3.6, color=fs.color(role), mfc='white')
    ax.plot([], [], ls='none', marker='o', color=INK2, ms=3.6, label='peak')
    ax.plot([], [], ls='none', marker='o', color=INK2, mfc='white', ms=3.6, label='RMS')
    ax.set_yscale('log')
    ax.set_ylim(0.05, 200)
    ax.set_xticks(range(len(cfgs)))
    ax.set_xticklabels([c[1] for c in cfgs], fontsize=7)
    ax.set_xlim(-0.5, len(cfgs) - 0.5)
    ax.set_ylabel(fs.label('Tool-tip offset, horizontal cuts', 'mm'))
    ax.legend(loc='upper left')
    fs.panel_label(ax, 'e', 'Cutting force in Gazebo (brick, 5.5 mm/s)')

    # (f) 规划收益随时间模型（执行方式）的变化
    ax = axs[2, 1]
    rows = load(os.path.join(EXP, 'cutting_force', 'planning_gain.csv'))
    names = {'align_v3': 'Point to point\n(align_v3)', 'traj_v1': 'Continuous, 50 mm/s\n(traj_v1)',
             'cut_brick_v1': 'Continuous, 5.5 mm/s\n(cut_brick_v1)'}
    x = np.arange(len(rows))
    w = 0.36
    for k, (key, lab, role) in enumerate(
            [('joint_vs_rule_pct', 'Joint vs. rule-based + DP (non-load-bearing, n = 156)',
              'proposed'),
             ('rule_vs_per_slot_pct', 'Rule-based + DP vs. per-slot (n = 275)', 'neutral')]):
        vals = np.array([float(r[key]) for r in rows])
        xi = x + (k - 0.5) * w
        if key == 'joint_vs_rule_pct':
            ci = np.array([[float(t) for t in r['joint_vs_rule_pct_ci'].split('..')] for r in rows])
            err = np.vstack([vals - ci[:, 0], ci[:, 1] - vals])
            ax.bar(xi, vals, w - 0.03, yerr=err, error_kw=dict(lw=0.6, capsize=1.5),
                   label=lab, **fs.bar(role))
            tops = ci[:, 1]
        else:
            ax.bar(xi, vals, w - 0.03, label=lab, **fs.bar(role))
            tops = vals
        for xx, v, t in zip(xi, vals, tops):
            ax.text(xx, t + 0.3, f'{v:.1f}', ha='center', va='bottom', fontsize=7)
    ax.set_xticks(x)
    ax.set_xticklabels([names[r['time_model']] for r in rows], fontsize=7)
    ax.set_ylabel(fs.label('Mean predicted saving', '%'))
    ax.set_ylim(0, 14.5)
    ax.legend(loc='upper right')
    fs.panel_label(ax, 'f', '400 random walls (offline prediction)')
    fs.save(fig, out)


if __name__ == '__main__':
    fig_architecture(os.path.join(HERE, 'fig_architecture.png'))
    fig_summary(os.path.join(HERE, 'fig_summary.png'))
    print('已保存 fig_architecture.{png,pdf}、fig_summary.{png,pdf}')
