#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""技术报告的新增图：系统架构图、结果汇总图。其余图直接引用各实验报告中的原图.

  python3 docs/report/make_figures.py

只读取仓库中已提交的 CSV（docs/experiments/**），不需要 ROS / Gazebo。
输出：docs/report/fig_architecture.png、docs/report/fig_summary.png
"""
import csv
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.patches as mpatches  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.join(HERE, '..', 'experiments')
INK, INK2, GRID, SURFACE = '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'
C_V1, C_V2, C_V3 = '#2a78d6', '#eb6834', '#1baf7a'
matplotlib.rcParams.update({
    'font.sans-serif': ['Noto Sans CJK SC', 'Noto Sans CJK JP', 'DejaVu Sans'],
    'axes.unicode_minus': False, 'axes.edgecolor': INK2, 'axes.labelcolor': INK,
    'xtick.color': INK2, 'ytick.color': INK2, 'axes.grid': True, 'grid.color': GRID,
    'grid.linewidth': 0.8, 'axes.spines.top': False, 'axes.spines.right': False,
    'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
    'font.size': 10})


def load(path):
    with open(path, encoding='utf-8') as f:
        return list(csv.DictReader(f))


# ---------------------------------------------------------------------------
# 系统架构图
# ---------------------------------------------------------------------------
def box(ax, x, y, w, h, title, lines=(), fc='#ffffff', ec=INK2, title_size=10):
    ax.add_patch(mpatches.FancyBboxPatch((x, y), w, h,
                                         boxstyle='round,pad=0.008,rounding_size=0.012',
                                         fc=fc, ec=ec, lw=1.0))
    ax.text(x + w / 2, y + h - 0.018, title, ha='center', va='top', fontsize=title_size,
            color=INK, fontweight='bold')
    for i, s in enumerate(lines):
        ax.text(x + w / 2, y + h - 0.052 - i * 0.03, s, ha='center', va='top', fontsize=8.2,
                color=INK2)


def group(ax, x, y, w, h, label, fc):
    ax.add_patch(mpatches.FancyBboxPatch((x, y), w, h,
                                         boxstyle='round,pad=0.006,rounding_size=0.015',
                                         fc=fc, ec='none'))
    ax.text(x + 0.008, y + h - 0.01, label, ha='left', va='top', fontsize=9, color=INK2)


def arrow(ax, p, q, text='', dashed=False, rad=0.0, color=INK2, tx=None):
    ax.annotate('', xy=q, xytext=p, arrowprops=dict(
        arrowstyle='-|>', color=color, lw=1.1, ls='--' if dashed else '-',
        shrinkA=2, shrinkB=2, connectionstyle=f'arc3,rad={rad}'))
    if text:
        mx, my = tx if tx else ((p[0] + q[0]) / 2, (p[1] + q[1]) / 2)
        ax.text(mx, my, text, fontsize=7.6, color=INK2, ha='center', va='center',
                bbox=dict(fc=SURFACE, ec='none', pad=0.6))


def fig_architecture(out):
    fig = plt.figure(figsize=(12.5, 7.6))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis('off')

    # 离线规划
    group(ax, 0.015, 0.70, 0.30, 0.285, '离线规划（不需要 ROS）  scr_planner', '#eef4fc')
    box(ax, 0.03, 0.865, 0.27, 0.085, '墙面任务 YAML', ['墙体类型 / 障碍 / 已有点位 / 起止线盒'])
    box(ax, 0.03, 0.715, 0.27, 0.13, 'plan_wall_slots',
        ['C++ OrientedAStar：规范约束单槽最优', 'task_plan：站感知选线 + 下界证书',
         'slot_plan：DP 划分 + 扫掠顺序 + 时间模型'])
    arrow(ax, (0.165, 0.865), (0.165, 0.845))

    box(ax, 0.03, 0.555, 0.27, 0.11, '规划结果 YAML',
        ['槽线（直角折线）/ 停靠站位姿', '每站机械臂切割点 / 预测时间'], fc='#fff8ec')
    arrow(ax, (0.165, 0.715), (0.165, 0.665))

    # 在线执行
    group(ax, 0.35, 0.40, 0.635, 0.585, '在线执行  navigation_cutting_task（scr_arm_controller）',
          '#f3f2ee')
    box(ax, 0.37, 0.825, 0.17, 0.10, '1 导航', ['Nav2 送到标记前 1.2 m'])
    box(ax, 0.565, 0.69, 0.40, 0.25, '2 精对位（到标记前 0.70 m）', [])
    box(ax, 0.58, 0.815, 0.115, 0.09, 'AprilTag 检测', [], title_size=8.5)
    ax.text(0.6375, 0.835, 'IPPE PnP + 协方差', ha='center', fontsize=7.4, color=INK2)
    box(ax, 0.58, 0.705, 0.115, 0.09, '激光墙面拟合', [], title_size=8.5)
    ax.text(0.6375, 0.725, 'TLS + 离群剔除', ha='center', fontsize=7.4, color=INK2)
    box(ax, 0.72, 0.705, 0.10, 0.20, 'EKF', ['状态 (d, u, ψ)', '里程计预测', '延迟补偿', 'χ² 门限'],
        title_size=9)
    box(ax, 0.84, 0.705, 0.11, 0.20, '饱和控制律', ['平方根整形', 'k = 2 /s', '收敛判据', '示教零位'],
        title_size=9)
    arrow(ax, (0.695, 0.86), (0.72, 0.84))
    arrow(ax, (0.695, 0.75), (0.72, 0.77))
    arrow(ax, (0.82, 0.805), (0.84, 0.805))
    arrow(ax, (0.54, 0.885), (0.565, 0.885))

    box(ax, 0.37, 0.43, 0.595, 0.235, '3 逐站执行（墙面原点 = EKF 估计的标记位置）', [])
    xs = [0.385, 0.53, 0.675, 0.82]
    titles = [('横移到站', ['同一 EKF', '防碰撞守护']), ('定点保持', ['base_hold', 'P 5 / 6，50 Hz']),
              ('残差补偿', ['按估计的横向 /', '离墙 / 朝向误差', '修正机械臂点']),
              ('切割', ['每刀单轴', '仿真中空切'])]
    for i, (x, (t, ls)) in enumerate(zip(xs, titles)):
        box(ax, x, 0.455, 0.13, 0.155, t, ls, title_size=9)
        if i:
            arrow(ax, (xs[i - 1] + 0.13, 0.53), (x, 0.53))
    arrow(ax, (0.765, 0.69), (0.765, 0.665))
    arrow(ax, (0.30, 0.61), (0.37, 0.61), '读取规划文件')

    # 导航与仿真
    group(ax, 0.015, 0.03, 0.30, 0.50, '导航（scr_navigation）', '#eef7f2')
    box(ax, 0.03, 0.36, 0.27, 0.12, 'Nav2', ['AMCL + NavFn + DWB', 'SLAM Toolbox 建图'])
    box(ax, 0.03, 0.20, 0.27, 0.12, 'mecanum_controller', ['全向底盘运动学', '/cmd_vel → 轮速'])
    # 执行 → Nav2 / 底盘：沿两组之间的空隙走折线，避免穿过方框
    ax.plot([0.37, 0.325, 0.325], [0.86, 0.86, 0.43], color=INK2, lw=1.1)
    arrow(ax, (0.325, 0.43), (0.30, 0.43))
    ax.text(0.317, 0.73, '/navigate_to_pose', rotation=90, fontsize=7.6, color=INK2,
            ha='center', va='center', bbox=dict(fc=SURFACE, ec='none', pad=0.6))
    ax.plot([0.37, 0.34, 0.34], [0.47, 0.47, 0.27], color=INK2, lw=1.1)
    arrow(ax, (0.34, 0.27), (0.30, 0.27))
    ax.text(0.348, 0.37, '/cmd_vel', rotation=90, fontsize=7.6, color=INK2,
            ha='center', va='center', bbox=dict(fc=SURFACE, ec='none', pad=0.6))
    arrow(ax, (0.165, 0.36), (0.165, 0.32), '/cmd_vel', tx=(0.21, 0.34))

    group(ax, 0.35, 0.03, 0.635, 0.33, 'Gazebo Classic 仿真（scr_description，仿真配置 v2）', '#f6f0fb')
    box(ax, 0.37, 0.155, 0.17, 0.12, 'scr_planar_move', ['力控底盘插件', '速度 P 环'])
    box(ax, 0.56, 0.155, 0.17, 0.12, '直角坐标机械臂', ['Z / Y / X / 刀片', 'position_pid 力控'])
    box(ax, 0.75, 0.155, 0.20, 0.12, '传感器', ['2D 激光 10 Hz / 相机 30 Hz', '里程计 50 Hz'])
    box(ax, 0.37, 0.045, 0.58, 0.075, '真值（/gazebo/model_states, link_states）',
        ['只用于评估：execute_slot_plan 记录器、alignment_benchmark'], fc='#ffffff')
    arrow(ax, (0.30, 0.215), (0.37, 0.215), '/cmd_vel_direct', tx=(0.335, 0.195))
    arrow(ax, (0.645, 0.455), (0.645, 0.275), 'follow_joint_trajectory', tx=(0.645, 0.40))
    arrow(ax, (0.85, 0.275), (0.85, 0.455), '/scan, 图像, /odom', dashed=True, tx=(0.885, 0.40))

    fig.savefig(out, dpi=170)
    plt.close(fig)


# ---------------------------------------------------------------------------
# 结果汇总图
# ---------------------------------------------------------------------------
def fig_summary(out):
    fig, axs = plt.subplots(2, 2, figsize=(12, 8.4))

    # (a) 精对位：第一版 vs 第二版，各条件位置误差
    rows = load(os.path.join(EXP, 'alignment_robustness', 'summary.csv'))
    conds = [('nominal', '名义'), ('odom_drift_10cm', '里程计\n漂移 10 cm'),
             ('light_cool', '冷白光'), ('occlusion_intermittent', '间歇遮挡'),
             ('occlusion_static', '全程遮挡'), ('camera_yaw_1deg', '相机偏角\n1°')]
    S = {(r['condition'], r['pipeline']): r for r in rows}
    ax = axs[0, 0]
    x = np.arange(len(conds))
    for j, (pipe, lab, c) in enumerate((('v1', '第一版', C_V1), ('v2', '第二版', C_V2),
                                        ('v2_taught', '第二版 + 示教零位', C_V3))):
        for i, (cond, _) in enumerate(conds):
            r = S.get((cond, pipe))
            if r is None:
                continue
            xi = x[i] + (j - 1) * 0.27
            if int(r['success']) == 0:
                ax.bar(xi, 13, 0.25, color='none', ec=c, hatch='///', lw=0.8)
                ax.text(xi, 13.3, '0/8', ha='center', fontsize=7, color=c)
                continue
            m, s = float(r['pos_mean_mm']), float(r['pos_std_mm'])
            first = i == 0 or (pipe == 'v2_taught' and cond == 'nominal')
            ax.bar(xi, m, 0.25, color=c, label=lab if first else None)
            ax.errorbar(xi, m, s, color=INK2, lw=0.8, capsize=2)
    ax.set_xticks(x)
    ax.set_xticklabels([c[1] for c in conds], fontsize=8.5)
    ax.set_ylabel('到位时刻位置误差 (mm)')
    ax.set_ylim(0, 15)
    ax.set_title('(a) 精对位：Gazebo 配对实验，n = 8 / 条件（斜线：8 次全部失败）', loc='left',
                 fontsize=10)
    h, lbl = ax.get_legend_handles_labels()
    ax.legend(h, lbl, frameon=False, fontsize=8.5, loc='upper left')
    ax.grid(axis='x', visible=False)

    # (b) 多站执行：每次运行的到站横向误差（均值 ± 站间标准差）
    ax = axs[0, 1]
    runs = [('dp', '第一版 1', C_V1), ('dp_repeat', '第一版 2', C_V1),
            ('per_slot', '第一版 基线 1', C_V1), ('per_slot_repeat', '第一版 基线 2', C_V1),
            ('dp_nav2_each_station', '第一版 逐站 Nav2', C_V1),
            ('fine_align_dp', '第二版 1', C_V2), ('fine_align_dp_repeat', '第二版 2', C_V2),
            ('fine_align_dp_nofold', '第二版 关闭原点修正', '#b9b8b2')]
    for i, (d, lab, c) in enumerate(runs):
        st = load(os.path.join(EXP, 'slot_execution', 'runs', d, 'stations.csv'))
        lat = np.array([float(r['lateral_mm']) for r in st])
        ax.errorbar(i, lat.mean(), lat.std(ddof=1), fmt='o', color=c, capsize=3, ms=6)
        ax.text(i, lat.mean() + lat.std(ddof=1) + 0.12, f'{lat.mean():+.2f}', ha='center',
                fontsize=7.5, color=INK2)
    ax.axhline(0, color=INK2, lw=0.8)
    ax.set_xticks(range(len(runs)))
    ax.set_xticklabels([r[1] for r in runs], rotation=30, ha='right', fontsize=8)
    ax.set_ylabel('到站横向误差（真值）(mm)')
    ax.set_title('(b) 多站执行：每次运行的到站横向误差，均值 ± 站间标准差', loc='left', fontsize=10)
    ax.grid(axis='x', visible=False)

    # (c) 时间模型：每站预测 vs 实测（精对位第二版的 33 个停靠站）
    ax = axs[1, 0]
    models = {'align_v1': (4.40, 0.0945, 2.034), 'align_v2': (2.83, 0.0984, 2.049)}
    st = []
    for d in ('fine_align_dp', 'fine_align_dp_repeat', 'fine_align_dp_nofold'):
        st += load(os.path.join(EXP, 'slot_execution', 'runs', d, 'stations.csv'))
    y = np.array([float(r['station_s']) for r in st])
    dist = np.array([float(r['move_dist_m']) for r in st])
    n = np.array([float(r['points']) for r in st])
    lo, hi = 8, 42
    ax.plot([lo, hi], [lo, hi], color=INK2, lw=0.8, ls=':')
    for (name, (ts, v, tp)), c, lab in zip(models.items(), (C_V1, C_V2),
                                           ('align_v1（第一版标定）', 'align_v2（重标定）')):
        p = ts + dist / v + n * tp
        rmse = np.sqrt(np.mean((p - y) ** 2))
        ax.scatter(y, p, s=22, color=c, alpha=0.8, label=f'{lab}：RMSE {rmse:.2f} s', lw=0)
    ax.set_xlim(lo, hi)
    ax.set_ylim(lo, hi)
    ax.set_xlabel('Gazebo 实测每站耗时 (s)')
    ax.set_ylabel('模型预测 (s)')
    ax.set_title('(c) 时间模型：精对位第二版下 33 个停靠站（3 次运行）', loc='left', fontsize=10)
    ax.legend(frameon=False, fontsize=8.5, loc='upper left')
    ax.set_aspect('equal')

    # (d) 任务级规划：配对墙上的平均预测时间（两种时间模型）
    ax = axs[1, 1]
    meth = [('per_slot', '逐槽划分\n（基线）'), ('current', '先选线后 DP\n（现有）'),
            ('joint', '联合规划'), ('lower_bound', '下界')]
    sets = [('align_v1', os.path.join(EXP, 'task_planning', 'summary.csv'), C_V1),
            ('align_v2', os.path.join(EXP, 'task_planning', 'align_v2', 'summary.csv'), C_V2)]
    x = np.arange(len(meth))
    for j, (name, path, c) in enumerate(sets):
        if not os.path.exists(path):
            continue
        S = {r['method']: r for r in load(path)}
        vals = [float(S[m]['t_mean']) for m, _ in meth]
        ax.bar(x + (j - 0.5) * 0.36, vals, 0.34, color=c,
               label=f'{name}（n = {S["joint"]["n"]}）')
        for xi, v in zip(x + (j - 0.5) * 0.36, vals):
            ax.text(xi, v + 2, f'{v:.0f}', ha='center', fontsize=7.5, color=INK2)
    ax.set_xticks(x)
    ax.set_xticklabels([m[1] for m in meth], fontsize=8.5)
    ax.set_ylabel('配对墙上的平均预测执行时间 (s)')
    ax.set_title('(d) 400 面随机墙（离线预测，非 Gazebo 实测）', loc='left', fontsize=10)
    ax.legend(frameon=False, fontsize=8.5, loc='upper right')
    ax.grid(axis='x', visible=False)

    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    fig_architecture(os.path.join(HERE, 'fig_architecture.png'))
    fig_summary(os.path.join(HERE, 'fig_summary.png'))
    print('已保存 fig_architecture.png、fig_summary.png')
