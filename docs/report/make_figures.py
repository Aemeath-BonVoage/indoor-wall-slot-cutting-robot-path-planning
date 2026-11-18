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
    ax.text(x + w / 2, y + h - 0.016, title, ha='center', va='top', fontsize=title_size,
            color=INK, fontweight='bold')
    for i, s in enumerate(lines):
        ax.text(x + w / 2, y + h - 0.046 - i * 0.026, s, ha='center', va='top', fontsize=8.2,
                color=INK2)


def group(ax, x, y, w, h, label, fc, lx=None):
    ax.add_patch(mpatches.FancyBboxPatch((x, y), w, h,
                                         boxstyle='round,pad=0.006,rounding_size=0.015',
                                         fc=fc, ec='none'))
    ax.text(x + 0.008 if lx is None else lx, y + h - 0.01, label, ha='left', va='top', fontsize=9,
            color=INK2)


def arrow(ax, p, q, text='', dashed=False, rad=0.0, color=INK2, tx=None):
    ax.annotate('', xy=q, xytext=p, arrowprops=dict(
        arrowstyle='-|>', color=color, lw=1.1, ls='--' if dashed else '-',
        shrinkA=2, shrinkB=2, connectionstyle=f'arc3,rad={rad}'))
    if text:
        mx, my = tx if tx else ((p[0] + q[0]) / 2, (p[1] + q[1]) / 2)
        ax.text(mx, my, text, fontsize=7.6, color=INK2, ha='center', va='center',
                bbox=dict(fc=SURFACE, ec='none', pad=0.6))


def vline(ax, pts, color=INK2, dashed=False):
    """折线（不带箭头），pts = [(x, y), ...]."""
    xs, ys = zip(*pts)
    ax.plot(xs, ys, color=color, lw=1.1, ls='--' if dashed else '-')


def vlabel(ax, x, y, text):
    ax.text(x, y, text, rotation=90, fontsize=7.6, color=INK2, ha='center', va='center',
            bbox=dict(fc=SURFACE, ec='none', pad=0.6))


def fig_architecture(out):
    # 方框带 0.008 的圆角外扩，相邻方框之间至少留 0.03
    fig = plt.figure(figsize=(13, 9.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis('off')

    # ---- 左列：离线规划、命令行 / 记录器、真值 ----
    group(ax, 0.015, 0.62, 0.24, 0.365, '离线规划（不需要 ROS）  scr_planner', '#eef4fc')
    box(ax, 0.03, 0.872, 0.21, 0.07, '墙面任务 YAML', ['墙体类型 / 障碍 / 起止线盒'])
    box(ax, 0.03, 0.741, 0.21, 0.096, 'plan_wall_slots',
        ['OrientedAStar：规范约束单槽最优', '联合规划 / DP 划分 / 时间模型'])
    box(ax, 0.03, 0.636, 0.21, 0.07, '规划结果 YAML', ['槽线 / 停靠站 / 机械臂点 / 预测时间'],
        fc='#fff8ec')
    arrow(ax, (0.135, 0.864), (0.135, 0.845))
    arrow(ax, (0.135, 0.733), (0.135, 0.714))

    group(ax, 0.015, 0.315, 0.24, 0.285, '命令行 / 记录器', '#f3f2ee')
    box(ax, 0.03, 0.46, 0.21, 0.096, 'navigation_cutting_task',
        ['任务文件 → 任务 action', '打印反馈与每站记录'])
    box(ax, 0.03, 0.33, 0.21, 0.096, 'execute_slot_plan.py',
        ['同一 action 的客户端', '+ 真值：刀尖轨迹 / 到站误差'])
    arrow(ax, (0.215, 0.628), (0.215, 0.564))

    group(ax, 0.015, 0.015, 0.24, 0.28, 'Gazebo 真值', '#f5f5f3')
    box(ax, 0.03, 0.03, 0.21, 0.215, '只用于评估',
        ['/ground_truth/odom', '/gazebo/model_states', '/gazebo/link_states',
         '不进入任何控制或估计'], fc='#ffffff')
    arrow(ax, (0.135, 0.253), (0.135, 0.322), dashed=True)

    # 客户端 → 行为树执行器
    vline(ax, [(0.248, 0.508), (0.272, 0.508), (0.272, 0.88)])
    vline(ax, [(0.248, 0.378), (0.272, 0.378), (0.272, 0.508)])
    arrow(ax, (0.272, 0.88), (0.297, 0.88))
    vlabel(ax, 0.272, 0.69, '/execute_slot_plan · /execute_cutting_task')

    # ---- 中列：任务层 ----
    group(ax, 0.29, 0.45, 0.365, 0.535, '任务层：行为树编排 + 技能 action', '#fdf3ec')
    box(ax, 0.305, 0.82, 0.335, 0.122, 'task_executor（scr_task，BehaviorTree.CPP v3）',
        ['BtActionServer × 2（与 Nav2 bt_navigator 同构）', 'slot_plan.xml / cutting_task.xml',
         '恢复：导航重试、回标记前重新对位、切割重试'], title_size=9.2)
    arrow(ax, (0.4725, 0.812), (0.4725, 0.783), '技能 action', tx=(0.53, 0.797))
    box(ax, 0.305, 0.465, 0.335, 0.31, 'task_skill_server（scr_arm_controller）',
        ['FineAlign · MoveToStation · CutStation', 'HoldBase · ScanMarker · RetractArm'],
        title_size=9.2)
    subs = [('精对位', ['AprilTag + 激光', 'EKF (d, u, ψ, s)', '饱和控制律', '收敛判据']),
            ('横移 / 保持', ['同一墙面 EKF', '多标记地图更新', '横移比例修正', '定点保持 50 Hz']),
            ('切割', ['残差补偿', '每刀单轴', '仿真中空切'])]
    for i, (t, ls) in enumerate(subs):
        box(ax, 0.326 + i * 0.107, 0.48, 0.079, 0.148, t, ls, title_size=8.8)

    # ---- 右列：Nav2、定位 ----
    group(ax, 0.70, 0.62, 0.285, 0.365, 'Nav2（scr_navigation）', '#eef7f2')
    box(ax, 0.715, 0.872, 0.11, 0.07, 'bt_navigator', ['NavigateToPose'], title_size=9)
    box(ax, 0.86, 0.872, 0.11, 0.07, 'amcl', ['map → odom'], title_size=9)
    box(ax, 0.715, 0.741, 0.11, 0.096, 'controller', ['MPPI（Omni）', '1000 采样，20 Hz'],
        title_size=9)
    box(ax, 0.86, 0.741, 0.11, 0.096, 'planner', ['Smac 2D'], title_size=9)
    ax.text(0.915, 0.67, 'DWB 基线：\nnav2_params_dwb.yaml', ha='center', va='center',
            fontsize=7.6, color=INK2)
    arrow(ax, (0.648, 0.905), (0.707, 0.905), 'NavigateToPose', tx=(0.677, 0.928))

    group(ax, 0.70, 0.315, 0.285, 0.285, '定位（robot_localization）', '#eef7f2', lx=0.795)
    box(ax, 0.715, 0.36, 0.255, 0.16, 'EKF（平面模式）',
        ['/wheel/odom 的 (vx, vy) + /imu 的 wz', '→ /odom 与 odom → base_link',
         '只服务导航与 AMCL；技能的局部', '闭环直接用 /wheel/odom'], title_size=9)
    arrow(ax, (0.77, 0.528), (0.77, 0.733), '/odom，TF', tx=(0.77, 0.635))

    # Nav2 控制器 → 底盘控制器（沿中列与右列之间的空隙）
    vline(ax, [(0.707, 0.79), (0.677, 0.79), (0.677, 0.22)])
    arrow(ax, (0.677, 0.22), (0.653, 0.22))
    vlabel(ax, 0.677, 0.40, '/cmd_vel（导航）')

    # ---- 底部：ros2_control + Gazebo ----
    group(ax, 0.29, 0.015, 0.695, 0.28, 'ros2_control + Gazebo Classic', '#f6f0fb', lx=0.708)
    box(ax, 0.305, 0.03, 0.085, 0.225, '传感器', ['2D 激光 10 Hz', '相机 30 Hz'], title_size=9)
    box(ax, 0.42, 0.185, 0.10, 0.07, 'arm_controller', ['关节轨迹控制器'], title_size=9)
    box(ax, 0.42, 0.03, 0.10, 0.11, 'GazeboSystem', ['机械臂关节', 'position_pid 力控'],
        title_size=9)
    box(ax, 0.545, 0.185, 0.10, 0.07, 'base_controller', ['cmd_vel ↔ 轮速 / 里程计'],
        title_size=9)
    box(ax, 0.545, 0.025, 0.10, 0.122, 'ScrMecanumBase', ['力驱动被控对象', '编码器误差模型',
                                                         '模拟陀螺 /imu'], title_size=8.6)
    box(ax, 0.715, 0.185, 0.255, 0.07, '早期配置  base_control:=plugin',
        ['scr_planar_move 力控插件，/odom = 真值'], fc='#faf8fc', title_size=8.8)
    box(ax, 0.715, 0.025, 0.255, 0.122, '其他选项',
        ['plant:=wheels  轮子驱动（实验）', 'localization:=wheel  只用轮式里程计',
         'odom_error:=false  理想编码器'], fc='#faf8fc', title_size=8.8)

    # 技能服务器 ↔ 传感器 / 机械臂 / 底盘
    arrow(ax, (0.3475, 0.263), (0.3475, 0.472), '/scan, 图像', dashed=True, tx=(0.3475, 0.40))
    arrow(ax, (0.47, 0.457), (0.47, 0.263), 'FollowJoint-\nTrajectory', tx=(0.47, 0.33))
    arrow(ax, (0.575, 0.457), (0.575, 0.263), '/cmd_vel', tx=(0.575, 0.41))
    arrow(ax, (0.625, 0.263), (0.625, 0.457), '/wheel/odom', dashed=True, tx=(0.625, 0.33))
    # 底盘 → EKF
    arrow(ax, (0.93, 0.295), (0.93, 0.352), dashed=True)
    ax.text(0.92, 0.322, '/wheel/odom, /imu', ha='right', va='center', fontsize=7.6, color=INK2)

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

    # (c) 时间模型：当前默认执行路径下的样本外停靠站（plan_validation 的 5 次验证运行）
    ax = axs[1, 0]
    models = {'align_v2': (2.83, 0.0984, 2.049), 'align_v3': (4.70, 0.0971, 2.030)}
    st = [r for r in load(os.path.join(EXP, 'plan_validation', 'stations_all.csv'))
          if r['run'] != 'demo_joint']        # 演示运行另加了墙面实体与相机，单独列出
    y = np.array([float(r['station_s']) for r in st])
    dist = np.array([float(r['move_dist_m']) for r in st])
    n = np.array([float(r['points']) for r in st])
    lo, hi = 8, 42
    ax.plot([lo, hi], [lo, hi], color=INK2, lw=0.8, ls=':')
    for (name, (ts, v, tp)), c, lab in zip(models.items(), ('#9a9992', C_V2),
                                           ('align_v2（此前路径的标定）',
                                            'align_v3（当前路径重标定）')):
        p = ts + dist / v + n * tp
        rmse = np.sqrt(np.mean((p - y) ** 2))
        ax.scatter(y, p, s=22, color=c, alpha=0.8, label=f'{lab}：RMSE {rmse:.2f} s', lw=0)
    ax.set_xlim(lo, hi)
    ax.set_ylim(lo, hi)
    ax.set_xlabel('Gazebo 实测每站耗时 (s)')
    ax.set_ylabel('模型预测 (s)')
    ax.set_title(f'(c) 时间模型：当前默认路径下 {len(y)} 个样本外停靠站（5 次运行）', loc='left',
                 fontsize=10)
    ax.legend(frameon=False, fontsize=8.5, loc='upper left')
    ax.set_aspect('equal')

    # (d) 任务级规划：配对墙上的平均预测时间（三种时间模型）
    ax = axs[1, 1]
    meth = [('per_slot', '逐槽划分\n（基线）'), ('current', '先选线后 DP\n（现有）'),
            ('joint', '联合规划'), ('lower_bound', '下界')]
    sets = [('align_v1', os.path.join(EXP, 'task_planning', 'summary.csv'), C_V1),
            ('align_v2', os.path.join(EXP, 'task_planning', 'align_v2', 'summary.csv'), '#b9b8b2'),
            ('align_v3', os.path.join(EXP, 'task_planning', 'align_v3', 'summary.csv'), C_V2)]
    x = np.arange(len(meth))
    w = 0.27
    for j, (name, path, c) in enumerate(sets):
        if not os.path.exists(path):
            continue
        S = {r['method']: r for r in load(path)}
        vals = [float(S[m]['t_mean']) for m, _ in meth]
        ax.bar(x + (j - 1) * w, vals, w - 0.02, color=c,
               label=f'{name}（n = {S["joint"]["n"]}）')
        for xi, v in zip(x + (j - 1) * w, vals):
            ax.text(xi, v + 2, f'{v:.0f}', ha='center', fontsize=7, color=INK2)
    ax.set_xticks(x)
    ax.set_xticklabels([m[1] for m in meth], fontsize=8.5)
    ax.set_ylabel('配对墙上的平均预测执行时间 (s)')
    ax.set_ylim(0, 215)
    ax.set_title('(d) 400 面随机墙（离线预测，非 Gazebo 实测）', loc='left', fontsize=10)
    ax.legend(frameon=False, fontsize=8.5, loc='upper right', ncol=3)
    ax.grid(axis='x', visible=False)

    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    fig_architecture(os.path.join(HERE, 'fig_architecture.png'))
    fig_summary(os.path.join(HERE, 'fig_summary.png'))
    print('已保存 fig_architecture.png、fig_summary.png')
