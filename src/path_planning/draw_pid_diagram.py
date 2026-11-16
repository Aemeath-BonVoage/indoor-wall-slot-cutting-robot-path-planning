#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成PID闭环控制系统框图"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False

fig, ax = plt.subplots(1, 1, figsize=(16, 7))
ax.set_xlim(0, 16)
ax.set_ylim(0, 7)
ax.axis('off')


def draw_block(ax, x, y, w, h, title, subtitle='', color='#E8F4FD', border='#2980B9'):
    rect = patches.FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.1',
                                  facecolor=color, edgecolor=border, linewidth=2)
    ax.add_patch(rect)
    if subtitle:
        ax.text(x + w / 2, y + h / 2 + 0.15, title, ha='center', va='center',
                fontsize=11, fontweight='bold')
        ax.text(x + w / 2, y + h / 2 - 0.25, subtitle, ha='center', va='center',
                fontsize=8, color='#555')
    else:
        ax.text(x + w / 2, y + h / 2, title, ha='center', va='center',
                fontsize=11, fontweight='bold')


def draw_arrow(ax, x1, y1, x2, y2, label=''):
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle='->', color='#333', lw=2))
    if label:
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2 + 0.2
        ax.text(mx, my, label, ha='center', va='bottom', fontsize=9, color='#555')


def draw_sum(ax, x, y, r=0.25):
    circle = plt.Circle((x, y), r, fill=True, facecolor='white',
                         edgecolor='#333', linewidth=2)
    ax.add_patch(circle)
    ax.text(x, y, '+', ha='center', va='center', fontsize=14, fontweight='bold')
    ax.text(x - 0.05, y - r - 0.15, '-', ha='center', va='center',
            fontsize=12, fontweight='bold', color='#E74C3C')


# Title
ax.text(8, 6.6, '机器人精确定位闭环控制系统框图',
        ha='center', va='center', fontsize=16, fontweight='bold')

y_main = 3.5

# Input label
ax.text(0.5, y_main + 0.15, '目标位姿', ha='center', va='center',
        fontsize=11, fontweight='bold', color='#2980B9')
ax.text(0.5, y_main - 0.2, '($x_t, y_t, \\theta_t$)', ha='center', va='center',
        fontsize=10, color='#2980B9')

# Arrow to sum
draw_arrow(ax, 1.1, y_main, 1.7, y_main)

# Sum circle
draw_sum(ax, 2.0, y_main)

# Arrow to coordinate transform
draw_arrow(ax, 2.25, y_main, 3.2, y_main, '误差信号')

# Coordinate Transform
draw_block(ax, 3.2, y_main - 0.5, 2.0, 1.0, '坐标变换', '全局 -> 机器人本体')

# Arrow to P controller
draw_arrow(ax, 5.2, y_main, 6.0, y_main)

# P Controller
draw_block(ax, 6.0, y_main - 0.5, 2.2, 1.0, 'P 控制器',
           'Kp=0.6  Kp_a=1.2', color='#FFF3E0', border='#E67E22')

# Arrow to velocity limit
draw_arrow(ax, 8.2, y_main, 9.0, y_main)

# Velocity Saturation
draw_block(ax, 9.0, y_main - 0.5, 1.8, 1.0, '速度限幅', '|v|<=0.2 m/s')

# Arrow to chassis
draw_arrow(ax, 10.8, y_main, 11.5, y_main)

# Mecanum Chassis
draw_block(ax, 11.5, y_main - 0.5, 2.2, 1.0, '麦克纳姆轮底盘',
           '全向移动平台', color='#E8F8F5', border='#27AE60')

# Arrow to output
draw_arrow(ax, 13.7, y_main, 14.5, y_main)

# Output label
ax.text(15.3, y_main + 0.15, '实际位姿', ha='center', va='center',
        fontsize=11, fontweight='bold', color='#27AE60')
ax.text(15.3, y_main - 0.2, '($x, y, \\theta$)', ha='center', va='center',
        fontsize=10, color='#27AE60')

# ===== Feedback path =====
# Down from output
ax.annotate('', xy=(14.8, 1.5), xytext=(14.8, y_main - 0.5),
            arrowprops=dict(arrowstyle='-', color='#333', lw=2))

# Feedback block
draw_block(ax, 5.5, 1.0, 3.5, 1.0, '位姿反馈',
           'Gazebo真值(优先) / 里程计(备选)', color='#FDEDEC', border='#E74C3C')

# Right horizontal to feedback
ax.annotate('', xy=(9.0, 1.5), xytext=(14.8, 1.5),
            arrowprops=dict(arrowstyle='-', color='#333', lw=2))

# Left from feedback to sum
ax.annotate('', xy=(2.0, 2.0), xytext=(5.5, 1.5),
            arrowprops=dict(arrowstyle='-', color='#333', lw=2))
ax.annotate('', xy=(2.0, y_main - 0.25), xytext=(2.0, 2.0),
            arrowprops=dict(arrowstyle='->', color='#333', lw=2))

# ===== Convergence check =====
draw_block(ax, 9.0, 5.2, 3.8, 1.1, '收敛判定',
           '位置<3cm, 角度<4.6, 连续5次', color='#F5EEF8', border='#8E44AD')

# Arrow from output to convergence
ax.annotate('', xy=(12.8, 5.2), xytext=(14.8, y_main + 0.5),
            arrowprops=dict(arrowstyle='->', color='#8E44AD', lw=1.5,
                            linestyle='dashed'))

# Convergence output
ax.text(8.5, 5.75, '定位完成', ha='right', va='center',
        fontsize=10, color='#8E44AD', fontweight='bold')
ax.annotate('', xy=(8.5, 5.75), xytext=(9.0, 5.75),
            arrowprops=dict(arrowstyle='->', color='#8E44AD', lw=1.5,
                            linestyle='dashed'))

plt.tight_layout()
save_path = r'D:\毕设\colcon_ws\colcon_ws\src\path_planning\pid_control_diagram.png'
fig.savefig(save_path, dpi=200, bbox_inches='tight', facecolor='white')
print(f'已保存: {save_path}')
plt.close()
