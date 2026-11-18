#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
面向室内装修的机械臂墙面自主开槽路径规划算法
——基于视觉感知的二维墙面映射与增强A*约束规划

作者: SCR Project
功能: 
    1. 模拟相机视觉感知，生成带有插座、横梁等障碍物的二维墙面栅格图
    2. 调用已实现的改进A*算法，规划避障且横平竖直的切槽轨迹
    3. 输出专业仿真图以供论文使用
注意: 本脚本完全独立运行，不影响原有ROS导航配置
=============================================================================
"""

import os
import sys
import numpy as np
import matplotlib.patches as patches


# 导入已经写好的极品A*算法模块
try:
    from improved_astar import EnhancedAStar, ImprovedAStar, StandardAStar, PathAnalyzer
except ImportError:
    print("错误: 找不到 improved_astar.py，请确保两个脚本在同一目录下。")
    sys.exit(1)

# 作图样式见 docs/figstyle（PDF + PNG）
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'docs'))
import figstyle as fs  # noqa: E402

# 障碍物的图注
LABEL_EN = {'强电配电箱': 'Power panel', '电源插座1': 'Outlet 1', '电源插座2': 'Outlet 2',
            '空调插座': 'A/C outlet', '承重钢筋横梁': 'Structural beam', '窗户区域': 'Window'}
ALGO_EN = ['Standard A*', 'Improved A*\n(turn penalty)', 'Enhanced A*\n(axis-aligned)']
ALGO_ROLE = ['neutral', 'alt2', 'proposed']


# ============================================================================
# 第一部分：墙面视觉感知与建模映射
# ============================================================================

class WallSurfaceMap:
    """墙面二维环境建模类"""
    
    def __init__(self, width_cm=300, height_cm=260, resolution_cm=1):
        """
        初始化一堵墙的二维映射图
        参数:
            width_cm: 墙面宽度 (厘米)，默认3米
            height_cm: 墙面高度 (厘米)，默认2.6米
            resolution_cm: 栅格分辨率 (1厘米/像素)，精度极高
        """
        self.width = int(width_cm / resolution_cm)
        self.height = int(height_cm / resolution_cm)
        self.resolution = resolution_cm / 100.0  # 转为米，保持兼容
        
        # 网格：0代表干净可开槽的墙面墙体，1代表绝对不能碰的障碍物
        self.grid = np.zeros((self.height, self.width), dtype=np.int8)
        self.inflated_grid = None
        self.obstacles_info = [] # 记录障碍物信息用于可视化绘图
        
    def add_obstacle_box(self, x_cm, y_cm, w_cm, h_cm, label):
        """在墙上添加矩形障碍物 (插座盒、配电箱等)"""
        x_start = max(0, int(x_cm / (self.resolution * 100)))
        y_start = max(0, int(y_cm / (self.resolution * 100)))
        x_end = min(self.width, int((x_cm + w_cm) / (self.resolution * 100)))
        y_end = min(self.height, int((y_cm + h_cm) / (self.resolution * 100)))
        
        self.grid[y_start:y_end, x_start:x_end] = 1
        self.obstacles_info.append({'type': 'box', 'rect': (x_start, y_start, x_end-x_start, y_end-y_start), 'label': label})
        print(f"视觉检测障碍物[{label}]: ({x_cm}cm, {y_cm}cm) 尺寸 {w_cm}x{h_cm}cm")

    def inflate_safe_margin(self, safe_radius_cm=5):
        """对视觉捕捉的障碍物进行安全边距膨胀"""
        print(f"执行安全边距膨胀: {safe_radius_cm}cm...")
        radius = int(safe_radius_cm / (self.resolution * 100))
        
        blocked = (self.grid == 1).astype(np.uint8)
        inflated = np.zeros_like(blocked)
        pad = radius
        padded = np.pad(blocked, pad, mode='constant', constant_values=1)
        
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx*dx + dy*dy <= radius*radius:
                    shifted = padded[pad+dy:pad+dy+blocked.shape[0], pad+dx:pad+dx+blocked.shape[1]]
                    inflated = np.maximum(inflated, shifted)
                    
        self.inflated_grid = inflated.astype(np.int8)
        
    def is_free(self, x, y, use_inflated=True):
        """实现A*算法要求的接口"""
        if 0 <= x < self.width and 0 <= y < self.height:
            grid = self.inflated_grid if use_inflated else self.grid
            return grid[y, x] == 0
        return False


# ============================================================================
# 第二部分：场景构建与路径规划求解
# ============================================================================

def setup_simulation_wall():
    """构建典型的室内墙面作业场景"""
    print("\n" + "="*50)
    print(" 初始化视觉墙面建模映射 ")
    print("="*50)
    
    # 构建 3.2米宽 x 2.8米高的墙面，精度1cm
    wall = WallSurfaceMap(width_cm=320, height_cm=280)
    
    # 模拟相机视觉识别出的墙面障碍物
    # 1. 强弱电箱 (左下角)
    wall.add_obstacle_box(x_cm=30, y_cm=40, w_cm=40, h_cm=30, label="强电配电箱")
    
    # 2. 已有插座 (墙面各个位置)
    wall.add_obstacle_box(x_cm=150, y_cm=50, w_cm=15, h_cm=15, label="电源插座1")
    wall.add_obstacle_box(x_cm=260, y_cm=50, w_cm=15, h_cm=15, label="电源插座2")
    wall.add_obstacle_box(x_cm=180, y_cm=120, w_cm=15, h_cm=15, label="空调插座")
    
    # 3. 结构承重梁 (不可破坏，必须绕开)
    wall.add_obstacle_box(x_cm=120, y_cm=150, w_cm=80, h_cm=30, label="承重钢筋横梁")
    
    # 4. 门洞/窗户边缘
    wall.add_obstacle_box(x_cm=280, y_cm=80, w_cm=40, h_cm=120, label="窗户区域")

    # 执行膨胀操作，给切割刀头/机械臂末端留出8厘米的安全缓冲距离
    wall.inflate_safe_margin(safe_radius_cm=8)
    
    return wall

def run_wall_cutting_planning():
    """执行自主开槽规划"""
    wall = setup_simulation_wall()
    
    # 定义切割任务：从配电箱引线到右上方的新增壁灯位置
    # (起点：配电箱右侧，终点：新壁灯位置)
    start_pt = (85, 55)    # 对应 x=85cm, y=55cm
    goal_pt = (250, 220)   # 对应 x=250cm, y=220cm
    
    print("\n" + "*"*50)
    print(" 启动机械臂自主开槽轨迹规划引擎 ")
    print("*"*50)
    print(f"作业起点 (配电箱引出): {start_pt}")
    print(f"作业终点 (目标暗盒位): {goal_pt}")
    
    # 核心调用：跑三种算法做对比
    print("="*40)
    print("1. 运行 标准A*算法 (基准)...")
    planner_std = StandardAStar(wall)
    path_std, info_std = planner_std.search(start_pt, goal_pt)
    
    print("2. 运行 改进A*算法 (转向惩罚)...")
    planner_imp = ImprovedAStar(wall, turn_penalty=8.0)
    path_imp, info_imp = planner_imp.search(start_pt, goal_pt)
    
    print("3. 运行 增强A*算法 (绝对横平竖直)...")
    planner_enh = EnhancedAStar(wall, turn_penalty=8.0, diagonal_penalty=15.0, 
                                use_manhattan=True, smooth_path=True)
    path_enh, info_enh = planner_enh.search(start_pt, goal_pt)
    print("="*40)
    
    # 结果打包
    paths = [path_std, path_imp, path_enh]
    infos = [info_std, info_imp, info_enh]
    names = ['标准A* (基准)', '改进A* (减转弯)', '增强A* (横平竖直)']
    colors = ['#FF4757', '#2ED573', '#1E90FF']  # 红，绿，蓝
    
    if all(paths):
        print(f"\n[成功] 所有算法轨迹规划成功！开始生成对比图表...")
        metrics_list = []
        for p, info in zip(paths, infos):
            m = {
                'turns': PathAnalyzer.count_turns(p),
                'length': PathAnalyzer.calculate_path_length(p, wall.resolution),
                'straightness': PathAnalyzer.calculate_straightness(p),
                'diagonal_ratio': PathAnalyzer.calculate_diagonal_ratio(p)
            }
            metrics_list.append(m)
        
        visualize_wall_comparison(wall, start_pt, goal_pt, paths, names, colors, infos)
        visualize_wall_metrics(metrics_list, names, colors)
    else:
        print(f"\n[失败] 部分算法规划失败，无法生成对比。")


# ============================================================================
# 第三部分：论文级绘图输出
# ============================================================================

def visualize_wall_comparison(wall, start, goal, paths, names, colors, infos):
    """三种算法的开槽轨迹对比图（墙面坐标，厘米）."""
    fig, ax = fs.subplots(width='single', height=3.6)
    for obs in wall.obstacles_info:
        x, y, w, h = obs['rect']
        ax.add_patch(patches.Rectangle((x, y), w, h, lw=0.5, edgecolor=fs.BLACK,
                                       facecolor=fs.LIGHT, zorder=5))
        name = LABEL_EN.get(obs['label'], obs['label'])
        if w >= 30:                       # 大障碍物：标签写在框内
            ax.text(x + w / 2, y + h / 2, name.replace(' panel', '\npanel'),
                    ha='center', va='center', fontsize=7, zorder=6,
                    rotation=90 if h > 2 * w else 0)
        elif y > 100:                     # 小插座：标签写在左侧 / 下方，避开轨迹
            ax.text(x - 3, y + h / 2, name, ha='right', va='center', fontsize=7, zorder=6)
        else:
            ax.text(x + w / 2, y - 3, name, ha='center', va='top', fontsize=7, zorder=6)
    # 安全边距（膨胀层）
    margin = np.ma.masked_where((wall.inflated_grid == 0) | (wall.grid == 1),
                                np.ones_like(wall.grid, dtype=float))
    ax.imshow(margin, origin='lower', cmap='Greys', vmin=0, vmax=8, alpha=1.0,
              extent=(0, wall.width, 0, wall.height), interpolation='nearest', zorder=1)
    for i, path in enumerate(paths):
        if not path:
            continue
        ax.plot([p[0] for p in path], [p[1] for p in path], zorder=10 + i,
                label=ALGO_EN[i].replace('\n', ' '),
                **fs.line(ALGO_ROLE[i], lw=1.4 if i == 2 else 1.0))
        if i == 2:
            turns = [path[j] for j in range(1, len(path) - 1)
                     if (path[j][0] - path[j - 1][0], path[j][1] - path[j - 1][1]) !=
                     (path[j + 1][0] - path[j][0], path[j + 1][1] - path[j][1])]
            if turns:
                ax.plot([t[0] for t in turns], [t[1] for t in turns], 's', ms=3.5,
                        mfc='white', mec=fs.BLUE, zorder=15, label='Turn (enhanced A*)')
    ax.plot(*start, 'o', color=fs.BLACK, ms=4, zorder=20, label='Start')
    ax.plot(*goal, '*', color=fs.BLACK, ms=6, zorder=20, label='Goal')
    ax.plot([], [], 's', color='0.88', ms=5, label='Safety margin (8 cm)')
    ax.set_xlabel(fs.label('Wall position x', 'cm'))
    ax.set_ylabel(fs.label('Wall height y', 'cm'))
    ax.set_xlim(0, wall.width)
    ax.set_ylim(0, wall.height)
    ax.set_aspect('equal')
    fig.legend(loc='outside lower center', ncol=2)
    save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             'wall_cutting_comparison.png')
    fs.save(fig, save_path)
    print(f"\n[成功] 墙面多算法路径对比图已保存: \n{save_path}")


def visualize_wall_metrics(metrics_list, names, colors):
    """开槽轨迹的量化指标对比."""
    fig, axes = fs.subplots(2, 2, width='single', height=3.6)
    metrics = [('turns', 'Number of turns'), ('straightness', 'Axis-aligned share (%)'),
               ('diagonal_ratio', 'Diagonal share (%)'), ('length', 'Slot length (m)')]
    x = np.arange(len(metrics_list))
    for ax, (key, ylab) in zip(axes.flat, metrics):
        values = [m[key] * (100 if key in ('straightness', 'diagonal_ratio') else 1)
                  for m in metrics_list]
        for i, v in enumerate(values):
            ax.bar(x[i], v, 0.6, label=ALGO_EN[i].replace('\n', ' '), **fs.bar(ALGO_ROLE[i]))
            txt = f'{v:.1f}' if key != 'turns' else f'{int(v)}'
            if key == 'length':
                txt = f'{v:.2f}'
            ax.text(x[i], v, txt, ha='center', va='bottom', fontsize=7)
        ax.set_ylim(0, max(max(values) * 1.25, 1))
        ax.set_xticks(x)
        ax.set_xticklabels(['Std.', 'Impr.', 'Enh.'], fontsize=7)
        ax.set_ylabel(ylab)
    fs.panel_labels(axes.flat)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='outside lower center', ncol=1)
    save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             'wall_cutting_metrics.png')
    fs.save(fig, save_path)
    print(f"[成功] 墙面多算法指标图已保存: \n{save_path}")

if __name__ == '__main__':
    run_wall_cutting_planning()
