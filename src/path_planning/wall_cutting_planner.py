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
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib

import matplotlib.font_manager as fm

# 直接加载系统自带的微软雅黑字体 (针对Windows系统)
font_path = r"C:\Windows\Fonts\msyh.ttc"
if os.path.exists(font_path):
    my_font = fm.FontProperties(fname=font_path)
else:
    # 兼容性备择方案
    matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'sans-serif']
    my_font = fm.FontProperties()
    
matplotlib.rcParams['axes.unicode_minus'] = False

# 导入已经写好的极品A*算法模块
try:
    from improved_astar import EnhancedAStar, ImprovedAStar, StandardAStar, PathAnalyzer
except ImportError:
    print("错误: 找不到 improved_astar.py，请确保两个脚本在同一目录下。")
    sys.exit(1)


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
    """绘制极具视觉冲击力的开槽多算法对比仿真图"""
    fig, ax = plt.subplots(figsize=(14, 11))
    
    # 墙面背景
    ax.set_facecolor('#F8F9FA')  # 浅灰白墙面色
    
    # 绘制实体障碍物
    for obs in wall.obstacles_info:
        x, y, w, h = obs['rect']
        rect = patches.Rectangle((x, y), w, h, linewidth=2, edgecolor='#2D3436', 
                                 facecolor='#B2BEC3', alpha=0.9, zorder=5)
        ax.add_patch(rect)
        ax.text(x + w/2, y + h/2, obs['label'], 
                ha='center', va='center', fontsize=10, fontweight='bold',
                fontproperties=my_font,
                color='black', bbox=dict(facecolor='white', alpha=0.7, edgecolor='none', boxstyle='round,pad=0.2'), zorder=6)

    # 绘制机械臂安全禁区(仅外框以保持整洁)
    y_inf, x_inf = np.where(wall.inflated_grid == 1)
    ax.scatter(x_inf, y_inf, color='#FFEAA7', s=1, alpha=0.3, zorder=1, label='机械臂避障安全区(膨胀层)')

    # 绘制三条路径
    path_styles = [
        {'ls': '--', 'lw': 2, 'alpha': 0.8}, # 标准
        {'ls': '-.', 'lw': 3, 'alpha': 0.9}, # 改进
        {'ls': '-', 'lw': 4, 'alpha': 1.0}   # 增强
    ]
    
    for i, (path, name, color, style) in enumerate(zip(paths, names, colors, path_styles)):
        if not path: continue
        px = [p[0] for p in path]
        py = [p[1] for p in path]
        
        # 路径发光
        ax.plot(px, py, color=color, linewidth=style['lw']+4, alpha=0.2, zorder=7+i)
        ax.plot(px, py, color=color, linewidth=style['lw'], linestyle=style['ls'], 
                alpha=style['alpha'], label=name, zorder=10+i)
        
        # 仅标注终极版(增强A*)的转向点
        if i == 2:
            turns = []
            for j in range(1, len(path) - 1):
                d1 = (path[j][0] - path[j-1][0], path[j][1] - path[j-1][1])
                d2 = (path[j+1][0] - path[j][0], path[j+1][1] - path[j][1])
                if d1 != d2:
                    turns.append(path[j])
            if turns:
                tx = [t[0] for t in turns]
                ty = [t[1] for t in turns]
                ax.scatter(tx, ty, color='#FDCB6E', s=80, marker='s', zorder=15, 
                           edgecolors='#2D3436', linewidths=1.5, label='规划打孔/转向点 (最佳方案)')

    # 起点和终点
    ax.plot(start[0], start[1], 'o', color='#00B894', markersize=14, markeredgecolor='white', markeredgewidth=2.5, zorder=20, label='起槽点(Start)')
    ax.plot(goal[0], goal[1], '*', color='#0984E3', markersize=20, markeredgecolor='white', markeredgewidth=2.0, zorder=20, label='收槽点(Goal)')

    # 图表设置
    ax.set_title('机器视觉感知：三种墙面开槽轨迹算法直观对比', fontsize=18, fontweight='bold', pad=15, fontproperties=my_font)
    ax.set_xlabel('墙面X轴长度定位 (cm)', fontsize=12, fontproperties=my_font)
    ax.set_ylabel('墙面Y轴高度定位 (cm)', fontsize=12, fontproperties=my_font)
    ax.set_xlim(0, wall.width)
    ax.set_ylim(0, wall.height)
    
    # 添加图例
    legend = ax.legend(loc='lower right', fontsize=11, framealpha=0.9, edgecolor='#7F8CA6')
    for text in legend.get_texts():
        text.set_fontproperties(my_font)
        text.set_fontsize(11)

    # 保存退出
    plt.tight_layout()
    save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'wall_cutting_comparison.png')
    fig.savefig(save_path, dpi=250, bbox_inches='tight')
    print(f"\n[成功] 墙面多算法路径对比图已保存: \n{save_path}")
    plt.show(block=False)
    plt.pause(1)
    plt.close()

def visualize_wall_metrics(metrics_list, names, colors):
    """绘制开槽策略量化指标对比图"""
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('墙面智能开槽算法核心性能量化指标对比', fontsize=16, fontweight='bold', fontproperties=my_font, y=0.95)
    
    metrics = {
        '转向次数 (越少越好)': ('turns', axes[0, 0]),
        '横平竖直段比例 (100%最佳)': ('straightness', axes[0, 1]),
        '绝对斜切比例 (0%最佳)': ('diagonal_ratio', axes[1, 0]),
        '切割总长度 / 米': ('length', axes[1, 1])
    }
    
    x = np.arange(len(names))
    width = 0.5
    
    for title, (key, ax) in metrics.items():
        values = [m[key] for m in metrics_list]
        
        # 针对百分比特化显示
        if key in ['straightness', 'diagonal_ratio']:
            display_values = [v * 100 for v in values]
            bars = ax.bar(x, display_values, width, color=colors, alpha=0.8, edgecolor='black', linewidth=1)
            ax.set_ylabel('百分比 (%)', fontproperties=my_font)
            ax.set_ylim(0, 110)
        else:
            bars = ax.bar(x, values, width, color=colors, alpha=0.8, edgecolor='black', linewidth=1)
            ax.set_ylabel('数值', fontproperties=my_font)
            # 给最高值多留出空间写字
            ax.set_ylim(0, max(values) * 1.25 if max(values) > 0 else 1)
            
        ax.set_title(title, fontsize=12, fontweight='bold', fontproperties=my_font)
        ax.set_xticks(x)
        ax.set_xticklabels(names, fontproperties=my_font)
        ax.grid(axis='y', linestyle='--', alpha=0.7)
        
        # 柱子上加字
        for bar in bars:
            height = bar.get_height()
            if key in ['straightness', 'diagonal_ratio']:
                label = f"{height:.1f}%"
            elif key == 'length':
                label = f"{height:.2f}m"
            else:
                label = f"{int(height)}次"
            ax.text(bar.get_x() + bar.get_width()/2., height + (max(values)*0.05 if key not in ['straightness', 'diagonal_ratio'] else 5),
                    label, ha='center', va='bottom', fontsize=10, fontweight='bold', fontproperties=my_font)

    plt.tight_layout()
    fig.subplots_adjust(top=0.88)
    save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'wall_cutting_metrics.png')
    fig.savefig(save_path, dpi=250, bbox_inches='tight')
    print(f"[成功] 墙面多算法指标图已保存: \n{save_path}")
    plt.show(block=False)
    plt.pause(1)
    plt.close()


if __name__ == '__main__':
    run_wall_cutting_planning()
