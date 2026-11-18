#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
面向室内装修的墙面开槽路径规划算法研究
——基于改进A*算法的横平竖直约束路径规划

作者: SCR Project
功能: 
    1. 标准A*算法（基准对比）
    2. 改进A*算法 - 带转向惩罚因子 (Turn Penalty)
    3. 增强A*算法 - 转向惩罚 + 横平竖直方向约束 + 自适应代价
    4. 多算法对比可视化与量化指标分析
=============================================================================
"""

import heapq
import math
import time
import os
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import LinearSegmentedColormap
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'KaiTi']
matplotlib.rcParams['axes.unicode_minus'] = False


# ============================================================================
# 第一部分：地图加载与预处理
# ============================================================================

class GridMap:
    """栅格地图类 - 加载并处理SLAM生成的.pgm地图"""
    
    def __init__(self, pgm_path, yaml_path=None):
        """
        加载地图文件
        
        参数:
            pgm_path: .pgm地图文件路径
            yaml_path: .yaml配置文件路径（可选）
        """
        # 加载地图图像
        img = Image.open(pgm_path)
        self.raw_map = np.array(img)
        self.height, self.width = self.raw_map.shape
        
        # 默认参数
        self.resolution = 0.05  # 每个像素对应0.05米
        self.origin = [0.0, 0.0, 0.0]
        self.occupied_thresh = 0.65
        self.free_thresh = 0.25
        
        # 读取yaml配置
        if yaml_path and os.path.exists(yaml_path):
            self._load_yaml(yaml_path)
        
        # 生成二值化栅格地图
        # 0 = 可通行, 1 = 障碍物, -1 = 未知
        # PGM中: 254=白色=自由空间, 0=黑色=障碍物, 205=灰色=未知区域
        self.grid = np.full_like(self.raw_map, -1, dtype=np.int8)  # 默认未知
        self.grid[self.raw_map > 220] = 0    # 亮白色 → 可通行 (254)
        self.grid[self.raw_map < 50] = 1     # 黑色 → 障碍物 (0)
        # 其余(如205)保持为-1(未知)，视为不可通行
        
        # 膨胀障碍物（安全边距），使用numpy加速
        self.inflated_grid = self._inflate_obstacles(radius=3)
        
        print(f"地图加载完成: {self.width}x{self.height}")
        print(f"分辨率: {self.resolution} m/pixel")
        print(f"障碍物像素: {np.sum(self.grid == 1)}")
        print(f"可通行像素: {np.sum(self.grid == 0)}")
        print(f"未知像素: {np.sum(self.grid == -1)}")
    
    def _load_yaml(self, yaml_path):
        """解析YAML配置文件"""
        with open(yaml_path, 'r') as f:
            for line in f:
                line = line.strip()
                if line.startswith('resolution:'):
                    self.resolution = float(line.split(':')[1].strip())
                elif line.startswith('origin:'):
                    origin_str = line.split(':')[1].strip().strip('[]')
                    self.origin = [float(x.strip()) for x in origin_str.split(',')]
                elif line.startswith('occupied_thresh:'):
                    self.occupied_thresh = float(line.split(':')[1].strip())
                elif line.startswith('free_thresh:'):
                    self.free_thresh = float(line.split(':')[1].strip())
    
    def _inflate_obstacles(self, radius=3):
        """
        膨胀障碍物 - 使用numpy卷积加速
        将障碍物和未知区域都视为不可通行进行膨胀
        
        参数:
            radius: 膨胀半径（像素）
        """
        # 创建不可通行掩码（障碍物 + 未知区域都不可通行）
        blocked = (self.grid != 0).astype(np.uint8)
        
        # 创建圆形膨胀核
        y, x = np.ogrid[-radius:radius+1, -radius:radius+1]
        kernel = (x*x + y*y <= radius*radius).astype(np.uint8)
        
        # 使用numpy手动实现膨胀（避免依赖scipy）
        inflated = np.zeros_like(blocked)
        pad = radius
        padded = np.pad(blocked, pad, mode='constant', constant_values=1)
        
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx*dx + dy*dy <= radius*radius:
                    shifted = padded[pad+dy:pad+dy+blocked.shape[0], 
                                     pad+dx:pad+dx+blocked.shape[1]]
                    inflated = np.maximum(inflated, shifted)
        
        # 转换回grid格式
        result = self.grid.copy()
        result[inflated == 1] = 1
        
        print(f"膨胀后障碍物像素: {np.sum(result == 1)}")
        print(f"膨胀后可通行像素: {np.sum(result == 0)}")
        return result
    
    def is_free(self, x, y, use_inflated=True):
        """检查坐标是否可通行"""
        if 0 <= x < self.width and 0 <= y < self.height:
            grid = self.inflated_grid if use_inflated else self.grid
            return grid[y, x] == 0
        return False
    
    def world_to_grid(self, wx, wy):
        """世界坐标转栅格坐标"""
        gx = int((wx - self.origin[0]) / self.resolution)
        gy = int((wy - self.origin[1]) / self.resolution)
        return gx, gy
    
    def grid_to_world(self, gx, gy):
        """栅格坐标转世界坐标"""
        wx = gx * self.resolution + self.origin[0]
        wy = gy * self.resolution + self.origin[1]
        return wx, wy


# ============================================================================
# 第二部分：A*算法核心实现
# ============================================================================

class Node:
    """搜索节点"""
    __slots__ = ['x', 'y', 'g', 'h', 'f', 'parent', 'direction']
    
    def __init__(self, x, y, g=0, h=0, parent=None, direction=None):
        self.x = x
        self.y = y
        self.g = g          # 起点到当前节点的实际代价
        self.h = h          # 当前节点到终点的启发式估计
        self.f = g + h      # 总代价
        self.parent = parent
        self.direction = direction  # 到达此节点的方向 (dx, dy)
    
    def __lt__(self, other):
        return self.f < other.f
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        return hash((self.x, self.y))


class StandardAStar:
    """标准A*算法 - 作为对比基准"""
    
    # 8方向移动: (dx, dy, cost)
    DIRECTIONS_8 = [
        (0, 1, 1.0),    # 上
        (0, -1, 1.0),   # 下
        (1, 0, 1.0),    # 右
        (-1, 0, 1.0),   # 左
        (1, 1, 1.414),  # 右上
        (1, -1, 1.414), # 右下
        (-1, 1, 1.414), # 左上
        (-1, -1, 1.414) # 左下
    ]
    
    def __init__(self, grid_map):
        self.grid_map = grid_map
        self.name = "标准A*算法"
        self.expanded_nodes = 0
    
    def heuristic(self, x1, y1, x2, y2):
        """启发式函数 - 欧几里得距离"""
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    
    def search(self, start, goal):
        """
        执行A*搜索
        
        参数:
            start: 起点 (x, y)
            goal: 终点 (x, y)
        
        返回:
            path: 路径点列表 [(x, y), ...]
            info: 搜索信息字典
        """
        start_time = time.time()
        self.expanded_nodes = 0
        
        sx, sy = start
        gx, gy = goal
        
        start_node = Node(sx, sy, g=0, h=self.heuristic(sx, sy, gx, gy))
        
        open_list = []
        heapq.heappush(open_list, start_node)
        
        closed_set = set()
        g_score = {(sx, sy): 0}
        all_expanded = []  # 记录所有扩展的节点（用于可视化）
        
        while open_list:
            current = heapq.heappop(open_list)
            
            if (current.x, current.y) in closed_set:
                continue
            
            closed_set.add((current.x, current.y))
            self.expanded_nodes += 1
            all_expanded.append((current.x, current.y))
            
            # 到达目标
            if current.x == gx and current.y == gy:
                path = self._reconstruct_path(current)
                elapsed = time.time() - start_time
                info = {
                    'time': elapsed,
                    'expanded': self.expanded_nodes,
                    'path_length': len(path),
                    'path_cost': current.g,
                    'all_expanded': all_expanded
                }
                return path, info
            
            # 扩展邻居
            for dx, dy, move_cost in self.DIRECTIONS_8:
                nx, ny = current.x + dx, current.y + dy
                
                if not self.grid_map.is_free(nx, ny):
                    continue
                
                if (nx, ny) in closed_set:
                    continue
                
                new_g = current.g + move_cost
                
                if (nx, ny) not in g_score or new_g < g_score[(nx, ny)]:
                    g_score[(nx, ny)] = new_g
                    h = self.heuristic(nx, ny, gx, gy)
                    neighbor = Node(nx, ny, g=new_g, h=h, parent=current,
                                    direction=(dx, dy))
                    heapq.heappush(open_list, neighbor)
        
        return [], {'time': time.time() - start_time, 'expanded': self.expanded_nodes,
                    'path_length': 0, 'path_cost': float('inf'), 'all_expanded': all_expanded}
    
    def _reconstruct_path(self, node):
        """回溯路径"""
        path = []
        while node:
            path.append((node.x, node.y))
            node = node.parent
        return path[::-1]


class ImprovedAStar(StandardAStar):
    """
    改进A*算法 - 带转向惩罚因子
    
    核心改进：在代价函数 F = G + H 中引入转向惩罚
    当路径需要转弯时，额外增加代价值，迫使算法优先选择直线段
    """
    
    def __init__(self, grid_map, turn_penalty=2.0):
        """
        参数:
            grid_map: 栅格地图
            turn_penalty: 转向惩罚因子（越大越倾向直线）
        """
        super().__init__(grid_map)
        self.turn_penalty = turn_penalty
        self.name = f"改进A*（转向惩罚={turn_penalty}）"
    
    def search(self, start, goal):
        """带转向惩罚的A*搜索"""
        start_time = time.time()
        self.expanded_nodes = 0
        
        sx, sy = start
        gx, gy = goal
        
        start_node = Node(sx, sy, g=0, h=self.heuristic(sx, sy, gx, gy))
        
        open_list = []
        heapq.heappush(open_list, start_node)
        
        closed_set = set()
        g_score = {(sx, sy): 0}
        all_expanded = []
        
        while open_list:
            current = heapq.heappop(open_list)
            
            if (current.x, current.y) in closed_set:
                continue
            
            closed_set.add((current.x, current.y))
            self.expanded_nodes += 1
            all_expanded.append((current.x, current.y))
            
            if current.x == gx and current.y == gy:
                path = self._reconstruct_path(current)
                elapsed = time.time() - start_time
                info = {
                    'time': elapsed,
                    'expanded': self.expanded_nodes,
                    'path_length': len(path),
                    'path_cost': current.g,
                    'all_expanded': all_expanded
                }
                return path, info
            
            for dx, dy, move_cost in self.DIRECTIONS_8:
                nx, ny = current.x + dx, current.y + dy
                
                if not self.grid_map.is_free(nx, ny):
                    continue
                
                if (nx, ny) in closed_set:
                    continue
                
                # ======== 核心改进：转向惩罚 ========
                penalty = 0.0
                if current.direction is not None:
                    prev_dx, prev_dy = current.direction
                    # 计算方向变化
                    if (dx, dy) != (prev_dx, prev_dy):
                        # 计算转角（点积）
                        dot = dx * prev_dx + dy * prev_dy
                        norm1 = math.sqrt(dx**2 + dy**2)
                        norm2 = math.sqrt(prev_dx**2 + prev_dy**2)
                        cos_angle = dot / (norm1 * norm2) if norm1 * norm2 > 0 else 1.0
                        cos_angle = max(-1.0, min(1.0, cos_angle))
                        angle = math.acos(cos_angle)
                        
                        # 转向惩罚与转角成正比
                        penalty = self.turn_penalty * (angle / math.pi)
                # ====================================
                
                new_g = current.g + move_cost + penalty
                
                if (nx, ny) not in g_score or new_g < g_score[(nx, ny)]:
                    g_score[(nx, ny)] = new_g
                    h = self.heuristic(nx, ny, gx, gy)
                    neighbor = Node(nx, ny, g=new_g, h=h, parent=current,
                                    direction=(dx, dy))
                    heapq.heappush(open_list, neighbor)
        
        return [], {'time': time.time() - start_time, 'expanded': self.expanded_nodes,
                    'path_length': 0, 'path_cost': float('inf'), 'all_expanded': all_expanded}


class EnhancedAStar(StandardAStar):
    """
    增强A*算法 - 多策略融合的横平竖直路径规划
    
    改进策略：
    1. 转向惩罚因子：惩罚频繁转弯，鼓励直线行驶
    2. 方向约束因子：惩罚斜向移动（45°），仅允许4方向（上下左右）
    3. 自适应启发函数：使用曼哈顿距离替代欧几里得，天然适配横平竖直
    4. 路径平滑：后处理消除冗余转折点
    """
    
    # 4方向移动（仅横平竖直）
    DIRECTIONS_4 = [
        (0, 1, 1.0),    # 上
        (0, -1, 1.0),   # 下
        (1, 0, 1.0),    # 右
        (-1, 0, 1.0),   # 左
    ]
    
    def __init__(self, grid_map, turn_penalty=5.0, diagonal_penalty=8.0,
                 use_manhattan=True, smooth_path=True):
        """
        参数:
            grid_map: 栅格地图
            turn_penalty: 转向惩罚因子
            diagonal_penalty: 斜向移动额外惩罚
            use_manhattan: 是否使用曼哈顿距离启发函数
            smooth_path: 是否进行路径平滑后处理
        """
        super().__init__(grid_map)
        self.turn_penalty = turn_penalty
        self.diagonal_penalty = diagonal_penalty
        self.use_manhattan = use_manhattan
        self.smooth_path_flag = smooth_path
        self.name = "增强A*（横平竖直约束）"
    
    def heuristic(self, x1, y1, x2, y2):
        """
        自适应启发函数
        曼哈顿距离天然匹配横平竖直的移动模式
        """
        if self.use_manhattan:
            return abs(x1 - x2) + abs(y1 - y2)
        else:
            return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    
    def search(self, start, goal):
        """增强A*搜索 - 多策略融合"""
        start_time = time.time()
        self.expanded_nodes = 0
        
        sx, sy = start
        gx, gy = goal
        
        start_node = Node(sx, sy, g=0, h=self.heuristic(sx, sy, gx, gy))
        
        open_list = []
        heapq.heappush(open_list, start_node)
        
        closed_set = set()
        g_score = {(sx, sy): 0}
        all_expanded = []
        
        # 使用8方向但对斜向加重惩罚，让算法"学会"只走横平竖直
        directions = self.DIRECTIONS_8
        
        while open_list:
            current = heapq.heappop(open_list)
            
            if (current.x, current.y) in closed_set:
                continue
            
            closed_set.add((current.x, current.y))
            self.expanded_nodes += 1
            all_expanded.append((current.x, current.y))
            
            if current.x == gx and current.y == gy:
                path = self._reconstruct_path(current)
                elapsed_search = time.time() - start_time
                
                # 路径后处理：平滑与冗余点消除
                if self.smooth_path_flag:
                    path = self._smooth_path(path)
                
                elapsed_total = time.time() - start_time
                info = {
                    'time': elapsed_total,
                    'search_time': elapsed_search,
                    'expanded': self.expanded_nodes,
                    'path_length': len(path),
                    'path_cost': current.g,
                    'all_expanded': all_expanded
                }
                return path, info
            
            for dx, dy, move_cost in directions:
                nx, ny = current.x + dx, current.y + dy
                
                if not self.grid_map.is_free(nx, ny):
                    continue
                
                if (nx, ny) in closed_set:
                    continue
                
                # ======== 多策略代价计算 ========
                total_penalty = 0.0
                
                # 策略1: 斜向移动惩罚（迫使优先走横平竖直）
                is_diagonal = (dx != 0 and dy != 0)
                if is_diagonal:
                    total_penalty += self.diagonal_penalty
                
                # 策略2: 转向惩罚（减少不必要的转弯）
                if current.direction is not None:
                    prev_dx, prev_dy = current.direction
                    if (dx, dy) != (prev_dx, prev_dy):
                        # 区分90°转弯和其他角度
                        dot = dx * prev_dx + dy * prev_dy
                        norm1 = math.sqrt(dx**2 + dy**2)
                        norm2 = math.sqrt(prev_dx**2 + prev_dy**2)
                        cos_angle = dot / (norm1 * norm2) if norm1 * norm2 > 0 else 1.0
                        cos_angle = max(-1.0, min(1.0, cos_angle))
                        angle = math.acos(cos_angle)
                        
                        # 非90°整数倍的转弯加更重的惩罚
                        is_right_angle = abs(angle - math.pi/2) < 0.01 or abs(angle - math.pi) < 0.01
                        if is_right_angle:
                            total_penalty += self.turn_penalty * 0.5  # 90°转弯惩罚较轻
                        else:
                            total_penalty += self.turn_penalty * 2.0  # 其他角度惩罚更重
                # ====================================
                
                new_g = current.g + move_cost + total_penalty
                
                if (nx, ny) not in g_score or new_g < g_score[(nx, ny)]:
                    g_score[(nx, ny)] = new_g
                    h = self.heuristic(nx, ny, gx, gy)
                    neighbor = Node(nx, ny, g=new_g, h=h, parent=current,
                                    direction=(dx, dy))
                    heapq.heappush(open_list, neighbor)
        
        return [], {'time': time.time() - start_time, 'expanded': self.expanded_nodes,
                    'path_length': 0, 'path_cost': float('inf'), 'all_expanded': all_expanded}
    
    def _smooth_path(self, path):
        """
        路径平滑 - 消除冗余共线点
        保留起点、终点和所有转折点
        """
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]
        
        for i in range(1, len(path) - 1):
            prev = path[i - 1]
            curr = path[i]
            next_pt = path[i + 1]
            
            # 计算前后两段的方向
            d1 = (curr[0] - prev[0], curr[1] - prev[1])
            d2 = (next_pt[0] - curr[0], next_pt[1] - curr[1])
            
            # 如果方向发生变化，保留该转折点
            if d1 != d2:
                smoothed.append(curr)
        
        smoothed.append(path[-1])
        return smoothed


# ============================================================================
# 第三部分：路径分析与量化指标
# ============================================================================

class PathAnalyzer:
    """路径分析器 - 计算各种量化指标"""
    
    @staticmethod
    def count_turns(path):
        """统计路径转弯次数"""
        if len(path) < 3:
            return 0
        
        turns = 0
        for i in range(1, len(path) - 1):
            d1 = (path[i][0] - path[i-1][0], path[i][1] - path[i-1][1])
            d2 = (path[i+1][0] - path[i][0], path[i+1][1] - path[i][1])
            if d1 != d2:
                turns += 1
        return turns
    
    @staticmethod
    def count_right_angle_turns(path):
        """统计90°转弯次数"""
        if len(path) < 3:
            return 0
        
        right_angles = 0
        for i in range(1, len(path) - 1):
            d1 = (path[i][0] - path[i-1][0], path[i][1] - path[i-1][1])
            d2 = (path[i+1][0] - path[i][0], path[i+1][1] - path[i][1])
            
            if d1 != d2:
                # 计算角度
                dot = d1[0]*d2[0] + d1[1]*d2[1]
                n1 = math.sqrt(d1[0]**2 + d1[1]**2)
                n2 = math.sqrt(d2[0]**2 + d2[1]**2)
                if n1 > 0 and n2 > 0:
                    cos_a = max(-1, min(1, dot / (n1 * n2)))
                    angle = math.degrees(math.acos(cos_a))
                    if abs(angle - 90) < 5:  # 允许5度误差
                        right_angles += 1
        return right_angles
    
    @staticmethod
    def calculate_path_length(path, resolution=0.05):
        """计算路径实际长度（米）"""
        length = 0.0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            length += math.sqrt(dx**2 + dy**2)
        return length * resolution
    
    @staticmethod
    def calculate_straightness(path):
        """
        计算路径直线度（横平竖直比例）
        返回: 横/竖直线段占总段数的比例
        """
        if len(path) < 2:
            return 1.0
        
        hv_segments = 0  # 横平竖直段数
        total_segments = 0
        
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            total_segments += 1
            
            # 横向或竖向移动
            if dx == 0 or dy == 0:
                hv_segments += 1
        
        return hv_segments / total_segments if total_segments > 0 else 0.0
    
    @staticmethod
    def calculate_diagonal_ratio(path):
        """计算斜向移动占比"""
        if len(path) < 2:
            return 0.0
        
        diagonal = 0
        total = len(path) - 1
        
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            if dx != 0 and dy != 0:
                diagonal += 1
        
        return diagonal / total if total > 0 else 0.0


# ============================================================================
# 第四部分：专业级可视化
# ============================================================================

class PathVisualizer:
    """路径可视化器 - 生成论文级别的对比图表"""
    
    def __init__(self, grid_map, save_dir=None):
        self.grid_map = grid_map
        self.save_dir = save_dir or os.path.dirname(os.path.abspath(__file__))
        self.analyzer = PathAnalyzer()
        
        # 配色方案
        self.colors = {
            'standard': '#E74C3C',    # 红色
            'improved': '#2ECC71',    # 绿色
            'enhanced': '#3498DB',    # 蓝色
        }
    
    # 图中的英文名称与语义角色（docs/figstyle）
    NAMES_EN = ['Standard A*', 'Improved A* (turn penalty)', 'Enhanced A* (axis-aligned)']
    ROLES = ['neutral', 'alt2', 'proposed']

    def _fs(self):
        import sys
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        '..', '..', 'docs'))
        import figstyle as fs
        return fs

    def visualize_comparison(self, results, start, goal):
        """
        多算法路径对比（2×2）：(a) 叠加，(b)–(d) 各算法的路径与搜索范围。

        图中不标注计算耗时：耗时随机器变化，量化指标见 visualize_metrics。
        """
        fs = self._fs()
        h, w = self.grid_map.grid.shape
        fig, axes = fs.subplots(2, 2, width='double', height=7.16 * h / w + 0.9)
        ax1 = axes[0, 0]
        self._draw_map(ax1)
        for i, (name, path, info, color) in enumerate(results):
            if path:
                ax1.plot([p[0] for p in path], [p[1] for p in path], label=self.NAMES_EN[i],
                         **fs.line(self.ROLES[i], lw=1.2))
        ax1.plot(start[0], start[1], 'o', color=fs.BLACK, ms=4, zorder=10, label='Start')
        ax1.plot(goal[0], goal[1], '*', color=fs.BLACK, ms=6, zorder=10, label='Goal')
        fs.panel_label(ax1, 'a', 'All paths')
        for idx, (name, path, info, color) in enumerate(results):
            ax = axes[(idx + 1) // 2, (idx + 1) % 2]
            self._draw_map(ax)
            if info.get('all_expanded'):
                ex = info['all_expanded']
                ax.scatter([e[0] for e in ex], [e[1] for e in ex], c=range(len(ex)),
                           cmap='Oranges', s=0.4, alpha=0.5, zorder=1, rasterized=True)
            if path:
                ax.plot([p[0] for p in path], [p[1] for p in path], zorder=5,
                        **fs.line(self.ROLES[idx], lw=1.2))
                turns = [path[i] for i in range(1, len(path) - 1)
                         if (path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]) !=
                         (path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])]
                if turns:
                    ax.plot([t[0] for t in turns], [t[1] for t in turns], 's', ms=2.5,
                            mfc='white', mec=fs.BLACK, mew=0.5, zorder=6)
            ax.plot(start[0], start[1], 'o', color=fs.BLACK, ms=4, zorder=10)
            ax.plot(goal[0], goal[1], '*', color=fs.BLACK, ms=6, zorder=10)
            fs.note(ax, f"Path points: {info.get('path_length', 0)}\n"
                        f"Expanded: {info.get('expanded', 0)} nodes", x=0.98, y=0.02,
                    va='bottom', bbox=dict(fc='white', ec='none', pad=1.0))
            fs.panel_label(ax, 'bcd'[idx], self.NAMES_EN[idx])
        from matplotlib.lines import Line2D
        handles, labels = ax1.get_legend_handles_labels()
        handles += [Line2D([], [], ls='none', marker='s', ms=2.5, mfc='white', mec=fs.BLACK,
                           label='Turn'),
                    Line2D([], [], ls='none', marker='o', ms=3, color='#F4A261',
                           label='Expanded node (colour: order)')]
        fig.legend(handles=handles, loc='outside lower center', ncol=4)
        save_path = os.path.join(self.save_dir, 'path_comparison.png')
        fs.save(fig, save_path)
        print(f"\n对比图已保存: {save_path}")

    def visualize_metrics(self, results):
        """量化指标对比柱状图（不含计算耗时：耗时随机器变化，见终端输出）."""
        fs = self._fs()
        metrics = {
            'Path length (m)': [],
            'Number of turns': [],
            'Right-angle turns (%)': [],
            'Axis-aligned share (%)': [],
            'Expanded nodes': [],
        }
        for name, path, info, color in results:
            path_len = self.analyzer.calculate_path_length(path, self.grid_map.resolution)
            turns = self.analyzer.count_turns(path)
            right_angles = self.analyzer.count_right_angle_turns(path)
            straightness = self.analyzer.calculate_straightness(path)
            metrics['Path length (m)'].append(round(path_len, 2))
            metrics['Number of turns'].append(turns)
            metrics['Right-angle turns (%)'].append(
                round(right_angles / turns * 100, 1) if turns > 0 else 100.0)
            metrics['Axis-aligned share (%)'].append(round(straightness * 100, 1))
            metrics['Expanded nodes'].append(info.get('expanded', 0))
        fig, axes = fs.subplots(1, 5, width='double', height=2.2)
        x = np.arange(len(results))
        for ax, (metric_name, values) in zip(axes, metrics.items()):
            for i, v in enumerate(values):
                ax.bar(x[i], v, 0.65, label=self.NAMES_EN[i], **fs.bar(self.ROLES[i]))
                ax.text(x[i], v, f'{v}', ha='center', va='bottom', fontsize=7)
            ax.set_ylim(0, max(max(values) * 1.2, 1))
            ax.set_xticks(x)
            ax.set_xticklabels(['Std.', 'Impr.', 'Enh.'], fontsize=7)
            ax.set_ylabel(metric_name)
        fs.panel_labels(axes)
        handles, labels = axes[0].get_legend_handles_labels()
        fig.legend(handles, labels, loc='outside lower center', ncol=3)
        fs.fixed_layout(fig, left=0.42, right=0.05, bottom=0.62, top=0.2, wspace=0.58)
        save_path = os.path.join(self.save_dir, 'metrics_comparison.png')
        fs.save(fig, save_path)
        print(f"指标图已保存: {save_path}")

    def print_metrics_table(self, results):
        """打印格式化的指标对比表"""
        analyzer = self.analyzer
        
        print("\n" + "=" * 80)
        print("                    路径规划算法量化指标对比")
        print("=" * 80)
        print(f"{'指标':<20}", end="")
        for name, _, _, _ in results:
            print(f"{name:<22}", end="")
        print()
        print("-" * 80)
        
        rows = []
        for name, path, info, color in results:
            row = {
                '路径长度(m)': f"{analyzer.calculate_path_length(path, self.grid_map.resolution):.2f}",
                '路径点数': str(len(path)),
                '转弯次数': str(analyzer.count_turns(path)),
                '90°转弯数': str(analyzer.count_right_angle_turns(path)),
                '横平竖直比例': f"{analyzer.calculate_straightness(path)*100:.1f}%",
                '斜向移动占比': f"{analyzer.calculate_diagonal_ratio(path)*100:.1f}%",
                '搜索节点数': str(info.get('expanded', 0)),
                '计算时间(ms)': f"{info.get('time', 0)*1000:.1f}",
            }
            rows.append(row)
        
        for key in rows[0].keys():
            print(f"{key:<20}", end="")
            for row in rows:
                print(f"{row[key]:<22}", end="")
            print()
        
        print("=" * 80)
    
    def _draw_map(self, ax):
        """在坐标轴上绘制地图"""
        display_map = np.ones((*self.grid_map.grid.shape, 3))
        
        # 可通行区域 - 白色
        display_map[self.grid_map.grid == 0] = [1.0, 1.0, 1.0]
        # 障碍物 - 深灰色
        display_map[self.grid_map.grid == 1] = [0.2, 0.2, 0.2]
        # 未知区域 - 浅灰色
        display_map[self.grid_map.grid == -1] = [0.75, 0.75, 0.75]
        
        ax.imshow(display_map, origin='lower', interpolation='nearest')
        ax.set_xlabel('x (cells, 0.05 m)')
        ax.set_ylabel('y (cells, 0.05 m)')


# ============================================================================
# 第五部分：主程序
# ============================================================================

def find_valid_points(grid_map, margin=20):
    """在地图上自动寻找合适的起点和终点（使用膨胀后的地图）"""
    # 使用膨胀后的地图确保选取的点是真正可通行的
    free_cells = np.where(grid_map.inflated_grid == 0)
    
    if len(free_cells[0]) == 0:
        print("警告：地图中没有可通行区域！")
        return None, None
    
    ys, xs = free_cells
    print(f"可用自由点数: {len(xs)}")
    
    # 策略：从所有自由点中选择距离较远的两个点
    # 先按区域划分，寻找不同区域的代表点
    h, w = grid_map.height, grid_map.width
    
    candidates = []
    regions = [
        (margin, w//3, margin, h//3),         # 左下
        (2*w//3, w-margin, margin, h//3),     # 右下
        (margin, w//3, 2*h//3, h-margin),     # 左上
        (2*w//3, w-margin, 2*h//3, h-margin), # 右上
        (w//3, 2*w//3, h//3, 2*h//3),         # 中间
    ]
    
    for x_min, x_max, y_min, y_max in regions:
        mask = (xs >= x_min) & (xs < x_max) & (ys >= y_min) & (ys < y_max)
        region_xs = xs[mask]
        region_ys = ys[mask]
        if len(region_xs) > 10:  # 至少有10个自由点才算有效区域
            idx = len(region_xs) // 2
            candidates.append((int(region_xs[idx]), int(region_ys[idx])))
    
    print(f"候选区域数: {len(candidates)}")
    
    if len(candidates) < 2:
        # 从全部自由点中等间距取样
        step = max(1, len(xs) // 10)
        for i in range(0, len(xs), step):
            candidates.append((int(xs[i]), int(ys[i])))
    
    if len(candidates) < 2:
        print("警告：自由空间过小，无法找到合适的起终点")
        return None, None
    
    # 选择距离最远的两个候选点
    max_dist = 0
    best_pair = (candidates[0], candidates[1])
    for i in range(len(candidates)):
        for j in range(i+1, len(candidates)):
            dist = math.sqrt((candidates[i][0] - candidates[j][0])**2 + 
                           (candidates[i][1] - candidates[j][1])**2)
            if dist > max_dist:
                max_dist = dist
                best_pair = (candidates[i], candidates[j])
    
    return best_pair


def main():
    """主函数"""
    print("=" * 60)
    print("  面向室内装修的改进A*路径规划算法研究")
    print("  ——横平竖直约束路径规划对比实验")
    print("=" * 60)
    
    # 1. 加载地图
    script_dir = os.path.dirname(os.path.abspath(__file__))
    map_dir = os.path.join(os.path.dirname(script_dir), 'scr_navigation', 'maps')
    
    pgm_path = os.path.join(map_dir, 'my_map.pgm')
    yaml_path = os.path.join(map_dir, 'my_map.yaml')
    
    if not os.path.exists(pgm_path):
        print(f"错误: 找不到地图文件 {pgm_path}")
        print("请确保SLAM地图文件存在于 scr_navigation/maps/ 目录下")
        return
    
    print(f"\n加载地图: {pgm_path}")
    grid_map = GridMap(pgm_path, yaml_path)
    
    # 2. 设置起点和终点
    start, goal = find_valid_points(grid_map)
    
    if start is None or goal is None:
        print("错误: 无法在地图上找到有效的起点和终点")
        return
    
    print(f"\n起点: {start} (像素)")
    print(f"终点: {goal} (像素)")
    print(f"直线距离: {math.sqrt((start[0]-goal[0])**2 + (start[1]-goal[1])**2):.1f} 像素 "
          f"({math.sqrt((start[0]-goal[0])**2 + (start[1]-goal[1])**2) * grid_map.resolution:.2f} 米)")
    
    # 3. 运行三种算法
    print("\n" + "-" * 60)
    algorithms = [
        ("标准A*算法", StandardAStar(grid_map), '#E74C3C'),
        ("改进A*（转向惩罚）", ImprovedAStar(grid_map, turn_penalty=3.0), '#2ECC71'),
        ("增强A*（横平竖直约束）", EnhancedAStar(grid_map, turn_penalty=5.0, 
                                                   diagonal_penalty=8.0), '#3498DB'),
    ]
    
    results = []
    
    for name, algo, color in algorithms:
        print(f"\n运行 {name}...")
        path, info = algo.search(start, goal)
        
        if path:
            print(f"  ✅ 路径找到！ 长度={len(path)}点, "
                  f"搜索节点={info['expanded']}, "
                  f"耗时={info['time']*1000:.1f}ms")
        else:
            print(f"  ❌ 未找到路径")
        
        results.append((name, path, info, color))
    
    # 4. 可视化与分析
    print("\n" + "-" * 60)
    print("生成可视化图表...")
    
    save_dir = os.path.join(script_dir)
    visualizer = PathVisualizer(grid_map, save_dir=save_dir)
    
    # 量化指标表格
    visualizer.print_metrics_table(results)
    
    # 路径对比图
    visualizer.visualize_comparison(results, start, goal)
    
    # 量化指标柱状图
    visualizer.visualize_metrics(results)
    
    print("\n✅ 所有分析完成！")
    print(f"   图表已保存到: {save_dir}")
    print("   可直接用于论文中的算法对比章节")


if __name__ == '__main__':
    main()
