"""
离线开槽规划文件的解析与到站补偿（纯函数，不依赖 ROS）.

规划文件格式见 scr_planner/python/slot_plan.py 的 plan_to_dict。墙面坐标：u 沿墙向右
（面对墙面时），v 为离地高度。行为树执行器（scr_task，C++）中的 SlotPlan 解析与这里的
station_arm_points 实现同一套规则，二者由各自的单元测试对照同一份规划文件检查。
"""
import math

import numpy as np


def is_slot_plan(config):
    """任务文件是否为离线开槽规划（而不是单站任务）."""
    return isinstance(config, dict) and 'stations' in config and 'wall' in config


def plan_wall_axes(plan):
    """墙面系 → map：返回 (p0 左端点, t 沿墙向右单位向量, n 指向房间的法向)."""
    p0 = np.array(plan['wall']['left_end'], dtype=float)
    p1 = np.array(plan['wall']['right_end'], dtype=float)
    t = (p1 - p0) / np.linalg.norm(p1 - p0)
    return p0, t, np.array([t[1], -t[0]])


def plan_wall_pose(plan, u, standoff):
    """正对墙面、base_link 中心离墙 standoff、位于墙面 u 处的 map 位姿 (x, y, yaw)."""
    p0, t, n = plan_wall_axes(plan)
    p = p0 + u * t + standoff * n
    return float(p[0]), float(p[1]), math.atan2(-n[1], -n[0])


def plan_marker_u(plan, marker_xy):
    """定位标记在墙面坐标中的 u（米）."""
    p0, t, _ = plan_wall_axes(plan)
    return float((np.asarray(marker_xy, dtype=float) - p0) @ t)


def station_arm_points(station, plan, dry_run_gap=None):
    """
    一个停靠站的机械臂点 → ([(x, y, z, rotation_rad)], [是否处于切深]).

    dry_run_gap（米）：空切，切深处的点改为刀尖停在规划墙面前 gap 处（Gazebo 的墙是
    刚体，真的下刀会把整台机器人顶开）；退刀点不变。
    """
    r = plan['robot']
    x_cut = r['x_cut']
    x_dry = None if dry_run_gap is None else r['standoff'] - dry_run_gap - r['tool_offset']
    pts, is_cut = [], []
    for p in station['points']:
        cut = abs(p['x'] - x_cut) < 1e-6
        x = x_dry if (cut and x_dry is not None) else p['x']
        pts.append((x, p['y'], p['z'], math.radians(p['rotation'])))
        is_cut.append(cut)
    return pts, is_cut


def compensate_points(points, is_cut, e_l, e_n, psi, tool_offset, x_range, y_range):
    """
    把机械臂点修正到实际停靠位姿下（小角度近似）.

    e_l：底盘横向误差（米，沿墙向右为正）；e_n：离墙偏远量（米）；psi：朝向误差（rad，
    逆时针为正）。刀尖在 base 系的横向位置 fy 对应墙面 u ≈ u_base - fx·psi - fy，令其等于
    名义值得 fy' = fy + e_l - fx·psi（fx = 刀尖在 base 系的前伸）。切深处的点离墙偏远则
    多伸出 e_n；退刀点不动。超出关节限位的分量钳制到限位，返回 (新点列, 被钳制点数)。
    """
    (xlo, xhi), (ylo, yhi) = x_range, y_range
    out, clamped = [], 0
    for (x, y, z, rot), cut in zip(points, is_cut):
        fx = x + tool_offset
        y2 = y + e_l - fx * psi
        x2 = x + e_n if cut else x
        y3, x3 = min(max(y2, ylo), yhi), min(max(x2, xlo), xhi)
        clamped += int(abs(y3 - y2) > 1e-9 or abs(x3 - x2) > 1e-9)
        out.append((x3, y3, z, rot))
    return out, clamped


def cut_phases(is_cut):
    """逐点运动的阶段标注：相邻两点都在切深时为 'cut'，否则为 'move'."""
    out, prev = [], False
    for cut in is_cut:
        out.append('cut' if (prev and cut) else 'move')
        prev = cut
    return out


def label_by_time(times, events, default=(-1, 'idle')):
    """
    按时间给样本标注 (停靠站, 阶段)：每个样本取生效时刻不晚于它的最后一个事件.

    events：[(生效时刻, 停靠站, 阶段)]，可以乱序（按时间表提前发布的阶段可能晚于生效时刻
    才送达）；同一时刻的事件保持给出的先后。第一个事件之前的样本取 default。
    """
    order = sorted(range(len(events)), key=lambda k: events[k][0])
    stamps = [events[k][0] for k in order]
    out = []
    for t in times:
        k = int(np.searchsorted(stamps, t, side='right')) - 1
        out.append(default if k < 0 else tuple(events[order[k]][1:3]))
    return out


def residual_is_compensable(e_l, e_n, psi, lateral_max=0.02, standoff_max=0.01,
                            yaw_max=math.radians(1.5)):
    """到站残差是否在机械臂补偿可接受的范围内（横移未按判据收敛时据此决定继续或恢复）."""
    return abs(e_l) <= lateral_max and abs(e_n) <= standoff_max and abs(psi) <= yaw_max
