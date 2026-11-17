# -*- coding: utf-8 -*-
"""
离线预览：把与 RViz 相同的 MarkerSpec 画成 PNG（墙面立面图 + 俯视图）.

无显示器 / Wayland 无法截 RViz 时，用它检查 Marker 几何并生成文档插图。

  ros2 run scr_viz plan_preview [--task task.yaml] [--plan plan.yaml] --out preview.png
      [--trace tool_trace.csv]

--trace 可选：execute_slot_plan 输出的 tool_trace.csv（列 u, v 为刀尖墙面坐标）。
"""
import argparse
import csv
import sys

from scr_viz import markers as mk
from scr_viz import wall_model as wm

# 立面图上画的 namespace（其余只在俯视图中有意义）
ELEVATION_NS = ('wall', 'edge_margin', 'reach_band', 'station_windows', 'obstacles', 'margins',
                'endpoints', 'slots', 'conduits', 'unreachable', 'station_window_ids',
                'tool_trace')
TOP_NS = ('wall', 'station_arrows', 'station_ids', 'station_footprints', 'station_order',
          'start', 'planned_tip_path', 'tool_trace')


def _cjk_font():
    from matplotlib import font_manager
    for name in ('Noto Sans CJK SC', 'Noto Serif CJK SC', 'WenQuanYi Zen Hei', 'AR PL UKai CN',
                 'Noto Sans CJK JP'):
        if any(f.name == name for f in font_manager.fontManager.ttflist):
            return name
    return None


def _uv(wall, p):
    u, v, _ = wall.from_map(p[0], p[1], p[2])
    return u, v


def _wall_box_uv(wall, s):
    """贴墙 CUBE → 墙面坐标矩形 (u0, v0, w, h)."""
    u, v = _uv(wall, s.position)
    return u - s.scale[0] / 2, v - s.scale[2] / 2, s.scale[0], s.scale[2]


def _trace_from_csv(path, wall):
    pts = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            u, v = float(row['u']), float(row['v'])
            depth = float(row.get('depth', 0.0) or 0.0)
            pts.append(wall.to_map(u, v, -depth))
    return pts


def render(scene, out, trace_points=None, dpi=130, title=None):
    """画立面图 + 俯视图到 out（PNG）."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyArrow, Polygon, Rectangle

    font = _cjk_font()
    if font:
        plt.rcParams['font.family'] = font
    plt.rcParams['axes.unicode_minus'] = False

    specs = mk.scene_markers(scene, ascii_text=False)
    if trace_points:
        tr = mk.TipTrace(scene.wall, min_step=0.0)
        for p in trace_points:
            tr.add(p)
        specs += tr.markers()
    w = scene.wall
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 9),
                                   gridspec_kw=dict(height_ratios=[1.25, 1]))

    # ---- 立面图（面对墙面：u 向右，v 向上）----
    for s in specs:
        if s.ns not in ELEVATION_NS:
            continue
        c = s.color
        if s.kind == 'cube':
            u0, v0, ww, hh = _wall_box_uv(w, s)
            ax1.add_patch(Rectangle((u0, v0), ww, hh, color=c[:3], alpha=c[3], lw=0))
        elif s.kind == 'line_strip':
            uv = [_uv(w, p) for p in s.points]
            lw = 3.5 if s.ns == 'slots' else max(0.6, s.scale[0] * 150)
            if s.colors:
                for (a, b), cc in zip(zip(uv[:-1], uv[1:]), s.colors[1:]):
                    ax1.plot([a[0], b[0]], [a[1], b[1]], color=cc[:3], lw=1.2)
            else:
                ax1.plot(*zip(*uv), color=c[:3], alpha=c[3], lw=lw,
                         solid_capstyle='butt', ls='--' if s.ns == 'margins' else '-')
        elif s.kind == 'text':
            u, v = _uv(w, s.position)
            ax1.text(u, v, s.text, color=c[:3], fontsize=7, ha='center', va='top')
    for o in scene.obstacles:
        ax1.text(o.u + o.w / 2, o.v + o.h + wm.margin_of(o.kind) + 0.02, o.label,
                 fontsize=7, ha='center', va='bottom', color='#f0c0c0')
    for e in scene.endpoints:
        ax1.text(e.u, e.v + 0.05, e.label, fontsize=7, ha='center', va='bottom',
                 color='#222' if mk.is_existing(e.label) else '#1563c0')
    ax1.set_xlim(-0.05, w.length + 0.05)   # 面对墙面：u 向右
    ax1.set_ylim(-0.05, w.height + 0.05)
    ax1.set_aspect('equal')
    ax1.set_facecolor('#3a3a3a')
    ax1.set_xlabel('u：沿墙向右（m）')
    ax1.set_ylabel('v：离地高度（m）')
    ax1.set_title('墙面立面图（面对墙面）：障碍 / 规范边距（虚线）/ 线盒 / 槽线 / 管线中心线 / 每站可达窗口')

    # ---- 俯视图（map）----
    for s in specs:
        if s.ns not in TOP_NS:
            continue
        c = s.color
        if s.ns == 'wall' and s.kind == 'cube':
            corners = [w.to_map(0, 0, 0), w.to_map(w.length, 0, 0),
                       w.to_map(w.length, 0, -0.075), w.to_map(0, 0, -0.075)]
            ax2.add_patch(Polygon([p[:2] for p in corners], color='#bbb'))
        elif s.kind == 'line_strip' and s.ns != 'wall':
            xs, ys = [p[0] for p in s.points], [p[1] for p in s.points]
            ax2.plot(xs, ys, color=c[:3], alpha=max(c[3], 0.5), lw=1.6 if s.ns ==
                     'station_order' else 1.0)
        elif s.kind == 'arrow':
            import math
            x, y, _ = s.position
            ax2.add_patch(FancyArrow(x, y, 0.12 * math.cos(s.yaw), 0.12 * math.sin(s.yaw),
                                     width=0.012, color=c[:3]))
        elif s.kind == 'text' and s.ns in ('station_ids', 'start'):
            x, y, z = s.position
            ax2.text(x, y + 0.08 + 0.07 * (z > 0.2), s.text, color=c[:3], fontsize=8,
                     ha='center')
        elif s.kind == 'cylinder':
            ax2.plot(s.position[0], s.position[1], 'o', color=c[:3])
    xs = [st.pose[0] for st in scene.stations] + [w.p0[0], w.to_map(w.length)[0]]
    ys = [st.pose[1] for st in scene.stations] + [w.p0[1]]
    if scene.stations:
        cx = (min(xs) + max(xs)) / 2
        span = max(xs) - min(xs)
        ax2.set_xlim(cx - span / 2 - 0.1, cx + span / 2 + 0.1)
        ax2.set_ylim(min(ys) - 0.15, max(ys) + 0.5)
    # 与立面图对齐：站在房间里面对墙面看（墙在上方，u 向右）
    if w.n[1] > 0:
        ax2.invert_xaxis()
        ax2.invert_yaxis()
    ax2.set_aspect('equal')
    ax2.set_xlabel('map x（m）')
    ax2.set_ylabel('map y（m）')
    ax2.set_title('俯视图（map，面对墙面方向）：停靠站位姿、序号、访问顺序（黄线）、底盘轮廓、计划刀尖轨迹（紫）')
    ax2.grid(alpha=0.3)
    if title:
        fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out, dpi=dpi)
    plt.close(fig)
    return out


def main(argv=None):
    """命令行入口."""
    ap = argparse.ArgumentParser(description='离线渲染墙面开槽规划预览图')
    ap.add_argument('--task')
    ap.add_argument('--plan')
    ap.add_argument('--trace')
    ap.add_argument('--out', required=True)
    ap.add_argument('--title')
    a = ap.parse_args(argv)
    scene = wm.load_scene(a.task, a.plan)
    tr = _trace_from_csv(a.trace, scene.wall) if a.trace else None
    render(scene, a.out, tr, title=a.title)
    print(f'已写出 {a.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
