# -*- coding: utf-8 -*-
"""
墙面实体（线盒 / 插座 / 开关 / 配电箱）与演示相机的 SDF 生成（纯函数）.

墙面实体是贴在墙面内表面的静态薄盒，默认只有 visual、没有 collision：
激光（ray 传感器只看 collision）和机械臂都不会碰到它，不影响对位与切割；
需要时可加 collision。
"""
import math
import re
from dataclasses import dataclass
from xml.sax.saxutils import escape

from scr_viz import wall_model as wm

# 种类 → (默认宽, 默认高, 厚度, 面板 RGBA)（米）
FIXTURE_STYLE = {
    'socket': (0.086, 0.086, 0.008, (0.96, 0.96, 0.94, 1.0)),       # 86 型插座面板
    'switch': (0.086, 0.086, 0.008, (0.96, 0.96, 0.94, 1.0)),       # 86 型开关面板
    'junction_box': (0.075, 0.075, 0.004, (0.25, 0.45, 0.85, 1.0)),  # 线盒（未装面板）
    'panelboard': (0.30, 0.40, 0.03, (0.80, 0.80, 0.78, 1.0)),       # 配电箱
    'planned': (0.086, 0.086, 0.002, (0.20, 0.60, 1.00, 0.6)),        # 新装位置（待开）
}
# 标签关键字 → 种类（顺序即优先级）
LABEL_KINDS = (
    ('配电箱', 'panelboard'), ('panel', 'panelboard'),
    ('开关', 'switch'), ('switch', 'switch'),
    ('插座', 'socket'), ('socket', 'socket'),
    ('线盒', 'junction_box'), ('box', 'junction_box'),
)
# 这些标签的物体 world 里已经有了（room.world 的 wall_marker），默认不再生成
DEFAULT_SKIP_LABELS = ('定位标记', 'marker')


@dataclass
class Fixture:
    """一个墙面实体（u, v 为中心，米）."""

    name: str
    kind: str
    u: float
    v: float
    w: float
    h: float
    thickness: float
    rgba: tuple
    label: str = ''


def infer_kind(label, default='junction_box'):
    """按标签关键字推断实体种类."""
    low = (label or '').lower()
    for key, kind in LABEL_KINDS:
        if key in low:
            return kind
    return default


def safe_name(s):
    """Gazebo 实体名只用 ASCII 字母数字和下划线."""
    return re.sub(r'[^A-Za-z0-9_]', '_', s)


def fixtures_from_task(task, skip_labels=DEFAULT_SKIP_LABELS, include_endpoints=True,
                       prefix='scr_fixture'):
    """
    墙面任务 dict → [Fixture].

    - obstacles 中 kind 为 box 的物体：按标签推断种类（配电箱 / 开关 / 插座 / 线盒），尺寸取 YAML；
    - tasks 的起点 / 终点：标签含“已有”的按标签推断种类（插座 / 开关），否则为“新装位置”
      （半透明蓝色，表示规划要开到的线盒位置）。
    - 标签含 skip_labels 任一关键字的不生成（定位标记已在 world 里）。
    """
    out = []

    def skip(label):
        return any(k in (label or '') for k in skip_labels)

    for k, o in enumerate(wm.obstacles_from(task.get('obstacles'))):
        if o.kind != 'box' or skip(o.label):
            continue
        kind = infer_kind(o.label)
        _, _, th, rgba = FIXTURE_STYLE[kind]
        out.append(Fixture(f'{prefix}_obs{k}_{kind}', kind, o.u + o.w / 2, o.v + o.h / 2,
                           o.w, o.h, th, rgba, o.label))
    if include_endpoints:
        tasks = task.get('tasks') or ([task['task']] if 'task' in task else [])
        for e in wm.endpoints_from(tasks):
            if skip(e.label):
                continue
            kind = infer_kind(e.label, 'socket') if '已有' in e.label else 'planned'
            w, h, th, rgba = FIXTURE_STYLE[kind]
            out.append(Fixture(f'{prefix}_slot{e.slot_index}_{e.role}_{kind}', kind, e.u, e.v,
                               w, h, th, rgba, e.label))
    return out


def fixture_pose(wall, f):
    """实体在 world 中的位姿 (x, y, z, roll, pitch, yaw)：盒子背面贴墙面."""
    x, y, z = wall.to_map(f.u, f.v, f.thickness / 2)
    return (x, y, z, 0.0, 0.0, wall.yaw)


def _fmt(vals):
    return ' '.join(f'{v:.6g}' for v in vals)


def _material(rgba):
    c = _fmt(rgba)
    emissive = _fmt((rgba[0] * 0.15, rgba[1] * 0.15, rgba[2] * 0.15, 1.0))
    return (f'<material><ambient>{c}</ambient><diffuse>{c}</diffuse>'
            f'<specular>0.1 0.1 0.1 1</specular><emissive>{emissive}</emissive></material>')


def _face_details(f):
    """面板上的细节（插孔 / 开关键 / 箱门缝），让相机里能认出是什么."""
    dark = (0.15, 0.15, 0.15, 1.0)
    y = -f.thickness / 2 - 0.0005   # 正面（朝房间）在模型 -y 一侧，见 fixture_sdf 的说明
    out = []
    if f.kind == 'socket':
        for dx in (-0.018, 0.018):
            out.append(('hole', (dx, y, 0.0), (0.006, 0.001, 0.014), dark))
    elif f.kind == 'switch':
        out.append(('rocker', (0.0, y, 0.0), (0.05, 0.002, 0.05), (0.88, 0.88, 0.86, 1.0)))
    elif f.kind == 'panelboard':
        out.append(('door_gap', (0.0, y, 0.0), (f.w * 0.9, 0.001, 0.004), dark))
        out.append(('window', (0.0, y, f.h * 0.25), (f.w * 0.6, 0.001, f.h * 0.18),
                    (0.3, 0.35, 0.4, 1.0)))
    return out


def fixture_sdf(wall, f, collision=False, with_pose=True):
    """
    一个墙面实体 → SDF 字符串（静态模型）.

    模型坐标：x 沿墙（u 方向），y 沿墙面法向的反方向（指向墙内），z 向上。
    yaw = wall.yaw 时模型 x 与 t 对齐，模型 -y 方向 = n（朝房间），
    所以面板正面（朝房间）的细节放在 y = -thickness/2 一侧。
    """
    size = _fmt((f.w, f.thickness, f.h))
    geom = f'<geometry><box><size>{size}</size></box></geometry>'
    col = f'<collision name="collision">{geom}</collision>' if collision else ''
    visuals = [f'<visual name="panel">{geom}{_material(f.rgba)}</visual>']
    for name, pos, sz, rgba in _face_details(f):
        visuals.append(f'<visual name="{name}"><pose>{_fmt(pos)} 0 0 0</pose>'
                       f'<geometry><box><size>{_fmt(sz)}</size></box></geometry>'
                       f'{_material(rgba)}</visual>')
    pose = f'<pose>{_fmt(fixture_pose(wall, f))}</pose>' if with_pose else ''
    label = escape(f.label)
    return (f'<?xml version="1.0"?>\n<sdf version="1.6">\n'
            f'  <model name="{f.name}">\n    <!-- {f.kind}: {label} -->\n'
            f'    <static>true</static>\n    {pose}\n'
            f'    <link name="link">{col}{"".join(visuals)}</link>\n'
            f'  </model>\n</sdf>\n')


# ---------------------------------------------------------------------------
# 演示相机
# ---------------------------------------------------------------------------
def look_at_rpy(eye, target):
    """相机在 eye、看向 target 时的 (roll, pitch, yaw)：Gazebo 相机沿模型 +x 看."""
    dx, dy, dz = (target[i] - eye[i] for i in range(3))
    horiz = math.hypot(dx, dy)
    if horiz < 1e-9 and abs(dz) < 1e-9:
        raise ValueError('eye 与 target 重合')
    yaw = math.atan2(dy, dx) if horiz > 1e-9 else 0.0
    pitch = math.atan2(-dz, horiz)        # 绕 y 正转使 +x 朝下
    return (0.0, pitch, yaw)


def rpy_forward(rpy):
    """(roll, pitch, yaw) 下模型 +x 轴在 world 中的方向（用于测试 look_at_rpy）."""
    _, p, y = rpy
    return (math.cos(y) * math.cos(p), math.sin(y) * math.cos(p), -math.sin(p))


# 预设视角（room3 南墙）：eye, target
CAMERA_PRESETS = {
    # 斜后方俯视：能同时看到底盘、机械臂和墙面上的线盒。机器人沿墙在 x ≈ 7.4–8.8 之间
    # 横移时刀具与定位标记都不被立柱遮挡；视野避开 (7, -2) 处的圆柱设备
    'oblique': ((6.3, -2.6, 1.8), (8.1, -3.85, 0.5)),
    # 正对墙面（机器人身后、略高）：看清槽线位置
    'front': ((7.95, -1.3, 1.5), (7.95, -3.92, 0.55)),
    # 东侧近景：从侧面看刀具进给
    'side': ((9.6, -3.0, 1.0), (8.0, -3.8, 0.5)),
    # 高处俯视：看底盘沿墙横移
    'top': ((7.95, -2.6, 3.6), (7.95, -3.4, 0.0)),
    # 住宅俯视（近似正上方，画面上方为北、右方为东）：看导航全程，
    # 地面视野约 10 m × 5.8 m，覆盖起点 (0, 0) 与 room3 南墙
    'overview': ((4.3, -2.3, 7.5), (4.3, -1.5, 0.0)),
}


def camera_topic(name='demo_camera', namespace=None):
    """
    camera_sdf 生成的相机的图像话题.

    gazebo_ros_camera 把图像发布在 <插件命名空间>/<camera_name>/image_raw，
    命名空间默认为 /<name>，所以默认话题是 /<name>/<name>/image_raw。
    """
    return f'{namespace or "/" + name}/{name}/image_raw'


def camera_sdf(name='demo_camera', eye=(0, 0, 1), target=(1, 0, 1), width=640, height=360,
               fps=10.0, hfov=1.2, namespace=None):
    """固定视角相机模型 SDF（静态、无碰撞），图像话题见 camera_topic()."""
    ns = namespace or f'/{name}'
    pose = _fmt(tuple(eye) + look_at_rpy(eye, target))
    return f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <pose>{pose}</pose>
    <link name="link">
      <visual name="body">
        <geometry><box><size>0.06 0.04 0.04</size></box></geometry>
        <transparency>1.0</transparency>
      </visual>
      <sensor type="camera" name="{name}_sensor">
        <always_on>true</always_on>
        <update_rate>{fps:g}</update_rate>
        <visualize>false</visualize>
        <camera name="{name}">
          <horizontal_fov>{hfov:g}</horizontal_fov>
          <image><width>{int(width)}</width><height>{int(height)}</height>
            <format>R8G8B8</format></image>
          <clip><near>0.05</near><far>50</far></clip>
        </camera>
        <plugin name="{name}_plugin" filename="libgazebo_ros_camera.so">
          <ros>
            <namespace>{ns}</namespace>
          </ros>
          <camera_name>{name}</camera_name>
          <frame_name>{name}_link</frame_name>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>
'''
