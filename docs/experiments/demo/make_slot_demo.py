#!/usr/bin/env python3
"""
合成开槽演示动画：Gazebo 演示相机帧 + 墙面立面图（规划几何 + 真值刀尖轨迹）.

  python3 make_slot_demo.py <run_dir> <camera_dir> --task <task.yaml> --plan <plan.yaml>
      --gif slot_demo.gif [--mp4 slot_demo.mp4] [--stills <目录>] [--fps 10]
      [--speed-nav 10 --speed-align 4 --speed-cut 10]

run_dir 为 execute_slot_plan.py 的结果目录（tool_trace.csv[.gz]、summary.yaml）；camera_dir 下
每个视角一个子目录（scr_viz demo_record 的输出：frame_*.jpg + frames.csv，时间戳为仿真时间）。
导航阶段用 overview 视角，到达墙前之后用 oblique 视角。

画面（宽 800 px）：上方为阶段说明；中间为 Gazebo 相机画面；下方为墙面立面图：可切高度带、
每站机械臂可达窗口（彩色竖带，当前站加框）、规划槽线（白）、线盒与定位标记，以及 Gazebo
真值刀尖轨迹（橙红 = 切割段，刀尖停在墙前 10 mm 的空切；青 = 移动 / 退刀）。立面图与
RViz 中 scr_viz plan_viz 的 Marker 使用同一份墙面几何（scr_viz.wall_model），刀尖轨迹为
记录器从 /gazebo/link_states 算得的真值。

各阶段按不同倍速抽帧（仿真时间）。GIF 用 ffmpeg 两遍调色板压缩；MP4 用 libx264。
"""
import argparse
import csv
import gzip
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont

from scr_viz import markers as mk
from scr_viz import wall_model as wm

W = 800                 # 画面宽度（px）
CAPTION_H = 44
CAM_H = 390             # 相机画面高度（800 × 450 的原图裁去上下各 30 px）
WALL_H = 270
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
FONT_BOLD = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
# 定位标记（room.world，ID 0–5）在规划墙面坐标中的 u 与中心高度；地图中的 ID 0 / 2 / 4
TAGS_X = {5: 8.72, 0: 8.42, 1: 8.12, 2: 7.82, 3: 7.52, 4: 7.22}
TAG_V, TAG_SIZE = 0.70, 0.10
MAPPED_TAGS = (0, 2, 4)
MAP_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'src',
                        'scr_navigation', 'maps', 'my_map.yaml')
NAV_GOAL = (8.42, -3.925 + 1.2)     # 导航目标：参考标记正前方 1.2 m（执行器的 Nav2 目标）


def load_trace(run_dir):
    """tool_trace.csv[.gz] → 列数组 t, station, u, v, depth, phase."""
    path = os.path.join(run_dir, 'tool_trace.csv')
    f = open(path, newline='') if os.path.exists(path) else gzip.open(path + '.gz', 'rt')
    with f:
        rows = list(csv.DictReader(f))
    return dict(t=np.array([float(r['t']) for r in rows]),
                station=np.array([int(r['station']) for r in rows]),
                u=np.array([float(r['u']) for r in rows]),
                v=np.array([float(r['v']) for r in rows]),
                depth=np.array([float(r['depth']) for r in rows]),
                phase=np.array([r['phase'] for r in rows]),
                base_u=np.array([float(r['base_u']) for r in rows]),
                base_d=np.array([float(r['base_d']) for r in rows]))


def phase_times(tr):
    """导航开始、精对位开始、第一站开始、结束（仿真时间 s）."""
    t = tr['t']
    i_align = int(np.argmax(tr['phase'] == 'align'))
    i_st = int(np.argmax(tr['station'] >= 0))
    i_move = int(np.argmax((tr['phase'] == 'move') & (t > t[i_align])))
    return dict(nav=float(t[0]), align=float(t[i_align]),
                stations=float(min(t[i_move], t[i_st])), end=float(t[-1]))


def schedule(ev, fps, speed_nav, speed_align, speed_cut, hold_start=1.0, hold_end=2.5):
    """输出帧序号 → (仿真时间, 倍速)."""
    out = [(ev['nav'], speed_nav)] * int(hold_start * fps)
    for a, b, s in ((ev['nav'], ev['align'], speed_nav), (ev['align'], ev['stations'],
                                                          speed_align),
                    (ev['stations'], ev['end'], speed_cut)):
        n = max(1, int(round((b - a) / s * fps)))
        out += [(a + k * (b - a) / n, s) for k in range(n)]
    out += [(ev['end'], speed_cut)] * int(hold_end * fps)
    return out


class CameraFrames:
    """一个视角的帧序列；按仿真时间取最近的一帧."""

    def __init__(self, d):
        with open(os.path.join(d, 'frames.csv'), newline='') as f:
            rows = list(csv.DictReader(f))
        self.dir = d
        self.stamps = np.array([float(r['stamp_s']) for r in rows])
        self.ids = [int(r['frame']) for r in rows]

    def at(self, t):
        i = int(np.clip(np.searchsorted(self.stamps, t, side='right') - 1, 0,
                        len(self.ids) - 1))
        img = Image.open(os.path.join(self.dir, f'frame_{self.ids[i]:05d}.jpg')).convert('RGB')
        if img.width != W:
            img = img.resize((W, round(img.height * W / img.width)), Image.LANCZOS)
        top = (img.height - CAM_H) // 2
        return img.crop((0, top, W, top + CAM_H))


class WallPanel:
    """墙面立面图：静态背景只画一次，每帧更新刀尖轨迹、当前刀尖与当前站."""

    def __init__(self, scene, marker_u):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.collections import LineCollection
        from matplotlib.patches import Rectangle
        self.plt = plt
        self.scene = scene
        dpi = 100
        self.fig = plt.figure(figsize=(W / dpi, WALL_H / dpi), dpi=dpi, facecolor='#2b2b2b')
        ax = self.fig.add_axes([0.055, 0.17, 0.935, 0.80])
        self.ax = ax
        ax.set_facecolor('#4a4a4a')
        lo, hi = scene.reach_band
        wins = [st.window for st in scene.stations]
        u0 = min(w[0] for w in wins) - 0.15
        u1 = max(w[1] for w in wins) + 0.3
        v0, v1 = 0.2, 0.86
        ax.add_patch(Rectangle((u0 - 5, lo), 10, hi - lo, color=(0.35, 0.55, 0.35), alpha=0.35,
                               lw=0))
        self.windows = []
        for k, st in enumerate(scene.stations):
            c = mk.station_rgba(k)[:3]
            ax.add_patch(Rectangle((st.window[0], lo), st.window[1] - st.window[0], hi - lo,
                                   color=c, alpha=0.18, lw=0))
            ax.text((st.window[0] + st.window[1]) / 2, lo - 0.012, str(k + 1), color=c,
                    fontsize=7, ha='center', va='top')
            self.windows.append(st.window)
        for tid, x in TAGS_X.items():
            u = scene.wall.from_map(x, -3.925)[0]
            ax.add_patch(Rectangle((u - TAG_SIZE / 2, TAG_V - TAG_SIZE / 2), TAG_SIZE, TAG_SIZE,
                                   color='#111' if tid in MAPPED_TAGS else '#333',
                                   alpha=0.9 if tid in MAPPED_TAGS else 0.6, lw=0, zorder=2))
            ax.text(u, TAG_V, f'{tid}', color='#ddd', fontsize=6, ha='center', va='center',
                    zorder=3)
        for e in scene.endpoints:
            s = 0.086
            ax.add_patch(Rectangle((e.u - s / 2, e.v - s / 2), s, s, fill=False,
                                   ec='#f5f5f5' if mk.is_existing(e.label) else '#6fb2ff', lw=1.2,
                                   zorder=4))
        for sl in scene.slots:
            if len(sl.cut_path) >= 2:
                uu, vv = zip(*sl.cut_path)
                ax.plot(uu, vv, color='white', alpha=0.55, lw=7, solid_capstyle='butt',
                        zorder=3)
        self.cut_lc = LineCollection([], colors=[(1.0, 0.35, 0.15)], linewidths=1.8, zorder=6)
        self.move_lc = LineCollection([], colors=[(0.3, 0.85, 0.95, 0.55)], linewidths=0.8,
                                      zorder=5)
        ax.add_collection(self.move_lc)
        ax.add_collection(self.cut_lc)
        self.tip, = ax.plot([], [], 'o', ms=6, mfc='yellow', mec='black', zorder=7)
        self.cur = Rectangle((0, lo), 0, hi - lo, fill=False, ec='yellow', lw=1.6, zorder=6)
        ax.add_patch(self.cur)
        ax.set_xlim(u0, u1)
        ax.set_ylim(v0, v1)
        for sp in ax.spines.values():
            sp.set_color('#888')
        ax.tick_params(colors='#ccc', labelsize=7, length=2)
        ax.set_xlabel('u along wall (m)   [u axis drawn about 1.35x the height scale]',
                      color='#ccc', fontsize=7, labelpad=1)
        ax.set_ylabel('height (m)', color='#ccc', fontsize=7, labelpad=1)
        ax.text(0.005, 0.98, 'Wall elevation: plan + ground-truth tool tip',
                transform=ax.transAxes, color='white', fontsize=8, va='top', fontweight='bold')
        ax.text(0.995, 0.98, 'white: planned slot   orange: tip cutting path   cyan: tip moves   '
                'bands: arm window per station', transform=ax.transAxes, color='#ddd',
                fontsize=6.5, va='top', ha='right')

    def render(self, tr, t, station):
        m = tr['t'] <= t
        if m.sum() >= 2:
            u, v = tr['u'][m], tr['v'][m]
            seg = np.stack([np.c_[u[:-1], v[:-1]], np.c_[u[1:], v[1:]]], axis=1)
            near = (np.abs(np.diff(u)) < 0.05) & (np.abs(np.diff(v)) < 0.05)
            cut = (tr['phase'][m][1:] == 'cut') & (tr['depth'][m][1:] > -0.02) & near
            self.cut_lc.set_segments(seg[cut])
            mv = near & ~cut & (tr['station'][m][1:] >= 0)
            self.move_lc.set_segments(seg[mv])
            self.tip.set_data([u[-1]], [v[-1]])
        if 0 <= station < len(self.windows):
            a, b = self.windows[station]
            self.cur.set_x(a)
            self.cur.set_width(b - a)
            self.cur.set_visible(True)
        else:
            self.cur.set_visible(False)
        self.fig.canvas.draw()
        buf = np.asarray(self.fig.canvas.buffer_rgba())[:, :, :3]
        return Image.fromarray(buf.copy())


class MapPanel:
    """导航阶段的下方画面：占据栅格地图 + 真值底盘轨迹 + 导航目标."""

    def __init__(self, wall):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        with open(MAP_YAML, encoding='utf-8') as f:
            meta = yaml.safe_load(f)
        img = np.asarray(Image.open(os.path.join(os.path.dirname(MAP_YAML), meta['image'])))
        res, (ox, oy, _) = meta['resolution'], meta['origin']
        h, w = img.shape[:2]
        self.wall = wall
        dpi = 100
        self.fig = plt.figure(figsize=(W / dpi, WALL_H / dpi), dpi=dpi, facecolor='#2b2b2b')
        ax = self.fig.add_axes([0.02, 0.03, 0.96, 0.94])
        ax.imshow(img, cmap='gray', vmin=0, vmax=255, origin='upper',
                  extent=(ox, ox + w * res, oy, oy + h * res), interpolation='nearest')
        (x0, y0, _), (x1, y1, _) = wall.to_map(0.0), wall.to_map(wall.length)
        ax.plot([x0, x1], [y0, y1], color=(1.0, 0.55, 0.1), lw=3)
        ax.plot(*NAV_GOAL, marker='*', ms=14, mfc='yellow', mec='black')
        ax.text(NAV_GOAL[0] - 0.4, NAV_GOAL[1] - 0.55, 'goal: 1.2 m in front of tag 0',
                color='#c8a000', fontsize=8, ha='center')
        ax.text(wall.p0[0] - 0.1, wall.p0[1] - 0.45, 'target wall (room 3, south)',
                color=(1.0, 0.65, 0.3), fontsize=8, ha='right')
        self.path, = ax.plot([], [], color=(0.2, 0.7, 1.0), lw=2)
        self.robot, = ax.plot([], [], 'o', ms=8, mfc=(0.2, 0.7, 1.0), mec='white')
        ax.set_xlim(-1.2, 10.3)
        ax.set_ylim(-4.4, 1.2)
        ax.set_aspect('equal', adjustable='box')
        ax.axis('off')
        ax.text(0.005, 0.98, 'Map (AMCL / Nav2) + ground-truth base path',
                transform=ax.transAxes, color='white', fontsize=8, va='top', fontweight='bold',
                bbox=dict(facecolor='#2b2b2b', edgecolor='none', alpha=0.8))

    def render(self, tr, t):
        m = tr['t'] <= t
        if m.any():
            u, d = tr['base_u'][m], tr['base_d'][m]
            x = self.wall.p0[0] + u * self.wall.t[0] + d * self.wall.n[0]
            y = self.wall.p0[1] + u * self.wall.t[1] + d * self.wall.n[1]
            self.path.set_data(x, y)
            self.robot.set_data([x[-1]], [y[-1]])
        self.fig.canvas.draw()
        buf = np.asarray(self.fig.canvas.buffer_rgba())[:, :, :3]
        return Image.fromarray(buf.copy())


def caption(text, sub, speed):
    img = Image.new('RGB', (W, CAPTION_H), (24, 24, 28))
    d = ImageDraw.Draw(img)
    d.text((12, 5), text, font=ImageFont.truetype(FONT_BOLD, 17), fill=(255, 255, 255))
    d.text((12, 26), sub, font=ImageFont.truetype(FONT, 12), fill=(200, 200, 200))
    if speed:
        f = ImageFont.truetype(FONT_BOLD, 15)
        s = f'x{speed:g}'
        d.text((W - 12 - d.textlength(s, font=f), 13), s, font=f, fill=(255, 210, 80))
    return img


def phase_text(t, ev, station, n_st):
    if t < ev['align']:
        return ('1  Navigation to the wall', 'Nav2: Smac 2D planner + MPPI controller, '
                'EKF odometry (wheel encoders + gyro) with AMCL')
    if t < ev['stations']:
        return ('2  Fine alignment at the reference tag',
                'AprilTag + LiDAR wall-line EKF, base pose loop to 2 mm / 0.3 deg')
    if t >= ev['end'] - 1e-6:
        return ('Done', '')
    k = max(station, 0) + 1
    return (f'3  Station {k}/{n_st}: strafe, hold, cut',
            'strafe along wall with tag-map EKF, station hold, residual compensation, '
            'single-axis cuts (dry run: tip 10 mm off the wall)')


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('run_dir')
    ap.add_argument('camera_dir')
    ap.add_argument('--task', required=True)
    ap.add_argument('--plan', required=True)
    ap.add_argument('--gif')
    ap.add_argument('--mp4')
    ap.add_argument('--stills')
    ap.add_argument('--fps', type=float, default=10.0)
    ap.add_argument('--speed-nav', type=float, default=10.0)
    ap.add_argument('--speed-align', type=float, default=4.0)
    ap.add_argument('--speed-cut', type=float, default=10.0)
    ap.add_argument('--overview', default='overview')
    ap.add_argument('--close', default='oblique')
    ap.add_argument('--frames-dir', help='保留中间帧（默认用临时目录）')
    a = ap.parse_args()

    scene = wm.load_scene(a.task, a.plan)
    with open(a.plan, encoding='utf-8') as f:
        plan = yaml.safe_load(f)
    with open(os.path.join(a.run_dir, 'summary.yaml'), encoding='utf-8') as f:
        summ = yaml.safe_load(f)
    tr = load_trace(a.run_dir)
    ev = phase_times(tr)
    print('阶段（仿真时间 s）：', {k: round(v, 1) for k, v in ev.items()})
    cams = {k: CameraFrames(os.path.join(a.camera_dir, k)) for k in (a.overview, a.close)
            if os.path.isdir(os.path.join(a.camera_dir, k))}
    panel = WallPanel(scene, plan.get('marker_u'))
    nav_panel = MapPanel(scene.wall)
    sched = schedule(ev, a.fps, a.speed_nav, a.speed_align, a.speed_cut)
    n_st = len(scene.stations)
    # 站间移动时记录为 −1：沿用上一个停靠站，标题与当前站框不跳回第 1 站
    last_station = np.maximum.accumulate(np.where(tr['station'] >= 0,
                                                  np.arange(len(tr['station'])), -1))
    last_station = np.where(last_station >= 0, tr['station'][np.maximum(last_station, 0)], -1)
    tmp = a.frames_dir or tempfile.mkdtemp(prefix='slot_demo_')
    os.makedirs(tmp, exist_ok=True)
    stills = {}
    want = {'nav': ev['nav'] + 0.55 * (ev['align'] - ev['nav']),
            'align': ev['align'] + 0.6 * (ev['stations'] - ev['align']),
            'cut': ev['stations'] + 0.62 * (ev['end'] - ev['stations'])}
    for i, (t, speed) in enumerate(sched):
        k = int(np.searchsorted(tr['t'], t, side='right')) - 1
        station = int(last_station[max(k, 0)])
        view = a.overview if (t < ev['align'] - 2.0 and a.overview in cams) else a.close
        cam = cams[view].at(t)
        title, sub = phase_text(t, ev, station, n_st)
        if title == 'Done':
            title = (f'Done: {n_st} stations, {summ["slot_length_m"]:.2f} m of slot in '
                     f'{summ["sim_time_s"]:.0f} s (simulated time)')
            sub = (f'tool path error mean {summ["path_err_mean_mm"]:.2f} mm, '
                   f'p95 {summ["path_err_p95_mm"]:.2f} mm; coverage '
                   f'{100 * summ["coverage"]:.0f} %; stations total '
                   f'{summ["stations_total_s"]:.0f} s vs predicted '
                   f'{plan["prediction"]["t_total"]:.0f} s')
            speed = 0
        frame = Image.new('RGB', (W, CAPTION_H + CAM_H + WALL_H))
        frame.paste(caption(title, sub, speed), (0, 0))
        frame.paste(cam, (0, CAPTION_H))
        d = ImageDraw.Draw(frame)
        d.text((W - 8, CAPTION_H + 6), f'Gazebo  t = {t - ev["nav"]:5.0f} s', anchor='ra',
               font=ImageFont.truetype(FONT, 12), fill=(255, 255, 255))
        lower = (nav_panel.render(tr, t) if t < ev['align'] - 2.0 else
                 panel.render(tr, t, station))
        frame.paste(lower, (0, CAPTION_H + CAM_H))
        frame.save(os.path.join(tmp, f'f_{i:05d}.png'))
        for name, tw in list(want.items()):
            if t >= tw:
                stills[name] = frame.copy()
                del want[name]
        if i % 50 == 0:
            print(f'{i}/{len(sched)}', flush=True)
    if a.stills:
        os.makedirs(a.stills, exist_ok=True)
        for name, img in stills.items():
            p = os.path.join(a.stills, f'demo_{name}.png')
            img.save(p, optimize=True)
            print('still', p)
    pattern = os.path.join(tmp, 'f_%05d.png')
    if a.mp4:
        subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', f'{a.fps:g}',
                        '-i', pattern, '-c:v', 'libx264', '-preset', 'slow', '-crf', '20',
                        '-pix_fmt', 'yuv420p', '-movflags', '+faststart', a.mp4], check=True)
        print(f'{a.mp4}: {os.path.getsize(a.mp4) / 1e6:.1f} MB')
    if a.gif:
        pal = os.path.join(tmp, 'palette.png')
        subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', f'{a.fps:g}',
                        '-i', pattern, '-vf', 'palettegen=max_colors=128:stats_mode=diff',
                        pal], check=True)
        subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', f'{a.fps:g}',
                        '-i', pattern, '-i', pal, '-lavfi',
                        'paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle',
                        '-loop', '0', a.gif], check=True)
        print(f'{a.gif}: {os.path.getsize(a.gif) / 1e6:.1f} MB, {len(sched)} 帧, '
              f'{len(sched) / a.fps:.1f} s')
    if not a.frames_dir:
        shutil.rmtree(tmp, ignore_errors=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
