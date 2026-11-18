#!/usr/bin/env python3
r"""
精对位鲁棒性基准测试.

每次试验：用 Gazebo 服务把机器人传送到墙前的随机起点 → 只运行精对位阶段 →
用 Gazebo 真值计算最终误差（法向 / 沿墙 / 朝向分开记录）。比较的流水线：
  v1        第一版：色块 + 激光，指数平滑，比例 + 最小速度 + 死区，1 cm / 1° 停止
  v2        AprilTag + 激光 → EKF → 平方根整形饱和律 → “容差内且静止”收敛判据
  v2_blob   v2 的估计与控制 + 色块检测器（消融：区分检测器与估计 / 控制的贡献）
  v2_taught v2 + 零位标定偏置（--calib 给出的 teach_alignment_offset 输出）

扰动条件：
  里程计漂移    控制器读到的里程计位姿叠加固定偏差（v2 用里程计增量做预测，偏差中的
                航向分量会使增量方向偏转）
  传感器噪声    标记像素坐标（色块质心 / 四个角点）与激光测距叠加高斯噪声
  光照变化      打开墙前工作灯（世界文件中默认关闭），/gazebo/set_light_properties 设置：
                light_strong 强白光（画面平均灰度 80 → 175）；light_cool 冷白光
                （RGB 0.85 / 1.0 / 0.9，荧光灯类偏绿光谱，平均灰度 → 127）
  遮挡          墙前 1 cm 处一根 20 mm 宽的竖直遮挡条，盖住两种标记靠一侧约 30 %：
                static 全程遮挡；intermittent 每 3 s 中遮挡 1.5 s
  相机安装偏角  仿真以 camera_yaw_error_deg:=1.0 启动（TF 不变），须用 --camera-yaw-deg 1.0 声明

所有条件使用同一组随机起点（固定随机种子），构成配对比较。

用法（先启动仿真：ros2 launch scr_description gazebo.launch.py gui:=false rtf:=2.5）:
  ros2 run scr_arm_controller alignment_benchmark --trials 8 --out results/
  ros2 run scr_arm_controller alignment_benchmark --trials 8 --out results_cam/ \\
      --camera-yaw-deg 1.0 --calib taught.yaml
"""
import argparse
import csv
import math
import os
import threading
import time

import numpy as np
import rclpy
import yaml
from gazebo_msgs.srv import (DeleteEntity, GetLightProperties, SetEntityState,
                             SetLightProperties, SpawnEntity)
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from scr_arm_controller.task_skills import CuttingSkills as NavigationCuttingTask

PIPELINES = {
    'v1': dict(mode='visual_v1'),
    'v2': dict(mode='visual', detector='apriltag'),
    'v2_blob': dict(mode='visual', detector='blob'),
    'v2_taught': dict(mode='visual', detector='apriltag', calib=True),
}

# (条件名, 流水线, 里程计漂移 (平移 m, 航向 deg), 像素噪声 px, 测距噪声 m, 场景, 相机偏角 deg)
CONDITIONS = [
    ('nominal', 'v1', (0.00, 0.0), 0.0, 0.00, None, 0.0),
    ('nominal', 'v2', (0.00, 0.0), 0.0, 0.00, None, 0.0),
    ('nominal', 'v2_blob', (0.00, 0.0), 0.0, 0.00, None, 0.0),
    ('nominal', 'v2_taught', (0.00, 0.0), 0.0, 0.00, None, 0.0),
    ('odom_drift_3cm', 'v1', (0.03, 1.0), 0.0, 0.00, None, 0.0),
    ('odom_drift_3cm', 'v2', (0.03, 1.0), 0.0, 0.00, None, 0.0),
    ('odom_drift_10cm', 'v1', (0.10, 3.0), 0.0, 0.00, None, 0.0),
    ('odom_drift_10cm', 'v2', (0.10, 3.0), 0.0, 0.00, None, 0.0),
    ('sensor_noise_low', 'v1', (0.00, 0.0), 2.0, 0.02, None, 0.0),
    ('sensor_noise_low', 'v2', (0.00, 0.0), 2.0, 0.02, None, 0.0),
    ('sensor_noise_high', 'v1', (0.00, 0.0), 5.0, 0.05, None, 0.0),
    ('sensor_noise_high', 'v2', (0.00, 0.0), 5.0, 0.05, None, 0.0),
    ('light_strong', 'v1', (0.00, 0.0), 0.0, 0.00, 'light_strong', 0.0),
    ('light_strong', 'v2', (0.00, 0.0), 0.0, 0.00, 'light_strong', 0.0),
    ('light_cool', 'v1', (0.00, 0.0), 0.0, 0.00, 'light_cool', 0.0),
    ('light_cool', 'v2', (0.00, 0.0), 0.0, 0.00, 'light_cool', 0.0),
    ('occlusion_intermittent', 'v1', (0.00, 0.0), 0.0, 0.00, 'occl_toggle', 0.0),
    ('occlusion_intermittent', 'v2', (0.00, 0.0), 0.0, 0.00, 'occl_toggle', 0.0),
    ('occlusion_static', 'v1', (0.00, 0.0), 0.0, 0.00, 'occl_static', 0.0),
    ('occlusion_static', 'v2', (0.00, 0.0), 0.0, 0.00, 'occl_static', 0.0),
    ('camera_yaw_1deg', 'v1', (0.00, 0.0), 0.0, 0.00, None, 1.0),
    ('camera_yaw_1deg', 'v2', (0.00, 0.0), 0.0, 0.00, None, 1.0),
    ('camera_yaw_1deg', 'v2_taught', (0.00, 0.0), 0.0, 0.00, None, 1.0),
]
COND_ORDER = list(dict.fromkeys(c[0] for c in CONDITIONS))

# 墙面内表面 y 坐标与定位标记沿墙位置（真值）
WALL_Y = -3.925
TAG_X = 8.42
DISTANCE = 0.70

# 工作灯设置：(RGB, 衰减 (constant, linear, quadratic))
LIGHT_SCENES = {
    'light_strong': ((1.0, 1.0, 1.0), (0.1, 0.0, 0.15)),
    'light_cool': ((0.85, 1.0, 0.9), (0.2, 0.0, 0.3)),
}

OCCLUDER_SDF = """<?xml version="1.0"?>
<sdf version="1.6"><model name="{name}"><static>true</static><link name="link">
<visual name="v"><geometry><box><size>0.02 0.004 0.40</size></box></geometry>
<material><ambient>0.25 0.25 0.28 1</ambient><diffuse>0.25 0.25 0.28 1</diffuse></material>
</visual></link></model></sdf>"""
OCCLUDER_ON = (TAG_X + 0.02, WALL_Y + 0.012, 0.66)    # 盖住两种标记 +x 一侧约 30 %
OCCLUDER_OFF = (TAG_X + 0.02, WALL_Y + 0.012, -1.0)   # 移到地面以下


def call(node, client, req, timeout=5.0):
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return fut.result()


def teleport(node, client, x, y, yaw, z=0.105, name=None):
    req = SetEntityState.Request()
    req.state.name = name or node.GAZEBO_ENTITY
    req.state.reference_frame = 'world'
    req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z = x, y, z
    req.state.pose.orientation.z = math.sin(yaw / 2)
    req.state.pose.orientation.w = math.cos(yaw / 2)
    res = call(node, client, req)
    return res is not None and res.success


def wall_state_true(gt):
    """Gazebo 真值 → 墙面相对状态 (d, u, ψ)，与 WallEKF 的约定一致."""
    yaw = gt['yaw'] + math.pi / 2
    return (gt['y'] - WALL_Y, -(gt['x'] - TAG_X), math.atan2(math.sin(yaw), math.cos(yaw)))


class Occluder:
    """遮挡条：生成 / 删除，以及按仿真时间间歇移入移出（独立节点 + 后台线程）."""

    NAME = 'benchmark_occluder'

    def __init__(self, node):
        self.node = node
        self.spawn_cli = node.create_client(SpawnEntity, '/spawn_entity')
        self.delete_cli = node.create_client(DeleteEntity, '/delete_entity')
        self.driver = rclpy.create_node('occluder_driver', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.set_cli = self.driver.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.period, self.duty = 3.0, 0.5
        self.toggling = False
        self._t0 = None
        self._state = None
        self.driver.create_timer(0.05, self._tick)
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self.driver)
        self._thread = threading.Thread(target=self._exec.spin, daemon=True)
        self._thread.start()

    def spawn(self, on=True):
        self.spawn_cli.wait_for_service(timeout_sec=5.0)
        req = SpawnEntity.Request()
        req.name = self.NAME
        req.xml = OCCLUDER_SDF.format(name=self.NAME)
        x, y, z = OCCLUDER_ON if on else OCCLUDER_OFF
        req.initial_pose.position.x, req.initial_pose.position.y = x, y
        req.initial_pose.position.z = z
        req.reference_frame = 'world'
        res = call(self.node, self.spawn_cli, req)
        self._state = on
        return res is not None and res.success

    def delete(self):
        self.toggling = False
        req = DeleteEntity.Request()
        req.name = self.NAME
        call(self.node, self.delete_cli, req)

    def start_toggle(self):
        """从“不遮挡”开始，每 period 秒中后 duty 比例的时间遮挡."""
        self._t0 = None
        self.toggling = True

    def stop_toggle(self):
        self.toggling = False

    def _move(self, on):
        req = SetEntityState.Request()
        req.state.name = self.NAME
        req.state.reference_frame = 'world'
        x, y, z = OCCLUDER_ON if on else OCCLUDER_OFF
        req.state.pose.position.x, req.state.pose.position.y = x, y
        req.state.pose.position.z = z
        req.state.pose.orientation.w = 1.0
        self.set_cli.call_async(req)
        self._state = on

    def _tick(self):
        if not self.toggling:
            return
        t = self.driver.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = t
        on = ((t - self._t0) % self.period) >= self.period * (1 - self.duty)
        if on != self._state:
            self._move(on)

    def shutdown(self):
        self._exec.shutdown()
        self._thread.join(timeout=2.0)
        self.driver.destroy_node()


class Lighting:
    def __init__(self, node, light='work_light'):
        self.node, self.light = node, light
        self.get_cli = node.create_client(GetLightProperties, '/gazebo/get_light_properties')
        self.set_cli = node.create_client(SetLightProperties, '/gazebo/set_light_properties')
        self.default = None

    def _get(self):
        if not self.get_cli.wait_for_service(timeout_sec=5.0):
            return None
        req = GetLightProperties.Request()
        req.light_name = self.light
        return call(self.node, self.get_cli, req)

    def set(self, rgb=None, attenuation=None):
        """设置光源颜色 rgb（0–1）与衰减 (constant, linear, quadratic)；rgb=None 恢复默认."""
        if self.default is None:
            self.default = self._get()
            if self.default is None:
                raise RuntimeError('没有 /gazebo/get_light_properties（世界文件需加载 '
                                   'gazebo_ros_properties 插件）')
        d = self.default
        req = SetLightProperties.Request()
        req.light_name = self.light
        if rgb is None:
            req.diffuse = d.diffuse
            attenuation = None
        else:
            req.diffuse.r, req.diffuse.g, req.diffuse.b = (float(c) for c in rgb)
            req.diffuse.a = 1.0
        if attenuation is None:
            attenuation = (d.attenuation_constant, d.attenuation_linear, d.attenuation_quadratic)
        (req.attenuation_constant, req.attenuation_linear,
         req.attenuation_quadratic) = (float(a) for a in attenuation)
        res = call(self.node, self.set_cli, req)
        return res is not None and res.success


def summarize(rows):
    keys = sorted({(r['condition'], r['pipeline']) for r in rows},
                  key=lambda k: (COND_ORDER.index(k[0]), list(PIPELINES).index(k[1])))
    out = []
    for cond, pipe in keys:
        rs = [r for r in rows if r['condition'] == cond and r['pipeline'] == pipe]
        ok = [r for r in rs if r['success']]
        # 误差统计用全部试验（失败试验也计入其最终真值误差），避免只看成功样本
        pe = np.array([r['pos_err_mm'] for r in rs])
        out.append({
            'condition': cond, 'pipeline': pipe, 'n': len(rs), 'success': len(ok),
            'collisions': sum(r['collision'] for r in rs),
            'pos_mean_mm': float(np.mean(pe)), 'pos_std_mm': float(np.std(pe)),
            'pos_max_mm': float(np.max(pe)),
            'lateral_mean_mm': float(np.mean([r['lateral_err_mm'] for r in rs])),
            'lateral_absmax_mm': float(np.max([abs(r['lateral_err_mm']) for r in rs])),
            'normal_mean_mm': float(np.mean([r['normal_err_mm'] for r in rs])),
            'yaw_abs_mean_deg': float(np.mean([abs(r['yaw_err_deg']) for r in rs])),
            'min_clearance_mm': float(np.min([r['clearance_mm'] for r in rs])),
            'after1s_pos_mean_mm': float(np.mean([r['after1s_pos_err_mm'] for r in rs])),
            'time_mean_s': float(np.mean([r['time_s'] for r in rs])),
        })
    return out


def plot(summary, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['font.family'] = ['Noto Sans CJK JP', 'DejaVu Sans']

    conds = [c for c in COND_ORDER if any(s['condition'] == c for s in summary)]
    pipes = [p for p in PIPELINES if any(s['pipeline'] == p for s in summary)]
    colors = {'v1': '#9AA5B1', 'v2': '#2F6FDE', 'v2_blob': '#7FB069', 'v2_taught': '#D9480F'}
    labels = {'v1': '第一版（色块 + 平滑 + 比例/死区）', 'v2': '本版（AprilTag + EKF + 平方根律）',
              'v2_blob': '本版估计与控制 + 色块', 'v2_taught': '本版 + 示教零位'}
    top = 16.0          # 纵轴上限（mm）；全部失败的条件不画柱高，只标注
    fig, ax = plt.subplots(figsize=(11, 4.4))
    width = 0.8 / len(pipes)
    labelled = set()
    for i, p in enumerate(pipes):
        for j, c in enumerate(conds):
            s = next((s for s in summary if s['condition'] == c and s['pipeline'] == p), None)
            if s is None:
                continue
            x = j + (i - (len(pipes) - 1) / 2) * width
            lab = None if p in labelled else labels[p]
            labelled.add(p)
            if s['success'] == 0:
                ax.bar(x, top, width, color='none', edgecolor=colors[p], hatch='///', label=lab)
                ax.text(x, top * 0.55, f"失败\n0/{s['n']}", ha='center', va='center', fontsize=7)
                continue
            ax.bar(x, s['pos_mean_mm'], width, yerr=s['pos_std_mm'], capsize=2,
                   color=colors[p], label=lab)
            note = f"{s['pos_mean_mm']:.1f}"
            if s['success'] < s['n']:
                note += f"\n{s['success']}/{s['n']}"
            ax.text(x, min(s['pos_mean_mm'] + s['pos_std_mm'] + 0.2, top - 1), note,
                    ha='center', va='bottom', fontsize=6.5)
    ax.set_xticks(range(len(conds)))
    ax.set_xticklabels(conds, rotation=18, fontsize=8)
    ax.set_ylabel('真值位置误差（mm，均值 ± 标准差，n = 8）')
    ax.set_ylim(0, top)
    ax.legend(frameon=False, fontsize=8, ncol=2, loc='upper left')
    ax.spines[['top', 'right']].set_visible(False)
    fig.tight_layout()
    fig.savefig(path, dpi=150)


def load_calib(path):
    with open(path, encoding='utf-8') as f:
        y = yaml.safe_load(f)
    # 新生成的标定文件以 /** 为键（任意节点名），早期文件以 navigation_cutting_task 为键
    a = (y.get('/**') or y['navigation_cutting_task'])['ros__parameters']['alignment']
    return {k: float(a[k]) for k in ('offset_normal', 'offset_lateral', 'offset_yaw_deg')}


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('--trials', type=int, default=8, help='每个条件的试验次数')
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--out', default='alignment_results')
    ap.add_argument('--only', nargs='*', help='只运行这些条件名')
    ap.add_argument('--pipelines', nargs='*', help='只运行这些流水线')
    ap.add_argument('--camera-yaw-deg', type=float, default=0.0,
                    help='仿真启动时的 camera_yaw_error_deg（只运行与之匹配的条件）')
    ap.add_argument('--calib', help='零位标定 YAML（v2_taught 使用）')
    ap.add_argument('--traces', action='store_true', help='保存 v2 逐周期估计记录')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    node = NavigationCuttingTask()
    node.distance_from_marker = DISTANCE
    client = node.create_client(SetEntityState, '/gazebo/set_entity_state')
    if not client.wait_for_service(timeout_sec=20.0):
        node.get_logger().error('找不到 /gazebo/set_entity_state，请确认仿真已启动')
        return
    occluder = Occluder(node)
    lighting = Lighting(node)
    calib = load_calib(args.calib) if args.calib else None
    zero = dict(offset_normal=0.0, offset_lateral=0.0, offset_yaw_deg=0.0)

    tx, ty, tyaw = TAG_X, WALL_Y + DISTANCE, -math.pi / 2

    # 同一组随机起点：离墙 1.0–1.5 m、横向 ±0.3 m、朝向 ±10°
    rng = np.random.default_rng(args.seed)
    starts = [(tx + rng.uniform(-0.3, 0.3), ty + rng.uniform(0.3, 0.8),
               tyaw + math.radians(rng.uniform(-10, 10))) for _ in range(args.trials)]
    # 每次试验的漂移方向：平移方向角 + 航向符号
    drift_dirs = [(rng.uniform(0, 2 * math.pi), rng.choice([-1.0, 1.0]))
                  for _ in range(args.trials)]

    rows, traces = [], []
    csv_path = os.path.join(args.out, 'trials.csv')
    conds = [c for c in CONDITIONS
             if (not args.only or c[0] in args.only)
             and (not args.pipelines or c[1] in args.pipelines)
             and abs(c[6] - args.camera_yaw_deg) < 1e-9
             and (calib is not None or not PIPELINES[c[1]].get('calib'))]
    scene_now = None
    for cond, pipe, bias, pix, rng_noise, scene, _ in conds:
        # ---- 场景
        if scene != scene_now:
            if scene_now in LIGHT_SCENES:
                lighting.set(None)
            if scene_now in ('occl_static', 'occl_toggle'):
                occluder.delete()
            if scene in LIGHT_SCENES:
                lighting.set(*LIGHT_SCENES[scene])
            if scene in ('occl_static', 'occl_toggle'):
                occluder.spawn(on=(scene == 'occl_static'))
            scene_now = scene
            node.sleep_sim(1.0)
        cfg = PIPELINES[pipe]
        node.positioning_mode = cfg['mode']
        node.apply_alignment_config(detector=cfg.get('detector', 'apriltag'),
                                    **(calib if cfg.get('calib') else zero))
        for k, (sx, sy, syaw) in enumerate(starts):
            ang, sgn = drift_dirs[k]
            node.odom_bias = (bias[0] * math.cos(ang), bias[0] * math.sin(ang),
                              sgn * math.radians(bias[1]))
            node.estimator.odom_bias = node.odom_bias
            node.servo.pixel_noise, node.servo.range_noise = pix, rng_noise
            node.servo.rng = np.random.default_rng(args.seed + k)

            occluder.stop_toggle()
            node.base_hold.release()
            node.stop_robot()
            if not teleport(node, client, sx, sy, syaw):
                node.get_logger().error('传送失败，跳过')
                continue
            node.sleep_sim(1.0)
            if scene == 'occl_toggle':
                occluder.start_toggle()

            t0 = node.get_clock().now()
            wall0 = time.time()
            ok = node.precise_positioning_to_marker()
            dt = (node.get_clock().now() - t0).nanoseconds / 1e9
            occluder.stop_toggle()
            # 主指标：控制器宣布到位时刻的真值（失败试验取结束时刻）。之后底盘在零速度指令下
            # 会缓慢滑移（仿真轮子摩擦为 0），v2 到位后进入定点保持，再取 1 s 后的真值作参考
            gt = node.align_gt or node.get_ground_truth_position()
            d_t, u_t, psi_t = wall_state_true(gt)
            node.sleep_sim(1.0)
            gt1 = node.get_ground_truth_position()
            d_1, u_1, _ = wall_state_true(gt1)
            node.base_hold.release()
            try:
                # 刀尖（按当前机械臂位姿）到墙面的真值间隙；机器人朝向近似正对墙面
                clearance = d_1 - node.servo.tool_front_x()
            except Exception:  # noqa: BLE001
                clearance = float('nan')
            row = {'condition': cond, 'pipeline': pipe, 'trial': k, 'success': bool(ok),
                   'pos_err_mm': math.hypot(d_t - DISTANCE, u_t) * 1000,
                   'normal_err_mm': (d_t - DISTANCE) * 1000, 'lateral_err_mm': u_t * 1000,
                   'yaw_err_deg': math.degrees(psi_t), 'time_s': dt,
                   'after1s_pos_err_mm': math.hypot(d_1 - DISTANCE, u_1) * 1000,
                   'after1s_lateral_err_mm': u_1 * 1000,
                   'clearance_mm': clearance * 1000, 'collision': bool(clearance < 0.0),
                   'drift_dx': node.odom_bias[0], 'drift_dy': node.odom_bias[1],
                   'drift_dyaw_deg': math.degrees(node.odom_bias[2]),
                   'camera_yaw_deg': args.camera_yaw_deg,
                   'offset_lateral_mm': 1000 * node.align_cfg['offset_lateral'],
                   'wall_s': time.time() - wall0,
                   'start_x': sx, 'start_y': sy, 'start_yaw_deg': math.degrees(syaw)}
            X = node.estimator.state() if cfg['mode'] == 'visual' else None
            if X is not None:
                row.update(est_normal_err_mm=(X[0] - d_t) * 1000,
                           est_lateral_err_mm=(X[1] - u_t) * 1000,
                           est_yaw_err_deg=math.degrees(X[2] - psi_t),
                           tag_accepted=node.estimator.ekf.stats['tag'][0],
                           tag_rejected=node.estimator.ekf.stats['tag'][1])
                if args.traces:
                    traces += [(cond, pipe, k, *r) for r in node.align_trace]
            rows.append(row)
            print(f'[{cond:>22s} | {pipe:9s} | #{k}] success={ok} '
                  f'pos={row["pos_err_mm"]:6.2f} mm (n {row["normal_err_mm"]:+6.2f}, '
                  f'u {row["lateral_err_mm"]:+6.2f}) yaw={row["yaw_err_deg"]:+5.2f}° '
                  f'time={dt:5.1f}s clearance={clearance*1000:5.1f}mm'
                  f'{" COLLISION" if row["collision"] else ""}', flush=True)
            fields = list(dict.fromkeys(k for r in rows for k in r))
            with open(csv_path, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=fields)
                w.writeheader()
                w.writerows(rows)
    if scene_now in LIGHT_SCENES:
        lighting.set(None)
    if scene_now in ('occl_static', 'occl_toggle'):
        occluder.delete()

    if traces:
        with open(os.path.join(args.out, 'traces.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['condition', 'pipeline', 'trial', 't', 'd', 'u', 'psi', 'sigma_d',
                        'sigma_u', 'sigma_psi', 'vx', 'vy', 'wz', 'tag_ok', 'wall_ok'])
            w.writerows(traces)
    if rows:
        summary = summarize(rows)
        with open(os.path.join(args.out, 'summary.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(summary[0].keys()))
            w.writeheader()
            w.writerows(summary)
        print('\n条件                    流水线     成功  位置 mean±std (max) mm  沿墙均值 mm  '
              '角度 mean°  耗时 s')
        for s in summary:
            print(f'{s["condition"]:23s} {s["pipeline"]:10s} {s["success"]}/{s["n"]}  '
                  f'{s["pos_mean_mm"]:6.2f} ± {s["pos_std_mm"]:4.2f} ({s["pos_max_mm"]:5.2f})  '
                  f'{s["lateral_mean_mm"]:+6.2f}      {s["yaw_abs_mean_deg"]:5.2f}     '
                  f'{s["time_mean_s"]:5.1f}')
        plot(summary, os.path.join(args.out, 'robustness.png'))
    print(f'\n结果已保存到 {os.path.abspath(args.out)}')
    node.stop_robot()
    occluder.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
