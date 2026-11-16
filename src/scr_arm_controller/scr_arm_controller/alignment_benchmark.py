#!/usr/bin/env python3
"""对位鲁棒性基准测试.

每次试验：用 Gazebo 服务把机器人传送到墙前的随机起点 → 只运行精确对位阶段 →
用 Gazebo 真值计算最终误差。比较两种对位模式在不同扰动下的表现：
  - odom   ：里程计 + 已知方块坐标（基线）
  - visual ：相机 + 激光融合 PBVS

扰动条件：
  - 里程计漂移：在控制器读到的里程计上叠加固定偏差（模拟长距离导航后的累积误差）
  - 传感器噪声：在方块像素坐标与激光测距上叠加高斯噪声

所有条件使用同一组随机起点（固定随机种子），构成配对比较。

用法（需先启动仿真，例如 ros2 launch scr_description gazebo.launch.py gui:=false）:
  ros2 run scr_arm_controller alignment_benchmark --trials 5 --out results/
"""
import argparse
import csv
import math
import os
import time

import numpy as np
import rclpy
from gazebo_msgs.srv import SetEntityState

from scr_arm_controller.navigation_cutting_task import NavigationCuttingTask

# (名称, 模式, 里程计漂移 (平移幅值 m, 航向幅值 deg), 像素噪声 px, 测距噪声 m, 防碰撞守护)
# 漂移方向每次试验随机（固定种子，两种模式使用相同的方向序列）
CONDITIONS = [
    ('nominal',        'odom',   (0.00, 0.0), 0.0, 0.00, True),
    ('nominal',        'visual', (0.00, 0.0), 0.0, 0.00, True),
    ('odom_drift_3cm', 'odom',   (0.03, 1.0), 0.0, 0.00, True),
    ('odom_drift_3cm', 'visual', (0.03, 1.0), 0.0, 0.00, True),
    ('odom_drift_10cm', 'odom',  (0.10, 3.0), 0.0, 0.00, True),
    ('odom_drift_10cm', 'visual', (0.10, 3.0), 0.0, 0.00, True),
    ('drift_10cm_noguard', 'odom', (0.10, 3.0), 0.0, 0.00, False),   # 消融：关闭守护
    ('sensor_noise_low', 'visual', (0.00, 0.0), 2.0, 0.02, True),
    ('sensor_noise_high', 'visual', (0.00, 0.0), 5.0, 0.05, True),
]

# 墙面内表面 y 坐标（真值，用于碰墙判定）
WALL_Y = -3.925


def teleport(node, client, x, y, yaw, z=0.105):
    req = SetEntityState.Request()
    req.state.name = node.GAZEBO_ENTITY
    req.state.reference_frame = 'world'
    req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z = x, y, z
    req.state.pose.orientation.z = math.sin(yaw / 2)
    req.state.pose.orientation.w = math.cos(yaw / 2)
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    return fut.result() is not None and fut.result().success


def gt_error(node, tx, ty, tyaw):
    gt = node.get_ground_truth_position()
    if gt is None:
        return float('nan'), float('nan')
    pos = math.hypot(tx - gt['x'], ty - gt['y'])
    dyaw = math.atan2(math.sin(tyaw - gt['yaw']), math.cos(tyaw - gt['yaw']))
    return pos, math.degrees(abs(dyaw))


def summarize(rows):
    keys = sorted({(r['condition'], r['mode']) for r in rows},
                  key=lambda k: ([c[0] for c in CONDITIONS].index(k[0]), k[1]))
    out = []
    for cond, mode in keys:
        rs = [r for r in rows if r['condition'] == cond and r['mode'] == mode]
        ok = [r for r in rs if r['success']]
        # 误差统计用全部试验（失败试验也计入其最终真值误差），避免只看成功样本
        pe = np.array([r['pos_err_mm'] for r in rs])
        ae = np.array([r['yaw_err_deg'] for r in rs])
        ts = np.array([r['time_s'] for r in rs])
        out.append({
            'condition': cond, 'mode': mode, 'n': len(rs), 'success': len(ok),
            'collisions': sum(r['collision'] for r in rs),
            'pos_mean_mm': float(np.mean(pe)), 'pos_std_mm': float(np.std(pe)),
            'pos_max_mm': float(np.max(pe)), 'yaw_mean_deg': float(np.mean(ae)),
            'time_mean_s': float(np.mean(ts)),
        })
    return out


def plot(summary, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    conds = [c for c in dict.fromkeys(s['condition'] for s in summary)]
    fig, ax = plt.subplots(figsize=(9, 4.2))
    width = 0.38
    colors = {'odom': '#9AA5B1', 'visual': '#2F6FDE'}
    for i, mode in enumerate(['odom', 'visual']):
        xs, ys, es, notes = [], [], [], []
        for j, c in enumerate(conds):
            s = next((s for s in summary if s['condition'] == c and s['mode'] == mode), None)
            if s is None:
                continue
            xs.append(j + (i - 0.5) * width)
            ys.append(s['pos_mean_mm'])
            es.append(s['pos_std_mm'])
            note = f"{s['success']}/{s['n']} ok"
            if s['collisions']:
                note += f"\n{s['collisions']} hit wall"
            notes.append(note)
        ax.bar(xs, ys, width, yerr=es, capsize=3, color=colors[mode],
               label={'odom': 'odom (baseline)', 'visual': 'visual PBVS'}[mode])
        for x, y, e, note in zip(xs, ys, es, notes):
            ax.text(x, y + e + 1, f'{y:.1f}\n{note}', ha='center', va='bottom', fontsize=7)
    ax.set_xticks(range(len(conds)))
    ax.set_xticklabels(conds, rotation=15)
    ax.set_ylabel('ground-truth position error (mm)')
    ax.set_yscale('symlog', linthresh=20)
    top = max(s['pos_mean_mm'] + s['pos_std_mm'] for s in summary)
    ax.set_ylim(0, top * 4)
    ax.set_title('Marker alignment robustness (mean ± std)')
    ax.legend(frameon=False)
    ax.spines[['top', 'right']].set_visible(False)
    fig.tight_layout()
    fig.savefig(path, dpi=150)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--trials', type=int, default=5, help='每个条件的试验次数')
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--out', default='alignment_results')
    ap.add_argument('--only', nargs='*', help='只运行这些条件名')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    node = NavigationCuttingTask()
    node.distance_from_marker = 0.70
    client = node.create_client(SetEntityState, '/gazebo/set_entity_state')
    if not client.wait_for_service(timeout_sec=20.0):
        node.get_logger().error('找不到 /gazebo/set_entity_state，请确认仿真已启动')
        return

    mx, my = node.marker_position['x'], node.marker_position['y']
    tx, ty, tyaw = mx, my + node.distance_from_marker, -math.pi / 2

    # 同一组随机起点：离墙 1.0–1.5 m、横向 ±0.3 m、朝向 ±10°
    rng = np.random.default_rng(args.seed)
    starts = [(tx + rng.uniform(-0.3, 0.3), ty + rng.uniform(0.3, 0.8),
               tyaw + math.radians(rng.uniform(-10, 10))) for _ in range(args.trials)]
    # 每次试验的漂移方向：平移方向角 + 航向符号
    drift_dirs = [(rng.uniform(0, 2 * math.pi), rng.choice([-1.0, 1.0]))
                  for _ in range(args.trials)]

    rows = []
    csv_path = os.path.join(args.out, 'trials.csv')
    conds = [c for c in CONDITIONS if not args.only or c[0] in args.only]
    for cond, mode, bias, pix, rng_noise, guard in conds:
        for k, (sx, sy, syaw) in enumerate(starts):
            node.positioning_mode = mode
            node.guard_enabled = guard
            ang, sgn = drift_dirs[k]
            node.odom_bias = (bias[0] * math.cos(ang), bias[0] * math.sin(ang),
                              sgn * math.radians(bias[1]))
            node.servo.pixel_noise, node.servo.range_noise = pix, rng_noise
            node.servo.rng = np.random.default_rng(args.seed + k)

            node.stop_robot()
            if not teleport(node, client, sx, sy, syaw):
                node.get_logger().error('传送失败，跳过')
                continue
            node.sleep_sim(1.0)

            t0 = node.get_clock().now()
            wall0 = time.time()
            ok = node.precise_positioning_to_marker()
            dt = (node.get_clock().now() - t0).nanoseconds / 1e9
            node.sleep_sim(0.5)
            pe, ae = gt_error(node, tx, ty, tyaw)
            gt = node.get_ground_truth_position()
            wall_dist = gt['y'] - WALL_Y if gt else float('nan')
            try:
                # 刀具前端（按当前机械臂位姿）到墙面的真值间隙；机器人朝向近似正对墙面
                clearance = wall_dist - node.servo.tool_front_x()
            except Exception:  # noqa: BLE001
                clearance = float('nan')
            row = {'condition': cond, 'mode': mode, 'trial': k, 'success': bool(ok),
                   'pos_err_mm': pe * 1000, 'yaw_err_deg': ae, 'time_s': dt,
                   'wall_dist_m': wall_dist, 'clearance_mm': clearance * 1000,
                   'collision': bool(clearance < 0.0), 'guard': guard,
                   'drift_dx': node.odom_bias[0], 'drift_dy': node.odom_bias[1],
                   'drift_dyaw_deg': math.degrees(node.odom_bias[2]),
                   'wall_s': time.time() - wall0,
                   'start_x': sx, 'start_y': sy, 'start_yaw_deg': math.degrees(syaw)}
            rows.append(row)
            print(f'[{cond:>17s} | {mode:6s} | #{k}] success={ok} '
                  f'err={pe*1000:6.1f} mm {ae:5.2f}° time={dt:5.1f}s '
                  f'clearance={clearance*1000:5.1f}mm'
                  f'{" COLLISION" if row["collision"] else ""}', flush=True)
            with open(csv_path, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)

    summary = summarize(rows)
    with open(os.path.join(args.out, 'summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0].keys()))
        w.writeheader()
        w.writerows(summary)
    print('\n条件               模式     成功  碰墙  位置误差 mean±std (max) mm   角度 mean°  耗时 s')
    for s in summary:
        print(f'{s["condition"]:18s} {s["mode"]:7s} {s["success"]}/{s["n"]}   {s["collisions"]}   '
              f'{s["pos_mean_mm"]:6.1f} ± {s["pos_std_mm"]:4.1f} ({s["pos_max_mm"]:5.1f})'
              f'           {s["yaw_mean_deg"]:5.2f}     {s["time_mean_s"]:5.1f}')
    plot(summary, os.path.join(args.out, 'robustness.png'))
    print(f'\n结果已保存到 {os.path.abspath(args.out)}')
    node.stop_robot()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
