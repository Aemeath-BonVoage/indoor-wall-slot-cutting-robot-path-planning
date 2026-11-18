#!/usr/bin/env python3
"""
底盘特性测试：按固定速度指令序列驱动底盘，用真值评估跟踪与里程计.

每段指令保持 hold 秒，取后半段的真值机体速度均值与指令之比（跟踪比，1.0 为理想；
旋转段即早期“旋转失真”检查的实际 / 指令转速比）。全程记录里程计（/odom，及可选的
/wheel/odom）相对真值的漂移，以及仿真实时率与 gzserver CPU 占用。

  ros2 run scr_base_control base_characterization.py --out <目录> --label force
只需 Gazebo + 底盘控制器（scr_description/gazebo.launch.py），不要启动 Nav2。
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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

# (名称, vx, vy, wz)：平移、横移、旋转、组合
SEGMENTS = [
    ('forward', 0.3, 0.0, 0.0), ('backward', -0.3, 0.0, 0.0),
    ('left', 0.0, 0.3, 0.0), ('right', 0.0, -0.3, 0.0),
    ('rotate_ccw', 0.0, 0.0, 0.6), ('rotate_cw', 0.0, 0.0, -0.6),
    ('diagonal', 0.2, 0.2, 0.0), ('arc', 0.25, 0.0, 0.4), ('strafe_turn', 0.0, 0.2, 0.4),
    ('slow_strafe', 0.0, 0.05, 0.0),
]


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _pose(msg):
    p = msg.pose.pose
    return (p.position.x, p.position.y, _yaw(p.orientation))


def align(ref0, est0, est):
    """把估计轨迹按起点对齐到真值：T = ref0 ∘ est0⁻¹."""
    dyaw = ref0[2] - est0[2]
    c, s = math.cos(dyaw), math.sin(dyaw)
    dx, dy = est[0] - est0[0], est[1] - est0[1]
    return (ref0[0] + c * dx - s * dy, ref0[1] + s * dx + c * dy, est[2] + dyaw)


def gz_cpu():
    """gzserver 累计 CPU 时间（秒）."""
    for p in os.listdir('/proc'):
        if p.isdigit():
            try:
                with open(f'/proc/{p}/comm') as f:
                    if f.read().strip() != 'gzserver':
                        continue
                with open(f'/proc/{p}/environ', 'rb') as f:
                    if b'ROS_DOMAIN_ID=' + os.environ.get('ROS_DOMAIN_ID', '0').encode() \
                            not in f.read().split(b'\0'):
                        continue
                with open(f'/proc/{p}/stat') as f:
                    fl = f.read().rsplit(')', 1)[1].split()
                return (int(fl[11]) + int(fl[12])) / os.sysconf('SC_CLK_TCK')
            except OSError:
                continue
    return math.nan


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('--out', required=True)
    ap.add_argument('--label', default='base')
    ap.add_argument('--hold', type=float, default=5.0)
    ap.add_argument('--wheel-topic', default='/wheel/odom')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    node = rclpy.create_node('base_characterization',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
    state = {'gt': None, 'gt_twist': None, 'odom': None, 'wheel': None}
    # 真值带消息时间戳：速度按真值自身的时间戳差分（订阅端采样时刻有抖动）
    node.create_subscription(Odometry, '/ground_truth/odom', lambda m: state.update(
        gt=_pose(m), gt_t=m.header.stamp.sec + m.header.stamp.nanosec * 1e-9), 20)
    node.create_subscription(Odometry, '/odom', lambda m: state.update(odom=_pose(m)), 20)
    node.create_subscription(Odometry, args.wheel_topic,
                             lambda m: state.update(wheel=_pose(m)), 20)
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def wait_sim(dt, cmd=None, log=None):
        t0 = now()
        while now() - t0 < dt:
            if cmd is not None:
                pub.publish(cmd)
            if log is not None and state['gt'] is not None:
                log.append((state['gt_t'], *state['gt'], *(state['odom'] or (math.nan,) * 3),
                            *(state['wheel'] or (math.nan,) * 3)))
            time.sleep(0.02)

    while state['gt'] is None or state['odom'] is None or now() <= 0:
        time.sleep(0.2)
    wait_sim(2.0)
    rows, traj = [], []
    gt0, od0, wh0 = state['gt'], state['odom'], state['wheel']
    wall0, sim0, cpu0 = time.time(), now(), gz_cpu()
    for name, vx, vy, wz in SEGMENTS:
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz
        k0 = len(traj)
        wait_sim(args.hold, cmd, traj)
        seg = np.array(traj[k0 + len(traj[k0:]) // 2:])
        seg = seg[np.concatenate([[True], np.diff(seg[:, 0]) > 0])]   # 去掉重复的真值样本
        # 后半段真值机体速度：位姿差分转到机体系
        t, x, y, yaw = seg[:, 0], seg[:, 1], seg[:, 2], np.unwrap(seg[:, 3])
        dt = t[-1] - t[0]
        c, s = np.cos(yaw[:-1]), np.sin(yaw[:-1])
        dx, dy = np.diff(x), np.diff(y)
        bvx = float(np.sum(c * dx + s * dy) / dt)
        bvy = float(np.sum(-s * dx + c * dy) / dt)
        bwz = float((yaw[-1] - yaw[0]) / dt)
        row = dict(label=args.label, segment=name, cmd_vx=vx, cmd_vy=vy, cmd_wz=wz,
                   act_vx=bvx, act_vy=bvy, act_wz=bwz)
        for k, cv, av in (('vx', vx, bvx), ('vy', vy, bvy), ('wz', wz, bwz)):
            row[f'ratio_{k}'] = av / cv if abs(cv) > 1e-9 else math.nan
        rows.append(row)
        wait_sim(1.0, Twist(), traj)
        node.get_logger().info(
            f'{name}: 指令 ({vx:+.2f}, {vy:+.2f}, {wz:+.2f}) 实际 ({bvx:+.3f}, {bvy:+.3f}, '
            f'{bwz:+.3f})')
    wall, sim, cpu = time.time() - wall0, now() - sim0, gz_cpu() - cpu0

    # 里程计漂移（起点对齐）
    tr = np.array(traj)
    path = float(np.sum(np.hypot(np.diff(tr[:, 1]), np.diff(tr[:, 2]))))
    turn = float(np.sum(np.abs(np.diff(np.unwrap(tr[:, 3])))))
    summary = dict(label=args.label, sim_s=sim, wall_s=wall, rtf=sim / wall,
                   gzserver_cpu_pct=100.0 * cpu / wall, path_m=path, turn_rad=turn)
    for key, col, ref in (('odom', 4, od0), ('wheel', 7, wh0)):
        if ref is None or np.isnan(tr[-1, col]):
            continue
        est = [align(gt0, ref, r[col:col + 3]) for r in tr]
        err = np.hypot([e[0] for e in est] - tr[:, 1], [e[1] for e in est] - tr[:, 2])
        yerr = np.array([math.atan2(math.sin(e[2] - g), math.cos(e[2] - g))
                         for e, g in zip(est, tr[:, 3])])
        summary.update({f'{key}_final_pos_err_m': float(err[-1]),
                        f'{key}_max_pos_err_m': float(err.max()),
                        f'{key}_final_yaw_err_deg': float(math.degrees(yerr[-1])),
                        f'{key}_drift_pct': float(100 * err[-1] / path)})
    for k in ('vx', 'vy', 'wz'):
        r = [row[f'ratio_{k}'] for row in rows if not math.isnan(row[f'ratio_{k}'])]
        summary[f'ratio_{k}_mean'] = float(np.mean(r))
    with open(os.path.join(args.out, 'segments.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(args.out, 'trajectory.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'gt_x', 'gt_y', 'gt_yaw', 'odom_x', 'odom_y', 'odom_yaw',
                    'wheel_x', 'wheel_y', 'wheel_yaw'])
        w.writerows(traj)
    with open(os.path.join(args.out, 'summary.yaml'), 'w') as f:
        yaml.safe_dump({k: (round(v, 4) if isinstance(v, float) else v)
                        for k, v in summary.items()}, f, sort_keys=False)
    node.get_logger().info('\n' + yaml.safe_dump(summary, sort_keys=False))
    pub.publish(Twist())
    ex.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
