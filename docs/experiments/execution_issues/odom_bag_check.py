#!/usr/bin/env python3
"""
从已有记录包检查慢速运动下 EKF 与轮式里程计的关系（不需要 Gazebo）.

  python3 odom_bag_check.py <记录包目录> <输出 CSV> [--from 60 --to 400 --window 8]

在 [from, to] 秒（相对包起点）内每隔 window / 2 秒取一个 window 秒的窗口，比较位移（mm）：
  truth   /ground_truth/odom
  wheel   /wheel/odom 的位姿（控制器每个周期积分）
  twist   /wheel/odom 的 twist 按消息时间戳积分（EKF 融合的量）
  ekf     /odom（robot_localization）
并给出 twist 相对位姿差分速度的平均偏差（机体系，mm/s）。
"""
import argparse
import csv
import math

import numpy as np


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read(bag, t_lo, t_hi):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = ['/wheel/odom', '/odom', '/ground_truth/odom']
    r.set_filter(rosbag2_py.StorageFilter(topics=want))
    out = {k: [] for k in want}
    t0 = None
    while r.has_next():
        topic, data, t = r.read_next()
        t0 = t if t0 is None else t0
        rel = (t - t0) * 1e-9
        if rel < t_lo:
            continue
        if rel > t_hi:
            break
        m = deserialize_message(data, get_message(types[topic]))
        p, tw = m.pose.pose, m.twist.twist
        out[topic].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.position.x,
                           p.position.y, yaw_of(p.orientation), tw.linear.x, tw.linear.y))
    return {k: np.array(v) for k, v in out.items()}


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('out')
    ap.add_argument('--from', dest='t_lo', type=float, default=60.0)
    ap.add_argument('--to', dest='t_hi', type=float, default=400.0)
    ap.add_argument('--window', type=float, default=8.0)
    args = ap.parse_args()
    d = read(args.bag, args.t_lo, args.t_hi)
    W, E, G = d['/wheel/odom'], d['/odom'], d['/ground_truth/odom']
    dt = np.diff(W[:, 0], prepend=W[0, 0])
    yaw = W[:, 3]
    ix = np.cumsum((np.cos(yaw) * W[:, 4] - np.sin(yaw) * W[:, 5]) * dt)
    iy = np.cumsum((np.sin(yaw) * W[:, 4] + np.cos(yaw) * W[:, 5]) * dt)

    def at(a, t):
        return min(max(np.searchsorted(a[:, 0], t), 0), len(a) - 1)

    rows = []
    for t in np.arange(W[0, 0] + 1.0, W[-1, 0] - args.window, args.window / 2):
        t1 = t + args.window
        g0, g1 = G[at(G, t)], G[at(G, t1)]
        w0, w1 = at(W, t), at(W, t1)
        e0, e1 = E[at(E, t)], E[at(E, t1)]
        # 位姿差分速度与 twist 在机体系下的平均偏差
        dx, dy = np.diff(W[w0:w1 + 1, 1]), np.diff(W[w0:w1 + 1, 2])
        ddt = np.diff(W[w0:w1 + 1, 0])
        yy = W[w0 + 1:w1 + 1, 3]
        ok = ddt > 0
        vbx = (np.cos(yy) * dx + np.sin(yy) * dy)[ok] / ddt[ok]
        vby = (-np.sin(yy) * dx + np.cos(yy) * dy)[ok] / ddt[ok]
        rows.append(dict(
            t_rel_s=round(t - W[0, 0] + args.t_lo, 1),
            truth_mm=1e3 * math.hypot(g1[1] - g0[1], g1[2] - g0[2]),
            wheel_mm=1e3 * math.hypot(W[w1, 1] - W[w0, 1], W[w1, 2] - W[w0, 2]),
            twist_mm=1e3 * math.hypot(ix[w1] - ix[w0], iy[w1] - iy[w0]),
            ekf_mm=1e3 * math.hypot(e1[1] - e0[1], e1[2] - e0[2]),
            ekf_minus_twist_mm=1e3 * (math.hypot(e1[1] - e0[1], e1[2] - e0[2])
                                      - math.hypot(ix[w1] - ix[w0], iy[w1] - iy[w0])),
            twist_bias_x_mm_s=1e3 * float(np.mean(W[w0 + 1:w1 + 1, 4][ok] - vbx)),
            twist_bias_y_mm_s=1e3 * float(np.mean(W[w0 + 1:w1 + 1, 5][ok] - vby))))
    with open(args.out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        for r in rows:
            w.writerow({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})
    slow = [r for r in rows if r['truth_mm'] < 20.0]
    print(f'{len(rows)} windows, {len(slow)} with truth displacement < 20 mm')
    for k in ('twist_bias_x_mm_s', 'twist_bias_y_mm_s', 'ekf_minus_twist_mm'):
        v = np.array([r[k] for r in slow])
        print(f'  {k}: mean {v.mean():+.3f}, sd {v.std():.3f}, max |.| {np.abs(v).max():.3f}')
    v = np.array([abs(r['wheel_mm'] - r['truth_mm']) for r in slow])
    print(f'  |wheel - truth| mm: mean {v.mean():.3f}, max {v.max():.3f}')
    v = np.array([abs(r['twist_mm'] - r['wheel_mm']) for r in slow])
    print(f'  |twist - wheel| mm: mean {v.mean():.3f}, max {v.max():.3f}')


if __name__ == '__main__':
    main()
