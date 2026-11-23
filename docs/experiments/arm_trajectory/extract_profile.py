#!/usr/bin/env python3
"""
从 rosbag2 导出一个停靠站切割期间的关节位置 / 速度曲线（作 fig_profile 用）.

  python3 extract_profile.py <bag 目录> <停靠站序号> <输出.csv> [--rate 100]

/joint_states 以控制周期（1 kHz）发布，按 --rate（Hz）分箱平均后输出；时间以该站 CutStation
开始执行机械臂点的时刻为零点，阶段取自 /task_skills/status（时间戳为阶段生效时刻）。
需要 ROS 环境（rosbag2_py）。
"""
import argparse
import csv

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

JOINTS = ('joint1', 'joint2', 'joint3', 'joint4')


def read(bag, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while reader.has_next():
        topic, data, _ = reader.read_next()
        yield topic, deserialize_message(data, types[topic])


def stamp(t):
    return t.sec + t.nanosec * 1e-9


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('station', type=int)
    ap.add_argument('out')
    ap.add_argument('--rate', type=float, default=100.0)
    args = ap.parse_args()

    status, joints = [], []
    for topic, msg in read(args.bag, ('/task_skills/status', '/joint_states')):
        if topic == '/task_skills/status':
            status.append((stamp(msg.stamp), msg.station, msg.phase))
        elif all(j in msg.name for j in JOINTS):
            idx = [msg.name.index(j) for j in JOINTS]
            joints.append((stamp(msg.header.stamp),) + tuple(msg.position[i] for i in idx)
                          + tuple(msg.velocity[i] for i in idx))
    # 该站机械臂点的执行时段：到站之后第一条 move 到之后第一条 idle
    arrived = next(k for k, s in enumerate(status) if s[1] == args.station
                   and s[2] == 'arrived')
    seq = [s for s in status[arrived:] if s[1] == args.station]
    first = next(k for k, s in enumerate(seq) if s[2] == 'move')
    last = next(k for k, s in enumerate(seq) if s[2] == 'idle' and k > first)
    seq = sorted(seq[first:last + 1])
    t0, t1 = seq[0][0], seq[-1][0]

    data = np.array([r for r in joints if t0 - 0.2 <= r[0] <= t1 + 0.2])
    bins = np.floor((data[:, 0] - t0) * args.rate).astype(int)
    with open(args.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'phase'] + [f'q_{j}' for j in JOINTS] + [f'v_{j}' for j in JOINTS])
        for b in np.unique(bins):
            m = data[bins == b].mean(axis=0)
            phase = [s[2] for s in seq if s[0] <= m[0]]
            w.writerow([f'{m[0] - t0:.4f}', phase[-1] if phase else 'arrived']
                       + [f'{v:.6f}' for v in m[1:]])
    print(f'S{args.station}: {t1 - t0:.2f} s, {len(data)} samples → {args.out}')


if __name__ == '__main__':
    main()
