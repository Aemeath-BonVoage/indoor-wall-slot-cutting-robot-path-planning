#!/usr/bin/env python3
"""
从规划验证实验 joint_s1 的记录包中提取导航到达停滞的时间线（第 2 节图 1(b)）.

  python3 extract_stall_example.py ~/scr_bags/plan_validation/joint_s1 stall_example_joint_s1.csv

记录包不在仓库中（约 200 MB）；提交的是提取出的 CSV：导航开始后 70 s 内
  - 每条 /cmd_vel（vx, vy, wz）及其来源的判别：线速度为 0、|wz| ≥ 0.4 rad/s 的指令来自
    behavior_server 的 Spin（Spin 只发纯转动指令，最小转速 min_rotational_vel = 0.4），其余来自
    控制器（MPPI 经 velocity_smoother）；
  - 真值位姿（降采样到 10 Hz）与到导航目标的距离；
  - map → odom 变换（AMCL 的修正）。
"""
import csv
import math
import sys

GOAL = (8.42, -2.7245)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    bag, out = sys.argv[1], sys.argv[2]
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=['/cmd_vel', '/ground_truth/odom', '/tf']))
    rows, t0, last_gt = [], None, -1.0
    while r.has_next():
        topic, data, t = r.read_next()
        ts = t * 1e-9
        t0 = ts if t0 is None else t0
        if ts - t0 > 70.0:
            break
        m = deserialize_message(data, get_message(types[topic]))
        if topic == '/cmd_vel':
            spin = m.linear.x == 0.0 and m.linear.y == 0.0 and abs(m.angular.z) >= 0.4
            rows.append(dict(t=ts - t0, kind='cmd', source='spin' if spin else 'controller',
                             vx=m.linear.x, vy=m.linear.y, wz=m.angular.z))
        elif topic == '/ground_truth/odom':
            if ts - last_gt < 0.1:
                continue
            last_gt = ts
            p = m.pose.pose
            rows.append(dict(t=ts - t0, kind='truth', x=p.position.x, y=p.position.y,
                             yaw=yaw_of(p.orientation),
                             dist=math.hypot(p.position.x - GOAL[0], p.position.y - GOAL[1])))
        else:
            for tr in m.transforms:
                if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                    q = tr.transform
                    rows.append(dict(t=ts - t0, kind='map_odom', x=q.translation.x,
                                     y=q.translation.y, yaw=yaw_of(q.rotation)))
    fields = ['t', 'kind', 'source', 'vx', 'vy', 'wz', 'x', 'y', 'yaw', 'dist']
    with open(out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for row in rows:
            w.writerow({k: (f'{v:.4f}' if isinstance(v, float) else v) for k, v in row.items()})
    print(f'{len(rows)} rows -> {out}')


if __name__ == '__main__':
    main()
