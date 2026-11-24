#!/usr/bin/env python3
"""
从沿墙精度实验录制的数据包中提取离线回放与噪声拟合需要的原始测量（单进程，只读数据包）.

  source install/setup.bash          # 需要 scr_interfaces 的消息定义
  python3 docs/experiments/tag_map_estimation/extract_bags.py ~/scr_bags/along_wall

每个数据包输出到 bag_data/<名称>/（gzip 压缩的 CSV）：
  wheel_odom.csv.gz   /wheel/odom（50 Hz）：t, x, y, yaw —— 技能的局部闭环与估计器使用的里程计
  truth.csv.gz        /ground_truth/odom（50 Hz）：t, x, y, yaw
  gyro.csv.gz         /imu 角速度 z（100 Hz → 每两帧平均为 50 Hz）：t, wz
  wall_fit.csv.gz     /scan 每帧用 visual_servo.fit_wall 拟合的前方墙面，变换到 base_link：
                      t, distance, phi（墙面法向在 base_link 中的方位角）, var_d, cov_dphi,
                      var_phi, inliers, rms
  wall_estimate.csv.gz  /task_skills/wall_estimate：估计器每个周期的状态与画面中的标记
                      （t, d, u, psi, sigma_*, lateral_scale, sigma_scale, u_known, wall_ok,
                      tag_state, tags = "id:x:y:accepted;..."，标记位置已做延迟补偿）
  status.csv          /task_skills/status：t, station, phase, detail
  lidar_tf.yaml       base_link → lidar_link 的静态变换
"""
import csv
import gzip
import math
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'src', 'scr_arm_controller'))
from scr_arm_controller import visual_servo as vs  # noqa: E402

TOPICS = ('/wheel/odom', '/ground_truth/odom', '/imu', '/scan', '/task_skills/wall_estimate',
          '/task_skills/status', '/tf_static')


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _t(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def _quat_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _static_tf(tf_msgs, parent='base_link', child='lidar_link'):
    """在 /tf_static 中沿树链求 parent → child（只支持 child 在 parent 子树中的情形）."""
    edges = {}
    for m in tf_msgs:
        for tr in m.transforms:
            edges[tr.child_frame_id] = (tr.header.frame_id, tr.transform)
    p, R = np.zeros(3), np.eye(3)
    frame = child
    chain = []
    while frame != parent:
        if frame not in edges:
            raise KeyError(f'{child} 不在 {parent} 的子树中')
        par, tr = edges[frame]
        chain.append(tr)
        frame = par
    for tr in reversed(chain):
        t = tr.translation
        Rc = _quat_rot(tr.rotation)
        p = p + R @ np.array([t.x, t.y, t.z])
        R = R @ Rc
    return p, R


def wall_in_base(scan, p_lb, R_lb):
    """与 visual_servo_ros.WallMarkerServo.wall_in_base 相同的变换（不加注入噪声）."""
    wall_l = vs.fit_wall(np.asarray(scan.ranges, dtype=float), scan.angle_min,
                         scan.angle_increment)
    if wall_l is None:
        return None
    n_b = R_lb @ np.array([wall_l['normal'][0], wall_l['normal'][1], 0.0])
    n_b2 = n_b[:2] / np.linalg.norm(n_b[:2])
    d_b = wall_l['distance'] + float(n_b2 @ p_lb[:2])
    t_b = np.array([-n_b2[1], n_b2[0]])
    J = np.array([[1.0, float(t_b @ p_lb[:2])], [0.0, 1.0]])
    cov = J @ wall_l['cov'] @ J.T
    return (d_b, math.atan2(n_b2[1], n_b2[0]), cov[0, 0], cov[0, 1], cov[1, 1],
            wall_l['inliers'], wall_l['rms'])


def extract(bag_dir, out_dir):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
    cls = {t: get_message(types[t]) for t in TOPICS if t in types}

    rows = {k: [] for k in ('wheel_odom', 'truth', 'gyro', 'scan', 'wall_estimate', 'status')}
    tf_static = []
    gyro_buf = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, cls[topic])
        if topic in ('/wheel/odom', '/ground_truth/odom'):
            p = msg.pose.pose
            rows['wheel_odom' if topic == '/wheel/odom' else 'truth'].append(
                (_t(msg.header.stamp), p.position.x, p.position.y, _yaw(p.orientation)))
        elif topic == '/imu':
            gyro_buf.append((_t(msg.header.stamp), msg.angular_velocity.z))
            if len(gyro_buf) == 2:
                rows['gyro'].append((0.5 * (gyro_buf[0][0] + gyro_buf[1][0]),
                                     0.5 * (gyro_buf[0][1] + gyro_buf[1][1])))
                gyro_buf = []
        elif topic == '/scan':
            rows['scan'].append(msg)
        elif topic == '/tf_static':
            tf_static.append(msg)
        elif topic == '/task_skills/wall_estimate':
            tags = ';'.join(f'{i}:{q.x:.6f}:{q.y:.6f}:{int(a)}'
                            for i, q, a in zip(msg.tag_ids, msg.tag_positions, msg.tag_accepted))
            rows['wall_estimate'].append((
                _t(msg.header.stamp), msg.d, msg.u, msg.psi, msg.sigma_d, msg.sigma_u,
                msg.sigma_psi, msg.lateral_scale, msg.sigma_scale, int(msg.u_known),
                int(msg.wall_ok), msg.tag_state, tags))
        elif topic == '/task_skills/status':
            rows['status'].append((_t(msg.stamp), msg.station, msg.phase, msg.detail))

    p_lb, R_lb = _static_tf(tf_static)
    fits = []
    for s in rows['scan']:
        w = wall_in_base(s, p_lb, R_lb)
        if w is not None:
            fits.append((_t(s.header.stamp),) + tuple(w))

    os.makedirs(out_dir, exist_ok=True)

    def write(name, header, data, gz=True):
        path = os.path.join(out_dir, name + ('.csv.gz' if gz else '.csv'))
        op = gzip.open if gz else open
        with op(path, 'wt', newline='') as f:
            w = csv.writer(f)
            w.writerow(header)
            for r in data:
                w.writerow([f'{v:.7g}' if isinstance(v, float) else v for v in r])
        return path

    write('wheel_odom', ('t', 'x', 'y', 'yaw'), rows['wheel_odom'])
    write('truth', ('t', 'x', 'y', 'yaw'), rows['truth'])
    write('gyro', ('t', 'wz'), rows['gyro'])
    write('wall_fit', ('t', 'distance', 'phi', 'var_d', 'cov_dphi', 'var_phi', 'inliers',
                       'rms'), fits)
    write('wall_estimate', ('t', 'd', 'u', 'psi', 'sigma_d', 'sigma_u', 'sigma_psi',
                            'lateral_scale', 'sigma_scale', 'u_known', 'wall_ok', 'tag_state',
                            'tags'), rows['wall_estimate'])
    write('status', ('t', 'station', 'phase', 'detail'), rows['status'], gz=False)
    with open(os.path.join(out_dir, 'lidar_tf.yaml'), 'w') as f:
        yaml.safe_dump({'parent': 'base_link', 'child': 'lidar_link',
                        'translation': [round(float(v), 6) for v in p_lb],
                        'yaw': round(float(math.atan2(R_lb[1, 0], R_lb[0, 0])), 6)}, f)
    print(f'{bag_dir}: odom {len(rows["wheel_odom"])}, truth {len(rows["truth"])}, '
          f'gyro {len(rows["gyro"])}, wall fits {len(fits)}/{len(rows["scan"])}, '
          f'estimates {len(rows["wall_estimate"])}, status {len(rows["status"])}')


def main():
    root = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/scr_bags/along_wall')
    for name in sorted(os.listdir(root)):
        bag = os.path.join(root, name)
        if os.path.isfile(os.path.join(bag, 'metadata.yaml')):
            extract(bag, os.path.join(HERE, 'bag_data', name))


if __name__ == '__main__':
    main()
