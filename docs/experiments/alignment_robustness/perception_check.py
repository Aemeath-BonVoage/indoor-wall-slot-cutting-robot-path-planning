#!/usr/bin/env python3
"""
感知精度诊断：在一组已知位姿下比较 AprilTag / 色块 / 激光测量与 Gazebo 真值.

27 个位姿 = 离墙 0.7 / 0.9 / 1.2 m × 沿墙 −0.15 / 0 / +0.15 m × 朝向 −6° / 0 / +6°；
每个位姿静置后取 10 帧，记录标记中心在 base_link 中的位置误差、激光墙面距离 / 角度误差。

  ros2 launch scr_description gazebo.launch.py gui:=false rtf:=2.5
  python3 docs/experiments/alignment_robustness/perception_check.py --out <目录>
"""
import argparse
import csv
import math
import os

import numpy as np
import rclpy
from gazebo_msgs.srv import SetEntityState

from scr_arm_controller.alignment_benchmark import TAG_X, WALL_Y, teleport
from scr_arm_controller.navigation_cutting_task import NavigationCuttingTask

TAG_FACE_Y = WALL_Y + 0.0005          # 标记板厚 0.5 mm
BLOB = (8.42, -3.924)                 # 色块中心 x、正面 y


def to_base(gt, px, py):
    c, s = math.cos(gt['yaw']), math.sin(gt['yaw'])
    dx, dy = px - gt['x'], py - gt['y']
    return c * dx + s * dy, -s * dx + c * dy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='.')
    ap.add_argument('--frames', type=int, default=10)
    args = ap.parse_args()
    rclpy.init()
    node = NavigationCuttingTask()
    cli = node.create_client(SetEntityState, '/gazebo/set_entity_state')
    cli.wait_for_service(timeout_sec=20.0)
    node.servo.detectors = {'apriltag', 'blob'}
    node.move_arm_for_approach(joint1=node.observe_joint1)
    rows = []
    for D in (0.7, 0.9, 1.2):
        for du in (-0.15, 0.0, 0.15):
            for yaw_deg in (-6.0, 0.0, 6.0):
                x, y = TAG_X + du, WALL_Y + D
                teleport(node, cli, x, y, -math.pi / 2 + math.radians(yaw_deg))
                node.sleep_sim(1.5)
                got, last = 0, None
                for _ in range(args.frames * 6):
                    node.sleep_sim(0.1)
                    gt = node.get_ground_truth_position()
                    tag, _ = node.servo.tag_in_base(stamp_after=last)
                    wall, _ = node.servo.wall_in_base()
                    if tag is None or wall is None:
                        continue
                    last = tag['stamp']
                    tb = to_base(gt, TAG_X, TAG_FACE_Y)
                    blob, _ = node.servo.blob_in_base(wall)
                    bb = to_base(gt, *BLOB)
                    d_true = gt['y'] - WALL_Y          # 点到直线的垂距，与朝向无关
                    phi_true = -(gt['yaw'] + math.pi / 2)
                    rows.append(dict(
                        D=D, du=du, yaw_deg=yaw_deg,
                        tag_dx_mm=(tag['p'][0] - tb[0]) * 1000,
                        tag_dy_mm=(tag['p'][1] - tb[1]) * 1000,
                        blob_dy_mm=((blob['p'][1] - bb[1]) * 1000) if blob else float('nan'),
                        wall_dd_mm=(wall['distance'] - d_true) * 1000,
                        wall_dphi_deg=math.degrees(wall['yaw_error'] - phi_true)))
                    got += 1
                    if got >= args.frames:
                        break
                r = [q for q in rows if q['D'] == D and q['du'] == du and q['yaw_deg'] == yaw_deg]
                print(f'D={D} du={du:+.2f} yaw={yaw_deg:+.0f}: {got} 帧  标记 dx '
                      f'{np.mean([q["tag_dx_mm"] for q in r]) if r else float("nan"):+.2f} '
                      f'dy {np.mean([q["tag_dy_mm"] for q in r]) if r else float("nan"):+.2f} mm',
                      flush=True)
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, 'perception_check.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    a = {k: np.array([r[k] for r in rows]) for k in rows[0]}
    for k in ('tag_dx_mm', 'tag_dy_mm', 'blob_dy_mm', 'wall_dd_mm', 'wall_dphi_deg'):
        v = a[k][np.isfinite(a[k])]
        print(f'{k:14s} mean {v.mean():+.3f}  std {v.std():.3f}  max|.| {np.abs(v).max():.3f}  n {len(v)}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
