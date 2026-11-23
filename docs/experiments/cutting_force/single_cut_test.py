#!/usr/bin/env python3
"""
单刀切割试验：只启动仿真与技能服务器，在原地执行一刀横槽（或竖槽）并记录真值与切削力.

用于快速检查切割力插件与定点保持的配合（整墙任务一次十几分钟，这里一刀约 1 分钟）。
不需要墙面与导航：机械臂按空切的点位运动，切深由执行器按轨迹时间表发布。

  ros2 launch scr_description gazebo.launch.py gui:=false cutting_force:=true \\
      base_max_ang_accel:=75 cutting_force_params:="apply_to=base;tip_offset=0.2044"
  ros2 run scr_arm_controller task_skill_server --ros-args --params-file <参数文件> \\
      -p use_sim_time:=true -p odom_topic:=/wheel/odom
  python3 single_cut_test.py --out runs/single/<名称>.csv [--vertical] [--length 0.12]

输出 CSV（50 Hz）：时间、底盘真值相对起点的偏差（机体系）、插件施加的力旋量、执行器预计的
力旋量；并打印进给期间的刀尖偏移峰值 / 均方根与力的均值。
"""
import argparse
import csv
import math
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from scr_interfaces.action import CutStation, HoldBase
from scr_interfaces.msg import ArmPoint
from std_msgs.msg import Float64

TIP_X = 0.735
X_RETRACT, X_DRY = 0.52, 0.5306


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('--out', default='')
    ap.add_argument('--vertical', action='store_true')
    ap.add_argument('--length', type=float, default=0.12)
    ap.add_argument('--height', type=float, default=0.482, help='tool height in base_link (m)')
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('single_cut_test',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
    state = {'pose': None, 't': None, 'w': (0.0,) * 3, 'e': (0.0,) * 3, 'depth': 0.0,
             'ref': None}
    rows = []

    def on_truth(msg):
        p = msg.pose.pose
        pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        state['pose'] = pose
        state['t'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ref = state['ref']
        if ref is not None:
            c, s = math.cos(ref[2]), math.sin(ref[2])
            dx, dy = pose[0] - ref[0], pose[1] - ref[1]
            dyaw = math.atan2(math.sin(pose[2] - ref[2]), math.cos(pose[2] - ref[2]))
            rows.append((state['t'], c * dx + s * dy, -s * dx + c * dy, dyaw, state['depth'],
                         *state['w'], *state['e']))

    def on_wrench(key):
        def cb(msg):
            w = msg.wrench
            state[key] = (w.force.x, w.force.y, w.torque.z)
        return cb

    node.create_subscription(Odometry, '/ground_truth/odom', on_truth, 50)
    node.create_subscription(WrenchStamped, '/cutting_force/wrench', on_wrench('w'), 50)
    node.create_subscription(WrenchStamped, '/cutting_force/expected_wrench', on_wrench('e'), 50)
    node.create_subscription(Float64, '/cutting_force/engagement',
                             lambda m: state.__setitem__('depth', m.data), 50)
    hold = ActionClient(node, HoldBase, '/task_skills/hold_base')
    cut = ActionClient(node, CutStation, '/task_skills/cut_station')

    def spin_until(cond, timeout):
        t0 = time.time()
        while not cond():
            rclpy.spin_once(node, timeout_sec=0.02)
            if time.time() - t0 > timeout:
                raise RuntimeError('timeout')

    spin_until(lambda: state['pose'] is not None, 30.0)
    if not (hold.wait_for_server(timeout_sec=30.0) and cut.wait_for_server(timeout_sec=30.0)):
        sys.exit('skill server not available')
    hf = hold.send_goal_async(HoldBase.Goal())
    spin_until(hf.done, 10.0)
    t0 = state['t']
    spin_until(lambda: state['t'] > t0 + 1.0, 30.0)
    state['ref'] = state['pose']

    half = 0.5 * args.length
    if args.vertical:
        ends, rot = ((0.07, args.height - half), (0.07, args.height + half)), 0.0
    else:
        ends, rot = ((0.01 + half, args.height), (0.01 - half, args.height)), 90.0
    goal = CutStation.Goal()
    goal.station = 0
    for x, (y, z), deep in ((X_RETRACT, ends[0], False), (X_DRY, ends[0], True),
                            (X_DRY, ends[1], True), (X_RETRACT, ends[1], False)):
        goal.points.append(ArmPoint(x=x, y=y, z=z, rotation=math.radians(rot), at_depth=deep))
    goal.tool_offset = 0.1594
    goal.standoff = 0.70        # 预计力旋量按离墙 0.70 m 计算力臂
    cf = cut.send_goal_async(goal)
    spin_until(cf.done, 10.0)
    rf = cf.result().get_result_async()
    spin_until(rf.done, 600.0)
    t1 = state['t']
    spin_until(lambda: state['t'] > t1 + 2.0, 30.0)
    hf.result().cancel_goal_async()
    spin_until(lambda: False, 0.5) if False else None

    a = np.array(rows)
    tip = np.hypot(a[:, 1], a[:, 2] + TIP_X * a[:, 3]) * 1e3
    feeding = (a[:, 4] > 0.0349) & (np.abs(a[:, 9]) + np.abs(a[:, 8]) > 0.0)
    eng = a[:, 4] > 0.0
    print(f'success {rf.result().result.success}, engaged {eng.sum() / 50:.1f} s')
    print(f'tip deviation: engaged peak {tip[eng].max():.2f} mm rms '
          f'{math.sqrt(np.mean(tip[eng] ** 2)):.2f} mm; at depth peak {tip[feeding].max():.2f} '
          f'rms {math.sqrt(np.mean(tip[feeding] ** 2)):.2f}; after retract peak '
          f'{tip[~eng & (a[:, 0] > a[eng, 0].max())].max():.2f}')
    for k, name in ((5, 'fx'), (6, 'fy'), (7, 'mz')):
        print(f'{name}: applied mean {a[feeding, k].mean():+.2f} std {a[feeding, k].std():.2f}'
              f' | expected mean {a[feeding, k + 3].mean():+.2f}')
    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_s', 'x_m', 'y_m', 'yaw_rad', 'depth_m', 'fx_N', 'fy_N', 'mz_Nm',
                        'expected_fx_N', 'expected_fy_N', 'expected_mz_Nm'])
            w.writerows([[f'{v:.6g}' for v in r] for r in rows])
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
