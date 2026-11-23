#!/usr/bin/env python3
"""
Gazebo 中的外力阶跃试验：定点保持 + 切割力插件的 test_wrench，并与离线模型的预测对照.

前提：仿真已启动并加载切割力插件（cutting_force:=true），底盘静止在任意位置。
脚本以给定控制律开始定点保持，2 s 后在底盘原点施加常值外力（base_link 系），持续
--duration 秒后撤去，再记录 --tail 秒；真值取 /gazebo/model_states。结束时打印实测的
稳态偏差、峰值与刀尖偏移，以及 plant_sim.ForcePlant 对同一试验的预测。

  ros2 launch scr_description gazebo.launch.py gui:=false cutting_force:=true \\
      cutting_force_params:="apply_to=base"
  python3 docs/experiments/cutting_force/hold_step_test.py --mode dob --wrench 0 5 0 \\
      --out docs/experiments/cutting_force/runs/step_dob_fy5.csv

放宽限幅的试验（偏航力矩大于 9.2 N·m）需同时给启动文件加 base_max_ang_accel:=75，
并给本脚本加 --max-ang 4.0 --limits raised。
"""
import argparse
import csv
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import plant_sim as ps  # noqa: E402

TIP_X = ps.PF.tool_tip(ps.cf.SLOT_DEPTH)[0]


def predict(mode, wrench, duration, tail, limits):
    """离线模型对同一试验的预测：返回 (稳态偏差 (x, y, ψ), 刀尖偏移峰值 mm)."""
    name = {'p': 'P', 'pi': 'PI', 'pid': 'PID', 'dob': 'DOB', 'lqg': 'LQG'}[mode]
    hold, _ = ps.make_hold(name, limits)
    proc = ps.ConstantWrench(wrench, 1.0, 1.0 + duration)
    lim = ps.LIMITS[limits]
    plant = ps.ForcePlant(lim['a_max'], lim['alpha_max'])
    rec = ps.simulate(hold, proc, 1.0 + duration + tail, plant)
    on = (rec['t'] > duration) & (rec['t'] < 1.0 + duration)
    steady = tuple(float(np.mean(rec[k][on])) for k in ('x', 'y', 'psi'))
    return steady, float(np.hypot(rec['tipx'], rec['tipy']).max() * 1e3)


def body(ref, pose):
    """pose 相对 ref 的偏差，表示在 ref 的机体系."""
    c, s = math.cos(ref[2]), math.sin(ref[2])
    dx, dy = pose[0] - ref[0], pose[1] - ref[1]
    dyaw = math.atan2(math.sin(pose[2] - ref[2]), math.cos(pose[2] - ref[2]))
    return c * dx + s * dy, -s * dx + c * dy, dyaw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', default='p', choices=['p', 'pi', 'pid', 'dob', 'lqg'])
    ap.add_argument('--wrench', type=float, nargs=3, default=[0.0, 5.0, 0.0],
                    metavar=('FX', 'FY', 'MZ'), help='N, N, N·m in base_link')
    ap.add_argument('--duration', type=float, default=6.0)
    ap.add_argument('--tail', type=float, default=3.0)
    ap.add_argument('--odom-topic', default='/wheel/odom')
    ap.add_argument('--max-lin', type=float, default=None)
    ap.add_argument('--max-ang', type=float, default=None)
    ap.add_argument('--limits', default='default', choices=['default', 'raised'],
                    help='limits assumed by the offline prediction')
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    import rclpy
    from geometry_msgs.msg import Wrench
    from scr_arm_controller.base_hold import BaseHold

    ros_args = ['--ros-args', '-p', f'hold.mode:={args.mode}']
    for name, val in (('hold.max_lin', args.max_lin), ('hold.max_ang', args.max_ang)):
        if val is not None:
            ros_args += ['-p', f'{name}:={val}']
    rclpy.init(args=ros_args)
    hold = BaseHold(odom_topic=args.odom_topic)
    pub = hold.node.create_publisher(Wrench, '/cutting_force/test_wrench', 10)

    def now():
        return hold.node.get_clock().now().nanoseconds * 1e-9

    def wait(cond, timeout, what):
        t0 = time.time()
        while not cond():
            if time.time() - t0 > timeout:
                raise RuntimeError(f'timeout waiting for {what}')
            time.sleep(0.02)

    def send(w):
        msg = Wrench()
        msg.force.x, msg.force.y, msg.torque.z = w
        pub.publish(msg)

    rows = []
    try:
        wait(lambda: hold._odom is not None and hold._gt is not None and now() > 0.0, 20.0,
             'odometry and ground truth')
        send((0.0, 0.0, 0.0))
        hold.hold()
        ref, t0 = hold._gt, now()
        t_on, t_off = 2.0, 2.0 + args.duration
        state = 'pre'
        while now() - t0 < t_off + args.tail:
            t = now() - t0
            if state == 'pre' and t >= t_on:
                send(tuple(args.wrench))
                state = 'on'
            elif state == 'on' and t >= t_off:
                send((0.0, 0.0, 0.0))
                state = 'post'
            dev = body(ref, hold._gt)
            est = hold.disturbance_estimate() or (float('nan'),) * 3
            rows.append((t - t_on, *dev, *body(hold._target, hold._odom), *est))
            time.sleep(0.01)
    finally:
        send((0.0, 0.0, 0.0))
        hold.release()
        hold.shutdown()
        rclpy.shutdown()

    a = np.array(rows)
    on = (a[:, 0] > args.duration - 1.0) & (a[:, 0] < args.duration)
    steady = a[on, 1:4].mean(axis=0)
    tip = np.hypot(a[:, 1], a[:, 2] + TIP_X * a[:, 3]) * 1e3
    p_steady, p_peak = predict(args.mode, args.wrench, args.duration, args.tail, args.limits)
    print(f'mode {args.mode}, wrench {args.wrench}')
    print('steady deviation  measured: x {:+.2f} mm  y {:+.2f} mm  yaw {:+.3f} deg'.format(
        steady[0] * 1e3, steady[1] * 1e3, math.degrees(steady[2])))
    print('                 predicted: x {:+.2f} mm  y {:+.2f} mm  yaw {:+.3f} deg'.format(
        p_steady[0] * 1e3, p_steady[1] * 1e3, math.degrees(p_steady[2])))
    print(f'tool-tip deviation peak  measured {tip.max():.2f} mm, predicted {p_peak:.2f} mm')
    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_s', 'x_m', 'y_m', 'yaw_rad', 'odom_x_m', 'odom_y_m', 'odom_yaw_rad',
                        'est_fx_N', 'est_fy_N', 'est_mz_Nm'])
            w.writerows([[f'{v:.6g}' for v in r] for r in rows])


if __name__ == '__main__':
    main()
