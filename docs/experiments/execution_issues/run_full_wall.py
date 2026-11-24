#!/usr/bin/env python3
"""
整面墙（多站开槽，空切）的单次运行：默认配置 + 可选的定点保持控制律 / 机械臂运动覆盖.

  python3 run_full_wall.py <规划文件> <结果目录> --seed 1 [--hold dob]
      [--set mode=point_to_point ...] [--bag <目录>]

- 与 arm_trajectory/run_experiment.py 相同：精对位参数取默认的 fine_alignment.yaml，
  机械臂运动参数取默认的 arm_motion.yaml（--set 覆盖 arm_motion.*），不加载切割力插件，
  底盘限幅取默认值；
- --hold 只改 hold.mode（p / pi / pid / dob / lqg），其余 hold.* 取 base_hold.py 的默认值；
- 启动看门狗与重跑同 plan_validation/run_validation.py，启动失败写入 startup_failures.txt；
- 完整启动日志 launch.log 保留在结果目录（不提交；第 3 节的停顿分析从中提取机械臂轨迹
  目标的计时行，见 pause_timing.py）。
仿真与 ROS 通信隔离在 ROS_DOMAIN_ID = 49、GAZEBO_MASTER_URI = http://localhost:11353。
"""
import argparse
import os
import sys
import time

import yaml

import sim_session as ss

sys.path.insert(0, os.path.join(ss.HERE, '..', 'arm_trajectory'))
from run_experiment import portable_logs  # noqa: E402

rv = ss.rv
rv.BAG_TOPICS = ['/odom', '/wheel/odom', '/ground_truth/odom', '/task_skills/wall_estimate',
                 '/task_skills/status', '/task_executor/station_records', '/cmd_vel', '/imu',
                 '/clock', '/arm_controller/controller_state', '/plan', '/cmd_vel_nav',
                 '/behavior_tree_log', '/rosout']


def skill_params(hold, overrides):
    with open(os.path.join(ss.REPO, 'src', 'scr_arm_controller', 'config',
                           'fine_alignment.yaml'), encoding='utf-8') as f:
        params = yaml.safe_load(f)
    p = params['/**']['ros__parameters']
    if hold:
        p['hold'] = {'mode': hold}
    arm = {}
    for item in overrides:
        key, _, value = item.partition('=')
        arm[key] = yaml.safe_load(value)
    if arm:
        p['arm_motion'] = arm
    return params


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('plan')
    ap.add_argument('out')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--hold', default='', help='hold.mode（空 = 默认 p）')
    ap.add_argument('--set', action='append', default=[], metavar='KEY=VALUE',
                    help='arm_motion.* 覆盖，可重复')
    ap.add_argument('--launch', action='append', default=[], metavar='KEY:=VALUE',
                    help='额外的 launch 参数，可重复')
    ap.add_argument('--bag', help='同时录制 rosbag2 到该目录（仓库之外）')
    ap.add_argument('--fastdds-profile', default='',
                    help='Fast DDS XML 配置（作为 FASTRTPS_DEFAULT_PROFILES_FILE 传给全部进程）')
    ap.add_argument('--timeout', type=float, default=1500.0)
    ap.add_argument('--startup-timeout', type=float, default=240.0)
    ap.add_argument('--retries', type=int, default=2)
    args = ap.parse_args()
    plan = os.path.abspath(os.path.expanduser(args.plan))
    out = os.path.abspath(os.path.expanduser(args.out))
    os.makedirs(out, exist_ok=True)
    env = ss.env()
    if args.fastdds_profile:
        env['FASTRTPS_DEFAULT_PROFILES_FILE'] = os.path.abspath(args.fastdds_profile)
    ss.kill_stray_gazebo()

    params_file = os.path.join(out, 'skill_params.yaml')
    with open(params_file, 'w', encoding='utf-8') as f:
        yaml.safe_dump(skill_params(args.hold, args.set), f, allow_unicode=True,
                       sort_keys=False)
    launch = ss.launch_cmd([f'odom_seed:={args.seed}', f'alignment_params:={params_file}']
                           + args.launch)
    rec = ['ros2', 'run', 'scr_planner', 'execute_slot_plan.py', plan, out,
           '--dry-run-gap=0.01']
    with open(os.path.join(out, 'command.txt'), 'w') as f:
        if args.fastdds_profile:
            f.write('FASTRTPS_DEFAULT_PROFILES_FILE=' + rv.portable(
                env['FASTRTPS_DEFAULT_PROFILES_FILE']) + '\n')
        f.write(rv.portable(' '.join(launch)) + '\n' + rv.portable(' '.join(rec)) + '\n')

    t0 = time.time()
    rc = 1
    for k in range(args.retries + 1):
        rc, started = rv.attempt(args, env, out, launch, None, {}, rec)
        if started:
            break
        with open(os.path.join(out, 'startup_failures.txt'), 'a') as f:
            f.write(f'attempt {k + 1}: task layer did not accept the goal within '
                    f'{args.startup_timeout:.0f} s\n')
        print(f'第 {k + 1} 次启动失败，重跑', flush=True)
        ss.kill_stray_gazebo()
    with open(os.path.join(out, 'launch.log'), errors='replace') as f, \
            open(os.path.join(out, 'task_layer.log'), 'w') as g:
        for line in f:
            if '[task_skills' in line or '[task_executor' in line:
                g.write(line)
    portable_logs(out)
    ss.kill_stray_gazebo()
    print(f'{os.path.basename(out)} seed {args.seed}: rc={rc}, {time.time() - t0:.0f} s',
          flush=True)
    return rc


if __name__ == '__main__':
    sys.exit(main())
