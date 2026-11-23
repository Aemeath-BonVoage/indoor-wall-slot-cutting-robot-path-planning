#!/usr/bin/env python3
"""
机械臂轨迹实验的单次运行：无界面仿真 + 任务层 → 执行一份规划文件并记录真值 → 结束全部进程.

  python3 run_experiment.py <规划文件> <结果目录> --seed 1
      [--set mode=point_to_point] [--set feed_rate=0.1 --set rapid_accel=2.0 ...] [--bag <目录>]

--set 覆盖技能服务器的 arm_motion.* 参数（见 scr_arm_controller/task_skills.py 的
ARM_MOTION_PARAMS），其余设置全部取默认值，与规划验证实验（plan_validation）相同。本次运行
实际使用的技能服务器参数写入 <结果目录>/skill_params.yaml。

仿真与 ROS 通信隔离在 ROS_DOMAIN_ID = 49、GAZEBO_MASTER_URI = http://localhost:11353；
启动看门狗、进程清理与 run_validation.py 相同（直接复用其实现）。运行前先 source 工作空间。
"""
import argparse
import os
import sys
import time

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(HERE, '..', 'plan_validation'))
import run_validation as rv  # noqa: E402

rv.DOMAIN = '49'
rv.MASTER = 'http://localhost:11353'
rv.BAG_TOPICS = rv.BAG_TOPICS + ['/arm_controller/controller_state']


def skill_params(overrides):
    """默认精对位参数 + arm_motion 覆盖 → 技能服务器参数文件内容."""
    with open(os.path.join(REPO, 'src', 'scr_arm_controller', 'config', 'fine_alignment.yaml'),
              encoding='utf-8') as f:
        params = yaml.safe_load(f)
    arm = {}
    for item in overrides:
        key, _, value = item.partition('=')
        arm[key] = yaml.safe_load(value)
    if arm:
        params['/**']['ros__parameters']['arm_motion'] = arm
    return params


def portable_logs(out):
    """结果目录里的日志改用仓库相对路径与 ~（完整启动日志 launch.log 不提交，不处理）."""
    for name in os.listdir(out):
        path = os.path.join(out, name)
        if name == 'launch.log' or not name.endswith(('.log', '.txt', '.yaml')):
            continue
        with open(path, errors='replace') as f:
            text = f.read()
        clean = rv.portable(text)
        if clean != text:
            with open(path, 'w') as f:
                f.write(clean)


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('plan')
    ap.add_argument('out')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--set', action='append', default=[], metavar='KEY=VALUE',
                    help='arm_motion.* 参数覆盖，可重复')
    ap.add_argument('--bag', help='同时录制 rosbag2 到该目录（仓库之外）')
    ap.add_argument('--timeout', type=float, default=1500.0, help='整次运行的上限（墙钟秒）')
    ap.add_argument('--startup-timeout', type=float, default=240.0)
    ap.add_argument('--retries', type=int, default=2, help='启动失败时的重跑次数')
    args = ap.parse_args()
    plan = os.path.abspath(os.path.expanduser(args.plan))
    out = os.path.abspath(os.path.expanduser(args.out))
    os.makedirs(out, exist_ok=True)
    env = rv.isolated_env()

    params_file = os.path.join(out, 'skill_params.yaml')
    with open(params_file, 'w', encoding='utf-8') as f:
        yaml.safe_dump(skill_params(args.set), f, allow_unicode=True, sort_keys=False)
    launch = ['ros2', 'launch', 'scr_bringup', 'sim_nav.launch.py', 'gui:=false', 'rviz:=false',
              f'odom_seed:={args.seed}', f'alignment_params:={params_file}']
    rec = ['ros2', 'run', 'scr_planner', 'execute_slot_plan.py', plan, out, '--dry-run-gap=0.01']
    with open(os.path.join(out, 'command.txt'), 'w') as f:
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
    with open(os.path.join(out, 'launch.log'), errors='replace') as f, \
            open(os.path.join(out, 'task_layer.log'), 'w') as g:
        for line in f:
            if '[task_skills' in line or '[task_executor' in line:
                g.write(line)
    portable_logs(out)
    print(f'{os.path.basename(out)} seed {args.seed}: rc={rc}, {time.time() - t0:.0f} s',
          flush=True)
    return rc


if __name__ == '__main__':
    sys.exit(main())
