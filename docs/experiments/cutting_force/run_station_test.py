#!/usr/bin/env python3
"""
带切割力的多站切割试验的单次运行：无界面仿真 + 任务层 → 执行规划文件并记录真值 → 结束进程.

  python3 run_station_test.py <规划文件> <结果目录> --seed 1 --hold dob [--ff 0.7]
      [--feed 0.0055 --plunge 0.0025 | --force-limit] [--ramp 2.0] [--no-split]
      [--material brick] [--apply-to base|tool] [--no-force] [--limits raised|default]
      [--set key=value ...] [--bag <目录>]

- 切割力插件以话题方式取切深（执行器按轨迹时间表发布），力加在底盘（默认）或刀具上；
  空切时刀尖停在墙前 10 mm，插件的 tip_offset 加上“间隙 + 槽深”，使刀片中心位于真实切到
  槽深时的位置（底盘前方 0.66 m）；
- --hold 选择定点保持的控制律，--ff 给出前馈比例（0 = 不用前馈；0.7 表示假设模型低估
  切削力 30 %）；
- --limits raised（默认）：底盘偏航力矩上限 I_z × 75 rad/s²、指令限幅 0.25 m/s / 4 rad/s；
- --feed / --plunge 直接给出进给与下刀速度（m/s）；--force-limit 改由切割力模型给出上限；
- --set 覆盖其余 arm_motion.* 参数。
实际使用的技能服务器参数写入 <结果目录>/skill_params.yaml。运行前先 source 工作空间。
仿真与 ROS 通信隔离在 ROS_DOMAIN_ID = 49、GAZEBO_MASTER_URI = http://localhost:11353。
"""
import argparse
import os
import sys
import time

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(HERE, '..', 'plan_validation'))
sys.path.insert(0, os.path.join(HERE, '..', 'arm_trajectory'))
import run_validation as rv  # noqa: E402
from run_experiment import portable_logs  # noqa: E402  （同时把 rv 的域设为 49 / 11353）

# 记录包的话题：不含 /tf、/joint_states（1 kHz）与激光，切割一趟十几分钟，否则每包超过 1 GB
rv.BAG_TOPICS = ['/odom', '/wheel/odom', '/ground_truth/odom', '/task_skills/wall_estimate',
                 '/task_skills/status', '/task_executor/station_records', '/cmd_vel', '/imu',
                 '/clock', '/arm_controller/controller_state', '/cutting_force/wrench',
                 '/cutting_force/expected_wrench', '/cutting_force/engagement']
DRY_GAP, SLOT_DEPTH, TOOL_OFFSET = 0.010, 0.035, 0.1594
LIMITS = {
    'default': dict(launch=[], hold={}),
    'raised': dict(launch=['base_max_ang_accel:=75'], hold={'max_lin': 0.25, 'max_ang': 4.0}),
}


def skill_params(args):
    """默认精对位参数 + 定点保持与机械臂运动参数 → 技能服务器参数文件内容."""
    with open(os.path.join(REPO, 'src', 'scr_arm_controller', 'config', 'fine_alignment.yaml'),
              encoding='utf-8') as f:
        params = yaml.safe_load(f)
    hold = dict(LIMITS[args.limits]['hold'], mode=args.hold)
    if args.ff > 0.0:
        hold.update(feedforward_topic='/cutting_force/expected_wrench',
                    feedforward_gain=args.ff)
    arm = dict(material=args.material, feed_ramp_time=args.ramp, split_plunge=not args.no_split,
               scale_dry_plunge=True, cut_dwell=args.cut_dwell)
    if args.force_limit:
        arm['feed_basis'] = 'force_limit'
    else:
        arm.update(feed_rate=args.feed, plunge_rate=args.plunge)
    for item in args.set:
        key, _, value = item.partition('=')
        arm[key] = yaml.safe_load(value)
    p = params['/**']['ros__parameters']
    p['hold'], p['arm_motion'] = hold, arm
    return params


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('plan')
    ap.add_argument('out')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--hold', default='p', choices=['p', 'pi', 'pid', 'dob', 'lqg'])
    ap.add_argument('--ff', type=float, default=0.0, help='前馈比例，0 = 不用')
    ap.add_argument('--feed', type=float, default=0.0055)
    ap.add_argument('--plunge', type=float, default=0.0025)
    ap.add_argument('--force-limit', action='store_true')
    ap.add_argument('--ramp', type=float, default=2.0, help='进给 / 下刀速度的建立时间（秒）')
    ap.add_argument('--cut-dwell', type=float, default=1.0, help='切完后退刀前的等待（秒）')
    ap.add_argument('--no-split', action='store_true', help='下刀段不在接触点停顿')
    ap.add_argument('--material', default='brick')
    ap.add_argument('--apply-to', default='base', choices=['base', 'tool'])
    ap.add_argument('--velocity-tau', type=float, default=None,
                    help='插件中进给速度低通的时间常数（秒），默认取插件的默认值')
    ap.add_argument('--no-force', action='store_true', help='不加载切割力插件（空切对照）')
    ap.add_argument('--force-seed', type=int, default=None,
                    help='材料不均匀性的种子（默认与 --seed 相同）')
    ap.add_argument('--limits', default='raised', choices=sorted(LIMITS))
    ap.add_argument('--set', action='append', default=[], metavar='KEY=VALUE')
    ap.add_argument('--bag', help='同时录制 rosbag2 到该目录（仓库之外）')
    ap.add_argument('--timeout', type=float, default=3600.0, help='整次运行的上限（墙钟秒）')
    ap.add_argument('--startup-timeout', type=float, default=240.0)
    ap.add_argument('--retries', type=int, default=2)
    args = ap.parse_args()
    plan = os.path.abspath(os.path.expanduser(args.plan))
    out = os.path.abspath(os.path.expanduser(args.out))
    os.makedirs(out, exist_ok=True)
    env = rv.isolated_env()

    params_file = os.path.join(out, 'skill_params.yaml')
    with open(params_file, 'w', encoding='utf-8') as f:
        yaml.safe_dump(skill_params(args), f, allow_unicode=True, sort_keys=False)
    launch = ['ros2', 'launch', 'scr_bringup', 'sim_nav.launch.py', 'gui:=false', 'rviz:=false',
              f'odom_seed:={args.seed}', f'alignment_params:={params_file}']
    launch += LIMITS[args.limits]['launch']
    if not args.no_force:
        from scr_arm_controller.cutting_force import MATERIALS
        seed = args.seed if args.force_seed is None else args.force_seed
        plugin = (f'apply_to={args.apply_to};engagement=topic;seed={seed};'
                  f'specific_energy={MATERIALS[args.material]:.4g};'
                  f'tip_offset={TOOL_OFFSET + DRY_GAP + SLOT_DEPTH:.4f}')
        if args.velocity_tau is not None:
            plugin += f';velocity_tau={args.velocity_tau}'
        launch += ['cutting_force:=true', f'cutting_force_params:={plugin}']
    rec = ['ros2', 'run', 'scr_planner', 'execute_slot_plan.py', plan, out,
           f'--dry-run-gap={DRY_GAP}']
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
            if '[task_skills' in line or '[task_executor' in line or 'scr_cutting_force' in line:
                g.write(line)
    portable_logs(out)
    print(f'{os.path.basename(out)} seed {args.seed}: rc={rc}, {time.time() - t0:.0f} s',
          flush=True)
    return rc


if __name__ == '__main__':
    sys.exit(main())
