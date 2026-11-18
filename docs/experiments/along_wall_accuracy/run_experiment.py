#!/usr/bin/env python3
"""
沿墙精度实验的单次运行：启动无界面仿真 + 任务层 → 执行 11 站规划并记录真值 → 结束全部进程.

  python3 run_experiment.py <配置名> --seed 1 [--out runs/<配置名>_s1] [--bag <目录>]
  python3 run_experiment.py --list

每个配置只改技能服务器的精对位参数（标记地图、横移时的标记更新、横移比例修正）与
行为树的重新对位阈值，其余（仿真配置、规划、空切间隙、补偿）完全相同。打滑过程的随机
种子由 --seed 给出（sim_nav.launch.py 的 odom_seed），不同配置用同一组种子。

运行前先 source 工作空间（install/setup.bash）。仿真与 ROS 通信隔离在 ROS_DOMAIN_ID = 47、
GAZEBO_MASTER_URI = http://localhost:11351；结束时按进程组结束本次启动的全部进程，
再执行 fastdds shm clean。
"""
import argparse
import os
import shutil
import signal
import subprocess
import sys
import time

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
PLAN = os.path.join(REPO, 'docs', 'experiments', 'slot_execution', 'runs', 'dp', 'slot_plan.yaml')
MAPS = os.path.join(HERE, 'maps')

# 配置名 → (精对位参数覆盖, 重新对位阈值 m；0 = 关闭)
CONFIGS = {
    # 1 基线：一个标记，站间只用里程计外推 / 横移时视野内的标记继续更新
    'single_off': (dict(tag_map='', strafe_tag_updates=False, estimate_scale=False), 0.0),
    'single_on': (dict(tag_map='', strafe_tag_updates=True, estimate_scale=False), 0.0),
    # 同上但横移前不回观测位姿（与系统层改造报告中的 slot_ekf_tag 相同）
    'single_on_noobs': (dict(tag_map='', strafe_tag_updates=True, estimate_scale=False,
                             observe_between_stations=False), 0.0),
    # 2 多标记地图（间距 0.3 / 0.6 / 1.2 m；地图位置误差 ±3 mm）
    'tags_0.3': (dict(tag_map='tags_0.3', strafe_tag_updates=True, estimate_scale=False), 0.0),
    'tags_0.6': (dict(tag_map='tags_0.6', strafe_tag_updates=True, estimate_scale=False), 0.0),
    'tags_1.2': (dict(tag_map='tags_1.2', strafe_tag_updates=True, estimate_scale=False), 0.0),
    'tags_0.6_err3': (dict(tag_map='tags_0.6_err3', strafe_tag_updates=True,
                           estimate_scale=False), 0.0),
    # 3 一个标记 + 按行程重新对位
    'realign_0.15': (dict(tag_map='', strafe_tag_updates=True, estimate_scale=False), 0.15),
    'realign_0.3': (dict(tag_map='', strafe_tag_updates=True, estimate_scale=False), 0.3),
    'realign_0.6': (dict(tag_map='', strafe_tag_updates=True, estimate_scale=False), 0.6),
    # 4 横移比例修正（单独 / 与 2、3 组合）
    'scale_single': (dict(tag_map='', strafe_tag_updates=True, estimate_scale=True), 0.0),
    'scale_tags_1.2': (dict(tag_map='tags_1.2', strafe_tag_updates=True, estimate_scale=True),
                       0.0),
    'scale_tags_0.6': (dict(tag_map='tags_0.6', strafe_tag_updates=True, estimate_scale=True),
                       0.0),
    'scale_tags_0.6_err3': (dict(tag_map='tags_0.6_err3', strafe_tag_updates=True,
                                 estimate_scale=True), 0.0),
    'scale_realign_0.3': (dict(tag_map='', strafe_tag_updates=True, estimate_scale=True), 0.3),
    # 默认配置（不覆盖任何参数，验证配置文件与启动文件的默认值）
    'default': (None, None),
}

BAG_TOPICS = ['/odom', '/wheel/odom', '/ground_truth/odom', '/task_skills/wall_estimate',
              '/task_skills/status', '/cmd_vel', '/joint_states', '/imu', '/tf', '/tf_static',
              '/task_executor/station_records', '/scan', '/clock']


def portable(cmd):
    """记录的命令行改用仓库相对路径与 ~，便于在其他机器上复现."""
    repo = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
    return cmd.replace(repo + os.sep, '').replace(os.path.expanduser('~') + os.sep, '~/')


def isolated_env():
    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = '47'
    env['GAZEBO_MASTER_URI'] = 'http://localhost:11351'
    env['PYTHONUNBUFFERED'] = '1'
    return env


def params_file(name, out):
    """把配置写成技能服务器的参数文件（标记地图写绝对路径），返回路径；默认配置返回 None."""
    over, _ = CONFIGS[name]
    if over is None:
        return None
    a = dict(over)
    if a.get('tag_map'):
        a['tag_map'] = os.path.join(MAPS, a['tag_map'] + '.yaml')
    base = os.path.join(REPO, 'src', 'scr_arm_controller', 'config', 'fine_alignment.yaml')
    with open(base, encoding='utf-8') as f:
        p = yaml.safe_load(f)
    p['/**']['ros__parameters']['alignment'].update(a)
    path = os.path.join(out, 'alignment_params.yaml')
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(p, f, allow_unicode=True, sort_keys=False)
    return path


def kill_group(proc, name):
    if proc is None or proc.poll() is not None:
        return
    for sig, wait in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=wait)
            return
        except subprocess.TimeoutExpired:
            print(f'{name}: {sig.name} 后仍未退出', flush=True)


def leftover(env):
    """本域遗留的仿真 / ROS 进程（按环境变量 GAZEBO_MASTER_URI / ROS_DOMAIN_ID 识别）."""
    pids = []
    for pid in os.listdir('/proc'):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            with open(f'/proc/{pid}/environ', 'rb') as f:
                e = f.read().split(b'\0')
        except OSError:
            continue
        if (b'GAZEBO_MASTER_URI=' + env['GAZEBO_MASTER_URI'].encode()) in e and \
                (b'ROS_DOMAIN_ID=' + env['ROS_DOMAIN_ID'].encode()) in e:
            pids.append(int(pid))
    return pids


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('config', nargs='?')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--out')
    ap.add_argument('--bag', help='同时录制 rosbag2 到该目录（仓库之外）')
    ap.add_argument('--timeout', type=float, default=1500.0, help='整次运行的上限（墙钟秒）')
    ap.add_argument('--list', action='store_true')
    args = ap.parse_args()
    if args.list or not args.config:
        for k, (over, realign) in CONFIGS.items():
            print(f'{k:22s} {over}  realign={realign}')
        return 0
    if args.config not in CONFIGS:
        sys.exit(f'未知配置 {args.config}')
    out = os.path.abspath(args.out or os.path.join(HERE, 'runs', f'{args.config}_s{args.seed}'))
    os.makedirs(out, exist_ok=True)
    env = isolated_env()
    pf = params_file(args.config, out)
    realign = CONFIGS[args.config][1]

    launch = ['ros2', 'launch', 'scr_bringup', 'sim_nav.launch.py', 'gui:=false', 'rviz:=false',
              f'odom_seed:={args.seed}']
    if pf:
        launch.append(f'alignment_params:={pf}')
    rec = ['ros2', 'run', 'scr_planner', 'execute_slot_plan.py', PLAN, out, '--dry-run-gap=0.01']
    if realign is not None:
        rec.append(f'--realign-distance={realign if realign > 0 else -1.0}')
    with open(os.path.join(out, 'command.txt'), 'w') as f:
        f.write(portable(' '.join(launch)) + '\n' + portable(' '.join(rec)) + '\n')

    t0 = time.time()
    procs = {}
    launch_log = open(os.path.join(out, 'launch.log'), 'w')
    run_log = open(os.path.join(out, 'run.log'), 'w')
    rc = 1
    try:
        procs['launch'] = subprocess.Popen(launch, env=env, stdout=launch_log,
                                           stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(20.0)
        if args.bag:
            if os.path.exists(args.bag):
                shutil.rmtree(args.bag)
            os.makedirs(os.path.dirname(os.path.abspath(args.bag)), exist_ok=True)
            procs['bag'] = subprocess.Popen(
                ['ros2', 'bag', 'record', '-o', args.bag, '--use-sim-time'] + BAG_TOPICS,
                env=env, stdout=open(os.path.join(out, 'bag.log'), 'w'),
                stderr=subprocess.STDOUT, start_new_session=True)
        procs['rec'] = subprocess.Popen(rec, env=env, stdout=run_log, stderr=subprocess.STDOUT,
                                        start_new_session=True)
        while procs['rec'].poll() is None:
            if time.time() - t0 > args.timeout:
                print('超时，结束运行', flush=True)
                break
            if procs['launch'].poll() is not None:
                print('启动文件提前退出', flush=True)
                break
            time.sleep(2.0)
        rc = procs['rec'].poll()
        rc = 1 if rc is None else rc
    finally:
        kill_group(procs.get('rec'), 'recorder')
        kill_group(procs.get('bag'), 'bag')
        kill_group(procs.get('launch'), 'launch')
        launch_log.close()
        run_log.close()
        time.sleep(2.0)
        for pid in leftover(env):
            try:
                os.kill(pid, signal.SIGKILL)
                print(f'结束遗留进程 {pid}', flush=True)
            except ProcessLookupError:
                pass
        subprocess.run('fastdds shm clean', shell=True, env=env, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
    # 任务层日志（技能服务器与行为树执行器）单独保存，完整启动日志不提交
    with open(os.path.join(out, 'launch.log'), errors='replace') as f, \
            open(os.path.join(out, 'task_layer.log'), 'w') as g:
        for line in f:
            if '[task_skills' in line or '[task_executor' in line:
                g.write(line)
    print(f'{args.config} seed {args.seed}: rc={rc}, {time.time() - t0:.0f} s → {out}',
          flush=True)
    return rc


if __name__ == '__main__':
    sys.exit(main())
