#!/usr/bin/env python3
"""
规划验证实验的单次运行：无界面仿真 + 任务层 → 执行一份规划文件并记录真值 → 结束全部进程.

  python3 run_validation.py <规划文件> <结果目录> --seed 1 [--bag <目录>]
      [--fixtures <墙面任务.yaml>] [--camera <帧目录> --presets oblique overview --rate 2]

- 仿真、规划之外的设置全部取默认值（精对位参数、行为树、空切间隙 10 mm、残差补偿），
  与沿墙精度实验的默认配置相同；打滑过程的随机种子由 --seed 给出（odom_seed）。
- --bag：同时录制 rosbag2（仓库之外），话题见 BAG_TOPICS；
- --fixtures：执行前在墙面生成线盒 / 开关等外观实体（无碰撞体，scr_viz spawn_wall_fixtures）；
- 启动看门狗：Nav2 组件偶尔加载超时（load_node 无响应），行为树执行器等不到依赖而退出；
  这时整次重跑（--retries），失败记录写入 startup_failures.txt；
- --camera：执行期间用演示相机按仿真时间存帧（scr_viz demo_record），每个预设视角一台相机，
  帧存到 <帧目录>/<预设>/。

运行前先 source 工作空间（install/setup.bash）。仿真与 ROS 通信隔离在 ROS_DOMAIN_ID = 48、
GAZEBO_MASTER_URI = http://localhost:11352；结束时按进程组结束本次启动的全部进程，
再执行 fastdds shm clean。
"""
import argparse
import os
import shutil
import signal
import subprocess
import sys
import time

DOMAIN = '48'
MASTER = 'http://localhost:11352'
BAG_TOPICS = ['/odom', '/wheel/odom', '/ground_truth/odom', '/task_skills/wall_estimate',
              '/task_skills/status', '/task_executor/station_records', '/cmd_vel',
              '/joint_states', '/imu', '/scan', '/tf', '/tf_static', '/clock', '/plan',
              '/map', '/robot_description', '/gazebo/link_states']


def isolated_env():
    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = DOMAIN
    env['GAZEBO_MASTER_URI'] = MASTER
    env['PYTHONUNBUFFERED'] = '1'
    return env


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


def startup_failed(out, t_rec, timeout):
    """记录器启动 timeout 秒后仍未收到任务反馈，或执行器报告依赖没有起来."""
    with open(os.path.join(out, 'launch.log'), errors='replace') as f:
        if 'Dependencies did not come up' in f.read():
            return True
    with open(os.path.join(out, 'run.log'), errors='replace') as f:
        fed = 'NavigateToMarker' in f.read()
    return not fed and time.time() - t_rec > timeout


def attempt(args, env, out, launch, fix, cams, rec):
    """一次完整运行；返回 (记录器返回码, 任务层是否正常启动)."""
    t0 = time.time()
    procs = {}
    launch_log = open(os.path.join(out, 'launch.log'), 'w')
    run_log = open(os.path.join(out, 'run.log'), 'w')
    rc, started = 1, True
    try:
        procs['launch'] = subprocess.Popen(launch, env=env, stdout=launch_log,
                                           stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(20.0)
        if fix:
            r = subprocess.run(fix, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, timeout=120)
            with open(os.path.join(out, 'fixtures.log'), 'w') as f:
                f.write(r.stdout)
            print(f'墙面实体：rc={r.returncode}', flush=True)
        if args.bag:
            if os.path.exists(args.bag):
                shutil.rmtree(args.bag)
            os.makedirs(os.path.dirname(os.path.abspath(args.bag)), exist_ok=True)
            procs['bag'] = subprocess.Popen(
                ['ros2', 'bag', 'record', '-o', args.bag, '--use-sim-time'] + BAG_TOPICS,
                env=env, stdout=open(os.path.join(out, 'bag.log'), 'w'),
                stderr=subprocess.STDOUT, start_new_session=True)
        for p, c in cams.items():
            procs[f'cam_{p}'] = subprocess.Popen(
                c, env=env, stdout=open(os.path.join(out, f'camera_{p}.log'), 'w'),
                stderr=subprocess.STDOUT, start_new_session=True)
            time.sleep(5.0)
        procs['rec'] = subprocess.Popen(rec, env=env, stdout=run_log, stderr=subprocess.STDOUT,
                                        start_new_session=True)
        t_rec = time.time()
        while procs['rec'].poll() is None:
            if startup_failed(out, t_rec, args.startup_timeout):
                print('任务层没有启动', flush=True)
                started = False
                break
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
        for p in cams:
            kill_group(procs.get(f'cam_{p}'), f'camera {p}')
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
    return rc, started


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('plan')
    ap.add_argument('out')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--bag', help='同时录制 rosbag2 到该目录（仓库之外）')
    ap.add_argument('--fixtures', help='墙面任务 YAML：执行前生成墙面外观实体')
    ap.add_argument('--camera', help='演示相机帧目录（仓库之外）')
    ap.add_argument('--presets', nargs='+', default=['oblique'])
    ap.add_argument('--rate', type=float, default=2.0, help='演示相机每仿真秒帧数')
    ap.add_argument('--cam-size', type=int, nargs=2, default=(800, 450))
    ap.add_argument('--timeout', type=float, default=1500.0, help='整次运行的上限（墙钟秒）')
    ap.add_argument('--startup-timeout', type=float, default=240.0,
                    help='记录器启动后多久仍无任务反馈即判为启动失败（墙钟秒）')
    ap.add_argument('--retries', type=int, default=2, help='启动失败时的重跑次数')
    args = ap.parse_args()
    plan = os.path.abspath(os.path.expanduser(args.plan))
    out = os.path.abspath(os.path.expanduser(args.out))
    os.makedirs(out, exist_ok=True)
    env = isolated_env()

    launch = ['ros2', 'launch', 'scr_bringup', 'sim_nav.launch.py', 'gui:=false', 'rviz:=false',
              f'odom_seed:={args.seed}']
    rec = ['ros2', 'run', 'scr_planner', 'execute_slot_plan.py', plan, out, '--dry-run-gap=0.01']
    fix = (['ros2', 'run', 'scr_viz', 'spawn_wall_fixtures',
            os.path.abspath(os.path.expanduser(args.fixtures)), '--spawn']
           if args.fixtures else None)
    cams = {p: ['ros2', 'run', 'scr_viz', 'demo_record', '--out',
                os.path.join(os.path.abspath(os.path.expanduser(args.camera)), p),
                '--name', f'demo_{p}', '--preset', p, '--rate', f'{args.rate:g}',
                '--width', str(args.cam_size[0]), '--height', str(args.cam_size[1]),
                '--cam-fps', f'{max(4.0, 2 * args.rate):g}', '--duration', str(int(args.timeout))]
            for p in (args.presets if args.camera else [])}
    with open(os.path.join(out, 'command.txt'), 'w') as f:
        for c in [launch, fix] + list(cams.values()) + [rec]:
            if c:
                f.write(' '.join(c) + '\n')

    t0 = time.time()
    rc = 1
    for k in range(args.retries + 1):
        rc, started = attempt(args, env, out, launch, fix, cams, rec)
        if started:
            break
        # 启动失败（Nav2 组件或行为树执行器没有起来）：与规划无关，清理后整次重跑，并记录
        with open(os.path.join(out, 'startup_failures.txt'), 'a') as f:
            f.write(f'attempt {k + 1}: task layer did not accept the goal within '
                    f'{args.startup_timeout:.0f} s; launch.log tail:\n')
            with open(os.path.join(out, 'launch.log'), errors='replace') as g:
                f.writelines(g.readlines()[-15:])
        print(f'第 {k + 1} 次启动失败，重跑', flush=True)
    # 任务层日志（技能服务器与行为树执行器）单独保存，完整启动日志不提交
    with open(os.path.join(out, 'launch.log'), errors='replace') as f, \
            open(os.path.join(out, 'task_layer.log'), 'w') as g:
        for line in f:
            if '[task_skills' in line or '[task_executor' in line:
                g.write(line)
    print(f'{os.path.basename(out)} seed {args.seed}: rc={rc}, {time.time() - t0:.0f} s',
          flush=True)
    return rc


if __name__ == '__main__':
    sys.exit(main())
