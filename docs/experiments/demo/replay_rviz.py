#!/usr/bin/env python3
"""
用 rosbag2 回放一次执行，在 RViz 中检查 scr_viz plan_viz 的刀尖轨迹并按仿真时间截图.

  python3 replay_rviz.py <bag> --task <task.yaml> --plan <plan.yaml> --out <目录>
      [--at 120 200 ...] [--rate 1.0] [--config rviz_replay.rviz]

plan_viz 与 RViz 都用仿真时间（/clock 来自包内录制）；plan_viz 从回放的 /gazebo/link_states
推算刀尖轨迹，与执行时在线运行的计算完全相同。--at 为相对包内第一条 /clock 的仿真秒数，
到达时用 xwd 抓取 RViz 窗口（Wayland 下 RViz 走 XWayland），存为 <目录>/rviz_<t>.png；
回放结束时再抓一张 rviz_end.png。

运行在 ROS_DOMAIN_ID = 48；结束时按进程组结束本脚本启动的全部进程，再执行 fastdds shm clean。
"""
import argparse
import os
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))


def env():
    e = dict(os.environ)
    e['ROS_DOMAIN_ID'] = '48'
    e.setdefault('DISPLAY', ':0')
    return e


def rviz_window():
    """标题含 RViz 的顶层 X 窗口 id（没有时返回 None）."""
    out = subprocess.run(['xwininfo', '-root', '-tree'], capture_output=True, text=True).stdout
    for line in out.splitlines():
        if 'RViz' in line and '"' in line:
            wid = line.split()[0]
            if wid.startswith('0x'):
                return wid
    return None


def grab(wid, path):
    xwd = subprocess.run(['xwd', '-silent', '-id', wid], capture_output=True, check=True)
    subprocess.run(['convert', 'xwd:-', path], input=xwd.stdout, check=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--task', required=True)
    ap.add_argument('--plan', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--at', type=float, nargs='*', default=[])
    ap.add_argument('--rate', type=float, default=1.0)
    ap.add_argument('--config', default=os.path.join(HERE, 'rviz_replay.rviz'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    e = env()
    procs = []

    def start(cmd, log):
        p = subprocess.Popen(cmd, env=e, stdout=open(os.path.join(a.out, log), 'w'),
                             stderr=subprocess.STDOUT, start_new_session=True)
        procs.append(p)
        return p

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    try:
        start(['ros2', 'run', 'scr_viz', 'plan_viz', '--ros-args', '-p',
               f'task_file:={os.path.abspath(a.task)}', '-p',
               f'plan_file:={os.path.abspath(a.plan)}', '-p', 'use_sim_time:=true'],
              'plan_viz.log')
        start(['rviz2', '-d', a.config, '--ros-args', '-p', 'use_sim_time:=true'], 'rviz.log')
        wid = None
        for _ in range(60):
            wid = rviz_window()
            if wid:
                break
            time.sleep(1.0)
        if not wid:
            print('没有找到 RViz 窗口')
            return 1
        time.sleep(5.0)
        os.environ['ROS_DOMAIN_ID'] = e['ROS_DOMAIN_ID']
        rclpy.init()
        node = rclpy.create_node('replay_capture')
        clock = {'t0': None, 't': None}

        def on_clock(m):
            t = m.clock.sec + m.clock.nanosec * 1e-9
            if clock['t0'] is None:
                clock['t0'] = t
            clock['t'] = t - clock['t0']
        # 回放的 /clock 为 best effort
        node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
        play = start(['ros2', 'bag', 'play', a.bag, '--rate', f'{a.rate:g}'], 'play.log')
        targets = sorted(a.at)
        while play.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.05)
            if targets and clock['t'] is not None and clock['t'] >= targets[0]:
                t = targets.pop(0)
                p = os.path.join(a.out, f'rviz_{t:.0f}.png')
                grab(wid, p)
                print(f'{p}（仿真 {clock["t"]:.1f} s）', flush=True)
        time.sleep(2.0)
        grab(wid, os.path.join(a.out, 'rviz_end.png'))
        print('回放结束', flush=True)
        node.destroy_node()
        rclpy.shutdown()
        return 0
    finally:
        for p in procs[::-1]:
            if p.poll() is None:
                try:
                    os.killpg(p.pid, signal.SIGINT)
                    p.wait(timeout=10)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    try:
                        os.killpg(p.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
        subprocess.run('fastdds shm clean', shell=True, env=e, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)


if __name__ == '__main__':
    sys.exit(main())
