#!/usr/bin/env python3
"""
冷启动试验：反复启动整套仿真（Gazebo + Nav2 + 任务层），记录任务层是否在限定时间内就绪.

  python3 cold_start.py --out runs/cold_start_composition --count 15
      [--set use_composition:=False ...] [--timeout 180] [--hold 15] [--stress 3]

每次启动：
  - 等行为树执行器打印 “Task executor active”（它在 Nav2 激活、技能服务器就绪后才激活）；
  - 就绪后再观察 --hold 秒，检查是否有进程退出；
  - 未就绪时记录诊断：启动日志中已加载 / 缺少的 Nav2 组件、退出的进程、
    容器里实际存在的组件（ros2 component list）、本域的节点列表，以及启动日志末尾。
--stress N：启动期间另开 N 个占满一个核的忙循环进程，模拟批量实验时机器被其他计算任务
占用的情形（此前的启动失败都出现在这种时候）。
结果：<out>/cold_starts.csv，失败的启动日志摘录在 <out>/failures/。
"""
import argparse
import csv
import os
import re
import subprocess
import time

import sim_session as ss


def probe(cmd, env, timeout=25):
    try:
        r = subprocess.run(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           text=True, timeout=timeout)
        return r.stdout
    except subprocess.TimeoutExpired:
        return '<timeout>'


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('--out', required=True)
    ap.add_argument('--count', type=int, default=15)
    ap.add_argument('--set', action='append', default=[], help='额外的 launch 参数 key:=value')
    ap.add_argument('--timeout', type=float, default=180.0, help='就绪上限（墙钟秒）')
    ap.add_argument('--hold', type=float, default=15.0, help='就绪后继续观察的时间（墙钟秒）')
    ap.add_argument('--label', default='')
    ap.add_argument('--stress', type=int, default=0, help='启动期间的忙循环进程数')
    args = ap.parse_args()
    out = os.path.abspath(args.out)
    os.makedirs(os.path.join(out, 'failures'), exist_ok=True)
    csv_path = os.path.join(out, 'cold_starts.csv')
    done = 0
    if os.path.exists(csv_path):
        with open(csv_path) as f:
            done = sum(1 for _ in csv.DictReader(f))
    fields = ['attempt', 'label', 'launch_args', 'ready', 't_ready_s', 'loaded', 'missing',
              'died_before_ready', 'died_after_ready', 'container_components', 'note']
    with open(os.path.join(out, 'command.txt'), 'w') as f:
        f.write(ss.rv.portable(' '.join(ss.launch_cmd(args.set))) + '\n')
    for k in range(done, args.count):
        burners = [subprocess.Popen(['python3', '-c', 'while True: pass'])
                   for _ in range(args.stress)]
        try:
            ready, t_ready, diag, died_before, died_after, comps = one_start(args, out, k)
        finally:
            for b in burners:
                b.kill()
                b.wait()
        row = dict(attempt=k + 1, label=args.label, launch_args=' '.join(args.set),
                   ready=int(ready), t_ready_s=round(t_ready, 1), loaded=len(diag['loaded']),
                   missing=' '.join(diag['missing']) if not ready else '',
                   died_before_ready=' '.join(died_before),
                   died_after_ready=' '.join(died_after),
                   container_components=len(re.findall(r'^\s+\d+\s+/', comps, re.M)) if comps
                   else '', note='' if ready else 'not ready')
        new = not os.path.exists(csv_path)
        with open(csv_path, 'a', newline='') as f:
            w = csv.DictWriter(f, fieldnames=fields)
            if new:
                w.writeheader()
            w.writerow(row)
        print(f'[{k + 1}/{args.count}] ready={ready} t={t_ready:.0f}s '
              f'loaded={len(diag["loaded"])} died={died_before + died_after}', flush=True)


def one_start(args, out, k):
    """一次冷启动；返回 (就绪, 墙钟秒, 诊断, 就绪前退出的进程, 就绪后退出的进程, 组件列表)."""
    with ss.Session(out, args.set, log_name='launch.log') as s:
        ready, t_ready = s.wait_ready(args.timeout)
        died_before = ss.launch_diagnosis(s.log_text())['died']
        comps = nodes = ''
        if ready:
            time.sleep(args.hold)
        else:
            comps = probe(['ros2', 'component', 'list'], s.env)
            nodes = probe(['ros2', 'node', 'list'], s.env)
        text = s.log_text()
        diag = ss.launch_diagnosis(text)
        died_after = [d for d in diag['died'] if d not in died_before]
        if not ready or died_after:
            with open(os.path.join(out, 'failures', f'attempt_{k + 1:02d}.log'), 'w') as f:
                f.write(f'ready={ready} t={t_ready:.1f}s\n--- ros2 component list\n{comps}\n')
                if not ready:
                    f.write(f'--- ros2 node list\n{nodes}\n')
                f.write('--- launch.log (lines mentioning nav2 / container / errors, '
                        'and the last 60 lines)\n')
                lines = text.splitlines()
                keep = [ln for ln in lines if re.search(
                    r'nav2_container|Loaded node|lifecycle_manager|ERROR|WARN|died|'
                    r'Dependencies|Waiting for Nav2', ln)]
                f.write(ss.rv.portable('\n'.join(keep[-150:] + ['...'] + lines[-60:])))
    return ready, t_ready, diag, died_before, died_after, comps


if __name__ == '__main__':
    main()
