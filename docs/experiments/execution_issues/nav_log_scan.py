#!/usr/bin/env python3
"""
历史运行日志中的 Nav2 导航段统计（第 2 节）：扫描 ROS 日志目录中 Nav2 组件容器的日志文件
（component_container_isolated_*.log，每次启动一个），把每次导航（“Begin navigating” 到
“Goal succeeded / failed”）整理成一行.

  python3 nav_log_scan.py ~/.ros/log --out nav_history.csv

每行：启动序号、控制器类型、起点与目标、耗时（墙钟）、bt_navigator 报告的“等待服务器确认
目标超时”次数（按 action 分）、Spin / Wait / BackUp 的次数、Spin 超时、控制器“Failed to
make progress”、规划失败、结果。日志本身不提交（含本机路径），只提交 CSV。
"""
import argparse
import csv
import glob
import os
import re

LINE = re.compile(r'\[(?P<lvl>\w+)\] \[(?P<t>\d+\.\d+)\] \[(?P<who>[\w.]+)\]: (?P<msg>.*)')


def scan(path, launch_id):
    ctrl = ''
    navs, cur = [], None
    with open(path, errors='replace') as f:
        for line in f:
            m = LINE.search(line)
            if not m:
                continue
            msg, t = m['msg'], float(m['t'])
            if 'Created controller' in msg and 'FollowPath' in msg:
                ctrl = 'MPPI' if 'MPPI' in msg else ('DWB' if 'DWB' in msg or 'dwb' in msg
                                                     else msg.split('type')[-1].strip())
            if msg.startswith('Begin navigating'):
                if cur:
                    cur['result'] = 'preempted'
                    navs.append(cur)
                g = re.findall(r'\((-?[\d.]+), (-?[\d.]+)\)', msg)
                cur = dict(launch=launch_id, controller=ctrl, t0=t, start=g[0] if g else '',
                           goal=g[1] if len(g) > 1 else '', ack_follow_path=0,
                           ack_compute_path=0, ack_behavior=0, spins=0, waits=0, backups=0,
                           spin_time_exceeded=0, no_progress=0, plan_failed=0, result='')
                continue
            if cur is None:
                continue
            if 'acknowledge goal request for follow_path' in msg:
                cur['ack_follow_path'] += 1
            elif 'acknowledge goal request for compute_path' in msg:
                cur['ack_compute_path'] += 1
            elif 'acknowledge goal request for' in msg:
                cur['ack_behavior'] += 1
            elif msg == 'Running spin':
                cur['spins'] += 1
            elif msg == 'Running wait':
                cur['waits'] += 1
            elif msg == 'Running backup':
                cur['backups'] += 1
            elif 'Exceeded time allowance before reaching the Spin goal' in msg:
                cur['spin_time_exceeded'] += 1
            elif msg.startswith('Failed to make progress'):
                cur['no_progress'] += 1
            elif 'failed to generate a valid path' in msg:
                cur['plan_failed'] += 1
            elif msg.startswith('Goal succeeded') or msg.startswith('Goal failed') or \
                    msg.startswith('Goal canceled'):
                cur['result'] = msg.split()[1]
                cur['time_s'] = round(t - cur['t0'], 2)
                navs.append(cur)
                cur = None
    if cur:
        cur['result'] = 'unfinished'
        navs.append(cur)
    return navs


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('logdir')
    ap.add_argument('--out', default='nav_history.csv')
    args = ap.parse_args()
    files = sorted(glob.glob(os.path.join(os.path.expanduser(args.logdir),
                                          'component_container_isolated_*.log')),
                   key=lambda p: int(re.findall(r'_(\d+)\.log$', p)[0]))
    rows = []
    for k, path in enumerate(files):
        for n in scan(path, k):
            n.pop('t0')
            n['start'] = ' '.join(n['start']) if n['start'] else ''
            n['goal'] = ' '.join(n['goal']) if n['goal'] else ''
            rows.append(n)
    fields = ['launch', 'controller', 'start', 'goal', 'result', 'time_s', 'ack_follow_path',
              'ack_compute_path', 'ack_behavior', 'spins', 'waits', 'backups',
              'spin_time_exceeded', 'no_progress', 'plan_failed']
    with open(args.out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, '') for k in fields})
    print(f'{len(files)} container logs, {len(rows)} navigations')
    for ctrl in sorted({r['controller'] for r in rows}):
        sel = [r for r in rows if r['controller'] == ctrl]
        fp = [r for r in sel if r['ack_follow_path'] > 0]
        sp = [r for r in sel if r['spins'] > 0]
        sp_fp = [r for r in sp if r['ack_follow_path'] + r['ack_behavior'] > 0]
        print(f'{ctrl or "?"}: {len(sel)} navigations, {len(fp)} with follow_path ack timeout, '
              f'{len(sp)} with Spin ({len(sp_fp)} of them after an ack timeout)')


if __name__ == '__main__':
    main()
