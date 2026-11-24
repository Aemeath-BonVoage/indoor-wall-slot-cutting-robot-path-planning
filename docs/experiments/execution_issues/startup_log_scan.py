#!/usr/bin/env python3
"""
历史启动记录（第 4 节）：扫描 ROS 日志目录中行为树执行器的日志（task_executor_*.log，每次启动
一个），按结果分类；对失败的启动，找出同一时刻启动的 Nav2 组件容器日志，统计其中的加载记录.

  python3 startup_log_scan.py ~/.ros/log --out startup_history.csv

结果：active（执行器激活）、failed（“Dependencies did not come up”）、incomplete（日志没有
结论，即启动途中被结束）。日志本身不提交，只提交 CSV。
"""
import argparse
import csv
import glob
import os
import re


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('logdir')
    ap.add_argument('--out', default='startup_history.csv')
    args = ap.parse_args()
    d = os.path.expanduser(args.logdir)
    stamp = re.compile(r'_(\d{10})\d*\.log$')
    containers = {}
    for f in glob.glob(os.path.join(d, 'component_container_isolated_*.log')):
        m = stamp.search(f)
        if m:
            containers[int(m[1])] = f
    rows = []
    for k, f in enumerate(sorted(glob.glob(os.path.join(d, 'task_executor_*.log')),
                                 key=lambda p: int(stamp.search(p)[1]))):
        with open(f, errors='replace') as g:
            text = g.read()
        if 'Task executor active' in text:
            result = 'active'
        elif 'Dependencies did not come up' in text:
            result = 'failed'
        else:
            result = 'incomplete'
        t = int(stamp.search(f)[1])
        # 同一次启动的组件容器：启动时刻相差 10 s 以内
        near = [c for tc, c in containers.items() if abs(tc - t) <= 10]
        loads = ''
        if near:
            with open(near[0], errors='replace') as g:
                loads = sum(1 for line in g if 'Load Library' in line)
        rows.append(dict(start=k, result=result, container_log_found=int(bool(near)),
                         container_load_library_lines=loads))
    with open(args.out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    for res in ('active', 'failed', 'incomplete'):
        sel = [r for r in rows if r['result'] == res]
        print(f'{res}: {len(sel)}', [r['container_load_library_lines'] for r in sel]
              if res != 'active' else '')


if __name__ == '__main__':
    main()
