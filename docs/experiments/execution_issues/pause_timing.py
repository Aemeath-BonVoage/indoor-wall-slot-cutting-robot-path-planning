#!/usr/bin/env python3
"""
机械臂轨迹目标的往返计时（第 3 节，不需要 Gazebo）：从运行目录的 launch.log 中提取
技能服务器打印的“轨迹计时”行与机械臂控制器（joint_trajectory_controller）的
“Received / Accepted new action goal”行，按墙钟对齐.

  python3 pause_timing.py <运行目录> [<运行目录> ...] --out goal_timing.csv

每个轨迹目标一行（全部为墙钟时间）：轨迹时长、发送→被接受、被接受→收到结果、两者之和、
往返墙钟时间减去名义时长（实时因子约 1）、控制器日志中收到 / 接受目标相对发送的时刻，
以及计时行给出的仿真时间额外耗时 extra_sim_s。extra_sim_s 只对 p2p_rule_s1 / s2 之后的运行
有效：此前计时取的是机械臂客户端节点的时钟，它只在等待动作时被 spin，会停在上一次 spin 的
时刻（这两次运行只用墙钟列）。
launch.log 不提交（含本机路径），提交的是提取后的 CSV。
"""
import argparse
import csv
import os
import re

TIMING = re.compile(
    r'\[INFO\] \[(?P<wall>\d+\.\d+)\] \[[\w.]+\]: 轨迹计时: '
    r'时长 (?P<dur>[\d.]+) s，发送→接受 (?P<ack_w>-?\d+) ms 墙钟 / (?P<ack_s>-?\d+) ms 仿真，'
    r'接受→结果 (?P<res_w>[\d.]+) s 墙钟 / (?P<res_s>[\d.]+) s 仿真，'
    r'额外 (?P<extra>[+-][\d.]+) s 仿真 \(发送 (?P<send>[\d.]+)，结果 (?P<result>[\d.]+)\)')
JTC = re.compile(r'\[INFO\] \[(?P<wall>\d+\.\d+)\] \[arm_controller\]: '
                 r'(?P<what>Received|Accepted) new action goal')


def parse(run):
    with open(os.path.join(run, 'launch.log'), errors='replace') as f:
        text = f.read()
    jtc = [(float(m['wall']), m['what']) for m in JTC.finditer(text)]
    rows = []
    for k, m in enumerate(TIMING.finditer(text)):
        w_result = float(m['wall'])
        res_w = float(m['res_w'])
        ack_w = float(m['ack_w']) / 1e3
        w_send = w_result - res_w - ack_w

        def near(what):
            # 控制器的日志行与发送时刻相差应在毫秒级；只在 [-20, +200] ms 内找
            c = [w - w_send for w, x in jtc if x == what and -0.02 <= w - w_send <= 0.2]
            return round(1e3 * min(c, key=abs), 1) if c else ''
        rows.append(dict(
            run=os.path.basename(os.path.normpath(run)), goal=k, duration_s=float(m['dur']),
            ack_wall_ms=float(m['ack_w']), result_wall_s=res_w,
            total_wall_s=round(ack_w + res_w, 3),
            # 仿真时间给出的额外耗时：收到结果 − 发送 − 名义时长（仅对计时取技能节点时钟之后的
            # 运行有效，见文件头）
            extra_sim_s=float(m['extra']),
            jtc_received_ms=near('Received'), jtc_accepted_ms=near('Accepted')))
    for r in rows:
        # 墙钟额外耗时：往返的墙钟时间 − 名义时长（实时因子约 1，略大于 0 是正常的）
        r['excess_wall_s'] = round(r['total_wall_s'] - r['duration_s'], 3)
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--out', default='goal_timing.csv')
    args = ap.parse_args()
    rows = []
    for r in args.runs:
        rows += parse(r)
    with open(args.out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    import numpy as np
    for run in sorted({r['run'] for r in rows}):
        sel = [r for r in rows if r['run'] == run]
        ack = np.array([r['ack_wall_ms'] for r in sel])
        ex = np.array([r['excess_wall_s'] for r in sel])
        big = [r for r in sel if r['ack_wall_ms'] > 100 or r['excess_wall_s'] > 0.8]
        print(f'{run}: {len(sel)} goals, ack median {np.median(ack):.0f} ms, '
              f'max {ack.max():.0f} ms;'
              f' excess p95 {np.percentile(ex, 95):+.3f} s, max {ex.max():+.3f} s; '
              f'ack > 100 ms or excess > 0.8 s: {len(big)}')
        for r in big:
            print('   ', r)


if __name__ == '__main__':
    main()
