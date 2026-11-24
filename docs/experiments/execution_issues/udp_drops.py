#!/usr/bin/env python3
"""
按进程统计 UDP 接收缓冲区溢出丢包（第 3 节）：读 /proc/net/udp 每个套接字的 drops 计数，
经 /proc/<pid>/fd 的套接字 inode 对应到进程，间隔 --period 秒采样两次，输出各进程的增量.

  python3 udp_drops.py --period 60 [--out drops.csv]
  python3 udp_drops.py --watch 2 --out drops_timeline.csv     # 每 2 s 记录一次，直到被结束

只统计本域（ROS_DOMAIN_ID = 49）的仿真 / ROS 进程；同时给出全机 UdpRcvbufErrors 的增量。
"""
import argparse
import csv
import os
import time


def socket_drops():
    out = {}
    for path in ('/proc/net/udp', '/proc/net/udp6'):
        try:
            with open(path) as f:
                next(f)
                for line in f:
                    p = line.split()
                    # inode → (drops, rx_queue)
                    out[int(p[9])] = (int(p[12]), int(p[4].split(':')[1], 16))
        except OSError:
            pass
    return out


def inode_owner(domain):
    owner = {}
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/environ', 'rb') as f:
                if f'ROS_DOMAIN_ID={domain}'.encode() not in f.read().split(b'\0'):
                    continue
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().split(b'\0')
            name = os.path.basename(cmd[0].decode())
            if name.startswith('python'):
                name = os.path.basename(cmd[1].decode()) if len(cmd) > 1 else name
            for fd in os.listdir(f'/proc/{pid}/fd'):
                try:
                    link = os.readlink(f'/proc/{pid}/fd/{fd}')
                except OSError:
                    continue
                if link.startswith('socket:['):
                    owner[int(link[8:-1])] = f'{name}[{pid}]'
        except OSError:
            continue
    return owner


def rcvbuf_errors():
    with open('/proc/net/snmp') as f:
        lines = [ln.split() for ln in f if ln.startswith('Udp:')]
    head, vals = lines[0], lines[1]
    return int(vals[head.index('RcvbufErrors')])


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('--period', type=float, default=60.0)
    ap.add_argument('--domain', default=os.environ.get('ROS_DOMAIN_ID', '49'))
    ap.add_argument('--out', default='')
    ap.add_argument('--watch', type=float, default=0.0,
                    help='> 0：每隔这么多秒记录各进程的累计丢包（墙钟时间戳），直到被结束')
    args = ap.parse_args()
    if args.watch > 0:
        return watch(args)
    owner = inode_owner(args.domain)
    d0, e0 = socket_drops(), rcvbuf_errors()
    time.sleep(args.period)
    d1, e1 = socket_drops(), rcvbuf_errors()
    per = {}
    for ino, (drops, _) in d1.items():
        if ino in owner:
            per[owner[ino]] = per.get(owner[ino], 0) + drops - d0.get(ino, (drops, 0))[0]
    rows = sorted(per.items(), key=lambda kv: -kv[1])
    print(f'machine UdpRcvbufErrors +{e1 - e0} in {args.period:.0f} s')
    for name, n in rows:
        if n:
            print(f'  {name}: +{n}')
    if args.out:
        with open(args.out, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['process', 'drops', 'period_s', 'machine_rcvbuf_errors'])
            for name, n in rows:
                w.writerow([name.split('[')[0], n, args.period, e1 - e0])


def watch(args):
    """时间序列：墙钟、进程、该进程全部套接字的累计 drops（进程重启后重新计数）."""
    with open(args.out, 'a', newline='') as f:
        w = csv.writer(f)
        if f.tell() == 0:
            w.writerow(['wall', 'process', 'pid', 'drops_total', 'machine_rcvbuf_errors'])
        while True:
            owner = inode_owner(args.domain)
            drops = socket_drops()
            tot = {}
            for ino, (n, _) in drops.items():
                if ino in owner:
                    tot[owner[ino]] = tot.get(owner[ino], 0) + n
            now, err = round(time.time(), 2), rcvbuf_errors()
            for name, n in tot.items():
                proc, pid = name[:-1].split('[')
                w.writerow([now, proc, pid, n, err])
            f.flush()
            time.sleep(args.watch)


if __name__ == '__main__':
    main()
