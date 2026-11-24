#!/usr/bin/env python3
"""
导航段到达过程的记录：启动整套仿真，在起点与 room3 南墙标记前的导航目标之间往返，
记录每段导航末端的速度指令、行为树事件、定位修正与真值.

  python3 nav_legs.py --out runs/nav_default_s1 --seed 1 --rounds 6
      [--set params_file:=<nav2 参数>] [--set <其他 launch 参数> ...] [--stress 3]
      [--server-timeout 20]

每段（一个 NavigateToPose 目标）记录到 <out>/：
  legs.csv    每段一行：结果、耗时、恢复次数、末端耗时分解、终点真值误差、定位跳变；
  trace.csv   20 Hz 采样（以真值里程计消息头为时钟）：真值、map 系估计位姿（TF）、
              最终速度指令 /cmd_vel、控制器输出 /cmd_vel_nav、Nav2 反馈的剩余距离；
  events.csv  行为树状态变化（/behavior_tree_log）与 Nav2 各服务器的日志（/rosout）。
任务层照常启动（与多站任务相同的负载），但不执行任务。
--stress N：导航期间另开 N 个占满一个核的忙循环进程（模拟多站任务时记录器、记录包等的负载），
用于提高“等待服务器确认目标超时”的发生率，便于修正前后对比。
--server-timeout MS：以默认的 nav2_params.yaml 为底，只把 bt_navigator 的
default_server_timeout 改为 MS，副本写入结果目录并作为 params_file 传入（修正前后在同一份
代码上对比）。
"""
import argparse
import csv
import math
import os
import threading
import time

import sim_session as ss

# 起点（map 系）与 room3 南墙参考标记前 1.2 m 的导航目标（与多站任务的 NavigateToMarker 相同：
# 标记 (8.42, -3.9245)，nav_standoff 1.2 m，面向墙面）
START = ('start', 0.0, 0.0, 0.0)
MARKER = ('marker', 8.42, -2.7245, -math.pi / 2)
XY_TOL, YAW_TOL = 0.25, 0.25          # 与 nav2_params.yaml 的 general_goal_checker 一致
NAV2_LOGGERS = ('controller_server', 'bt_navigator', 'behavior_server', 'planner_server',
                'amcl', 'velocity_smoother', 'lifecycle_manager_navigation')


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Recorder:
    def __init__(self):
        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
        from nav2_msgs.action import NavigateToPose
        from nav2_msgs.msg import BehaviorTreeLog
        from nav_msgs.msg import Odometry
        from rcl_interfaces.msg import Log
        from rclpy.action import ActionClient
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        import tf2_ros
        self.rclpy = rclpy
        self.NavigateToPose = NavigateToPose
        rclpy.init()
        n = rclpy.create_node('nav_legs_recorder',
                              parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.node = n
        self.lock = threading.Lock()
        self.cmd = (0.0, 0.0, 0.0, -1.0)
        self.cmd_nav = (0.0, 0.0, 0.0, -1.0)
        self.amcl = None
        self.feedback = (math.nan, 0)
        self.trace, self.events = [], []
        self.leg = -1
        self._last_t = -1.0
        n.create_subscription(Twist, '/cmd_vel', lambda m: self._on_cmd(m, 'cmd'), 50)
        n.create_subscription(Twist, '/cmd_vel_nav', lambda m: self._on_cmd(m, 'cmd_nav'), 50)
        n.create_subscription(Odometry, '/ground_truth/odom', self._on_truth, 50)
        n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10)
        n.create_subscription(BehaviorTreeLog, '/behavior_tree_log', self._on_bt, 50)
        rosout_qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE)
        n.create_subscription(Log, '/rosout', self._on_log, rosout_qos)
        self.tf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tf, n)
        self.client = ActionClient(n, NavigateToPose, '/navigate_to_pose')
        self.ex = MultiThreadedExecutor(num_threads=3)
        self.ex.add_node(n)
        self.th = threading.Thread(target=self.ex.spin, daemon=True)
        self.th.start()

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, m, which):
        v = (m.linear.x, m.linear.y, m.angular.z, self.now())
        with self.lock:
            setattr(self, which, v)

    def _on_amcl(self, m):
        p = m.pose.pose
        with self.lock:
            self.amcl = (p.position.x, p.position.y, yaw_of(p.orientation))
            if self.leg >= 0:
                self.events.append((self.leg, self.now(), 'amcl_pose',
                                    f'{self.amcl[0]:.4f} {self.amcl[1]:.4f} '
                                    f'{self.amcl[2]:.5f}'))

    def _on_bt(self, m):
        # 事件内部的时间戳是墙钟；统一用消息头的 ROS 时间（仿真时间）
        t = m.timestamp.sec + m.timestamp.nanosec * 1e-9
        with self.lock:
            for e in m.event_log:
                self.events.append((self.leg, t, 'bt', f'{e.node_name} '
                                    f'{e.previous_status}->{e.current_status}'))

    def _on_log(self, m):
        if not any(x in m.name for x in NAV2_LOGGERS):
            return
        t = m.stamp.sec + m.stamp.nanosec * 1e-9
        with self.lock:
            self.events.append((self.leg, t, m.name, m.msg.replace('\n', ' ')[:300]))

    def _map_pose(self):
        from rclpy.time import Time
        try:
            tr = self.tf.lookup_transform('map', 'base_link', Time())
        except Exception:
            return (math.nan,) * 3
        return (tr.transform.translation.x, tr.transform.translation.y,
                yaw_of(tr.transform.rotation))

    def _on_truth(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t - self._last_t < 0.049 or self.leg < 0:
            return
        self._last_t = t
        p = m.pose.pose
        tw = m.twist.twist
        est = self._map_pose()
        with self.lock:
            self.trace.append((self.leg, t, p.position.x, p.position.y, yaw_of(p.orientation),
                               tw.linear.x, tw.linear.y, tw.angular.z, *est,
                               *self.cmd, *self.cmd_nav, self.feedback[0], self.feedback[1]))

    def _on_feedback(self, fb):
        f = fb.feedback
        self.feedback = (f.distance_remaining, f.number_of_recoveries)

    def navigate(self, goal, timeout=180.0):
        _, x, y, yaw = goal
        g = self.NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.node.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = x, y
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.feedback = (math.nan, 0)
        t0 = self.now()
        fut = self.client.send_goal_async(g, feedback_callback=self._on_feedback)
        w0 = time.time()
        while not fut.done() and time.time() - w0 < 30:
            time.sleep(0.02)
        if not fut.done() or not fut.result().accepted:
            return 'rejected', t0, self.now()
        h = fut.result()
        res = h.get_result_async()
        while not res.done():
            if self.now() - t0 > timeout:
                c = h.cancel_goal_async()
                w0 = time.time()
                while not c.done() and time.time() - w0 < 10:
                    time.sleep(0.05)
                return 'timeout', t0, self.now()
            time.sleep(0.02)
        st = res.result().status
        return {4: 'succeeded', 5: 'canceled', 6: 'aborted'}.get(st, str(st)), t0, self.now()

    def shutdown(self):
        self.ex.shutdown()
        self.node.destroy_node()
        self.rclpy.shutdown()


def leg_metrics(rows, goal, t0, t1):
    """一段的末端分解（真值与 map 系估计）."""
    _, gx, gy, gyaw = goal
    out = {}

    def first_within(ix, iy, iyaw):
        for r in rows:
            if math.hypot(r[ix] - gx, r[iy] - gy) <= XY_TOL and \
                    abs(wrap(r[iyaw] - gyaw)) <= YAW_TOL:
                return r[1]
        return math.nan

    def first_xy(ix, iy):
        for r in rows:
            if math.hypot(r[ix] - gx, r[iy] - gy) <= XY_TOL:
                return r[1]
        return math.nan
    out['t_est_in_tol'] = first_within(8, 9, 10) - t0
    out['t_true_in_tol'] = first_within(2, 3, 4) - t0
    out['t_est_in_xy'] = first_xy(8, 9) - t0
    out['t_true_in_xy'] = first_xy(2, 3) - t0
    # 末端：估计位姿第一次进入 1 m 之后
    near = [r for r in rows if math.hypot(r[8] - gx, r[9] - gy) <= 1.0]
    tn = near[0][1] if near else math.nan
    out['t_est_within_1m'] = tn - t0
    tail = [r for r in rows if r[1] >= tn] if near else []
    dt = 0.05
    out['tail_still_s'] = sum(dt for r in tail if math.hypot(r[5], r[6]) < 0.01
                              and abs(r[7]) < 0.02)
    out['tail_creep_s'] = sum(dt for r in tail if 0.01 <= math.hypot(r[5], r[6]) < 0.05
                              and abs(r[7]) < 0.1)
    out['tail_rot_s'] = sum(dt for r in tail if abs(r[7]) >= 0.1)
    if rows:
        r = rows[-1]
        out['final_true_xy_m'] = math.hypot(r[2] - gx, r[3] - gy)
        out['final_true_yaw_deg'] = math.degrees(wrap(r[4] - gyaw))
        out['final_est_xy_m'] = math.hypot(r[8] - gx, r[9] - gy)
        out['final_est_yaw_deg'] = math.degrees(wrap(r[10] - gyaw))
        out['final_loc_err_m'] = math.hypot(r[8] - r[2], r[9] - r[3])
    # 定位误差（估计 - 真值）在末端的最大值
    errs = [math.hypot(r[8] - r[2], r[9] - r[3]) for r in tail if not math.isnan(r[8])]
    out['tail_max_loc_err_m'] = max(errs) if errs else math.nan
    out['max_loc_err_m'] = max((math.hypot(r[8] - r[2], r[9] - r[3]) for r in rows
                                if not math.isnan(r[8])), default=math.nan)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--rounds', type=int, default=6, help='往返次数（每次 2 段）')
    ap.add_argument('--set', action='append', default=[], help='额外的 launch 参数 key:=value')
    ap.add_argument('--ready-timeout', type=float, default=240.0)
    ap.add_argument('--settle', type=float, default=3.0, help='两段之间的间隔（墙钟秒）')
    ap.add_argument('--stress', type=int, default=0, help='导航期间的忙循环进程数')
    ap.add_argument('--server-timeout', type=int, default=None,
                    help='bt_navigator 的 default_server_timeout（毫秒）；默认取参数文件中的值')
    args = ap.parse_args()
    out = os.path.abspath(args.out)
    os.makedirs(out, exist_ok=True)
    extra = [f'odom_seed:={args.seed}'] + args.set
    if args.server_timeout is not None:
        import re
        src = os.path.join(ss.REPO, 'src', 'scr_navigation', 'config', 'nav2_params.yaml')
        with open(src, encoding='utf-8') as f:
            text = f.read()
        text, n = re.subn(r'(\n    default_server_timeout: )\d+', rf'\g<1>{args.server_timeout}',
                          text)
        assert n == 1, 'default_server_timeout not found'
        params = os.path.join(out, 'nav2_params.yaml')
        with open(params, 'w', encoding='utf-8') as f:
            f.write(text)
        extra.append(f'params_file:={params}')
    with open(os.path.join(out, 'command.txt'), 'w') as f:
        f.write(ss.rv.portable(' '.join(ss.launch_cmd(extra))) + '\n')
    legs = []
    with ss.Session(out, extra) as s:
        ready, t_ready = s.wait_ready(args.ready_timeout)
        print(f'ready={ready} after {t_ready:.0f} s', flush=True)
        if not ready:
            with open(os.path.join(out, 'startup_failure.txt'), 'w') as f:
                f.write(f'not ready after {t_ready:.0f} s\n')
            return 1
        rec = Recorder()
        time.sleep(5.0)
        import subprocess
        burners = [subprocess.Popen(['python3', '-c', 'while True: pass'])
                   for _ in range(args.stress)]

        seq = [MARKER, START] * args.rounds
        prev = START
        for k, goal in enumerate(seq):
            with rec.lock:
                rec.leg = k
            status, t0, t1 = rec.navigate(goal)
            time.sleep(0.5)
            with rec.lock:
                rows = [r for r in rec.trace if r[0] == k and t0 <= r[1] <= t1]
                ev = [e for e in rec.events if e[0] == k]
                rec.leg = -1
            m = leg_metrics(rows, goal, t0, t1)
            spins = sum(1 for e in ev if e[2] == 'bt' and e[3].startswith('Spin ')
                        and e[3].endswith('->RUNNING'))
            waits = sum(1 for e in ev if e[2] == 'bt' and e[3].startswith('Wait ')
                        and e[3].endswith('->RUNNING'))
            backups = sum(1 for e in ev if e[2] == 'bt' and e[3].startswith('BackUp ')
                          and e[3].endswith('->RUNNING'))
            fp_fail = sum(1 for e in ev if e[2] == 'bt' and e[3].startswith('FollowPath ')
                          and e[3].endswith('->FAILURE'))
            cp_fail = sum(1 for e in ev if e[2] == 'bt' and e[3].startswith('ComputePathToPose ')
                          and e[3].endswith('->FAILURE'))
            no_progress = sum(1 for e in ev if 'Failed to make progress' in e[3])
            row = dict(leg=k, start=prev[0], goal=goal[0], status=status,
                       time_s=round(t1 - t0, 2), recoveries=rec.feedback[1], spins=spins,
                       waits=waits, backups=backups, followpath_failures=fp_fail,
                       computepath_failures=cp_fail, no_progress=no_progress)
            row.update({k2: (round(v, 4) if isinstance(v, float) else v)
                        for k2, v in m.items()})
            legs.append(row)
            print(f'leg {k} {prev[0]}->{goal[0]}: {status} {t1 - t0:.1f} s '
                  f'rec={rec.feedback[1]} spin={spins} wait={waits} '
                  f'tail_still={m.get("tail_still_s", 0):.1f}s', flush=True)
            prev = goal
            time.sleep(args.settle)
        for b in burners:
            b.kill()
            b.wait()

        with rec.lock:
            trace, events = list(rec.trace), list(rec.events)
        rec.shutdown()
    with open(os.path.join(out, 'legs.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(legs[0]))
        w.writeheader()
        w.writerows(legs)
    with open(os.path.join(out, 'trace.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['leg', 't', 'gt_x', 'gt_y', 'gt_yaw', 'gt_vx', 'gt_vy', 'gt_wz',
                    'est_x', 'est_y', 'est_yaw', 'cmd_vx', 'cmd_vy', 'cmd_wz', 'cmd_t',
                    'nav_vx', 'nav_vy', 'nav_wz', 'nav_t', 'dist_remaining', 'recoveries'])
        for r in trace:
            w.writerow([f'{v:.4f}' if isinstance(v, float) else v for v in r])
    with open(os.path.join(out, 'events.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['leg', 't', 'source', 'text'])
        for e in sorted(events, key=lambda e: e[1]):
            w.writerow([e[0], f'{e[1]:.3f}', e[2], e[3]])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
