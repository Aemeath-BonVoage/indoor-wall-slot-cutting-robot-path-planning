#!/usr/bin/env python3
"""
Nav2 导航对比实验：按固定目标序列反复导航，用 Gazebo 真值评估每段导航.

每段（一个目标）记录：
  success        Nav2 返回 SUCCEEDED
  time_s         仿真时间
  path_m         真值轨迹长度
  clearance_m    底盘矩形外廓到地图障碍的最小距离（真值位姿 + 静态地图）
  jerk_rms       线加速度变化率的均方根（m/s³，真值 10 Hz 采样、中心差分）
  dw_rms         角速度变化率的均方根（rad/s²，同上）
  cmd_dw_rms     相邻两次速度指令（/cmd_vel）角速度之差的均方根（rad/s）
  final_err_m    结束时真值位置与目标的距离
  cpu_*          该段期间各 Nav2 进程的 CPU 占用（单核百分比；需 use_composition:=False）

同时按 10 Hz 记录轨迹（真值、/odom、可选的 /wheel/odom、map→base_link），供定位漂移分析。

  ros2 run scr_navigation nav_benchmark.py --out <目录> --rounds 3 --label mppi_smac
"""
import argparse
import csv
import math
import os
import threading
import time
import warnings

import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
import tf2_ros

# 固定目标序列（map 系 x, y, yaw）：大房间 → 东侧房间（经门洞）→ 大房间西南 → 南侧房间（经门洞）
# → 起点。一圈约 40 m，包含两处约 1.9 m 宽的门洞。
GOALS = [
    ('G1_big_room_ne', 3.0, 2.5, 0.0),
    ('G2_wall_target', 8.6, -2.0, -1.5708),
    ('G3_east_room_n', 7.0, 2.6, 3.1416),
    ('G4_big_room_sw', -3.5, -2.5, 1.5708),
    ('G5_south_room', 0.0, -6.5, -1.5708),
    ('G6_start', 0.0, 0.0, 0.0),
]
ENTITY = 'scr_robot_gazebo'
FOOTPRINT = (0.7467, 0.5662)        # base_link 碰撞盒（scr_robot_gazebo.urdf）
NAV2_PROCS = ('controller_serv', 'planner_server', 'bt_navigator', 'amcl',
              'component_conta', 'velocity_smoot')


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def footprint_points(x, y, yaw, step=0.05):
    """矩形外廓上的采样点（map 系）."""
    lx, ly = FOOTPRINT[0] / 2, FOOTPRINT[1] / 2
    pts = []
    for a, b in (((-lx, -ly), (lx, -ly)), ((lx, -ly), (lx, ly)), ((lx, ly), (-lx, ly)),
                 ((-lx, ly), (-lx, -ly))):
        n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
        for k in range(n):
            s = k / n
            pts.append((a[0] + s * (b[0] - a[0]), a[1] + s * (b[1] - a[1])))
    c, s_ = math.cos(yaw), math.sin(yaw)
    return np.array([(x + c * px - s_ * py, y + s_ * px + c * py) for px, py in pts])


def smoothness(samples):
    """samples: [(t, x, y, yaw)] 10 Hz → (jerk_rms, dw_rms)."""
    if len(samples) < 7:
        return math.nan, math.nan
    a = np.array(samples)
    t, x, y, yaw = a[:, 0], a[:, 1], a[:, 2], np.unwrap(a[:, 3])

    def d(v):
        return np.gradient(v, t)
    vx, vy, w = d(x), d(y), d(yaw)
    ax, ay, dw = d(vx), d(vy), d(w)
    jx, jy = d(ax), d(ay)
    return float(np.sqrt(np.mean(jx ** 2 + jy ** 2))), float(np.sqrt(np.mean(dw ** 2)))


class CpuSampler:
    """按进程名统计本域 Nav2 进程的 CPU 时间（/proc）."""

    def __init__(self, domain):
        self.domain = str(domain)
        self.hz = os.sysconf('SC_CLK_TCK')
        self.pids = self._find()

    def _find(self):
        out = {}
        for p in os.listdir('/proc'):
            if not p.isdigit():
                continue
            try:
                with open(f'/proc/{p}/comm') as f:
                    comm = f.read().strip()
                if comm not in NAV2_PROCS:
                    continue
                with open(f'/proc/{p}/environ', 'rb') as f:
                    env = f.read().split(b'\0')
                if f'ROS_DOMAIN_ID={self.domain}'.encode() in env:
                    out[comm] = int(p)
            except OSError:
                continue
        return out

    def snapshot(self):
        snap = {}
        for name, pid in self.pids.items():
            try:
                with open(f'/proc/{pid}/stat') as f:
                    fields = f.read().rsplit(')', 1)[1].split()
                snap[name] = (int(fields[11]) + int(fields[12])) / self.hz
            except OSError:
                continue
        with open('/proc/stat') as f:
            cpu = [int(v) for v in f.readline().split()[1:]]
        snap['_total'] = (sum(cpu) - cpu[3] - cpu[4], sum(cpu))
        snap['_wall'] = time.time()
        return snap

    @staticmethod
    def usage(s0, s1):
        dt = max(s1['_wall'] - s0['_wall'], 1e-6)
        out = {f'cpu_{k}': 100.0 * (s1[k] - s0[k]) / dt
               for k in s0 if not k.startswith('_') and k in s1}
        busy = s1['_total'][0] - s0['_total'][0]
        total = max(s1['_total'][1] - s0['_total'][1], 1)
        out['cpu_system'] = 100.0 * busy / total     # 全机占用（4 核平均）
        return out


class Bench:
    def __init__(self, wheel_topic):
        self.node = rclpy.create_node(
            'nav_benchmark', parameter_overrides=[Parameter('use_sim_time', value=True)])
        n = self.node
        self.truth = None
        self.odom = None
        self.wheel = None
        self.cmd = []
        self.lock = threading.Lock()
        self.obstacles = None
        n.create_subscription(ModelStates, '/gazebo/model_states', self._on_truth, 10)
        n.create_subscription(Odometry, '/odom', self._on_odom, 20)
        if wheel_topic:
            n.create_subscription(Odometry, wheel_topic, self._on_wheel, 20)
        n.create_subscription(Twist, '/cmd_vel', self._on_cmd, 20)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        n.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, n)
        self.client = ActionClient(n, NavigateToPose, '/navigate_to_pose')
        self.traj = []          # 10 Hz 轨迹
        self.recording = False
        n.create_timer(0.1, self._sample)
        self.ex = MultiThreadedExecutor(num_threads=3)
        self.ex.add_node(n)
        self.thread = threading.Thread(target=self.ex.spin, daemon=True)
        self.thread.start()

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _on_truth(self, msg):
        try:
            p = msg.pose[msg.name.index(ENTITY)]
        except ValueError:
            return
        self.truth = (p.position.x, p.position.y, _yaw(p.orientation))

    def _on_odom(self, msg):
        p = msg.pose.pose
        self.odom = (p.position.x, p.position.y, _yaw(p.orientation))

    def _on_wheel(self, msg):
        p = msg.pose.pose
        self.wheel = (p.position.x, p.position.y, _yaw(p.orientation))

    def _on_cmd(self, msg):
        if self.recording:
            with self.lock:
                self.cmd.append((self.now(), msg.linear.x, msg.linear.y, msg.angular.z))

    def _on_map(self, msg):
        w, h, r = msg.info.width, msg.info.height, msg.info.resolution
        o = msg.info.origin.position
        grid = np.array(msg.data, dtype=np.int16).reshape(h, w)
        iy, ix = np.nonzero(grid >= 65)
        pts = np.column_stack([o.x + (ix + 0.5) * r, o.y + (iy + 0.5) * r])
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            from scipy.spatial import cKDTree
        self.obstacles = cKDTree(pts)

    def _map_pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time())
        except tf2_ros.TransformException:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y, _yaw(q))

    def _sample(self):
        if self.truth is None:
            return
        nan3 = (math.nan,) * 3
        row = (self.now(), *self.truth, *(self.odom or nan3), *(self.wheel or nan3),
               *(self._map_pose() or nan3))
        with self.lock:
            self.traj.append(row)

    def clearance(self, pose):
        if self.obstacles is None:
            return math.nan
        d, _ = self.obstacles.query(footprint_points(*pose))
        return float(np.min(d))

    def navigate(self, x, y, yaw, timeout=240.0):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        fut = self.client.send_goal_async(goal)
        if not _wait(fut, 30.0) or not fut.result().accepted:
            return GoalStatus.STATUS_ABORTED
        handle = fut.result()
        res = handle.get_result_async()
        t0 = self.now()
        while not res.done():
            if self.now() - t0 > timeout:
                c = handle.cancel_goal_async()
                _wait(c, 10.0)
                _wait(res, 10.0)
                return GoalStatus.STATUS_CANCELED
            time.sleep(0.05)
        return res.result().status

    def shutdown(self):
        self.ex.shutdown()
        self.thread.join(timeout=2.0)
        self.node.destroy_node()


def _wait(fut, timeout):
    t0 = time.time()
    while not fut.done() and time.time() - t0 < timeout:
        time.sleep(0.05)
    return fut.done()


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('--out', required=True)
    ap.add_argument('--label', default='nav')
    ap.add_argument('--rounds', type=int, default=3)
    ap.add_argument('--wheel-topic', default='', help='额外记录的轮式里程计话题')
    ap.add_argument('--domain', default=os.environ.get('ROS_DOMAIN_ID', '0'))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    b = Bench(args.wheel_topic)
    log = b.node.get_logger()
    while not b.client.wait_for_server(timeout_sec=5.0):
        log.info('等待 /navigate_to_pose ...')
    t_wait = time.time()
    while (b.truth is None or b.obstacles is None or b._map_pose() is None) and \
            time.time() - t_wait < 60:
        time.sleep(0.5)
    cpu = CpuSampler(args.domain)
    log.info(f'CPU 统计进程: {sorted(cpu.pids)}')
    rows = []
    b.recording = True
    for rnd in range(args.rounds):
        for name, x, y, yaw in GOALS:
            with b.lock:
                b.cmd.clear()
                k0 = len(b.traj)
            s0, t0 = cpu.snapshot(), b.now()
            status = b.navigate(x, y, yaw)
            t1, s1 = b.now(), cpu.snapshot()
            time.sleep(0.2)
            with b.lock:
                seg = [(r[0], r[1], r[2], r[3]) for r in b.traj[k0:]]
                cmd = list(b.cmd)
            path = sum(math.hypot(q[1] - p[1], q[2] - p[2]) for p, q in zip(seg, seg[1:]))
            clear = min((b.clearance(p[1:]) for p in seg), default=math.nan)
            jerk, dw = smoothness(seg)
            dwc = np.diff([c[3] for c in cmd]) if len(cmd) > 2 else np.array([math.nan])
            fin = b.truth
            row = dict(label=args.label, round=rnd, goal=name, status=int(status),
                       success=int(status == GoalStatus.STATUS_SUCCEEDED), time_s=t1 - t0,
                       path_m=path, clearance_m=clear, jerk_rms=jerk, dw_rms=dw,
                       cmd_dw_rms=float(np.sqrt(np.mean(dwc ** 2))),
                       final_err_m=math.hypot(fin[0] - x, fin[1] - y),
                       final_yaw_err_deg=math.degrees(math.atan2(math.sin(fin[2] - yaw),
                                                                 math.cos(fin[2] - yaw))))
            row.update(cpu.usage(s0, s1))
            rows.append(row)
            log.info(f'[{rnd}] {name}: status {status} {row["time_s"]:.1f} s '
                     f'{path:.2f} m 间隙 {clear:.3f} m jerk {jerk:.2f} '
                     f'CPU ' + ' '.join(f'{k[4:]}={v:.0f}%' for k, v in row.items()
                                        if k.startswith('cpu_')))
    b.recording = False
    fields = sorted({k for r in rows for k in r}, key=lambda k: list(rows[0]).index(k)
                    if k in rows[0] else 99)
    with open(os.path.join(args.out, 'legs.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(args.out, 'trajectory.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'gt_x', 'gt_y', 'gt_yaw', 'odom_x', 'odom_y', 'odom_yaw',
                    'wheel_x', 'wheel_y', 'wheel_yaw', 'map_x', 'map_y', 'map_yaw'])
        w.writerows(b.traj)
    ok = [r for r in rows if r['success']]
    summary = dict(label=args.label, legs=len(rows), success=len(ok),
                   success_rate=len(ok) / max(1, len(rows)))
    for k in ('time_s', 'path_m', 'clearance_m', 'jerk_rms', 'dw_rms', 'cmd_dw_rms',
              'final_err_m') + tuple(k for k in rows[0] if k.startswith('cpu_')):
        v = np.array([r[k] for r in ok if k in r], dtype=float)
        if len(v):
            summary[f'{k}_mean'] = float(np.nanmean(v))
            summary[f'{k}_std'] = float(np.nanstd(v))
            summary[f'{k}_min'] = float(np.nanmin(v))
            summary[f'{k}_max'] = float(np.nanmax(v))
    with open(os.path.join(args.out, 'summary.yaml'), 'w') as f:
        yaml.safe_dump({k: (round(v, 4) if isinstance(v, float) else v)
                        for k, v in summary.items()}, f, sort_keys=False)
    log.info(f'完成：{len(ok)}/{len(rows)} 成功 → {args.out}')
    b.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
