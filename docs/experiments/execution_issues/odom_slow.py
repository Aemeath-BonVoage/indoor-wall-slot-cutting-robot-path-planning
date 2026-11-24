#!/usr/bin/env python3
"""
慢速运动下的里程计与 EKF：只启动 Gazebo（底盘 ros2_control + 编码器误差模型 + EKF），
按固定的速度指令序列驱动底盘，比较各阶段的位移.

  python3 odom_slow.py --out runs/odom_slow --seed 1

每个阶段比较四个位移（mm，取各自坐标系下的位移向量的模，并给出沿指令方向的分量）：
  truth   /ground_truth/odom（真值）
  wheel   /wheel/odom 的位姿（控制器每个周期积分）
  twist   /wheel/odom 的 twist 按消息时间戳积分（EKF 实际融合的量）
  ekf     robot_localization 的 /odom
阶段（每段 8 s 仿真时间）：
  open_vx_0.7 / open_vx_3 / open_vy_0.7  恒定开环指令 −0.7 / −3 mm/s（vx）与 −0.7 mm/s（vy）
  zero                                   零指令
  hold                                   比例定点保持（与 BaseHold 相同的触发方式：每收到一条
                                         /wheel/odom 就计算并发布一次指令，k = 5 / s），16 s
  hold_arm                               同上，同时机械臂按 2 s 周期往复（joint2 横向 ±5 cm、
                                         joint3 前后 ±2 cm），底盘受机械臂的反作用，16 s
整个序列先以 average_twist = false（发布那一拍的瞬时速度，修正前的行为）运行，再以
average_twist = true（两次发布之间的平均速度）运行；参数在运行中切换，同一次启动内配对。
结果：<out>/phases.csv。
"""
import argparse
import csv
import math
import os
import subprocess
import threading
import time

import sim_session as ss

PHASES = [('open_vx_0.7', (-0.0007, 0.0, 0.0), 8.0), ('open_vx_3', (-0.003, 0.0, 0.0), 8.0),
          ('open_vy_0.7', (0.0, -0.0007, 0.0), 8.0), ('zero', (0.0, 0.0, 0.0), 8.0),
          ('hold', None, 16.0), ('hold_arm', 'arm', 16.0)]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Probe:
    def __init__(self):
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.parameter import Parameter
        self.rclpy, self.Twist = rclpy, Twist
        rclpy.init()
        n = rclpy.create_node('odom_slow_probe',
                              parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.node = n
        self.lock = threading.Lock()
        self.truth = self.wheel = self.ekf = None
        self.twist_int = [0.0, 0.0]
        self._tw_last = None
        self.hold_target = None
        self.pub = n.create_publisher(Twist, '/cmd_vel', 10)
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        self.FJT = FollowJointTrajectory
        self.arm = ActionClient(n, FollowJointTrajectory,
                                '/arm_controller/follow_joint_trajectory')
        n.create_subscription(Odometry, '/ground_truth/odom', self._on_truth, 50)
        n.create_subscription(Odometry, '/wheel/odom', self._on_wheel, 50)
        n.create_subscription(Odometry, '/odom', self._on_ekf, 50)
        self.ex = MultiThreadedExecutor(num_threads=2)
        self.ex.add_node(n)
        threading.Thread(target=self.ex.spin, daemon=True).start()

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _pose(m):
        p = m.pose.pose
        return (p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_truth(self, m):
        with self.lock:
            self.truth = self._pose(m)

    def _on_ekf(self, m):
        with self.lock:
            self.ekf = self._pose(m)

    def _on_wheel(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        x, y, yaw = self._pose(m)
        vx, vy = m.twist.twist.linear.x, m.twist.twist.linear.y
        with self.lock:
            self.wheel = (x, y, yaw)
            # twist 的积分：与 robot_localization 相同，用相邻消息的时间间隔
            if self._tw_last is not None:
                dt = t - self._tw_last[0]
                if 0.0 < dt < 0.5:
                    pvx, pvy, pyaw = self._tw_last[1:]
                    c, s = math.cos(pyaw), math.sin(pyaw)
                    self.twist_int[0] += (c * pvx - s * pvy) * dt
                    self.twist_int[1] += (s * pvx + c * pvy) * dt
            self._tw_last = (t, vx, vy, yaw)
            target = self.hold_target
        if target is not None:
            # 定点保持：与 BaseHold 相同由里程计消息触发（机体系比例控制）
            ex, ey = target[0] - x, target[1] - y
            c, s = math.cos(yaw), math.sin(yaw)
            cmd = self.Twist()
            cmd.linear.x = max(-0.05, min(0.05, 5.0 * (c * ex + s * ey)))
            cmd.linear.y = max(-0.05, min(0.05, 5.0 * (-s * ex + c * ey)))
            cmd.angular.z = max(-0.2, min(0.2, 6.0 * math.remainder(target[2] - yaw, 2 * math.pi)))
            self.pub.publish(cmd)

    def snapshot(self):
        with self.lock:
            return dict(truth=self.truth, wheel=self.wheel, twist=tuple(self.twist_int),
                        ekf=self.ekf)

    def run_open(self, v, dur):
        t0 = self.now()
        cmd = self.Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = v
        while self.now() - t0 < dur:
            self.pub.publish(cmd)
            time.sleep(0.05)

    def run_hold(self, dur):
        with self.lock:
            self.hold_target = self.wheel
        t0 = self.now()
        while self.now() - t0 < dur:
            time.sleep(0.05)
        with self.lock:
            self.hold_target = None
        self.pub.publish(self.Twist())

    def move_arm(self, dur):
        """机械臂往复运动（不等结果）：周期 2 s，每 0.5 s 一个结点."""
        from builtin_interfaces.msg import Duration
        from trajectory_msgs.msg import JointTrajectoryPoint
        self.arm.wait_for_server(timeout_sec=10.0)
        g = self.FJT.Goal()
        g.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        seq = [(0.0, -0.01), (0.04, 0.01), (0.0, -0.01), (-0.06, -0.03)]
        k = 0
        t = 0.5
        while t <= dur + 1e-9:
            j2, j3 = seq[k % 4]
            pt = JointTrajectoryPoint()
            pt.positions = [0.0, j2, j3, 0.0]
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            g.trajectory.points.append(pt)
            k += 1
            t += 0.5
        self.arm.send_goal_async(g)

    def shutdown(self):
        self.ex.shutdown()
        self.node.destroy_node()
        self.rclpy.shutdown()


def set_average_twist(env, value, attempts=10):
    """运行中切换底盘控制器的 average_twist，并读回确认（ros2 CLI 刚启动时可能还没发现节点）."""
    for _ in range(attempts):
        subprocess.run(['ros2', 'param', 'set', '--no-daemon', '/base_controller',
                        'average_twist', value], env=env, stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT, text=True, timeout=60)
        r = subprocess.run(['ros2', 'param', 'get', '--no-daemon', '/base_controller',
                            'average_twist'], env=env, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, text=True, timeout=60)
        print(f'average_twist -> {value}: {r.stdout.strip()}', flush=True)
        if r.stdout.strip().endswith(value.capitalize()):
            return True
        time.sleep(3.0)
    return False


def disp(a, b, direction):
    """位移向量的模（mm）与沿指令方向（odom / 世界系下的单位向量）的分量（mm）."""
    if a is None or b is None:
        return math.nan, math.nan
    dx, dy = b[0] - a[0], b[1] - a[1]
    along = (dx * direction[0] + dy * direction[1]) if direction else math.nan
    return 1e3 * math.hypot(dx, dy), 1e3 * along


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--repeats', type=int, default=2, help='每种 average_twist 设置重复的序列数')
    args = ap.parse_args()
    out = os.path.abspath(args.out)
    os.makedirs(out, exist_ok=True)
    env = ss.env()
    cmd = ['ros2', 'launch', 'scr_description', 'gazebo.launch.py', 'gui:=false',
           f'odom_seed:={args.seed}']
    with open(os.path.join(out, 'command.txt'), 'w') as f:
        f.write(' '.join(cmd) + '\n')
    ss.kill_stray_gazebo()
    log = open(os.path.join(out, 'launch.log'), 'w')
    proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    rows = []
    try:
        probe = Probe()
        t_wait = time.time()
        while time.time() - t_wait < 120:
            s = probe.snapshot()
            if all(s[k] is not None for k in ('truth', 'wheel', 'ekf')):
                break
            time.sleep(1.0)
        time.sleep(10.0)
        for avg in ('false', 'true'):
            if not set_average_twist(env, avg):
                raise RuntimeError(f'could not set average_twist={avg}')
            time.sleep(2.0)
            for rep in range(args.repeats):
                for name, v, dur in PHASES:
                    a = probe.snapshot()
                    if v == 'arm':
                        probe.move_arm(dur)
                        probe.run_hold(dur + 0.5)
                        v = None
                    elif v is None:
                        probe.run_hold(dur)
                    else:
                        probe.run_open(v, dur)
                        probe.pub.publish(probe.Twist())
                    time.sleep(1.0)       # 让指令停下后的最后一段运动也计入
                    b = probe.snapshot()
                    row = dict(average_twist=avg, repeat=rep, phase=name, duration_s=dur)
                    # 指令方向：机体系 → 各自坐标系（真值 / odom 系下朝向几乎不变）
                    for key in ('truth', 'wheel', 'twist', 'ekf'):
                        pa, pb = a[key], b[key]
                        if key == 'twist':
                            yaw = a['wheel'][2]
                            pa, pb = (pa[0], pa[1]), (pb[0], pb[1])
                        else:
                            yaw = pa[2]
                        d = None
                        if v is not None and (v[0] or v[1]):
                            n = math.hypot(v[0], v[1])
                            bx, by = v[0] / n, v[1] / n
                            d = (math.cos(yaw) * bx - math.sin(yaw) * by,
                                 math.sin(yaw) * bx + math.cos(yaw) * by)
                        row[f'{key}_mm'], row[f'{key}_along_mm'] = disp(pa, pb, d)
                    rows.append(row)
                    print(' '.join(f'{k}={v:.2f}' if isinstance(v, float) else f'{k}={v}'
                                   for k, v in row.items()), flush=True)
        probe.shutdown()
    finally:
        ss.rv.kill_group(proc, 'gazebo')
        log.close()
        time.sleep(2.0)
        for pid in ss.rv.leftover(env):
            try:
                os.kill(pid, 9)
            except ProcessLookupError:
                pass
        ss.kill_stray_gazebo()
        subprocess.run('fastdds shm clean', shell=True, env=env, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
    with open(os.path.join(out, 'phases.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        for r in rows:
            w.writerow({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})


if __name__ == '__main__':
    main()
