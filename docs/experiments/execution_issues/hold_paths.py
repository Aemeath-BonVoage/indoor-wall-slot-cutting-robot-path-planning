#!/usr/bin/env python3
"""
定点保持控制律（hold.mode = p / dob）在多站空切之外的三条调用路径上的对照（第 6.4 节）.

  # 单站任务（cutting_task.xml：导航 → 精对位 → 定点保持下扫描与切割）
  python3 hold_paths.py task --hold dob --seed 1 --out runs/hold_task_dob_s1
  # 独立精对位（alignment_benchmark 的 v2 流程，到位后保持 HOLD_S 秒）+ 示教零位
  python3 hold_paths.py align --hold dob --seed 1 --out runs/hold_align_dob_s1

task：完整仿真（sim_nav.launch.py，默认配置；技能服务器的 hold.mode 由 --hold 给出），
用 navigation_cutting_task 执行单站任务 task_at_target.yaml，同时记录 /task_skills/status 与
Gazebo 真值（/gazebo/model_states，降采样到 50 Hz）。

align：只启动 Gazebo（scr_description gazebo.launch.py，不启动 Nav2 与任务层），本进程内
创建与 alignment_benchmark 相同的技能节点（hold.mode 由参数覆盖），对同一组随机起点
（种子 42，与 alignment_benchmark 相同的生成方式）各做一次精对位，到位后定点保持 HOLD_S 秒
并记录真值；之后在同一个仿真中运行 teach_alignment_offset（同样覆盖 hold.mode），记录
示教期间的真值。两个工具的里程计话题都设为 /wheel/odom（与任务层相同，50 Hz）：节点默认的
/odom 由 EKF 发布，仿真中只有 10 Hz，激光与图像的时间戳常落在里程计历史之外，无法做延迟
补偿，两个工具都得不到测量（第 6.4 节）。--odom 可改回。

仿真与 ROS 通信隔离在 ROS_DOMAIN_ID = 49、GAZEBO_MASTER_URI = http://localhost:11353。
运行前先 source 工作空间。
"""
import argparse
import csv
import math
import os
import subprocess
import sys
import threading
import time

import yaml

import sim_session as ss

sys.path.insert(0, os.path.join(ss.HERE, '..', 'arm_trajectory'))
from run_experiment import portable_logs  # noqa: E402
from run_full_wall import skill_params  # noqa: E402

rv = ss.rv
TASK_FILE = 'task_at_target.yaml'
HOLD_S = 8.0          # 精对位到位后保持并记录真值的时长（仿真秒）
TRIALS = 4            # 每个控制律的精对位试验数（同一组起点）
TEACH_S = 30.0        # 示教采样时长（teach_alignment_offset 默认）


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class TruthRecorder:
    """订阅 /gazebo/model_states 与 /task_skills/status，按仿真时间记录（独立线程 spin）."""

    def __init__(self, entity='scr_robot_gazebo', status=True, period=0.02):
        import rclpy
        from gazebo_msgs.msg import ModelStates
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        self.rclpy = rclpy
        self.node = Node('hold_paths_recorder', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.entity, self.period = entity, period
        self.gt, self.status, self._last = [], [], -1.0
        self.node.create_subscription(ModelStates, '/gazebo/model_states', self._on_states, 10)
        if status:
            from rclpy.qos import DurabilityPolicy, QoSProfile
            from scr_interfaces.msg import TaskStatus
            qos = QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.node.create_subscription(TaskStatus, '/task_skills/status', self._on_status,
                                          qos)
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.node)
        self.th = threading.Thread(target=self._spin, daemon=True)
        self.th.start()

    def _spin(self):
        while self.rclpy.ok():
            try:
                self.ex.spin_once(timeout_sec=0.05)
            except Exception:  # noqa: BLE001  （关闭时的竞争）
                break

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _on_states(self, msg):
        if self.entity not in msg.name:
            return
        t = self.now()
        if t - self._last < self.period:
            return
        self._last = t
        p = msg.pose[msg.name.index(self.entity)]
        self.gt.append((t, p.position.x, p.position.y, yaw_of(p.orientation)))

    def _on_status(self, msg):
        st = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        self.status.append((self.now(), st, msg.station, msg.phase, msg.detail))

    def close(self):
        self.ex.shutdown()
        self.node.destroy_node()


def write_csv(path, header, rows):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


# ---------------------------------------------------------------- 单站任务
def run_task(args, out):
    import rclpy
    params_file = os.path.join(out, 'skill_params.yaml')
    with open(params_file, 'w', encoding='utf-8') as f:
        yaml.safe_dump(skill_params(args.hold, []), f, allow_unicode=True, sort_keys=False)
    extra = [f'odom_seed:={args.seed}', f'alignment_params:={params_file}']
    client = ['ros2', 'run', 'scr_arm_controller', 'navigation_cutting_task', TASK_FILE]
    with open(os.path.join(out, 'command.txt'), 'w') as f:
        f.write(rv.portable(' '.join(ss.launch_cmd(extra))) + '\n' + ' '.join(client) + '\n')
    summary = dict(path='single_station_task', hold=args.hold or 'p', seed=args.seed)
    with ss.Session(out, extra) as s:
        ok, t_ready = s.wait_ready(args.startup_timeout)
        summary['startup_wall_s'] = round(t_ready, 1)
        if not ok:
            summary['error'] = 'task layer not ready'
        else:
            rclpy.init()
            rec = TruthRecorder()
            t0_wall = time.time()
            with open(os.path.join(out, 'client.log'), 'w') as log:
                try:
                    rc = subprocess.run(client, env=s.env, stdout=log, stderr=subprocess.STDOUT,
                                        timeout=args.timeout, start_new_session=True).returncode
                except subprocess.TimeoutExpired:
                    rc = -1
            time.sleep(1.0)
            summary.update(client_rc=rc, wall_s=round(time.time() - t0_wall, 1))
            write_csv(os.path.join(out, 'truth.csv'), ['t', 'x', 'y', 'yaw'], rec.gt)
            write_csv(os.path.join(out, 'status.csv'),
                      ['t_recv', 'stamp', 'station', 'phase', 'detail'], rec.status)
            rec.close()
            rclpy.shutdown()
    with open(os.path.join(out, 'launch.log'), errors='replace') as f, \
            open(os.path.join(out, 'task_layer.log'), 'w') as g:
        for line in f:
            if '[task_skills' in line or '[task_executor' in line:
                g.write(line)
    with open(os.path.join(out, 'summary.yaml'), 'w') as f:
        yaml.safe_dump(summary, f, sort_keys=False)
    portable_logs(out)
    return 0 if summary.get('client_rc') == 0 else 1


# ---------------------------------------------------------------- 精对位 + 示教
def gazebo_cmd(seed):
    return ['ros2', 'launch', 'scr_description', 'gazebo.launch.py', 'gui:=false',
            f'odom_seed:={seed}']


def run_align(args, out):
    import numpy as np
    import rclpy
    env = ss.env()
    ss.kill_stray_gazebo()
    for pid in rv.leftover(env):
        os.kill(pid, 9)
    hold = args.hold or 'p'
    ros_args = ['--ros-args', '-p', f'hold.mode:={hold}', '-p', f'odom_topic:={args.odom}']
    teach_cmd = ['ros2', 'run', 'scr_arm_controller', 'teach_alignment_offset', '--out',
                 os.path.join(out, 'taught.yaml')] + ros_args
    with open(os.path.join(out, 'command.txt'), 'w') as f:
        f.write(' '.join(gazebo_cmd(args.seed)) + '\n'
                + f'hold_paths.py align: v2 精对位 × {TRIALS}（起点种子 42），到位后保持 '
                  f'{HOLD_S:.0f} s；hold.mode:={hold}，odom_topic:={args.odom}\n'
                + rv.portable(' '.join(teach_cmd)) + '\n')
    log = open(os.path.join(out, 'launch.log'), 'w')
    sim = subprocess.Popen(gazebo_cmd(args.seed), env=env, stdout=log, stderr=subprocess.STDOUT,
                           start_new_session=True)
    rows, trace, summary = [], [], dict(path='align_teach', hold=hold, seed=args.seed)
    try:
        time.sleep(25.0)
        from gazebo_msgs.srv import SetEntityState
        from scr_arm_controller import alignment_benchmark as ab
        from scr_arm_controller.task_skills import CuttingSkills
        rclpy.init(args=ros_args)
        node = CuttingSkills()
        node.distance_from_marker = ab.DISTANCE
        cli = node.create_client(SetEntityState, '/gazebo/set_entity_state')
        if not cli.wait_for_service(timeout_sec=60.0):
            raise RuntimeError('no /gazebo/set_entity_state')
        summary['hold_law_in_node'] = type(node.base_hold._law).__name__
        tx, ty, tyaw = ab.TAG_X, ab.WALL_Y + ab.DISTANCE, -math.pi / 2
        rng = np.random.default_rng(42)
        starts = [(tx + rng.uniform(-0.3, 0.3), ty + rng.uniform(0.3, 0.8),
                   tyaw + math.radians(rng.uniform(-10, 10))) for _ in range(TRIALS)]
        node.positioning_mode = 'visual'
        node.apply_alignment_config(detector='apriltag', offset_normal=0.0, offset_lateral=0.0,
                                    offset_yaw_deg=0.0)
        for k, (sx, sy, syaw) in enumerate(starts):
            node.base_hold.release()
            node.stop_robot()
            if not ab.teleport(node, cli, sx, sy, syaw):
                continue
            node.sleep_sim(1.0)
            t0 = node.get_clock().now().nanoseconds * 1e-9
            ok = node.precise_positioning_to_marker()
            t1 = node.get_clock().now().nanoseconds * 1e-9
            gt0 = node.align_gt or node.get_ground_truth_position()
            d0, u0, p0 = ab.wall_state_true(gt0)
            dev = []
            while node.get_clock().now().nanoseconds * 1e-9 - t1 < HOLD_S:
                node.sleep_sim(0.05)
                g = node.get_ground_truth_position()
                dt = node.get_clock().now().nanoseconds * 1e-9 - t1
                dx, dy = g['x'] - gt0['x'], g['y'] - gt0['y']
                dyaw = math.atan2(math.sin(g['yaw'] - gt0['yaw']), math.cos(g['yaw'] - gt0['yaw']))
                dev.append((math.hypot(dx, dy), dyaw))
                trace.append((k, round(dt, 3), dx * 1000, dy * 1000, math.degrees(dyaw)))
            node.base_hold.release()
            node.stop_robot()
            dev = np.array(dev) if dev else np.zeros((1, 2))
            row = dict(trial=k, success=bool(ok), align_s=round(t1 - t0, 2),
                       normal_err_mm=(d0 - ab.DISTANCE) * 1000, lateral_err_mm=u0 * 1000,
                       yaw_err_deg=math.degrees(p0),
                       hold_drift_max_mm=float(dev[:, 0].max() * 1000),
                       hold_drift_end_mm=float(dev[-1, 0] * 1000),
                       hold_yaw_max_deg=float(np.degrees(np.abs(dev[:, 1]).max())))
            rows.append(row)
            print(f'[{hold} s{args.seed} #{k}] ok={ok} align {row["align_s"]:.1f} s, '
                  f'drift max {row["hold_drift_max_mm"]:.3f} mm, yaw max '
                  f'{row["hold_yaw_max_deg"]:.3f}°', flush=True)
        node.destroy_node()
        # 示教：独立进程（与文档用法相同），本进程只记录真值
        rec = TruthRecorder(status=False, period=0.05)
        t_start = rec.now()
        with open(os.path.join(out, 'teach.log'), 'w') as tl:
            rc = subprocess.run(teach_cmd, env=env, stdout=tl, stderr=subprocess.STDOUT,
                                timeout=300).returncode
        summary['teach_rc'] = rc
        write_csv(os.path.join(out, 'teach_truth.csv'), ['t', 'x', 'y', 'yaw'],
                  [r for r in rec.gt if r[0] >= t_start])
        rec.close()
        rclpy.shutdown()
    except Exception as e:  # noqa: BLE001
        summary['error'] = repr(e)
        print('错误：', repr(e), flush=True)
    finally:
        rv.kill_group(sim, 'gazebo')
        log.close()
        time.sleep(2.0)
        for pid in rv.leftover(env):
            try:
                os.kill(pid, 9)
            except ProcessLookupError:
                pass
        ss.kill_stray_gazebo()
        subprocess.run('fastdds shm clean', shell=True, env=env, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
    if rows:
        write_csv(os.path.join(out, 'align_trials.csv'), list(rows[0]),
                  [list(r.values()) for r in rows])
        write_csv(os.path.join(out, 'align_hold_trace.csv'),
                  ['trial', 't', 'dx_mm', 'dy_mm', 'dyaw_deg'], trace)
    taught = os.path.join(out, 'taught.yaml')
    if os.path.exists(taught):
        with open(taught, encoding='utf-8') as f:
            summary['taught'] = yaml.safe_load(f)['/**']['ros__parameters']['alignment']
    with open(os.path.join(out, 'summary.yaml'), 'w') as f:
        yaml.safe_dump(summary, f, sort_keys=False, allow_unicode=True)
    portable_logs(out)
    return 0 if rows and 'error' not in summary else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('path', choices=['task', 'align'])
    ap.add_argument('--hold', default='', help='hold.mode（空 = 默认 p）')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--out', required=True)
    ap.add_argument('--odom', default='/wheel/odom', help='align：两个工具的 odom_topic')
    ap.add_argument('--timeout', type=float, default=900.0)
    ap.add_argument('--startup-timeout', type=float, default=240.0)
    args = ap.parse_args()
    out = os.path.abspath(args.out)
    os.makedirs(out, exist_ok=True)
    t0 = time.time()
    rc = (run_task if args.path == 'task' else run_align)(args, out)
    print(f'{os.path.basename(out)}: rc={rc}, {time.time() - t0:.0f} s', flush=True)
    return rc


if __name__ == '__main__':
    sys.exit(main())
