#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多站开槽执行的 Gazebo 实验记录器（评估用；执行本身走主任务）.

执行主路径是 scr_arm_controller.navigation_cutting_task（NavigationCuttingTask.run_slot_plan）：
它直接读取 plan_wall_slots.py 的规划文件，Nav2 → 视觉对位 → 逐站横移 / 定点保持 /
残差补偿 / 切割。本脚本在其上只加 Gazebo 真值记录（刀尖轨迹、每站到位误差、底盘漂移），
并把结果与规划槽线比较，写出 stations.csv / tool_trace.csv / summary.yaml。

  ros2 run scr_planner execute_slot_plan.py <plan.yaml> [结果目录] [--dry-run-gap=0.01]
                                           [--no-compensation] [--max-strafe=1.5]

不需要真值评估时直接用主任务：
  ros2 run scr_arm_controller navigation_cutting_task <plan.yaml> --dry-run-gap=0.01

--dry-run-gap：空切（本项目仿真中的标准用法，只验证切割轨迹，不真正开槽）。切深处的点改为
刀尖停在墙面前 gap 处，其余动作与真实切割相同。Gazebo 的墙是刚体，真的下刀会把整台
机器人顶开，所以仿真中必须加这个参数。
"""
import csv
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
import rclpy.parameter
import yaml
from gazebo_msgs.msg import LinkStates
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import MarkerArray

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'python'))
try:
    import slot_ros  # noqa: F401
except ImportError:
    from ament_index_python.packages import get_package_share_directory
    sys.path.insert(0, os.path.join(get_package_share_directory('scr_planner'), 'python'))

import slot_plan as sp  # noqa: E402
import wall_rules as wr  # noqa: E402
from scr_arm_controller.navigation_cutting_task import NavigationCuttingTask  # noqa: E402
from slot_ros import build_markers  # noqa: E402

TOOL_LINK = 'scr_robot_gazebo::link4'
BASE_LINK = 'scr_robot_gazebo::base_link'


class SlotExecutor(NavigationCuttingTask):
    """主任务 + Gazebo 真值记录（刀尖轨迹、每站到位误差），只用于评估，不参与控制."""

    def __init__(self, wall):
        super().__init__()
        self.wall = wall            # slot_plan.WallFrame
        self.robot = sp.RobotModel()
        self.trace = []             # (t, station, u, v, depth, arm_x, phase, base_u, base_d)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, '/wall_slot_plan/markers', qos)
        # 真值记录放在独立节点 + 后台线程：机械臂运动时 cartesian_cutting 会阻塞在自己的
        # 节点上，主节点的回调此时得不到处理
        self.tracer = rclpy.create_node(
            'slot_tool_tracer',
            parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.tracer.create_subscription(LinkStates, '/gazebo/link_states', self._links_cb, 50)
        self._tracer_exec = SingleThreadedExecutor()
        self._tracer_exec.add_node(self.tracer)
        self._tracer_thread = threading.Thread(target=self._tracer_exec.spin, daemon=True)
        self._tracer_thread.start()

    def shutdown_tracer(self):
        """先停后台执行器并等线程退出，再销毁节点（否则退出时 C++ 层会 abort）."""
        self._tracer_exec.shutdown()
        self._tracer_thread.join(timeout=2.0)
        self.tracer.destroy_node()

    def _links_cb(self, msg):
        if TOOL_LINK not in msg.name or BASE_LINK not in msg.name:
            return
        b = msg.pose[msg.name.index(BASE_LINK)]
        p = msg.pose[msg.name.index(TOOL_LINK)].position
        q = b.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        heading = np.array([math.cos(yaw), math.sin(yaw)])
        # 机械臂伸出量（base_link x），用来区分“切割 / 抵墙”与“退刀”阶段
        arm_x = float((np.array([p.x, p.y]) - np.array([b.position.x, b.position.y])) @ heading)
        # 刀尖 = link4 原点沿 base_link x 方向前伸 tool_offset（visual_servo_ros.tool_front_x）
        tip = np.array([p.x, p.y]) + self.robot.tool_offset * heading
        rel = tip - self.wall.p0
        u = float(rel @ self.wall.t)
        depth = float(-(rel @ self.wall.n))      # >0：刀尖进入墙面（刚体墙上应约为 0）
        relb = np.array([b.position.x, b.position.y]) - self.wall.p0
        self.trace.append((msg_time(self.tracer), self.cur_station, u, float(p.z), depth, arm_x,
                           self.phase, float(relb @ self.wall.t), float(relb @ self.wall.n)))

    def on_station_arrived(self, st):
        """到站真值误差：横向 / 离墙 / 朝向（相对规划停靠位姿）."""
        self.sleep_sim(0.3)
        gt = self.get_ground_truth_position()
        if gt is None:
            return {}
        rel = np.array([gt['x'], gt['y']]) - self.wall.p0
        u, d = float(rel @ self.wall.t), float(rel @ self.wall.n)
        yaw_t = st['pose']['yaw']
        dyaw = math.atan2(math.sin(gt['yaw'] - yaw_t), math.cos(gt['yaw'] - yaw_t))
        err = dict(lateral_mm=(u - st['u_c']) * 1000,
                   standoff_mm=(d - self.robot.standoff) * 1000, yaw_deg=math.degrees(dyaw))
        self.get_logger().info('到站真值误差: ' + ', '.join(f'{k}={v:.2f}' for k, v in err.items()))
        return err


def msg_time(node):
    return node.get_clock().now().nanoseconds * 1e-9


def cutting_samples(trace, robot=None, depth=None):
    """切割阶段（相邻两个切割点之间的运动）的样本."""
    return [r for r in trace if r[1] >= 0 and r[6] == 'cut']


def evaluate(trace, plan, robot, depth_target):
    """真值刀尖轨迹（切割阶段）vs 规划槽线."""
    deep = cutting_samples(trace, robot, depth_target)
    res = dict(trace_samples=len(trace), cutting_samples=len(deep))
    if not deep:
        return res
    dist = np.array([min(_dist_poly((r[2], r[3]), cp) for cp in plan.cut_paths) for r in deep])
    res.update(path_err_mean_mm=float(dist.mean() * 1000), path_err_p95_mm=float(
        np.percentile(dist, 95) * 1000), path_err_max_mm=float(dist.max() * 1000),
        tip_depth_mean_mm=float(np.mean([r[4] for r in deep]) * 1000),
        tip_depth_max_mm=float(np.max([r[4] for r in deep]) * 1000),
        arm_x_mean=float(np.mean([r[5] for r in deep])))
    # 覆盖率：可达槽线每 5 mm 取样，3 mm 内有刀尖经过即算已切
    band = robot.reach_band
    pts = [q for cp in plan.cut_paths for piece in sp.clip_polyline(cp, band[0], band[1], axis=1)
           for q in sp._densify(piece, 0.005)]
    uv = np.array([(r[2], r[3]) for r in deep])
    covered = sum(np.min(np.hypot(uv[:, 0] - q[0], uv[:, 1] - q[1])) < 0.003 for q in pts)
    res['coverage'] = float(covered) / max(1, len(pts))
    return res


def _dist_poly(p, pts):
    p = np.asarray(p)
    best = math.inf
    for a, b in zip(pts[:-1], pts[1:]):
        a, b = np.asarray(a), np.asarray(b)
        t = np.clip(np.dot(p - a, b - a) / max(np.dot(b - a, b - a), 1e-18), 0, 1)
        best = min(best, float(np.linalg.norm(p - (a + t * (b - a)))))
    return best


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    flags = dict((a[2:].partition('=')[0], a[2:].partition('=')[2] or True)
                 for a in sys.argv[1:] if a.startswith('--'))
    dry_gap = float(flags['dry-run-gap']) if 'dry-run-gap' in flags else None
    plan_path = args[0]
    out_dir = args[1] if len(args) > 1 else os.getcwd()
    os.makedirs(out_dir, exist_ok=True)
    # 只读离线规划结果，执行中不重新规划
    with open(plan_path, encoding='utf-8') as f:
        plan_dict = yaml.safe_load(f)
    plan, wall, obstacles = sp.plan_from_dict(plan_dict)
    if plan.status != 'success' or not plan.stations:
        print(f'规划结果不可执行: {plan.status}')
        sys.exit(2)
    robot = sp.RobotModel()
    scene = wr.WallScene(int(round(wall.length * 100)), 300,
                         bool(plan_dict['wall'].get('load_bearing')), obstacles)
    with open(os.path.join(out_dir, 'slot_plan.yaml'), 'w', encoding='utf-8') as f:
        yaml.safe_dump(plan_dict, f, allow_unicode=True, sort_keys=False)

    rclpy.init()
    ex = SlotExecutor(wall)
    ex.compensate = 'no-compensation' not in flags
    if 'max-strafe' in flags:
        ex.max_strafe = float(flags['max-strafe'])
    ex.marker_pub.publish(build_markers(plan, scene, wall, robot, ex.get_clock().now().to_msg()))
    log = ex.get_logger()
    t_start = time.time()
    sim_start = ex.sim_now()
    ok, rows = ex.run_slot_plan(plan_dict, dry_gap)       # 主任务的多站执行路径
    sim_total = ex.sim_now() - sim_start
    if not rows:
        log.error('没有执行任何停靠站')
        shutdown(ex)
        sys.exit(3)
    # 每站底盘漂移（真值，切割期间相对到站时刻的最大位移）
    for r in rows:
        bs = [(t[7], t[8]) for t in ex.trace if t[1] == r['station']]
        r['base_drift_mm'] = (max(math.hypot(a - bs[0][0], d - bs[0][1]) for a, d in bs)
                              * 1000) if bs else float('nan')

    summary = evaluate(ex.trace, plan, robot, plan.slot_depth)
    summary.update(plan_partition=plan.partition, plan_order=plan.order,
                   predicted_t_total=plan.prediction.get('t_total'),
                   stations_total_s=float(sum(r['station_s'] for r in rows)),
                   dry_run_gap=dry_gap, compensation=ex.compensate,
                   stations=len(plan.stations), stations_done=len(rows), all_ok=ok,
                   sim_time_s=sim_total, wall_time_s=time.time() - t_start,
                   slot_length_m=plan.length)
    for k in ('lateral_mm', 'standoff_mm', 'yaw_deg', 'base_drift_mm', 'sensed_yaw_deg',
              'move_s'):
        vals = [abs(r[k]) for r in rows if k in r]
        if vals:
            summary[f'station_{k}_mean_abs'] = float(np.mean(vals))
            summary[f'station_{k}_max_abs'] = float(np.max(vals))
    summary['stations_converged'] = int(sum(r['moved'] for r in rows))
    with open(os.path.join(out_dir, 'stations.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(out_dir, 'tool_trace.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'station', 'u', 'v', 'depth', 'arm_x', 'phase', 'base_u', 'base_d'])
        w.writerows(ex.trace)
    with open(os.path.join(out_dir, 'summary.yaml'), 'w', encoding='utf-8') as f:
        yaml.safe_dump({k: (round(float(v), 4) if isinstance(v, (float, np.floating)) else
                            int(v) if isinstance(v, (np.integer, np.bool_)) else v)
                        for k, v in summary.items()}, f, allow_unicode=True, sort_keys=False)
    executed = [(r[2], r[3]) for r in cutting_samples(ex.trace)]
    ex.marker_pub.publish(build_markers(plan, scene, wall, robot, ex.get_clock().now().to_msg(),
                                        executed=executed[::5]))
    log.info('\n' + '\n'.join(f'  {k}: {v}' for k, v in summary.items()))
    shutdown(ex)
    sys.exit(0 if ok else 1)


def shutdown(ex):
    """按顺序停掉后台线程并销毁全部节点（否则退出时 C++ 层会 abort）."""
    ex.shutdown_tracer()
    ex.trajectory_visualizer.destroy_node()
    ex.arm_controller.destroy_node()
    ex.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
