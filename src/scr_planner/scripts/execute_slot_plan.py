#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多停靠站开槽执行（Gazebo）.

流程：规划 → Nav2 导航到定位标记前 → 视觉伺服对位（现有 PBVS，确定墙面坐标原点）
→ 依次沿墙横移到各停靠站（里程计控横向，激光拟合墙面控离墙距离与朝向）
→ 每站执行机械臂切割点 → 用 Gazebo 真值记录刀尖轨迹，与规划槽线比较。

复用 scr_arm_controller.navigation_cutting_task.NavigationCuttingTask（不修改它）。

  ros2 run scr_planner execute_slot_plan.py <task.yaml> [结果目录]
"""
import csv
import math
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Twist
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import MarkerArray

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'python'))
try:
    import slot_ros  # noqa: F401
except ImportError:
    from ament_index_python.packages import get_package_share_directory
    sys.path.insert(0, os.path.join(get_package_share_directory('scr_planner'), 'python'))

import slot_plan as sp  # noqa: E402
from scr_arm_controller.navigation_cutting_task import NavigationCuttingTask  # noqa: E402
from scr_arm_controller.visual_servo import pbvs_command  # noqa: E402
from scr_planner_py import OrientedAStar  # noqa: E402
from slot_ros import build_markers  # noqa: E402

TOOL_LINK = 'scr_robot_gazebo::link4'


class SlotExecutor(NavigationCuttingTask):
    def __init__(self):
        super().__init__()
        self.trace = []            # (t, station, u, v, depth) 真值刀尖
        self.cur_station = -1
        self.wall = None
        self.create_subscription(LinkStates, '/gazebo/link_states', self._links_cb, 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, '/wall_slot_plan/markers', qos)

    # --- 真值刀尖 -----------------------------------------------------------
    def _links_cb(self, msg):
        if self.wall is None or TOOL_LINK not in msg.name:
            return
        gt = self.get_ground_truth_position()
        if gt is None:
            return
        p = msg.pose[msg.name.index(TOOL_LINK)].position
        # 刀尖 = link4 原点沿 base_link x 方向前伸 tool_offset（visual_servo_ros.tool_front_x）
        tip = np.array([p.x, p.y]) + sp.RobotModel().tool_offset * np.array(
            [math.cos(gt['yaw']), math.sin(gt['yaw'])])
        rel = tip - self.wall.p0
        u = float(rel @ self.wall.t)
        depth = float(-(rel @ self.wall.n))      # >0：刀尖进入墙体
        self.trace.append((self.get_clock().now().nanoseconds * 1e-9, self.cur_station,
                           u, float(p.z), depth))

    # --- 沿墙横移 -------------------------------------------------------------
    def strafe_to(self, du_target, ref, tol_lat=0.003, tol_norm=0.005, tol_yaw=math.radians(0.5),
                  max_iter=600):
        """横移到相对参考位姿 du_target（沿墙向右为正，米）处，保持离墙 standoff."""
        D = self.distance_from_marker
        right = np.array([math.sin(ref['yaw']), -math.cos(ref['yaw'])])   # 面墙时的右方
        ok = 0
        for it in range(max_iter):
            pose = self.get_robot_position()
            wall, why = self.servo.wall_in_base()
            if pose is None or wall is None:
                self.cmd_vel_pub.publish(Twist())
                self.sleep_sim(0.1)
                continue
            du = float((np.array([pose['x'], pose['y']]) - ref['p']) @ right)
            n = np.array(wall['normal'])
            err = {'normal': wall['distance'] - D, 'lateral': -(du_target - du),
                   'yaw': wall['yaw_error'], 'n': n, 't': np.array([-n[1], n[0])}
            if abs(err['lateral']) < tol_lat and abs(err['normal']) < tol_norm and \
                    abs(err['yaw']) < tol_yaw:
                ok += 1
                if ok >= 5:
                    self.stop_robot()
                    return True
            else:
                ok = 0
            vx, vy, wz = pbvs_command(err, max_lin=0.1, pos_deadband=tol_lat * 0.5,
                                      ang_deadband=tol_yaw * 0.5)
            cmd = Twist()
            cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz
            self.cmd_vel_pub.publish(self.apply_wall_guard(cmd))
            self.sleep_sim(0.1)
        self.stop_robot()
        return False

    def gt_station_error(self, st, robot):
        self.sleep_sim(0.3)
        gt = self.get_ground_truth_position()
        if gt is None:
            return None
        rel = np.array([gt['x'], gt['y']]) - self.wall.p0
        u = float(rel @ self.wall.t)
        d = float(rel @ self.wall.n)
        yaw_t = st.pose[2]
        dyaw = math.atan2(math.sin(gt['yaw'] - yaw_t), math.cos(gt['yaw'] - yaw_t))
        return dict(lateral_mm=(u - st.u_c) * 1000, standoff_mm=(d - robot.standoff) * 1000,
                    yaw_deg=math.degrees(dyaw))


def evaluate(trace, plan, robot, depth_target):
    """真值刀尖轨迹 vs 规划槽线."""
    cut = [(s, u, v, d) for _, s, u, v, d in trace if d > 0.002]
    deep = [(s, u, v, d) for s, u, v, d in cut if d > depth_target - 0.005]
    res = dict(trace_samples=len(trace), in_wall_samples=len(cut))
    if not deep:
        return res
    dist = np.array([_dist_poly((u, v), plan.conduit) for _, u, v, _ in deep])
    res.update(path_err_mean_mm=float(dist.mean() * 1000), path_err_p95_mm=float(
        np.percentile(dist, 95) * 1000), path_err_max_mm=float(dist.max() * 1000),
        depth_mean_mm=float(np.mean([d for *_, d in deep]) * 1000))
    # 覆盖率：可达槽线每 5 mm 取样，3 mm 内有刀尖经过即算已切
    band = robot.reach_band
    pts = [q for piece in sp.clip_polyline(plan.conduit, band[0], band[1], axis=1)
           for q in sp._densify(piece, 0.005)]
    uv = np.array([(u, v) for _, u, v, _ in deep])
    covered = sum(np.min(np.hypot(uv[:, 0] - q[0], uv[:, 1] - q[1])) < 0.003 for q in pts)
    res['coverage'] = covered / max(1, len(pts))
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
    task_file = sys.argv[1]
    out_dir = sys.argv[2] if len(sys.argv) > 2 else os.getcwd()
    os.makedirs(out_dir, exist_ok=True)
    with open(task_file, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    scene, wall = sp.scene_from_config(cfg)
    robot = sp.RobotModel()
    opt = cfg.get('options', {})
    plan = sp.plan_task(scene, wall, robot, OrientedAStar(),
                        restrict_to_reach=opt.get('restrict_to_reach', True),
                        retract_between_turns=opt.get('retract_between_turns', True))
    if plan.status != 'success' or not plan.stations:
        print(f'规划失败: {plan.status}')
        sys.exit(2)
    duration = opt.get('duration', 2.0)
    with open(os.path.join(out_dir, 'slot_plan.yaml'), 'w', encoding='utf-8') as f:
        yaml.safe_dump(sp.plan_to_dict(plan, wall, robot, duration), f, allow_unicode=True,
                       sort_keys=False)

    rclpy.init()
    ex = SlotExecutor()
    ex.wall = wall
    ex.distance_from_marker = robot.standoff
    ex.positioning_mode = 'visual'
    ex.marker_pub.publish(build_markers(plan, scene, wall, robot, ex.get_clock().now().to_msg()))
    log = ex.get_logger()
    t_start = time.time()
    sim_start = ex.get_clock().now()

    # 定位标记在墙面坐标中的位置 = 参考原点
    m = np.array([ex.marker_position['x'], ex.marker_position['y']])
    u_marker = float((m - wall.p0) @ wall.t)
    log.info(f'定位标记 u = {u_marker:.3f} m；停靠站 {len(plan.stations)} 个')

    # 1. Nav2 导航到标记正前方 1.2 m
    nx, ny, nyaw = wall.station_pose(u_marker, 1.2)
    if not ex.navigate_to_pose(nx, ny, nyaw):
        log.error('导航失败')
        sys.exit(3)
    ex.sleep_sim(2.0)
    # 2. 视觉伺服对位到标记正前方 standoff
    if not ex.precise_positioning_to_marker():
        log.error('对位失败')
        sys.exit(4)
    pose = ex.get_robot_position()
    ref = {'p': np.array([pose['x'], pose['y']]), 'yaw': pose['yaw']}

    rows = []
    for st in plan.stations:
        log.info(f'\n===== 停靠站 S{st.index}: u = {st.u_c:.3f} m（横移 '
                 f'{(st.u_c - u_marker) * 100:+.1f} cm），{len(st.points)} 个点 =====')
        t0 = ex.get_clock().now()
        ok_move = ex.strafe_to(st.u_c - u_marker, ref)
        t_move = (ex.get_clock().now() - t0).nanoseconds * 1e-9
        gt_err = ex.gt_station_error(st, robot) or {}
        log.info(f'到位 {"✓" if ok_move else "✗"}，用时 {t_move:.1f} s，真值误差 '
                 + ', '.join(f'{k}={v:.2f}' for k, v in gt_err.items()))
        ex.cur_station = st.index
        pts = [(p['x'], p['y'], p['z'], math.radians(p['rotation'])) for p in st.points]
        t1 = ex.get_clock().now()
        ok_cut = ex.execute_cutting_points(pts, duration, return_home=False)
        t_cut = (ex.get_clock().now() - t1).nanoseconds * 1e-9
        ex.cur_station = -1
        rows.append(dict(station=st.index, u_c=st.u_c, points=len(st.points),
                         cut_cm=st.cut_length * 100, moved=int(ok_move), cut_ok=int(ok_cut),
                         move_s=t_move, cut_s=t_cut, **gt_err))
        if not ok_cut:
            log.error(f'S{st.index} 切割失败，停止')
            break
    ex.arm_controller.move_to_joints([0.0, 0.0, ex.approach_joint3, 0.0], duration=2.0)
    sim_total = (ex.get_clock().now() - sim_start).nanoseconds * 1e-9

    summary = evaluate(ex.trace, plan, robot, plan.slot_depth)
    summary.update(stations=len(plan.stations), stations_done=len(rows),
                   all_ok=all(r['moved'] and r['cut_ok'] for r in rows) and len(rows) == len(
                       plan.stations), sim_time_s=sim_total, wall_time_s=time.time() - t_start,
                   slot_length_m=plan.length)
    for k in ('lateral_mm', 'standoff_mm', 'yaw_deg'):
        vals = [abs(r[k]) for r in rows if k in r]
        if vals:
            summary[f'station_{k}_mean_abs'] = float(np.mean(vals))
            summary[f'station_{k}_max_abs'] = float(np.max(vals))
    with open(os.path.join(out_dir, 'stations.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(out_dir, 'tool_trace.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'station', 'u', 'v', 'depth'])
        w.writerows(ex.trace)
    with open(os.path.join(out_dir, 'summary.yaml'), 'w', encoding='utf-8') as f:
        yaml.safe_dump({k: (round(v, 4) if isinstance(v, float) else v)
                        for k, v in summary.items()}, f, allow_unicode=True, sort_keys=False)
    executed = [(u, v) for _, s, u, v, d in ex.trace if d > plan.slot_depth - 0.005]
    ex.marker_pub.publish(build_markers(plan, scene, wall, robot, ex.get_clock().now().to_msg(),
                                        executed=executed[::5]))
    log.info('\n' + '\n'.join(f'  {k}: {v}' for k, v in summary.items()))
    ex.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
