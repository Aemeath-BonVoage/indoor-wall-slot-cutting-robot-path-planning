#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多停靠站开槽执行（Gazebo）.

流程：读规划文件 → Nav2 导航到定位标记前 → 视觉伺服对位（现有 PBVS，确定墙面坐标原点）
→ 依次沿墙横移到各停靠站（里程计控横向，激光拟合墙面控离墙距离与朝向）
→ 每站执行机械臂切割点 → 用 Gazebo 真值记录刀尖轨迹，与规划槽线比较。

复用 scr_arm_controller.navigation_cutting_task.NavigationCuttingTask（不修改它）。

  ros2 run scr_planner execute_slot_plan.py <plan.yaml> [结果目录] [--dry-run-gap=0.01]
                                           [--no-compensation]

到站后用里程计（横向）和激光拟合墙面（离墙距离、朝向）测出残余误差，按它修正本站的
机械臂点（刀尖离底盘中心约 0.69 m，0.5° 的朝向误差就会让刀尖横移约 6 mm）。

--dry-run-gap：空切。Gazebo 的墙是刚体，刀具下刀会把整台机器人顶开（实测约 35 mm），
切深无法仿真，接触力还会扭曲轨迹；空切时把切割位置改为刀尖停在墙面前 gap 处，
只验证墙面内 (u, v) 轨迹精度。真机上不加此参数。
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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
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
from scr_arm_controller.visual_servo import pbvs_command  # noqa: E402
from slot_ros import build_markers  # noqa: E402

TOOL_LINK = 'scr_robot_gazebo::link4'
BASE_LINK = 'scr_robot_gazebo::base_link'


class SlotExecutor(NavigationCuttingTask):
    def __init__(self):
        super().__init__()
        self.trace = []            # (t, station, u, v, depth) 真值刀尖
        self.cur_station = -1
        self.phase = 'idle'         # idle / move / cut
        self.wall = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, '/wall_slot_plan/markers', qos)
        # 真值记录放在独立节点 + 后台线程：机械臂运动时 cartesian_cutting 会阻塞在自己的
        # 节点上，主节点的回调此时得不到处理
        self.robot = sp.RobotModel()
        self.tracer = rclpy.create_node(
            'slot_tool_tracer',
            parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.tracer.create_subscription(LinkStates, '/gazebo/link_states', self._links_cb, 50)
        # 定点保持：机械臂运动的反作用力会推动底盘（Gazebo 中轮子无摩擦、底盘由 planar_move
        # 驱动，实测一站内可漂移 10 cm）。切割期间用里程计闭环把底盘保持在到站位姿。
        self.hold_target = None
        self._odom = None
        self.tracer.create_subscription(Odometry, '/odom', self._odom_cb, 20)
        self._hold_pub = self.tracer.create_publisher(Twist, '/cmd_vel', 10)
        self.tracer.create_timer(0.05, self._hold_step)
        self._tracer_exec = SingleThreadedExecutor()
        self._tracer_exec.add_node(self.tracer)
        threading.Thread(target=self._tracer_exec.spin, daemon=True).start()

    # --- 定点保持 -----------------------------------------------------------
    def _odom_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self._odom = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y),
                                          1 - 2 * (q.y * q.y + q.z * q.z)))

    def _hold_step(self, k_lin=1.0, k_ang=1.5, max_lin=0.05, max_ang=0.2):
        if self.hold_target is None or self._odom is None:
            return
        x, y, yaw = self._odom
        tx, ty, tyaw = self.hold_target
        ex, ey = tx - x, ty - y
        c, s_ = math.cos(yaw), math.sin(yaw)
        bx, by = c * ex + s_ * ey, -s_ * ex + c * ey          # 误差转到 base_link
        eyaw = math.atan2(math.sin(tyaw - yaw), math.cos(tyaw - yaw))
        cmd = Twist()
        cmd.linear.x = max(-max_lin, min(max_lin, k_lin * bx))
        cmd.linear.y = max(-max_lin, min(max_lin, k_lin * by))
        cmd.angular.z = max(-max_ang, min(max_ang, k_ang * eyaw))
        self._hold_pub.publish(cmd)

    def hold_here(self):
        self.hold_target = self._odom

    def release_hold(self):
        self.hold_target = None
        self._hold_pub.publish(Twist())

    # --- 真值刀尖 -----------------------------------------------------------
    def _links_cb(self, msg):
        if self.wall is None or TOOL_LINK not in msg.name or BASE_LINK not in msg.name:
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

    # --- 沿墙横移 -------------------------------------------------------------
    def strafe_to(self, du_target, ref, tol_lat=0.003, tol_norm=0.005, tol_yaw=math.radians(1.0),
                  max_iter=600, alpha=0.3):
        """横移到相对参考位姿 du_target（沿墙向右为正，米）处，保持离墙 standoff.

        横向用里程计（短距离内几乎无漂移、无噪声）；离墙距离与朝向用激光拟合墙面，
        单帧拟合 rms 约 9 mm、朝向抖动约 ±1°，故做指数平滑（alpha）。朝向容差 1°（再收紧到
        0.3° 时实测横移常常 60 s 内不收敛）；剩余朝向误差由到站后的机械臂点补偿消除。
        """
        D = self.distance_from_marker
        right = np.array([math.sin(ref['yaw']), -math.cos(ref['yaw'])])   # 面墙时的右方
        ok = 0
        filt = None
        for it in range(max_iter):
            pose = self.get_robot_position()
            wall, why = self.servo.wall_in_base()
            if pose is None or wall is None:
                self.cmd_vel_pub.publish(Twist())
                self.sleep_sim(0.1)
                continue
            raw = np.array([wall['distance'] - D, wall['yaw_error']])
            filt = raw if filt is None else (1 - alpha) * filt + alpha * raw
            du = float((np.array([pose['x'], pose['y']]) - ref['p']) @ right)
            n = np.array(wall['normal'])
            err = {'normal': float(filt[0]), 'lateral': -(du_target - du),
                   'yaw': float(filt[1]), 'n': n, 't': np.array([-n[1], n[0]])}
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

    def run_points(self, points, is_cut, duration):
        """逐点执行机械臂点，并标注阶段：相邻两点都在切割深度时为 'cut'."""
        prev_cut = False
        for (x, y, z, rot), cut in zip(points, is_cut):
            self.phase = 'cut' if (prev_cut and cut) else 'move'
            if not self.arm_controller.move_to_cartesian(x, y, z, rot, duration=duration):
                self.phase = 'idle'
                return False
            prev_cut = cut
        self.phase = 'idle'
        return True

    def measure_residual(self, du_target, ref, n=10):
        """到站后的残余误差（只用机器人自身传感器）：横向 e_l（里程计，米，沿墙向右为正）、
        离墙 e_n（激光拟合墙面，米，>0 表示离墙偏远）、朝向 psi（rad，逆时针为正）."""
        right = np.array([math.sin(ref['yaw']), -math.cos(ref['yaw'])])
        ls, ns, ys = [], [], []
        for _ in range(n * 3):
            pose = self.get_robot_position()
            wall, _ = self.servo.wall_in_base()
            if pose is not None and wall is not None:
                ls.append(float((np.array([pose['x'], pose['y']]) - ref['p']) @ right) - du_target)
                ns.append(wall['distance'] - self.distance_from_marker)
                ys.append(-wall['yaw_error'])   # yaw_error > 0 需逆时针转 → 机器人偏顺时针
                if len(ls) >= n:
                    break
            self.sleep_sim(0.1)
        if not ls:
            return 0.0, 0.0, 0.0
        return float(np.mean(ls)), float(np.mean(ns)), float(np.mean(ys))

    @staticmethod
    def compensate(points, is_cut, e_l, e_n, psi, robot):
        """把机械臂点修正到实际停靠位姿下（小角度近似）.

        刀尖在 base 系的横向位置 fy 对应墙面 u ≈ u_base - fx·psi - fy；令其等于名义值
        得 fy' = fy + e_l - fx·psi（u_base = 目标 + e_l）。切割深度处的点离墙偏远则多伸出
        e_n；退刀点不动。超出关节限位的分量钳制到限位并计数。
        """
        (xlo, xhi), (ylo, yhi) = robot.x_range, robot.y_range
        out, clamped = [], 0
        for (x, y, z, rot), cut in zip(points, is_cut):
            fx = x + robot.tool_offset
            y2 = y + e_l - fx * psi
            x2 = x + e_n if cut else x
            y3, x3 = min(max(y2, ylo), yhi), min(max(x2, xlo), xhi)
            clamped += int(abs(y3 - y2) > 1e-9 or abs(x3 - x2) > 1e-9)
            out.append((x3, y3, z, rot))
        return out, clamped

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
    dry_gap = None
    compensate = '--no-compensation' not in sys.argv
    for a in sys.argv[1:]:
        if a.startswith('--dry-run-gap='):
            dry_gap = float(a.split('=', 1)[1])
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
    duration = plan_dict['stations'][0]['motion']['duration']
    with open(os.path.join(out_dir, 'slot_plan.yaml'), 'w', encoding='utf-8') as f:
        yaml.safe_dump(plan_dict, f, allow_unicode=True, sort_keys=False)

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
    if 'marker_u' in plan_dict and abs(plan_dict['marker_u'] - u_marker) > 1e-3:
        log.warn(f"规划时的标记位置 u={plan_dict['marker_u']:.3f} 与执行器不一致 ({u_marker:.3f})")
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
    prev_u = u_marker
    for st in plan.stations:
        log.info(f'\n===== 停靠站 S{st.index}: u = {st.u_c:.3f} m（横移 '
                 f'{(st.u_c - u_marker) * 100:+.1f} cm），{len(st.points)} 个点 =====')
        t0 = ex.tracer.get_clock().now()
        ok_move = ex.strafe_to(st.u_c - u_marker, ref)
        ex.hold_here()      # 到站即定点保持，直到本站切完
        t_move = (ex.tracer.get_clock().now() - t0).nanoseconds * 1e-9
        gt_err = ex.gt_station_error(st, robot) or {}
        log.info(f'到位 {"✓" if ok_move else "✗"}，用时 {t_move:.1f} s，真值误差 '
                 + ', '.join(f'{k}={v:.2f}' for k, v in gt_err.items()))
        ex.cur_station = st.index
        x_cut = robot.x_cut(plan.slot_depth)
        x_exec = x_cut if dry_gap is None else robot.standoff - dry_gap - robot.tool_offset
        is_cut = [abs(p['x'] - x_cut) < 1e-6 for p in st.points]
        pts = [(x_exec if c else p['x'], p['y'], p['z'], math.radians(p['rotation']))
               for p, c in zip(st.points, is_cut)]
        e_l, e_n, psi = ex.measure_residual(st.u_c - u_marker, ref)
        if compensate:
            pts, clamped = ex.compensate(pts, is_cut, e_l, e_n, psi, robot)
            if clamped:
                log.warn(f'S{st.index}: {clamped} 个点的补偿量被关节限位钳制')
        log.info(f'残余误差（传感器）: 横向 {e_l * 1000:+.1f} mm, 离墙 {e_n * 1000:+.1f} mm, '
                 f'朝向 {math.degrees(psi):+.2f}°' + ('，已补偿' if compensate else ''))
        t1 = ex.tracer.get_clock().now()
        ok_cut = ex.run_points(pts, is_cut, duration)
        ex.release_hold()
        t_cut = (ex.tracer.get_clock().now() - t1).nanoseconds * 1e-9
        ex.cur_station = -1
        bs = [(r[7], r[8]) for r in ex.trace if r[1] == st.index]
        drift = (max(math.hypot(a - bs[0][0], d - bs[0][1]) for a, d in bs) * 1000) if bs else float('nan')
        station_s = (ex.tracer.get_clock().now() - t0).nanoseconds * 1e-9
        move_dist = abs(st.u_c - prev_u)
        prev_u = st.u_c
        rows.append(dict(station=st.index, u_c=st.u_c, points=len(st.points),
                         move_dist_m=move_dist, station_s=station_s,
                         sensed_lateral_mm=e_l * 1000, sensed_standoff_mm=e_n * 1000,
                         sensed_yaw_deg=math.degrees(psi),
                         base_drift_mm=drift,
                         cut_cm=st.cut_length * 100, moved=int(ok_move), cut_ok=int(ok_cut),
                         move_s=t_move, cut_s=t_cut, **gt_err))
        if not ok_cut:
            log.error(f'S{st.index} 切割失败，停止')
            break
    ex.arm_controller.move_to_joints([0.0, 0.0, ex.approach_joint3, 0.0], duration=2.0)
    sim_total = (ex.get_clock().now() - sim_start).nanoseconds * 1e-9

    summary = evaluate(ex.trace, plan, robot, plan.slot_depth)
    summary.update(plan_partition=plan.partition, plan_order=plan.order,
                   predicted_t_total=plan.prediction.get('t_total'),
                   stations_total_s=float(sum(r['station_s'] for r in rows)),
                   dry_run_gap=dry_gap, compensation=compensate, stations=len(plan.stations), stations_done=len(rows),
                   all_ok=all(r['moved'] and r['cut_ok'] for r in rows) and len(rows) == len(
                       plan.stations), sim_time_s=sim_total, wall_time_s=time.time() - t_start,
                   slot_length_m=plan.length)
    for k in ('lateral_mm', 'standoff_mm', 'yaw_deg', 'base_drift_mm'):
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
    ex.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
