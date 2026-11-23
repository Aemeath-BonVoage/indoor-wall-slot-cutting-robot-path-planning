#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多站开槽执行的 Gazebo 实验记录器（评估用；执行本身走行为树任务层）.

执行由行为树执行器完成（scr_task/task_executor 的 /execute_slot_plan action，Nav2 →
精对位 → 逐站横移 / 定点保持 / 残差补偿 / 切割）。本脚本作为该 action 的客户端发送规划
文件，同时只做 Gazebo 真值记录：刀尖轨迹、每站到位误差、底盘漂移；阶段与停靠站序号取自
技能服务器的 /task_skills/status。结果与规划槽线比较，写出 stations.csv / tool_trace.csv /
summary.yaml（字段与早期版本相同，另增 recoveries、realigned、since_fix_m），以及墙面相对
状态估计的逐周期记录 ekf_trace.csv（/task_skills/wall_estimate 与同一时刻的真值）和机械臂
控制器的指令 / 实际关节位置 arm_state.csv（/arm_controller/controller_state，50 Hz，时间戳
取自消息头，不受 /clock 10 Hz 发布节拍的限制）。加载了切割力插件时另写 wrench.csv：插件
实际施加的力旋量（/cutting_force/wrench）、执行器预计的力旋量
（/cutting_force/expected_wrench）与同一时刻的底盘真值位姿。

  ros2 run scr_planner execute_slot_plan.py <plan.yaml> [结果目录] [--dry-run-gap=0.01]
                                           [--no-compensation] [--max-strafe=1.5] [--bt=<xml>]
                                           [--realign-distance=0.3]

--realign-distance：无标记更新的沿墙行程超过该值（米）时先回参考标记重新对位；
不给出时用执行器参数 realign_distance，负值关闭。

不需要真值评估时直接用任务入口：
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
from action_msgs.msg import GoalStatus
from control_msgs.msg import JointTrajectoryControllerState
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import MarkerArray

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'python'))
try:
    import slot_ros  # noqa: F401
except ImportError:
    from ament_index_python.packages import get_package_share_directory
    sys.path.insert(0, os.path.join(get_package_share_directory('scr_planner'), 'python'))

import slot_plan as sp  # noqa: E402
import wall_rules as wr  # noqa: E402
from scr_arm_controller.slot_execution import label_by_time  # noqa: E402
from scr_interfaces.action import ExecuteSlotPlan  # noqa: E402
from scr_interfaces.msg import TaskStatus, WallEstimate  # noqa: E402
from slot_ros import build_markers  # noqa: E402

TOOL_LINK = 'scr_robot_gazebo::link4'
BASE_LINK = 'scr_robot_gazebo::base_link'
RECORD_FIELDS = ('station', 'u_c', 'points', 'move_dist_m', 'nav_leg', 'station_s', 'move_s',
                 'cut_s', 'sensed_lateral_mm', 'sensed_standoff_mm', 'sensed_yaw_deg', 'clamped',
                 'cut_cm', 'moved', 'cut_ok', 'origin_residual_mm', 'recoveries', 'realigned',
                 'since_fix_m')
CLOCK_PERIOD = 0.1          # Gazebo 发布 /clock 的周期（秒）
ARM_JOINTS = ('joint1', 'joint2', 'joint3', 'joint4')
ARM_FIELDS = (('t', 'station', 'phase') + tuple(f'ref_{j}' for j in ARM_JOINTS)
              + tuple(f'act_{j}' for j in ARM_JOINTS))
EKF_FIELDS = ('t', 'station', 'phase', 'd', 'u', 'psi', 'sigma_d', 'sigma_u', 'sigma_psi',
              'lateral_scale', 'sigma_scale', 'odom_since_fix', 'u_known', 'tag_ids',
              'tags_accepted', 'true_u', 'true_d', 'true_psi', 'tag_xy')


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class SlotRecorder:
    """行为树执行的 action 客户端 + Gazebo 真值记录（只用于评估，不参与控制）."""

    def __init__(self, wall, plan_dict, marker_u):
        self.node = rclpy.create_node(
            'slot_plan_recorder',
            parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.wall = wall            # slot_plan.WallFrame
        self.plan_dict = plan_dict
        self.robot = sp.RobotModel()
        self.trace = []             # (t, station, u, v, depth, arm_x, phase, base_u, base_d)
        self.truth = {}             # station → 到站真值误差
        self.marker_u = marker_u    # 墙面原点（参考标记）在规划墙面坐标中的 u
        self.ekf_trace = []
        self.arm_trace = []         # (t, station, phase, 指令关节位置 ×4, 实际关节位置 ×4)
        self.wrench_trace = []      # (t, 底盘真值 x y yaw, 施加的 Fx Fy Mz, 预计的 Fx Fy Mz)
        self._expected = (0.0, 0.0, 0.0)
        self._truth = None
        self.station, self.phase = -1, 'idle'
        self._pending = []          # 尚未生效的阶段 [(生效时刻, 停靠站, 阶段)]
        self._events = []           # 全部阶段事件 [(生效时刻, 停靠站, 阶段)]，结束时据此重新标注
        self._t_fine = None         # 细粒度仿真时间：/ground_truth/odom 的消息头（50 Hz）
        self._base = None
        self._t_goal = None
        self._lock = threading.Lock()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.node.create_publisher(
            MarkerArray, '/wall_slot_plan/markers', qos)
        self.node.create_subscription(LinkStates, '/gazebo/link_states', self._links_cb, 50)
        self.node.create_subscription(Odometry, '/ground_truth/odom', self._clock_cb, 10)
        status_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(TaskStatus, '/task_skills/status', self._status_cb,
                                      status_qos)
        self.node.create_subscription(WallEstimate, '/task_skills/wall_estimate',
                                      self._estimate_cb, 50)
        self.node.create_subscription(JointTrajectoryControllerState,
                                      '/arm_controller/controller_state', self._arm_cb, 50)
        self.node.create_subscription(WrenchStamped, '/cutting_force/wrench',
                                      self._wrench_cb, 50)
        self.node.create_subscription(WrenchStamped, '/cutting_force/expected_wrench',
                                      self._expected_cb, 50)
        self.client = ActionClient(self.node, ExecuteSlotPlan, '/execute_slot_plan')
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def fine_now(self):
        """
        细粒度仿真时间（秒）.

        节点时钟跟随 /clock，Gazebo 以 10 Hz 发布，分辨率只有 0.1 s；真值里程计的消息头
        带仿真时间、50 Hz。没有该话题时退回节点时钟。
        """
        return self._t_fine if self._t_fine is not None else self.now()

    def _clock_cb(self, msg):
        self._t_fine = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose
        self._truth = (p.position.x, p.position.y, _yaw(p.orientation))

    def _expected_cb(self, msg):
        w = msg.wrench
        self._expected = (w.force.x, w.force.y, w.torque.z)

    def _wrench_cb(self, msg):
        """切割力插件实际施加的力旋量（50 Hz）与同一时刻的底盘真值位姿."""
        if self._t_goal is None or self._truth is None:
            return
        w = msg.wrench
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.wrench_trace.append((t, *self._truth, w.force.x, w.force.y, w.torque.z,
                                  *self._expected))

    def _current_phase(self):
        """到当前时刻为止已生效的 (停靠站, 阶段)."""
        t = self.fine_now()
        with self._lock:
            while self._pending and self._pending[0][0] <= t:
                _, self.station, self.phase = self._pending.pop(0)
            return self.station, self.phase

    def shutdown(self):
        """先停后台执行器并等线程退出，再销毁节点（否则退出时 C++ 层会 abort）."""
        self.executor.shutdown()
        self._thread.join(timeout=2.0)
        self.node.destroy_node()

    # ---------- 回调 ----------
    def _status_cb(self, msg):
        t = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        if self._t_goal is None or t < self._t_goal:
            return              # 本次目标之前的状态（TRANSIENT_LOCAL 历史）
        # 时间戳是阶段的生效时刻。实时标注（估计与控制器状态的记录用）：已到的立即生效，
        # 未到的排队，由 _current_phase 在该时刻之后的第一个样本上生效。刀尖轨迹在运行结束
        # 后按事件时间重新标注（relabel），与状态消息的送达延迟无关：
        # - 按时间表提前发布的阶段（detail = scheduled）取消息里的生效时刻，晚到也不影响；
        # - 即时状态取到达时刻（与逐点执行的标注方式相同），并丢弃生效时刻在它之后的旧时间表
        #   （例如轨迹被中止）。它的时间戳取自技能服务器的节点时钟（/clock，10 Hz），真实
        #   发布时刻在时间戳之后 0.1 s 以内，到达时刻超出这个范围（送达延迟）时取其上界
        with self._lock:
            now = self.fine_now()
            if msg.detail == 'scheduled':
                when = t
            else:
                when = min(max(t, now), t + CLOCK_PERIOD)
                self._events = [e for e in self._events if e[0] <= when]
            self._events.append((when, msg.station, msg.phase))
            if t <= now:
                if msg.detail != 'scheduled':
                    self._pending.clear()
                self.station, self.phase = msg.station, msg.phase
            else:
                self._pending.append((t, msg.station, msg.phase))
        if msg.phase == 'arrived' and msg.station >= 0:
            # 到站并进入定点保持 0.3 s 后取真值（与早期版本的采样时刻相同）
            st = msg.station
            timer = []

            def sample():
                timer[0].cancel()
                self._sample_station(st)
            timer.append(self.node.create_timer(0.3, sample))

    def _sample_station(self, index):
        if self._base is None:
            return
        x, y, yaw = self._base
        st = next(s for s in self.plan_dict['stations'] if s['index'] == index)
        rel = np.array([x, y]) - self.wall.p0
        u, d = float(rel @ self.wall.t), float(rel @ self.wall.n)
        yaw_t = st['pose']['yaw']
        dyaw = math.atan2(math.sin(yaw - yaw_t), math.cos(yaw - yaw_t))
        err = dict(lateral_mm=(u - st['u_c']) * 1000,
                   standoff_mm=(d - self.robot.standoff) * 1000, yaw_deg=math.degrees(dyaw))
        self.truth[index] = err
        self.node.get_logger().info(
            f'S{index} 到站真值误差: ' + ', '.join(f'{k}={v:.2f}' for k, v in err.items()))

    def _estimate_cb(self, msg):
        """估计与同一时刻的真值（墙面相对状态，原点为参考标记）."""
        if self._t_goal is None or self._base is None:
            return
        x, y, yaw = self._base
        rel = np.array([x, y]) - self.wall.p0
        # 正对墙面时底盘朝向 = 墙面法向的反方向（指向墙内）
        n_in = -self.wall.n
        psi = math.atan2(math.sin(yaw - math.atan2(n_in[1], n_in[0])),
                         math.cos(yaw - math.atan2(n_in[1], n_in[0])))
        station, phase = self._current_phase()
        acc = [i for i, ok in zip(msg.tag_ids, msg.tag_accepted) if ok]
        self.ekf_trace.append((
            self.now(), station, phase, msg.d, msg.u, msg.psi, msg.sigma_d, msg.sigma_u,
            msg.sigma_psi, msg.lateral_scale, msg.sigma_scale, msg.odom_since_fix,
            int(msg.u_known), ' '.join(str(i) for i in msg.tag_ids),
            ' '.join(str(i) for i in acc), float(rel @ self.wall.t) - self.marker_u,
            float(rel @ self.wall.n), psi,
            ' '.join(f'{i}:{q.x:.5f}:{q.y:.5f}' for i, q in zip(msg.tag_ids, msg.tag_positions))))

    def _arm_cb(self, msg):
        """机械臂控制器的指令（reference）与实际（feedback）关节位置."""
        if self._t_goal is None or len(msg.reference.positions) < len(ARM_JOINTS):
            return
        try:
            idx = [msg.joint_names.index(j) for j in ARM_JOINTS]
        except ValueError:
            return
        station, phase = self._current_phase()
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.arm_trace.append((t, station, phase)
                              + tuple(msg.reference.positions[i] for i in idx)
                              + tuple(msg.feedback.positions[i] for i in idx))

    def _links_cb(self, msg):
        if TOOL_LINK not in msg.name or BASE_LINK not in msg.name:
            return
        b = msg.pose[msg.name.index(BASE_LINK)]
        p = msg.pose[msg.name.index(TOOL_LINK)].position
        yaw = _yaw(b.orientation)
        self._base = (b.position.x, b.position.y, yaw)
        heading = np.array([math.cos(yaw), math.sin(yaw)])
        # 机械臂伸出量（base_link x），用来区分“切割 / 抵墙”与“退刀”阶段
        arm_x = float((np.array([p.x, p.y]) - np.array([b.position.x, b.position.y])) @ heading)
        # 刀尖 = link4 原点沿 base_link x 方向前伸 tool_offset（visual_servo_ros.tool_front_x）
        tip = np.array([p.x, p.y]) + self.robot.tool_offset * heading
        rel = tip - self.wall.p0
        u = float(rel @ self.wall.t)
        depth = float(-(rel @ self.wall.n))      # >0：刀尖进入墙面（刚体墙上应约为 0）
        relb = np.array([b.position.x, b.position.y]) - self.wall.p0
        station, phase = self._current_phase()
        self.trace.append((self.fine_now(), station, u, float(p.z), depth, arm_x, phase,
                           float(relb @ self.wall.t), float(relb @ self.wall.n), yaw))

    def relabel(self):
        """运行结束后按事件时间重新标注刀尖轨迹的 (停靠站, 阶段)."""
        with self._lock:
            events = list(self._events)
        labels = label_by_time([r[0] for r in self.trace], events)
        self.trace = [(r[0], st, *r[2:6], ph, *r[7:]) for r, (st, ph) in zip(self.trace, labels)]

    # ---------- 执行 ----------
    def execute(self, goal):
        """发送规划目标并等待结果；返回 (成功, 结果消息)."""
        log = self.node.get_logger()
        while not self.client.wait_for_server(timeout_sec=10.0):
            log.info('等待行为树执行器 /execute_slot_plan ...')
        last = [None]

        def on_feedback(msg):
            fb = msg.feedback
            key = (fb.station, fb.phase)
            if key != last[0]:
                last[0] = key
                log.info(f'S{fb.station}/{fb.station_count} {fb.phase}'
                         + (f'（已恢复 {fb.recoveries} 次）' if fb.recoveries else ''))

        self._t_goal = self.now()
        handle = _wait(self.client.send_goal_async(goal, feedback_callback=on_feedback))
        if handle is None or not handle.accepted:
            log.error('目标被拒绝')
            return False, None
        res = _wait(handle.get_result_async())
        if res is None:
            return False, None
        return res.status == GoalStatus.STATUS_SUCCEEDED and res.result.success, res.result


def _wait(future, timeout=None):
    t0 = time.time()
    while rclpy.ok() and not future.done():
        if timeout is not None and time.time() - t0 > timeout:
            return None
        time.sleep(0.05)
    return future.result() if future.done() else None


def cutting_samples(trace, robot=None, depth=None):
    """切割阶段（相邻两个切割点之间的运动）的样本."""
    return [r for r in trace if r[1] >= 0 and r[6] == 'cut']


def cutting_runs(trace):
    """切割阶段的样本按连续的切割段分组 → [[(u, v), ...], ...]（每组是一段刀尖折线）."""
    runs, prev = [], None
    for k, r in enumerate(trace):
        if r[1] >= 0 and r[6] == 'cut':
            if prev != k - 1:
                runs.append([])
            runs[-1].append((r[2], r[3]))
            prev = k
    return runs


def path_coverage(runs, pts, tol=0.003):
    """
    覆盖率（与采样率无关）：槽线取样点到“相邻刀尖样本连成的折线”的距离在 tol 内即算已切.

    刀尖真值 20 Hz 采样，进给较快时相邻样本的间距超过 tol，逐点最近距离会把样本之间的槽线
    误判为未切；刀尖在两个样本之间走直线（单轴直线切割），所以用折线距离。
    """
    lines = [r for r in runs if len(r) >= 2]
    if not lines or not pts:
        return 0.0
    covered = sum(min(_dist_poly(q, line) for line in lines) < tol for q in pts)
    return float(covered) / len(pts)


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
    res['coverage_path'] = path_coverage(cutting_runs(trace), pts)
    return res


def _dist_poly(p, pts):
    p = np.asarray(p)
    best = math.inf
    for a, b in zip(pts[:-1], pts[1:]):
        a, b = np.asarray(a), np.asarray(b)
        t = np.clip(np.dot(p - a, b - a) / max(np.dot(b - a, b - a), 1e-18), 0, 1)
        best = min(best, float(np.linalg.norm(p - (a + t * (b - a)))))
    return best


def segment_labels(ekf_trace):
    """
    每个估计样本所属的停靠站：横移阶段（station = −1）记入之后到达的那一站.

    技能服务器在横移开始时把停靠站置为 −1，到站后才给出站号，因此按时间倒序传递。
    精对位（含按行程触发的重新对位）同样记入之后的那一站。
    """
    labels, nxt = [], -1
    for e in reversed(ekf_trace):
        if e[1] >= 0:
            nxt = e[1]
        labels.append(e[1] if e[1] >= 0 else nxt)
    return labels[::-1]


def record_to_row(r):
    row = {k: getattr(r, k) for k in RECORD_FIELDS}
    for k in ('nav_leg', 'moved', 'cut_ok', 'realigned'):
        row[k] = int(row[k])
    return row


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    argv = rclpy.utilities.remove_ros_args(sys.argv)
    args = [a for a in argv[1:] if not a.startswith('--')]
    flags = dict((a[2:].partition('=')[0], a[2:].partition('=')[2] or True)
                 for a in argv[1:] if a.startswith('--'))
    gap_arg = flags.get('dry-run-gap')
    if gap_arg is True:          # 只写 --dry-run-gap 不带数值：取主任务的默认间隙
        gap_arg = '0.01'
    dry_gap = float(gap_arg) if gap_arg is not None else None
    plan_path = os.path.abspath(args[0])
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
    marker_x = float(flags.get('marker-x', 8.42))
    marker_y = float(flags.get('marker-y', -3.9245))
    marker_u = float((np.array([marker_x, marker_y]) - wall.p0) @ wall.t)
    ex = SlotRecorder(wall, plan_dict, marker_u)
    log = ex.node.get_logger()
    while ex.now() <= 0.0:          # 等仿真时钟
        time.sleep(0.1)
    ex.marker_pub.publish(build_markers(plan, scene, wall, robot, ex.node.get_clock().now()
                                        .to_msg()))
    goal = ExecuteSlotPlan.Goal()
    goal.plan_file = plan_path
    goal.dry_run = dry_gap is not None
    goal.dry_run_gap = dry_gap or 0.0
    goal.compensate = 'no-compensation' not in flags
    goal.max_strafe = float(flags.get('max-strafe', 0.0))
    goal.behavior_tree = flags['bt'] if isinstance(flags.get('bt'), str) else ''
    realign_arg = flags.get('realign-distance', 0.0)
    if realign_arg is True:      # 只写 --realign-distance 不带数值：视为未设置
        realign_arg = 0.0
    goal.realign_distance = float(realign_arg)
    t_start = time.time()
    sim_start = ex.now()
    ok, result = ex.execute(goal)
    sim_total = ex.now() - sim_start
    time.sleep(0.5)                  # 最后一站的真值采样
    ex.relabel()
    rows = [record_to_row(r) for r in result.stations] if result is not None else []
    if not rows:
        log.error('没有执行任何停靠站' + (f'：{result.error}' if result is not None else ''))
        shutdown(ex)
        sys.exit(3)
    segments = segment_labels(ex.ekf_trace)
    for r in rows:
        r.update(ex.truth.get(r['station'], {}))
        # 每站底盘漂移（真值，到站之后相对到站时刻的最大位移）
        bs = [(t[7], t[8]) for t in ex.trace if t[1] == r['station']]
        r['base_drift_mm'] = (max(math.hypot(a - bs[0][0], d - bs[0][1]) for a, d in bs)
                              * 1000) if bs else float('nan')
        # 本站（到站横移 + 残差测量）中接受过更新的标记
        es = [e for e, s in zip(ex.ekf_trace, segments) if s == r['station']]
        ids = sorted({int(i) for e in es for i in e[14].split()})
        r['tags_used'] = ' '.join(str(i) for i in ids)
        r['tag_updates'] = sum(1 for e in es if e[14])

    summary = evaluate(ex.trace, plan, robot, plan.slot_depth)
    summary.update(plan_partition=plan.partition, plan_order=plan.order,
                   predicted_t_total=plan.prediction.get('t_total'),
                   stations_total_s=float(sum(r['station_s'] for r in rows)),
                   dry_run_gap=dry_gap, compensation=goal.compensate,
                   stations=len(plan.stations), stations_done=len(rows), all_ok=ok,
                   sim_time_s=sim_total, wall_time_s=time.time() - t_start,
                   slot_length_m=plan.length, executor='behavior_tree',
                   recoveries=int(sum(r['recoveries'] for r in rows)),
                   error=(result.error if result is not None else 'no result') or None)
    for k in ('lateral_mm', 'standoff_mm', 'yaw_deg', 'base_drift_mm', 'sensed_yaw_deg',
              'move_s'):
        vals = [abs(r[k]) for r in rows if k in r]
        if vals:
            summary[f'station_{k}_mean_abs'] = float(np.mean(vals))
            summary[f'station_{k}_max_abs'] = float(np.max(vals))
    summary['stations_converged'] = int(sum(r['moved'] for r in rows))
    summary['realignments'] = int(sum(r['realigned'] for r in rows))
    if ex.ekf_trace:
        last = ex.ekf_trace[-1]
        summary['final_lateral_scale'] = float(last[9])
        summary['final_sigma_scale'] = float(last[10])
    fields = list(rows[0].keys())
    for r in rows:
        fields += [k for k in r if k not in fields]
    with open(os.path.join(out_dir, 'stations.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(out_dir, 'tool_trace.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'station', 'u', 'v', 'depth', 'arm_x', 'phase', 'base_u', 'base_d',
                    'base_yaw'])
        w.writerows(ex.trace)
    if ex.wrench_trace:
        with open(os.path.join(out_dir, 'wrench.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'base_x', 'base_y', 'base_yaw', 'fx', 'fy', 'mz',
                        'expected_fx', 'expected_fy', 'expected_mz'])
            w.writerows([f'{v:.5f}' for v in r] for r in ex.wrench_trace)
    with open(os.path.join(out_dir, 'ekf_trace.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(EKF_FIELDS + ('segment',))
        w.writerows(e + (s,) for e, s in zip(ex.ekf_trace, segments))
    with open(os.path.join(out_dir, 'arm_state.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(ARM_FIELDS)
        w.writerows([f'{r[0]:.4f}', r[1], r[2]] + [f'{v:.6f}' for v in r[3:]]
                    for r in ex.arm_trace)
    with open(os.path.join(out_dir, 'summary.yaml'), 'w', encoding='utf-8') as f:
        yaml.safe_dump({k: (round(float(v), 4) if isinstance(v, (float, np.floating)) else
                            int(v) if isinstance(v, (np.integer, np.bool_)) else v)
                        for k, v in summary.items()}, f, allow_unicode=True, sort_keys=False)
    executed = [(r[2], r[3]) for r in cutting_samples(ex.trace)]
    ex.marker_pub.publish(build_markers(plan, scene, wall, robot,
                                        ex.node.get_clock().now().to_msg(),
                                        executed=executed[::5]))
    log.info('\n' + '\n'.join(f'  {k}: {v}' for k, v in summary.items()))
    shutdown(ex)
    sys.exit(0 if ok else 1)


def shutdown(ex):
    """按顺序停掉后台线程并销毁节点（否则退出时 C++ 层会 abort）."""
    ex.shutdown()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
