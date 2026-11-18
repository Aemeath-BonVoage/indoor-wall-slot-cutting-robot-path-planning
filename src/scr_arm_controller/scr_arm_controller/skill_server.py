#!/usr/bin/env python3
"""
切割任务技能的 ROS 2 action 服务器（由 scr_task 的行为树调用）.

  /task_skills/fine_align       scr_interfaces/action/FineAlign
  /task_skills/move_to_station  scr_interfaces/action/MoveToStation
  /task_skills/cut_station      scr_interfaces/action/CutStation
  /task_skills/hold_base        scr_interfaces/action/HoldBase
  /task_skills/scan_marker      scr_interfaces/action/ScanMarker
  /task_skills/retract_arm      scr_interfaces/action/RetractArm
  /task_skills/status           scr_interfaces/msg/TaskStatus（阶段变化时发布）

各技能共享同一个墙面相对状态估计（EKF）与定点保持：精对位确定的墙面原点由之后的横移
沿用，横移收敛时开始的定点保持一直持续到切割结束。因此技能放在同一进程内，而不是拆成
多个节点。运动类技能（除 HoldBase 外）互斥：同一时刻只接受一个，其余目标被拒绝。

  ros2 run scr_arm_controller task_skill_server [--ros-args --params-file fine_alignment.yaml]
"""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from scr_arm_controller.slot_execution import compensate_points, residual_is_compensable
from scr_arm_controller.task_skills import CuttingSkills
from scr_interfaces.action import (CutStation, FineAlign, HoldBase, MoveToStation,
                                   RetractArm, ScanMarker)
from scr_interfaces.msg import TaskStatus


class SkillServer:
    """把 CuttingSkills 的技能包装成 action 服务器."""

    def __init__(self):
        sk = CuttingSkills(node_name='task_skills')
        sk.executor_driven = True
        self.skills = sk
        self.log = sk.get_logger()
        self._motion_lock = threading.Lock()
        self._motion_goal = None
        sk.abort_requested = self._motion_cancel_requested
        sk.on_phase = self._publish_status

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = sk.create_publisher(TaskStatus, '/task_skills/status', qos)
        cb = ReentrantCallbackGroup()

        def server(action, name, execute, motion=True):
            return ActionServer(
                sk, action, '/task_skills/' + name, execute_callback=execute,
                goal_callback=self._accept_motion if motion else (lambda _: GoalResponse.ACCEPT),
                cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=cb)

        self._servers = [
            server(FineAlign, 'fine_align', self._fine_align),
            server(MoveToStation, 'move_to_station', self._move_to_station),
            server(CutStation, 'cut_station', self._cut_station),
            server(ScanMarker, 'scan_marker', self._scan_marker),
            server(RetractArm, 'retract_arm', self._retract_arm),
            server(HoldBase, 'hold_base', self._hold_base, motion=False),
        ]
        self._publish_status(-1, 'idle')
        self.log.info('技能服务器就绪：fine_align / move_to_station / cut_station / hold_base / '
                      'scan_marker / retract_arm')

    # ---------- 公共 ----------
    def _accept_motion(self, _goal):
        if self._motion_lock.locked():
            self.log.error('已有运动类技能在执行，拒绝新目标')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _motion_cancel_requested(self):
        g = self._motion_goal
        return g is not None and g.is_cancel_requested

    def _publish_status(self, station, phase, detail=''):
        msg = TaskStatus()
        msg.stamp = self.skills.get_clock().now().to_msg()
        msg.station, msg.phase, msg.detail = int(station), phase, detail
        self.status_pub.publish(msg)

    def _run_motion(self, goal_handle, body, result):
        """执行一个运动类技能：互斥、取消、异常都在这里统一处理."""
        with self._motion_lock:
            self._motion_goal = goal_handle
            t0 = self.skills.sim_now()
            try:
                ok = body(goal_handle.request, result)
            except Exception as e:  # noqa: BLE001  技能异常按失败上报，不让服务器退出
                self.log.error(f'技能执行异常: {e!r}')
                result.message = repr(e)
                ok = False
            finally:
                self._motion_goal = None
            if hasattr(result, 'duration'):
                result.duration = self.skills.sim_now() - t0
            result.success = bool(ok)
            if goal_handle.is_cancel_requested:
                self.skills.stop_robot()
                goal_handle.canceled()
            elif ok:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result

    # ---------- 技能 ----------
    def _fine_align(self, goal_handle):
        sk = self.skills

        def feedback(X, sig, info):
            fb = FineAlign.Feedback()
            fb.normal_error = float(X[0] - sk.distance_from_marker)
            fb.lateral_error, fb.yaw_error = float(X[1]), float(X[2])
            fb.sigma_lateral = float(sig[1])
            fb.tag_state = str(info.get('tag'))
            goal_handle.publish_feedback(fb)

        def body(g, res):
            if g.standoff > 0.0:
                sk.distance_from_marker = g.standoff
            if g.mode:
                sk.positioning_mode = g.mode
            sk.set_phase('align', station=-1)
            sk.servo_feedback = feedback
            try:
                sk.sleep_sim(max(g.settle_time, 0.0))
                ok = sk.precise_positioning_to_marker()
            finally:
                sk.servo_feedback = None
            res.normal_error, res.lateral_error, res.yaw_error = sk.last_align_errors
            if ok:
                res.origin_residual = sk.set_wall_origin()
                res.message = 'converged'
            else:
                res.message = 'fine alignment did not converge'
            sk.set_phase('hold' if ok else 'idle')
            return ok

        return self._run_motion(goal_handle, body, FineAlign.Result())

    def _move_to_station(self, goal_handle):
        sk = self.skills

        def feedback(X, _sig, _info):
            fb = MoveToStation.Feedback()
            g = goal_handle.request
            fb.lateral_error = float(X[1] - g.lateral_offset)
            fb.standoff_error = float(X[0] - sk.distance_from_marker)
            fb.yaw_error = float(X[2])
            goal_handle.publish_feedback(fb)

        def body(g, res):
            if g.standoff > 0.0:
                sk.distance_from_marker = g.standoff
            sk.set_phase('move', station=-1)
            sk.observe_before_strafe()
            ok = sk.strafe_to(g.lateral_offset, max_lin=g.max_speed or 0.1,
                              timeout=g.timeout or 60.0, ready_cb=feedback)
            if sk.abort_requested():
                return False
            # 到站（或未收敛）后立即定点保持，残差由 CutStation 测量并补偿
            sk.base_hold.enabled = sk.hold_base
            sk.base_hold.hold()
            X = sk.estimator.state()
            if X is None:
                res.message = 'no wall estimate'
                return False
            res.converged = bool(ok)
            res.lateral_error = float(X[1] - g.lateral_offset)
            res.standoff_error = float(X[0] - sk.distance_from_marker)
            res.yaw_error = float(X[2])
            res.odom_since_fix = float(sk.estimator.odom_since_fix)
            accept = ok or residual_is_compensable(res.lateral_error, res.standoff_error,
                                                   res.yaw_error)
            res.message = ('converged' if ok else
                           'not converged, residual compensable' if accept else
                           'not converged, residual too large')
            if not ok:
                sk.get_logger().warn(
                    f'横移未按判据收敛：残差 横向 {res.lateral_error * 1000:+.1f} mm, 离墙 '
                    f'{res.standoff_error * 1000:+.1f} mm, 朝向 '
                    f'{math.degrees(res.yaw_error):+.2f}° → {res.message}')
            sk.set_phase('arrived', station=g.station)
            return accept

        return self._run_motion(goal_handle, body, MoveToStation.Result())

    def _cut_station(self, goal_handle):
        sk = self.skills

        def body(g, res):
            pts = [(p.x, p.y, p.z, p.rotation) for p in g.points]
            is_cut = [bool(p.at_depth) for p in g.points]
            if len(pts) < 1:
                res.message = 'no points'
                return False
            if g.standoff > 0.0:
                sk.distance_from_marker = g.standoff
            if g.station >= 0:
                sk.cur_station = g.station
            if g.compensate:
                e_l, e_n, psi = sk.measure_residual(g.lateral_offset)
                lim = sk.arm_controller.cartesian_limits
                pts, res.clamped = compensate_points(pts, is_cut, e_l, e_n, psi, g.tool_offset,
                                                     lim['x'], lim['y'])
                res.sensed_lateral, res.sensed_standoff, res.sensed_yaw = e_l, e_n, psi
                sk.get_logger().info(
                    f'S{g.station} 残余误差（传感器）: 横向 {e_l * 1000:+.1f} mm, 离墙 '
                    f'{e_n * 1000:+.1f} mm, 朝向 {math.degrees(psi):+.2f}°，已补偿'
                    + (f'；{res.clamped} 个点被关节限位钳制' if res.clamped else ''))
            if g.return_home or g.wall_gap > 0.0:
                # 单站任务：可达性检查 → 回缩回位姿 → 逐点 → 回缩回位姿（空切时按实测墙面后移）
                sk.dry_run_gap = g.wall_gap if g.wall_gap > 0.0 else None
                sk.set_phase('cut')
                ok = sk.execute_cutting_points(pts, g.point_duration, g.return_home)
                sk.set_phase('idle')
                if ok and g.station < 0:
                    sk._visualize_cutting_result()
            else:
                ok = sk.run_points(pts, is_cut, g.point_duration)
            sk.set_phase('idle', station=-1)
            res.message = 'done' if ok else 'arm motion failed'
            return ok

        return self._run_motion(goal_handle, body, CutStation.Result())

    def _scan_marker(self, goal_handle):
        sk = self.skills

        def body(g, res):
            sk.set_phase('scan')
            p = sk.scan_for_marker(samples=g.samples or 5)
            sk.set_phase('idle')
            if p is None:
                return False
            res.position = Point(x=p[0], y=p[1], z=p[2])
            return True

        return self._run_motion(goal_handle, body, ScanMarker.Result())

    def _retract_arm(self, goal_handle):
        sk = self.skills

        def body(g, _res):
            sk.set_phase('arm')
            sk.move_arm_for_approach(joint1=g.lift)
            sk.set_phase('idle')
            return True

        return self._run_motion(goal_handle, body, RetractArm.Result())

    def _hold_base(self, goal_handle):
        """保持到目标被取消（行为树中与切割并行，切割结束时由 Parallel 节点停止）."""
        sk, hold = self.skills, self.skills.base_hold
        if not hold.active:
            hold.enabled = sk.hold_base
            hold.hold()
        gen = hold.generation
        res, max_err = HoldBase.Result(), 0.0
        while rclpy.ok() and not goal_handle.is_cancel_requested and hold.generation == gen:
            err = hold.error()
            if err is not None:
                max_err = max(max_err, err[0])
                fb = HoldBase.Feedback()
                fb.position_error, fb.yaw_error = float(err[0]), float(err[1])
                goal_handle.publish_feedback(fb)
            time.sleep(0.05)
        # 只释放本目标开始时的那次保持（之后的技能可能已经开始新的保持）
        if hold.generation == gen:
            drift = hold.release()
            if drift is not None:
                self.log.info(f'定点保持期间底盘真值最大漂移: {drift[0] * 1000:.2f} mm, '
                              f'{drift[1]:.3f}°')
        res.success, res.max_drift = True, max_err
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.succeed()
        return res

    def destroy(self):
        sk = self.skills
        sk.trajectory_visualizer.destroy_node()
        sk.arm_controller.destroy_node()
        sk.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    server = SkillServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server.skills)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        server.destroy()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
