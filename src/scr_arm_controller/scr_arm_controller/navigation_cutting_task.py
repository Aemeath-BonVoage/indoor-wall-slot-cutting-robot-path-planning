#!/usr/bin/env python3
"""
导航+切割任务的命令行入口（保持原用法）.

任务流程由行为树执行（scr_task 包的 task_executor，BehaviorTree.CPP）。本程序判断任务
文件类型，把它作为 action 目标发给行为树执行器，打印执行反馈与每站记录：

  单站任务（task_at_target.yaml 格式）   → /execute_cutting_task
  离线开槽规划（plan_wall_slots.py 输出） → /execute_slot_plan

  ros2 run scr_arm_controller navigation_cutting_task <task.yaml | plan.yaml>
      [--dry-run-gap=0.01] [--no-compensation] [--max-strafe=1.5] [--bt=<tree.xml>]

需要先启动仿真、Nav2 与任务层：ros2 launch scr_bringup sim_nav.launch.py（默认 task:=true）。
精对位参数（alignment.*）属于技能服务器，启动时用 alignment_params:=<yaml> 指定。

为兼容已有脚本，本模块继续导出规划解析的纯函数（见 slot_execution）与
NavigationCuttingTask（即 task_skills.CuttingSkills，对位实验脚本直接调用其中的技能）。
"""

import os
import sys

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient

from scr_arm_controller.slot_execution import (  # noqa: F401  兼容导出
    compensate_points, is_slot_plan, plan_wall_axes, plan_wall_pose, station_arm_points)
from scr_arm_controller.task_skills import ALIGNMENT_PARAMS  # noqa: F401  兼容导出
from scr_arm_controller.task_skills import CuttingSkills as NavigationCuttingTask  # noqa: F401
from scr_interfaces.action import ExecuteCuttingTask, ExecuteSlotPlan


def parse_args(argv):
    """--key[=value] 选项与位置参数（--ros-args 之后的参数留给 rclpy）."""
    argv = rclpy.utilities.remove_ros_args(argv)
    flags = dict((a[2:].partition('=')[0], a[2:].partition('=')[2] or True)
                 for a in argv[1:] if a.startswith('--'))
    pos = [a for a in argv[1:] if not a.startswith('--')]
    return pos, flags


def resolve_task_file(name):
    """相对路径先在当前目录找，再到本包的 config/ 中找."""
    if os.path.isabs(name) or os.path.exists(name):
        return os.path.abspath(name)
    try:
        from ament_index_python.packages import get_package_share_directory
        cand = os.path.join(get_package_share_directory('scr_arm_controller'), 'config', name)
        if os.path.exists(cand):
            return cand
    except Exception:  # noqa: BLE001
        pass
    return os.path.abspath(name)


def make_goal(path, config, flags):
    """任务文件 → (action 类型, action 名, 目标)."""
    if is_slot_plan(config):
        goal = ExecuteSlotPlan.Goal()
        goal.plan_file = path
        gap = flags.get('dry-run-gap')
        goal.dry_run = gap is not None
        goal.dry_run_gap = float(gap) if gap not in (None, True) else 0.01
        goal.compensate = 'no-compensation' not in flags
        goal.max_strafe = float(flags.get('max-strafe', 0.0))
        goal.behavior_tree = flags.get('bt', '') if flags.get('bt') is not True else ''
        return ExecuteSlotPlan, '/execute_slot_plan', goal
    goal = ExecuteCuttingTask.Goal()
    goal.task_file = path
    goal.behavior_tree = flags.get('bt', '') if flags.get('bt') is not True else ''
    return ExecuteCuttingTask, '/execute_cutting_task', goal


def format_record(r):
    return (f'  S{r.station:>2}: {r.station_s:6.1f} s（到站 {r.move_s:5.1f} s，切割 {r.cut_s:5.1f} s）'
            f'  残差 横向 {r.sensed_lateral_mm:+5.1f} mm 朝向 {r.sensed_yaw_deg:+5.2f}°'
            f'  {"收敛" if r.moved else "未收敛"}'
            + (f'  恢复 {r.recoveries} 次' if r.recoveries else ''))


def send_and_wait(node, action, name, goal):
    """发送目标、打印反馈，返回 (状态, 结果)；Ctrl-C 时取消目标."""
    client = ActionClient(node, action, name)
    log = node.get_logger()
    while not client.wait_for_server(timeout_sec=10.0):
        log.info(f'等待行为树执行器 {name} ...')
    last = {'phase': None, 'station': None}

    def on_feedback(msg):
        fb = msg.feedback
        station = getattr(fb, 'station', None)
        if fb.phase != last['phase'] or station != last['station']:
            last.update(phase=fb.phase, station=station)
            where = (f'S{station}/{fb.station_count} ' if station is not None and station >= 0
                     else '')
            log.info(f'{where}{fb.phase}' + (f'（已恢复 {fb.recoveries} 次）'
                                             if fb.recoveries else ''))

    future = client.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, future)
    handle = future.result()
    if not handle.accepted:
        log.error('目标被行为树执行器拒绝')
        return GoalStatus.STATUS_ABORTED, None
    result_future = handle.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, result_future)
    except KeyboardInterrupt:
        log.warn('中断：取消任务')
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel, timeout_sec=5.0)
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)
    res = result_future.result()
    return (res.status, res.result) if res is not None else (GoalStatus.STATUS_ABORTED, None)


def main(args=None):
    pos, flags = parse_args(sys.argv)
    if not pos:
        print(__doc__)
        return
    path = resolve_task_file(pos[0])
    if not os.path.exists(path):
        print(f'错误: 任务文件不存在: {path}')
        sys.exit(2)
    with open(path, encoding='utf-8') as f:
        config = yaml.safe_load(f)
    action, name, goal = make_goal(path, config, flags)

    rclpy.init(args=args)
    node = rclpy.create_node('navigation_cutting_task_client')
    log = node.get_logger()
    log.info(f'任务文件 {path} → {name}')
    status, res = send_and_wait(node, action, name, goal)
    ok = status == GoalStatus.STATUS_SUCCEEDED and res is not None and res.success
    if res is not None:
        for r in getattr(res, 'stations', []):
            log.info(format_record(r))
        done = (f'，{res.stations_done} 站' if hasattr(res, 'stations_done') else '')
        log.info(f'任务{"完成" if ok else "失败"}{done}，用时 {res.total_time:.1f} s'
                 + (f'：{res.error}' if res.error else ''))
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
