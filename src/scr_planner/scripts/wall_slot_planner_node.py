#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""墙面开槽规划节点.

读取墙面任务 YAML（墙在 map 中的位置、墙体类型、已有物体、起终线盒）→ 按开槽规范
规划正交槽线 → 弯管圆弧 → 按机械臂工作空间划分底盘停靠站 → 生成每站的机械臂切割点。
结果写成 YAML（每站 points 与 navigation_cutting_task 的格式相同），并在
/wall_slot_plan/markers 上发布 RViz 可视化（transient local，后连的 RViz 也能看到）。

  ros2 run scr_planner wall_slot_planner_node.py --ros-args \
      -p task_file:=$(ros2 pkg prefix scr_planner)/share/scr_planner/config/wall_task_room3_south.yaml \
      -p output_file:=/tmp/slot_plan.yaml
"""
import os
import sys

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import MarkerArray

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'python'))
try:
    from slot_ros import package_python_dir
except ImportError:  # 安装树：python/ 在 share/scr_planner 下
    from ament_index_python.packages import get_package_share_directory
    sys.path.insert(0, os.path.join(get_package_share_directory('scr_planner'), 'python'))
    from slot_ros import package_python_dir
sys.path.insert(0, package_python_dir())

import slot_plan as sp  # noqa: E402
from scr_planner_py import OrientedAStar  # noqa: E402
from slot_ros import build_markers  # noqa: E402


def load_and_plan(task_file, planner=None):
    with open(task_file, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    scene, wall = sp.scene_from_config(cfg)
    robot = sp.RobotModel()
    opt = cfg.get('options', {})
    plan = sp.plan_task(scene, wall, robot, planner or OrientedAStar(),
                        restrict_to_reach=opt.get('restrict_to_reach', True),
                        retract_between_turns=opt.get('retract_between_turns', True))
    return cfg, scene, wall, robot, plan


class WallSlotPlanner(Node):
    def __init__(self):
        super().__init__('wall_slot_planner')
        self.declare_parameter('task_file', '')
        self.declare_parameter('output_file', '')
        task_file = self.get_parameter('task_file').value
        if not task_file:
            raise SystemExit('参数 task_file 未设置')
        cfg, scene, wall, robot, plan = load_and_plan(task_file)
        self.log_plan(plan, robot)
        out = self.get_parameter('output_file').value
        if out:
            d = sp.plan_to_dict(plan, wall, robot, cfg.get('options', {}).get('duration', 2.0))
            with open(out, 'w', encoding='utf-8') as f:
                yaml.safe_dump(d, f, allow_unicode=True, sort_keys=False)
            self.get_logger().info(f'规划结果已写入 {out}')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, '/wall_slot_plan/markers', qos)
        self.pub.publish(build_markers(plan, scene, wall, robot, self.get_clock().now().to_msg()))

    def log_plan(self, plan, robot):
        log = self.get_logger()
        if plan.status != 'success':
            log.error(f'规划失败: {plan.status}（可能需要设计同意、改走吊顶 / 地面，或调整线盒位置）')
            return
        log.info(f'槽线: {len(plan.corners_m) - 2} 个弯, 长 {plan.length:.3f} m, '
                 f'槽深 {plan.slot_depth * 1000:.0f} mm, 槽宽 {plan.slot_width * 1000:.0f} mm, '
                 f'规划 {plan.planner["time_ms"]:.1f} ms')
        if plan.unreachable:
            log.warn(f'机械臂够不到的部分 {plan.unreachable_length:.3f} m '
                     f'（可达高度 {robot.reach_band[0]:.2f}–{robot.reach_band[1]:.2f} m）')
        log.info(f'停靠站 {len(plan.stations)} 个（每站可切宽度 {robot.window_width * 100:.0f} cm）')
        for st in plan.stations:
            log.info(f'  S{st.index}: u={st.u_c:.3f} m 位姿=({st.pose[0]:.3f}, {st.pose[1]:.3f}, '
                     f'{st.pose[2]:.3f}) 切割 {st.cut_length * 100:.1f} cm, {len(st.points)} 个点')


def main():
    rclpy.init()
    node = WallSlotPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
