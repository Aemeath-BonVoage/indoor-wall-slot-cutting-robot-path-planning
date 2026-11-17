# -*- coding: utf-8 -*-
"""
RViz 可视化节点：墙面任务 + 规划结果 → MarkerArray，并实时画出实际刀尖轨迹.

参数
  task_file / plan_file   墙面任务 YAML / 规划结果 YAML（至少给一个）
  ascii_labels            RViz 文字只用 ASCII（中文标签换成种类 + 序号），默认 true
  show_footprints         画每站底盘轮廓，默认 true
  show_planned_tip        画计划刀尖轨迹（由每站机械臂点换算），默认 true
  republish_period        静态 Marker 重发周期（s），默认 2.0；文件变化时自动重新加载
  tool_tip_topic          可选刀尖话题（geometry_msgs/PointStamped，map 坐标），默认 /scr_viz/tool_tip
  use_link_states         同时从 /gazebo/link_states 推算刀尖（link4 + tool_offset），默认 true
  base_link_name / tool_link_name   link_states 中的名字
  trace_min_step / trace_max_points / contact_threshold   轨迹抽稀、限长、贴墙判定

话题
  ~/markers     visualization_msgs/MarkerArray（transient local）：静态场景
  ~/tool_trace  visualization_msgs/MarkerArray：实际刀尖轨迹（红 = 贴墙切割，青 = 移动）
服务
  ~/clear_trace std_srvs/Empty：清空轨迹
"""
import math
import os

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Empty
from visualization_msgs.msg import MarkerArray

from scr_viz import markers as mk
from scr_viz import wall_model as wm
from scr_viz.ros_adapter import to_marker_array

try:  # gazebo_msgs 只在需要 link_states 时用到
    from gazebo_msgs.msg import LinkStates
except ImportError:  # pragma: no cover
    LinkStates = None


class PlanVizNode(Node):
    """读墙面任务 / 规划结果并发布 RViz Marker."""

    def __init__(self):
        super().__init__('scr_plan_viz')
        p = self.declare_parameter
        self.task_file = p('task_file', '').value
        self.plan_file = p('plan_file', '').value
        self.ascii_labels = p('ascii_labels', True).value
        self.show_footprints = p('show_footprints', True).value
        self.show_planned_tip = p('show_planned_tip', True).value
        period = float(p('republish_period', 2.0).value)
        tip_topic = p('tool_tip_topic', '/scr_viz/tool_tip').value
        use_links = p('use_link_states', True).value
        self.base_link = p('base_link_name', 'scr_robot_gazebo::base_link').value
        self.tool_link = p('tool_link_name', 'scr_robot_gazebo::link4').value
        self.trace = mk.TipTrace(None, float(p('trace_min_step', 0.002).value),
                                 int(p('trace_max_points', 20000).value),
                                 float(p('contact_threshold', 0.02).value))

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.scene_pub = self.create_publisher(MarkerArray, '~/markers', latched)
        self.trace_pub = self.create_publisher(MarkerArray, '~/tool_trace', 10)
        self.create_service(Empty, '~/clear_trace', self._clear_cb)
        if tip_topic:
            self.create_subscription(PointStamped, tip_topic, self._tip_cb, 50)
        if use_links and LinkStates is not None:
            self.create_subscription(LinkStates, '/gazebo/link_states', self._links_cb, 10)

        self.scene = None
        self._mtimes = None
        self._trace_dirty = False
        self._load(force=True)
        self.create_timer(max(0.2, period), self._republish)
        self.create_timer(0.2, self._publish_trace)

    # --- 场景 ------------------------------------------------------------------
    def _file_mtimes(self):
        return tuple(os.path.getmtime(f) if f and os.path.exists(f) else None
                     for f in (self.task_file, self.plan_file))

    def _load(self, force=False):
        mt = self._file_mtimes()
        if not force and mt == self._mtimes:
            return False
        self._mtimes = mt
        if not (self.task_file or self.plan_file):
            self.get_logger().error('需要参数 task_file 或 plan_file')
            return False
        try:
            self.scene = wm.load_scene(self.task_file or None, self.plan_file or None)
        except Exception as e:  # noqa: BLE001 — 文件写到一半等情况，下个周期再试
            self.get_logger().error(f'读取失败：{e}')
            self._mtimes = None
            return False
        self.trace.wall = self.scene.wall
        s = self.scene
        self.get_logger().info(
            f'已加载：墙长 {s.wall.length:.2f} m，障碍 {len(s.obstacles)}，线盒 '
            f'{len(s.endpoints)}，槽 {len(s.slots)}，停靠站 {len(s.stations)}')
        return True

    def _republish(self):
        self._load()
        if self.scene is None:
            return
        specs = mk.scene_markers(self.scene, self.ascii_labels, self.show_footprints,
                                 self.show_planned_tip)
        self.scene_pub.publish(to_marker_array(specs, self.scene.wall.frame_id,
                                               self.get_clock().now().to_msg()))

    # --- 刀尖轨迹 ----------------------------------------------------------------
    def _add_tip(self, p):
        if self.trace.add(p):
            self._trace_dirty = True

    def _tip_cb(self, msg):
        self._add_tip((msg.point.x, msg.point.y, msg.point.z))

    def _links_cb(self, msg):
        try:
            b = msg.pose[msg.name.index(self.base_link)]
            t = msg.pose[msg.name.index(self.tool_link)].position
        except ValueError:
            return
        q = b.orientation
        yaw = wm.yaw_from_quaternion(q.x, q.y, q.z, q.w)
        tip = wm.tip_from_link_poses((b.position.x, b.position.y), yaw, (t.x, t.y, t.z),
                                     self.scene.tool_offset if self.scene else wm.TOOL_OFFSET)
        if all(math.isfinite(c) for c in tip):
            self._add_tip(tip)

    def _publish_trace(self):
        if not self._trace_dirty:
            return
        self._trace_dirty = False
        frame = self.scene.wall.frame_id if self.scene else 'map'
        self.trace_pub.publish(to_marker_array(self.trace.markers(), frame,
                                               self.get_clock().now().to_msg(), False))

    def _clear_cb(self, req, resp):
        self.trace.clear()
        frame = self.scene.wall.frame_id if self.scene else 'map'
        self.trace_pub.publish(to_marker_array([], frame, self.get_clock().now().to_msg()))
        return resp


def main(args=None):
    """节点入口."""
    rclpy.init(args=args)
    node = PlanVizNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
