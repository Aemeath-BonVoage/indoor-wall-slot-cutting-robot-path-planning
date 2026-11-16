"""底盘定点保持 + 漂移测量.

机械臂运动的反作用力会推动底盘：仿真中轮子摩擦为 0、底盘由 planar_move 驱动，
实测机械臂动作时一站内可漂移约 10 cm。切割 / 扫描期间需要把底盘闭环保持在到位位姿。

机械臂控制器在执行动作时会阻塞主节点的回调，因此本模块使用独立节点和独立线程运行：
- 以 20 Hz 用里程计做 P 控制，把底盘保持在 hold() 时的位姿；
- 同时订阅 Gazebo 真值（/gazebo/model_states），记录保持期间底盘偏离起点的最大距离与角度。
"""
import math
import threading

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class BaseHold:
    def __init__(self, entity='scr_robot_gazebo', rate_hz=20.0,
                 k_lin=1.0, k_ang=1.5, max_lin=0.05, max_ang=0.2):
        sim_time = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        self.node = rclpy.create_node('base_hold', parameter_overrides=[sim_time])
        self.entity = entity
        self.k_lin, self.k_ang, self.max_lin, self.max_ang = k_lin, k_ang, max_lin, max_ang
        self.enabled = True             # False 时只测量漂移、不发控制（用于对比实验）
        self._odom = None
        self._gt = None
        self._target = None             # 里程计系中的保持目标 (x, y, yaw)
        self._gt_ref = None             # 真值起点
        self._max_drift = 0.0           # 米
        self._max_yaw = 0.0             # 弧度
        self._lock = threading.Lock()

        self.node.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.node.create_subscription(ModelStates, '/gazebo/model_states', self._on_gt, 10)
        self._pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.node.create_timer(1.0 / rate_hz, self._step)
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self.node)
        self._thread = threading.Thread(target=self._exec.spin, daemon=True)
        self._thread.start()

    # ---------- 回调 ----------
    def _on_odom(self, msg):
        p = msg.pose.pose
        self._odom = (p.position.x, p.position.y, _yaw(p.orientation))

    def _on_gt(self, msg):
        try:
            p = msg.pose[msg.name.index(self.entity)]
        except ValueError:
            return
        self._gt = (p.position.x, p.position.y, _yaw(p.orientation))
        with self._lock:
            if self._gt_ref is not None:
                gx, gy, gyaw = self._gt_ref
                d = math.hypot(self._gt[0] - gx, self._gt[1] - gy)
                self._max_drift = max(self._max_drift, d)
                dyaw = math.atan2(math.sin(self._gt[2] - gyaw), math.cos(self._gt[2] - gyaw))
                self._max_yaw = max(self._max_yaw, abs(dyaw))

    def _step(self):
        if self._target is None or self._odom is None or not self.enabled:
            return
        x, y, yaw = self._odom
        tx, ty, tyaw = self._target
        ex, ey = tx - x, ty - y
        c, s = math.cos(yaw), math.sin(yaw)
        cmd = Twist()
        cmd.linear.x = max(-self.max_lin, min(self.max_lin, self.k_lin * (c * ex + s * ey)))
        cmd.linear.y = max(-self.max_lin, min(self.max_lin, self.k_lin * (-s * ex + c * ey)))
        eyaw = math.atan2(math.sin(tyaw - yaw), math.cos(tyaw - yaw))
        cmd.angular.z = max(-self.max_ang, min(self.max_ang, self.k_ang * eyaw))
        self._pub.publish(cmd)

    # ---------- 接口 ----------
    def hold(self):
        """从当前位姿开始保持（并开始记录真值漂移）."""
        with self._lock:
            self._target = self._odom
            self._gt_ref = self._gt
            self._max_drift = 0.0
            self._max_yaw = 0.0

    def release(self):
        """停止保持，返回保持期间的真值最大漂移 (米, 度)；无真值时为 None."""
        self._target = None
        self._pub.publish(Twist())
        with self._lock:
            had_ref = self._gt_ref is not None
            self._gt_ref = None
            return (self._max_drift, math.degrees(self._max_yaw)) if had_ref else None

    def shutdown(self):
        """停止后台线程并销毁节点；必须在 rclpy.shutdown() 之前调用，否则进程退出时会 abort."""
        self._target = None
        self._exec.shutdown()
        self._thread.join(timeout=2.0)
        self.node.destroy_node()
