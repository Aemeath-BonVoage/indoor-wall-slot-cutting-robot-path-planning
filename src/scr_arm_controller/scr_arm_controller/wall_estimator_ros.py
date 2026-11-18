"""
墙面相对状态估计的 ROS 接口：里程计预测 + 激光 / 定位标记更新（WallEKF），带延迟补偿.

延迟补偿（与 Nav2 opennav_docking 把检测位姿按时间戳经 TF 变换到固定系的做法相同）：
每个测量都带采集时间戳 t_m。保存里程计位姿历史，插值得到 t_m 与当前时刻的底盘位姿，
把测量从 t_m 时刻的 base_link 变换到当前时刻的 base_link 后再做更新。这样滤波器状态
始终对应“最新里程计时刻”，相机 / 激光 / 处理延迟不会在运动中引入滞后偏差。

标记短时丢失（遮挡、出视野）时只做里程计预测与激光更新，σ_u 随运动距离增长；
控制环继续运行，σ_u 超过阈值或丢失时间过长时由调用方停车。
"""
import bisect
import math
from collections import deque

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.time import Time

from scr_arm_controller.wall_ekf import WallEKF


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def relative_pose(a, b):
    """位姿 a → b 的相对变换，表示在 a 的机体系中：(dx, dy, dyaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    ex, ey = b[0] - a[0], b[1] - a[1]
    dyaw = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
    return c * ex + s * ey, -s * ex + c * ey, dyaw


class WallStateEstimator:
    """挂在节点上：订阅里程计（默认 /odom）建立位姿历史，step() 时预测并融合新测量."""

    def __init__(self, node, servo, history=4.0, odom_topic='/odom'):
        self.node = node
        self.servo = servo
        self.ekf = WallEKF()
        self.odom_bias = (0.0, 0.0, 0.0)   # 实验用：与任务节点相同的里程计漂移注入
        self.history = history
        self._hist = deque()               # (t 秒, (x, y, yaw), twist (vx, vy, wz))
        self._last_pose = None
        self._last_wall = None
        self._last_tag = None
        self.use_tag = True                # False：只用里程计 + 激光（多站执行的站间移动）
        self.u_known = False               # 是否已由标记确定过沿墙位置
        self.last_tag_time = None          # 最近一次接受标记更新的时刻（秒）
        self.tag_reject_streak = 0
        self.reacquire_after = 15          # 连续拒收这么多帧标记后重新初始化 u
        node.create_subscription(Odometry, odom_topic, self._on_odom, 50)

    # ---------- 里程计历史 ----------
    def _on_odom(self, msg):
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        p = msg.pose.pose
        bx, by, byaw = self.odom_bias
        pose = (p.position.x + bx, p.position.y + by, _yaw(p.orientation) + byaw)
        tw = msg.twist.twist
        self._hist.append((t, pose, (tw.linear.x, tw.linear.y, tw.angular.z)))
        while self._hist and t - self._hist[0][0] > self.history:
            self._hist.popleft()

    def pose_at(self, t):
        """插值得到 t 时刻的里程计位姿；超出历史范围返回 None."""
        if not self._hist:
            return None
        ts = [h[0] for h in self._hist]
        if t > ts[-1] + 0.05 or t < ts[0]:
            return None
        i = bisect.bisect_left(ts, t)
        if i >= len(ts):
            return self._hist[-1][1]
        if i == 0 or ts[i] == t:
            return self._hist[i][1]
        (t0, a, _), (t1, b, _) = self._hist[i - 1], self._hist[i]
        w = (t - t0) / max(t1 - t0, 1e-9)
        dyaw = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
        return (a[0] + w * (b[0] - a[0]), a[1] + w * (b[1] - a[1]), a[2] + w * dyaw)

    def velocity(self, window=0.5):
        """
        底盘实测速度 (vx, vy, wz)（base_link）：最近 window 秒里程计位移 / 时间.

        不直接用 /odom 的 twist：力控底盘插件的 twist 取自连杆瞬时速度，静止时也有
        ±7 mm/s、±0.3°/s 的抖动（关节 PID 与接触力引起的微振动，位移只有微米级），
        用它判断“已静止”会一直不成立。位移差分相当于 window 内的平均速度。
        """
        if not self._hist:
            return (0.0, 0.0, 0.0)
        t1, p1, _ = self._hist[-1]
        p0 = self.pose_at(t1 - window)
        if p0 is None:
            t0, p0, _ = self._hist[0]
            window = t1 - t0
        if window <= 1e-6:
            return (0.0, 0.0, 0.0)
        dx, dy, dyaw = relative_pose(p0, p1)
        return dx / window, dy / window, dyaw / window

    def now(self):
        return self._hist[-1][0] if self._hist else None

    # ---------- 控制 ----------
    def reset(self):
        """开始新一次对位：清空滤波器（标记位置重新由测量确定）."""
        self.ekf.x = None
        self.ekf.P = None
        self.ekf.stats = {'wall': [0, 0], 'tag': [0, 0]}
        self._last_pose = None
        self._last_wall = None
        self._last_tag = None
        self.u_known = False
        self.last_tag_time = None
        self.tag_reject_streak = 0

    def set_offset(self, d=0.0, u=0.0, psi=0.0):
        """零位标定偏置（传感器指示值 − 真值，见 WallEKF）."""
        self.ekf.offset = np.array([d, u, psi], dtype=float)

    def rebase_u(self, u=0.0):
        """把当前位置的沿墙坐标设为 u（第一版语义：以停车位姿为参照原点）."""
        if self.ekf.initialized:
            self.ekf.x[1] = u
            self.ekf.P[1, :] = 0.0
            self.ekf.P[:, 1] = 0.0
            self.ekf.P[1, 1] = 1e-8
            self.u_known = True

    def state(self):
        return None if not self.ekf.initialized else self.ekf.x.copy()

    # ---------- 主循环一步 ----------
    def _to_now(self, stamp, now_pose):
        """t_m 时刻 base_link → 当前 base_link 的变换 (R 2×2, t 2)，或 None."""
        t_m = stamp.nanoseconds * 1e-9
        pm = self.pose_at(t_m)
        if pm is None:
            return None
        dx, dy, dyaw = relative_pose(now_pose, pm)     # t_m 位姿在当前机体系中
        c, s = math.cos(dyaw), math.sin(dyaw)
        return np.array([[c, -s], [s, c]]), np.array([dx, dy])

    def step(self):
        """预测 + 融合本周期的新测量. 返回 (状态 or None, 状态说明 dict)."""
        info = {'wall': None, 'tag': None}
        if not self._hist:
            return None, info
        now_pose = self._hist[-1][1]
        ekf = self.ekf

        if ekf.initialized and self._last_pose is not None:
            ekf.predict(*relative_pose(self._last_pose, now_pose))
        self._last_pose = now_pose

        # 激光墙面
        wall, why = self.servo.wall_in_base()
        if wall is not None and (self._last_wall is None or wall['stamp'] > self._last_wall):
            self._last_wall = wall['stamp']
            T = self._to_now(wall['stamp'], now_pose)
            if T is not None:
                R, t = T
                n = R @ np.array(wall['normal'])
                dist = wall['distance'] + float(n @ t)
                phi = math.atan2(n[1], n[0])
                if not ekf.initialized:
                    d, u, psi, _ = WallEKF.init_from((dist, phi), offset=ekf.offset)
                    ekf.reset(d, u, psi)
                    info['wall'] = 'init'
                else:
                    ok, nis = ekf.update_wall(dist, phi, wall['cov'])
                    info['wall'] = 'ok' if ok else f'rejected (NIS {nis:.1f})'
        elif wall is None:
            info['wall'] = why

        if not ekf.initialized or not self.use_tag:
            return self.state(), info

        # 定位标记（AprilTag 或色块）
        if self.servo.detector == 'apriltag':
            m, why = self.servo.tag_in_base(stamp_after=self._last_tag)
        elif wall is not None:
            m, why = self.servo.blob_in_base(wall, stamp_after=self._last_tag)
        else:
            m, why = None, '无墙面，色块无法求交'
        if m is None:
            info['tag'] = why
            return self.state(), info
        self._last_tag = m['stamp']
        T = self._to_now(m['stamp'], now_pose)
        if T is None:
            info['tag'] = '时间戳超出里程计历史'
            return self.state(), info
        R, t = T
        p = R @ m['p'][:2] + t
        cov = R @ m['cov'][:2, :2] @ R.T
        if not self.u_known:
            # 首次看到标记：保留激光给出的 d、ψ，由标记位置反解 u
            _, u, _, _ = WallEKF.init_from((ekf.x[0] + ekf.offset[0], -(ekf.x[2] + ekf.offset[2])),
                                           p, offset=ekf.offset)
            ekf.x[1] = u
            ekf.P[1, :] = 0.0
            ekf.P[:, 1] = 0.0
            ekf.P[1, 1] = 0.01 ** 2
            self.u_known = True
        ok, nis = ekf.update_tag(p, cov)
        if ok:
            self.tag_reject_streak = 0
            self.last_tag_time = self.now()
            info['tag'] = 'ok'
        else:
            self.tag_reject_streak += 1
            info['tag'] = f'rejected (NIS {nis:.1f})'
            if self.tag_reject_streak >= self.reacquire_after:
                # 持续不一致：认为滤波器的 u 已失效（例如被外力推动），由标记重新初始化
                self.u_known = False
                self.tag_reject_streak = 0
                info['tag'] = 'reacquire'
        return self.state(), info
