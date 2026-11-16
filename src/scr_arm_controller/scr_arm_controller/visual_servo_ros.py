"""PBVS 的 ROS 接口：订阅激光 / 相机 / 相机内参，查 TF，产出伺服误差测量."""
import math

import numpy as np
import tf2_ros
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan

from scr_arm_controller import visual_servo as vs


def _quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class WallMarkerServo:
    """挂在一个已有节点上，持续缓存最新的激光、方块检测与相机内参."""

    def __init__(self, node, image_topic='/front_camera/image_raw',
                 info_topic='/front_camera/camera_info', scan_topic='/scan',
                 base_frame='base_link', camera_frame='camera_optical_link',
                 lidar_frame='lidar_link', max_age=0.5, smoothing=0.5):
        self.node = node
        self.base_frame = base_frame
        self.camera_frame = camera_frame
        self.lidar_frame = lidar_frame
        self.max_age = Duration(seconds=max_age)
        self.alpha = smoothing          # 指数滑动平均系数（越大越平滑）
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        self.scan = None
        self.detection = None           # (u, v, stamp)
        self.K = None                   # (fx, fy, cx, cy)
        self._filtered = None

        # 实验用：额外注入的测量噪声（标准差），默认为 0
        self.pixel_noise = 0.0          # 方块像素坐标噪声（像素）
        self.range_noise = 0.0          # 激光测距噪声（米）
        self.rng = np.random.default_rng()

        node.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        node.create_subscription(Image, image_topic, self._on_image, 1)
        node.create_subscription(CameraInfo, info_topic, self._on_info, 1)

    # ---------- 回调 ----------
    def _on_scan(self, msg):
        self.scan = msg

    def _on_info(self, msg):
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def _on_image(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        det = vs.detect_marker(img)
        stamp = Time.from_msg(msg.header.stamp)
        self.detection = None if det is None else (det['u'], det['v'], stamp)

    # ---------- 测量 ----------
    def _fresh(self, stamp):
        return (self.node.get_clock().now() - stamp) < self.max_age

    def _lookup(self, target, source):
        tf = self.tf_buffer.lookup_transform(target, source, Time(),
                                             timeout=Duration(seconds=0.1))
        t = tf.transform.translation
        return np.array([t.x, t.y, t.z]), _quat_to_rot(tf.transform.rotation)

    def wall_in_base(self):
        """仅用激光拟合前方墙面，返回 base_link 中的墙面 dict，或 (None, 原因)."""
        if self.scan is None or not self._fresh(Time.from_msg(self.scan.header.stamp)):
            return None, '激光数据过期'
        s = self.scan
        ranges = np.asarray(s.ranges, dtype=float)
        if self.range_noise > 0:
            ranges = ranges + self.rng.normal(0.0, self.range_noise, ranges.shape)
        wall_l = vs.fit_wall(ranges, s.angle_min, s.angle_increment)
        if wall_l is None:
            return None, '前方未拟合到墙面'
        try:
            p_lb, R_lb = self._lookup(self.base_frame, self.lidar_frame)
        except Exception as e:  # noqa: BLE001
            return None, f'TF 查询失败: {e}'
        # 激光系墙面 → base_link（雷达只有平移与绕 z 旋转时法向在水平面内）
        n_b = R_lb @ np.array([wall_l['normal'][0], wall_l['normal'][1], 0.0])
        n_b2 = n_b[:2] / np.linalg.norm(n_b[:2])
        d_b = wall_l['distance'] + float(n_b2 @ p_lb[:2])
        return {'normal': tuple(n_b2), 'distance': d_b,
                'yaw_error': math.atan2(n_b2[1], n_b2[0]),
                'rms': wall_l['rms'], 'inliers': wall_l['inliers']}, None

    def tool_front_x(self, link='link4', offset=0.1594):
        """刀具最前端在 base_link 中的 x 坐标.

        offset：末端网格沿 base_link x 方向相对 link4 原点的最大前伸量（由 link4.STL 包围盒得到）。
        """
        p, _ = self._lookup(self.base_frame, link)
        return float(p[0]) + offset

    def measure(self, desired_distance):
        """返回一次 PBVS 测量 dict，或 (None, 原因)."""
        if self.K is None:
            return None, '未收到相机内参'
        if self.detection is None or not self._fresh(self.detection[2]):
            return None, '未检测到方块'
        wall, why = self.wall_in_base()
        if wall is None:
            return None, why
        try:
            p_cb, R_cb = self._lookup(self.base_frame, self.camera_frame)
        except Exception as e:  # noqa: BLE001
            return None, f'TF 查询失败: {e}'
        n_b2, d_b = np.array(wall['normal']), wall['distance']

        u, v, _ = self.detection
        if self.pixel_noise > 0:
            u += self.rng.normal(0.0, self.pixel_noise)
            v += self.rng.normal(0.0, self.pixel_noise)
        ray = R_cb @ vs.pixel_to_ray(u, v, *self.K)
        marker = vs.intersect_ray_wall(p_cb, ray, n_b2, d_b)
        if marker is None:
            return None, '相机射线与墙面无交点'

        err = vs.pbvs_error(marker, wall, desired_distance)
        raw = np.array([err['yaw'], err['normal'], err['lateral']])
        if self._filtered is None:
            self._filtered = raw
        else:
            self._filtered = self.alpha * self._filtered + (1 - self.alpha) * raw
        err['yaw'], err['normal'], err['lateral'] = self._filtered.tolist()
        return {'err': err, 'wall': wall, 'marker_base': marker, 'pixel': (u, v)}, None

    def reset_filter(self):
        self._filtered = None
