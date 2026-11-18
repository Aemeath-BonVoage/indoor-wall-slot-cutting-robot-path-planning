"""
精对位的 ROS 接口：订阅激光 / 相机 / 相机内参，查 TF，产出墙面与定位标记测量.

- WallMarkerServo：传感器前端。激光拟合墙面；相机检测定位标记，检测器可选
  'apriltag'（tag36h11，有 ID、亚像素角点）或 'blob'（HSV 色块质心，第一版的检测器，
  保留作基线）。measure() 是第一版流水线（色块射线与墙面求交 + 指数滑动平均），
  供 'visual_v1' 对位模式与对比实验使用。
- 状态估计（EKF）与延迟补偿见 wall_estimator_ros.py。
"""
import math

import numpy as np
import tf2_ros
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan

from scr_arm_controller import fiducial as fd
from scr_arm_controller import visual_servo as vs


def _quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class WallMarkerServo:
    """挂在一个已有节点上，持续缓存最新的激光、标记检测与相机内参."""

    def __init__(self, node, image_topic='/front_camera/image_raw',
                 info_topic='/front_camera/camera_info', scan_topic='/scan',
                 base_frame='base_link', camera_frame='camera_optical_link',
                 lidar_frame='lidar_link', max_age=0.5, smoothing=0.5,
                 detector='apriltag', tag_family='tag36h11', tag_id=0, tag_size=0.10):
        self.node = node
        self.base_frame = base_frame
        self.camera_frame = camera_frame
        self.lidar_frame = lidar_frame
        self.max_age = Duration(seconds=max_age)
        self.alpha = smoothing          # 第一版：指数滑动平均系数（越大越平滑）
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        # 检测器：'apriltag' 或 'blob'；detectors 为当前需要运行的检测器集合
        self.detector = detector
        self.detectors = {detector}
        self.tag_family, self.tag_id, self.tag_size = tag_family, tag_id, tag_size
        # 需要输出的标记 ID 集合（多标记地图）；None 表示只用 tag_id
        self.tag_ids = None
        self._tag_detector = None
        self.tag_sigma_px = 0.2         # 角点噪声（像素，用于测量协方差）
        self.blob_sigma_px = 0.5        # 色块质心噪声（像素）

        self.scan = None
        self.detection = None           # 色块 (u, v, stamp)
        self.tag = None                 # AprilTag（tag_id）dict(id, corners, center, stamp)
        self.tags = {}                  # 本帧检测到的地图内标记 {id: dict}
        self.K = None                   # (fx, fy, cx, cy)
        self._filtered = None

        # 主点修正（像素）：gazebo_ros_camera 发布的 cx, cy = (W+1)/2, (H+1)/2，而按 OpenCV
        # “整数坐标位于像素中心”的约定应为 (W-1)/2, (H-1)/2，相差 1 px。27 个位姿实测检测像素
        # 相对真值投影偏 −0.99 / −1.01 px，导致标记横向估计偏 +1.5~3 mm（随距离增大）。
        # 真机上应由相机标定得到内参，此修正置 0。
        self.principal_point_shift = (-1.0, -1.0)

        # 实验用：额外注入的测量噪声（标准差），默认为 0
        self.pixel_noise = 0.0          # 色块质心 / 标记角点的像素噪声（像素）
        self.range_noise = 0.0          # 激光测距噪声（米）
        self.rng = np.random.default_rng()

        node.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        node.create_subscription(Image, image_topic, self._on_image, 1)
        node.create_subscription(CameraInfo, info_topic, self._on_info, 1)

    # ---------- 回调 ----------
    def _on_scan(self, msg):
        self.scan = msg

    def _on_info(self, msg):
        dx, dy = self.principal_point_shift
        self.K = (msg.k[0], msg.k[4], msg.k[2] + dx, msg.k[5] + dy)

    def _on_image(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        stamp = Time.from_msg(msg.header.stamp)
        if 'blob' in self.detectors:
            det = vs.detect_marker(img)
            self.detection = None if det is None else (det['u'], det['v'], stamp)
        if 'apriltag' in self.detectors:
            if self._tag_detector is None:
                self._tag_detector = fd.make_detector(self.tag_family, refine='subpix')
            wanted = {self.tag_id} if self.tag_ids is None else set(self.tag_ids)
            tags = fd.detect_apriltags(img, self._tag_detector, None)
            self.tags = {t['id']: dict(t, stamp=stamp) for t in tags if t['id'] in wanted}
            self.tag = self.tags.get(self.tag_id)

    # ---------- 工具 ----------
    def _fresh(self, stamp):
        return (self.node.get_clock().now() - stamp) < self.max_age

    def _lookup(self, target, source):
        tf = self.tf_buffer.lookup_transform(target, source, Time(),
                                             timeout=Duration(seconds=0.1))
        t = tf.transform.translation
        return np.array([t.x, t.y, t.z]), _quat_to_rot(tf.transform.rotation)

    def K_matrix(self):
        fx, fy, cx, cy = self.K
        return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])

    # ---------- 激光 ----------
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
        # 协方差：d_b = d_l + n(φ)·p，∂d_b/∂φ = t·p（φ 的平移不变）
        t_b = np.array([-n_b2[1], n_b2[0]])
        J = np.array([[1.0, float(t_b @ p_lb[:2])], [0.0, 1.0]])
        return {'normal': tuple(n_b2), 'distance': d_b,
                'yaw_error': math.atan2(n_b2[1], n_b2[0]),
                'cov': J @ wall_l['cov'] @ J.T,
                'rms': wall_l['rms'], 'inliers': wall_l['inliers'],
                'stamp': Time.from_msg(s.header.stamp)}, None

    def tool_front_x(self, link='link4', offset=0.1594):
        """
        刀尖在 base_link 中的 x 坐标.

        offset：末端网格沿 base_link x 方向相对 link4 原点的最大前伸量（由 link4.STL 包围盒得到）。
        """
        p, _ = self._lookup(self.base_frame, link)
        return float(p[0]) + offset

    # ---------- 相机：标记在 base_link 中的位置 ----------
    def tag_in_base(self, stamp_after=None):
        """
        参考标记（tag_id）中心在 base_link 中的位置与协方差 → (dict(id, p, cov, stamp, corners), None).

        stamp_after：只返回比该时间戳更新的检测（避免同一帧重复更新滤波器）。
        """
        tags, why = self.tags_in_base(stamp_after, ids=(self.tag_id,))
        return (tags[0], None) if tags else (None, why)

    def tags_in_base(self, stamp_after=None, ids=None):
        """
        本帧检测到的全部地图内标记 → ([dict(id, p, cov, stamp, corners), ...], 原因).

        ids：只返回这些 ID（None = 全部）。同一帧中的多个标记共享时间戳。
        """
        if self.K is None:
            return [], '未收到相机内参'
        tags = [t for i, t in sorted(self.tags.items()) if ids is None or i in ids]
        if not tags or not self._fresh(tags[0]['stamp']):
            return [], '未检测到 AprilTag'
        if stamp_after is not None and tags[0]['stamp'] <= stamp_after:
            return [], '无新图像'
        try:
            p_cb, R_cb = self._lookup(self.base_frame, self.camera_frame)
        except Exception as e:  # noqa: BLE001
            return [], f'TF 查询失败: {e}'
        sigma = math.hypot(self.tag_sigma_px, self.pixel_noise)
        out = []
        for tag in tags:
            corners = tag['corners']
            if self.pixel_noise > 0:
                corners = corners + self.rng.normal(0.0, self.pixel_noise, corners.shape)
            pose = fd.tag_pose(corners, self.K_matrix(), self.tag_size)
            if pose is None:
                continue
            _, t_cam = pose
            cov_cam = fd.tag_position_covariance(t_cam, self.K[0], self.tag_size, sigma_px=sigma)
            out.append({'id': tag['id'], 'p': p_cb + R_cb @ t_cam, 'cov': R_cb @ cov_cam @ R_cb.T,
                        'stamp': tag['stamp'], 'corners': corners})
        return out, (None if out else 'PnP 失败')

    def blob_in_base(self, wall, stamp_after=None):
        """
        色块质心射线与激光墙面的交点（base_link）及协方差 → (dict, None).

        新流水线搭配色块检测器时使用（消融对照）；深度来自激光墙面，协方差的深度项取大值，
        使法向距离仍由激光测量直接决定、色块只提供沿墙位置。
        """
        if self.K is None:
            return None, '未收到相机内参'
        if self.detection is None or not self._fresh(self.detection[2]):
            return None, '未检测到色块'
        u, v, stamp = self.detection
        if stamp_after is not None and stamp <= stamp_after:
            return None, '无新图像'
        if self.pixel_noise > 0:
            u += self.rng.normal(0.0, self.pixel_noise)
            v += self.rng.normal(0.0, self.pixel_noise)
        try:
            p_cb, R_cb = self._lookup(self.base_frame, self.camera_frame)
        except Exception as e:  # noqa: BLE001
            return None, f'TF 查询失败: {e}'
        ray = R_cb @ vs.pixel_to_ray(u, v, *self.K)
        p = vs.intersect_ray_wall(p_cb, ray, np.array(wall['normal']), wall['distance'])
        if p is None:
            return None, '相机射线与墙面无交点'
        t_cam = R_cb.T @ (p - p_cb)
        sigma = math.hypot(self.blob_sigma_px, self.pixel_noise)
        cov_cam = fd.tag_position_covariance(t_cam, self.K[0], 1.0, sigma_px=sigma)
        return {'p': p, 'cov': R_cb @ cov_cam @ R_cb.T, 'stamp': stamp}, None

    # ---------- 第一版流水线 ----------
    def measure(self, desired_distance):
        """第一版：色块射线与墙面求交 + 指数滑动平均，返回一次 PBVS 测量 dict 或 (None, 原因)."""
        if self.K is None:
            return None, '未收到相机内参'
        if self.detection is None or not self._fresh(self.detection[2]):
            return None, '未检测到色块'
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
