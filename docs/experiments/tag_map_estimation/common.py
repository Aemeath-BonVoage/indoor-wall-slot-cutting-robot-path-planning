"""
本实验各脚本共用的数据读取与几何换算（不依赖 ROS）.

墙面坐标：room3 南墙墙面 y = −3.925 m，参考标记 ID 0 在 x = 8.42 m；面向墙面向右（世界 −x）
为 u 的正方向。底盘真值 (x, y, yaw) → 墙面相对状态 (d, u, ψ)：
    d = y + 3.925,   u = 8.42 − x,   ψ = yaw + π/2（正对墙面时 yaw = −π/2）
"""
import csv
import gzip
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'src', 'scr_arm_controller'))
sys.path.insert(0, os.path.join(REPO, 'docs'))

BAG_DATA = os.path.join(HERE, 'bag_data')
AW_RUNS = os.path.join(REPO, 'docs', 'experiments', 'along_wall_accuracy', 'runs')
WALL_Y = -3.925
TAG0_X = 8.42
# room.world 中各标记的实际沿墙位置（ID k = 1–4 在 0.3·k，ID 5 在 −0.3）
WORLD_TAG_U = {0: 0.0, 1: 0.3, 2: 0.6, 3: 0.9, 4: 1.2, 5: -0.3}
MARKER_U_PLAN = 1.505     # 参考标记在规划墙面坐标中的 u（停靠站 u_c − 1.505 = 相对标记的位置）


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def read_csv(path):
    op = gzip.open if path.endswith('.gz') else open
    with op(path, 'rt', newline='') as f:
        return list(csv.DictReader(f))


def read_array(path, cols):
    rows = read_csv(path)
    return np.array([[float(r[c]) for c in cols] for r in rows])


def to_wall(x, y, yaw):
    """世界位姿 → (d, u, ψ)（numpy 数组或标量）."""
    d = np.asarray(y) + (-WALL_Y)
    u = TAG0_X - np.asarray(x)
    psi = np.arctan2(np.sin(np.asarray(yaw) + math.pi / 2), np.cos(np.asarray(yaw) + math.pi / 2))
    return d, u, psi


class Track:
    """时间序列位姿 (t, x, y, yaw)，按时间线性插值（朝向按展开角插值）."""

    def __init__(self, arr):
        self.t = arr[:, 0]
        self.x = arr[:, 1]
        self.y = arr[:, 2]
        self.yaw = np.unwrap(arr[:, 3])

    def at(self, t):
        return (np.interp(t, self.t, self.x), np.interp(t, self.t, self.y),
                np.interp(t, self.t, self.yaw))


def relative_pose(a, b):
    """位姿 a → b 的相对变换，表示在 a 的机体系中（与 wall_estimator_ros.relative_pose 相同）."""
    c, s = math.cos(a[2]), math.sin(a[2])
    ex, ey = b[0] - a[0], b[1] - a[1]
    return c * ex + s * ey, -s * ex + c * ey, wrap(b[2] - a[2])


def load_bag(name):
    """bag_data/<name> 的全部数据."""
    d = os.path.join(BAG_DATA, name)
    out = {
        'odom': Track(read_array(os.path.join(d, 'wheel_odom.csv.gz'), ('t', 'x', 'y', 'yaw'))),
        'truth': Track(read_array(os.path.join(d, 'truth.csv.gz'), ('t', 'x', 'y', 'yaw'))),
        'gyro': read_array(os.path.join(d, 'gyro.csv.gz'), ('t', 'wz')),
        'wall': read_array(os.path.join(d, 'wall_fit.csv.gz'),
                           ('t', 'distance', 'phi', 'var_d', 'cov_dphi', 'var_phi', 'inliers',
                            'rms')),
        'status': read_csv(os.path.join(d, 'status.csv')),
        'estimate': read_csv(os.path.join(d, 'wall_estimate.csv.gz')),
    }
    out['phases'] = phase_intervals(out['status'], out['truth'].t[-1])
    return out


def phase_intervals(status, t_end):
    """status.csv → [(t0, t1, station, phase)]（按 stamp 排序；后一条生效时前一条结束）."""
    rows = sorted(((float(r['t']), int(r['station']), r['phase']) for r in status),
                  key=lambda r: r[0])
    out = []
    for k, (t, st, ph) in enumerate(rows):
        t1 = rows[k + 1][0] if k + 1 < len(rows) else t_end
        if t1 > t:
            out.append((t, t1, st, ph))
    return out


def phase_at(phases, t):
    for t0, t1, st, ph in phases:
        if t0 <= t < t1:
            return st, ph
    return -1, 'idle'


def parse_tags(s):
    """'id:x:y:acc;...' → [(id, x, y, accepted)]."""
    out = []
    for item in filter(None, s.split(';')):
        i, x, y, a = item.split(':')
        out.append((int(i), float(x), float(y), bool(int(a))))
    return out


def parse_trace_tags(s):
    """along_wall ekf_trace 的 tag_xy 'id:x:y id:x:y' → [(id, x, y)]."""
    out = []
    for item in s.split():
        i, x, y = item.split(':')
        out.append((int(i), float(x), float(y)))
    return out


def h_tag(d, u, psi, u_tag):
    c, s = math.cos(psi), math.sin(psi)
    du = u - u_tag
    return np.array([d * c + du * s, -d * s + du * c])
