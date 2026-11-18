"""
墙面定位标记地图：多个 AprilTag 在墙面坐标中的已知位置（纯算法部分，不依赖 ROS）.

YAML 格式（u 相对参考标记，面向墙面向右为正；v 为中心离地高度，只作记录）::

    tag_map:
      family: tag36h11
      size: 0.10                 # 黑框边长（米）
      reference_id: 0            # 墙面坐标原点
      sigma_position: 0.002      # 每个标记相对参考标记的位置误差（米，1σ），手工粘贴的量级
      tags:
        - {id: 0, u: 0.0, v: 0.70}
        - {id: 1, u: 0.6, v: 0.70}

参考标记的 u 必须为 0：墙面坐标原点由它确定，精对位以它为目标。单个标记的情形
（第一版的“一个定位标记”）等价于只含参考标记的地图，见 TagMap.single()。
"""
import math
import os

import numpy as np
import yaml


class TagMap:
    """标记 ID → 墙面坐标 u（米）及位置误差."""

    def __init__(self, tags, reference_id=0, family='tag36h11', size=0.10, sigma_position=0.0):
        """tags：{id: u} 或 {id: (u, v)}；sigma_position：标量或 {id: σ}."""
        self.family, self.size = family, float(size)
        self.reference_id = int(reference_id)
        self.u, self.v = {}, {}
        for i, val in tags.items():
            u, v = (val, math.nan) if np.isscalar(val) else (val[0], val[1])
            self.u[int(i)] = float(u)
            self.v[int(i)] = float(v)
        if self.reference_id not in self.u:
            raise ValueError(f'参考标记 {self.reference_id} 不在地图中')
        if abs(self.u[self.reference_id]) > 1e-9:
            raise ValueError(f'参考标记的 u 必须为 0（实际 {self.u[self.reference_id]}）')
        if isinstance(sigma_position, dict):
            self._sigma = {int(k): float(v) for k, v in sigma_position.items()}
        else:
            self._sigma = {i: float(sigma_position) for i in self.u}
        self._sigma[self.reference_id] = 0.0   # 原点由参考标记定义，没有位置误差
        for i, s in self._sigma.items():
            if s < 0.0 or not math.isfinite(s):
                raise ValueError(f'标记 {i} 的位置误差无效: {s}')

    # ------------------------------------------------------------------
    @classmethod
    def single(cls, tag_id=0, family='tag36h11', size=0.10):
        """只有一个标记（即参考标记）的地图."""
        return cls({tag_id: 0.0}, reference_id=tag_id, family=family, size=size)

    @classmethod
    def from_dict(cls, data):
        m = data.get('tag_map', data)
        if not m or 'tags' not in m:
            raise ValueError('标记地图缺少 tags')
        tags = {}
        sig = {}
        default_sigma = float(m.get('sigma_position', 0.0))
        for t in m['tags']:
            i = int(t['id'])
            if i in tags:
                raise ValueError(f'标记 ID {i} 重复')
            tags[i] = (float(t['u']), float(t.get('v', math.nan)))
            sig[i] = float(t.get('sigma', default_sigma))
        ref = int(m.get('reference_id', min(tags)))
        return cls(tags, reference_id=ref, family=m.get('family', 'tag36h11'),
                   size=float(m.get('size', 0.10)), sigma_position=sig)

    @classmethod
    def load(cls, path):
        with open(path, encoding='utf-8') as f:
            return cls.from_dict(yaml.safe_load(f))

    def to_dict(self):
        return {'tag_map': {
            'family': self.family, 'size': self.size, 'reference_id': self.reference_id,
            'tags': [dict(id=i, u=round(self.u[i], 6), v=self.v[i], sigma=self._sigma[i])
                     for i in self.ids]}}

    # ------------------------------------------------------------------
    @property
    def ids(self):
        """按 u 排序的标记 ID."""
        return sorted(self.u, key=lambda i: (self.u[i], i))

    def __contains__(self, tag_id):
        return int(tag_id) in self.u

    def __len__(self):
        return len(self.u)

    def sigma(self, tag_id):
        return self._sigma.get(int(tag_id), 0.0)

    def spacing(self):
        """相邻标记的最大间距（米）；单个标记时为 inf."""
        us = sorted(self.u.values())
        return max(np.diff(us)) if len(us) > 1 else math.inf

    def nearest(self, u):
        """离沿墙位置 u 最近的标记 (id, 距离)."""
        i = min(self.u, key=lambda k: abs(self.u[k] - u))
        return i, abs(self.u[i] - u)

    def subset(self, ids):
        """只保留给定 ID 的子地图（参考标记必须保留）."""
        ids = set(int(i) for i in ids) | {self.reference_id}
        return TagMap({i: (self.u[i], self.v[i]) for i in ids if i in self.u},
                      reference_id=self.reference_id, family=self.family, size=self.size,
                      sigma_position={i: self._sigma[i] for i in ids if i in self.u})

    def perturbed(self, sigma, seed=0):
        """
        把每个非参考标记的 u 加上 N(0, σ²) 的误差：模拟“地图记录的位置 ≠ 实际粘贴位置”.

        实验中世界里的标记位置不变、改动地图，与“地图正确、粘贴有误差”等价。
        """
        rng = np.random.default_rng(seed)
        tags = {}
        for i in self.ids:
            e = 0.0 if i == self.reference_id else float(rng.normal(0.0, sigma))
            tags[i] = (self.u[i] + e, self.v[i])
        return TagMap(tags, reference_id=self.reference_id, family=self.family, size=self.size,
                      sigma_position=dict(self._sigma))


def resolve_path(path, config_dir=None):
    """相对路径按 config_dir（默认本包的 share/config）解析；空串返回 None."""
    if not path:
        return None
    if os.path.isabs(path) or config_dir is None:
        return path
    return os.path.join(config_dir, path)
