"""TagMap 单元测试：加载与校验、子地图、位置误差扰动."""
import math
import os

import pytest

from scr_arm_controller.tag_map import resolve_path, TagMap

MAP_YAML = """
tag_map:
  family: tag36h11
  size: 0.10
  reference_id: 0
  sigma_position: 0.002
  tags:
    - {id: 2, u: 0.6, v: 0.70}
    - {id: 0, u: 0.0, v: 0.70}
    - {id: 5, u: -0.3, v: 0.70, sigma: 0.004}
    - {id: 4, u: 1.2}
"""


@pytest.fixture
def map_file(tmp_path):
    p = tmp_path / 'tag_map.yaml'
    p.write_text(MAP_YAML, encoding='utf-8')
    return str(p)


def test_load_sorts_by_position_and_reads_sigma(map_file):
    m = TagMap.load(map_file)
    assert m.ids == [5, 0, 2, 4]
    assert len(m) == 4 and 2 in m and 3 not in m
    assert m.u[2] == pytest.approx(0.6)
    assert math.isnan(m.v[4])
    assert m.sigma(2) == pytest.approx(0.002)
    assert m.sigma(5) == pytest.approx(0.004)        # 单个标记可覆盖默认误差
    assert m.sigma(0) == 0.0                          # 参考标记定义原点，没有位置误差
    assert m.spacing() == pytest.approx(0.6)
    assert m.nearest(0.95) == (4, pytest.approx(0.25))


def test_round_trip_through_dict(map_file):
    m = TagMap.load(map_file)
    m2 = TagMap.from_dict(m.to_dict())
    assert m2.ids == m.ids
    assert all(m2.u[i] == pytest.approx(m.u[i]) and m2.sigma(i) == m.sigma(i) for i in m.ids)


@pytest.mark.parametrize('text, msg', [
    ('tag_map: {reference_id: 0, tags: [{id: 1, u: 0.3}]}', '不在地图中'),
    ('tag_map: {reference_id: 1, tags: [{id: 1, u: 0.3}]}', '必须为 0'),
    ('tag_map: {tags: [{id: 0, u: 0.0}, {id: 0, u: 0.3}]}', '重复'),
    ('tag_map: {reference_id: 0}', 'tags'),
    ('tag_map: {sigma_position: -1, tags: [{id: 0, u: 0.0}, {id: 1, u: 0.3}]}', '误差无效'),
])
def test_invalid_maps_are_rejected(text, msg):
    import yaml
    with pytest.raises(ValueError, match=msg):
        TagMap.from_dict(yaml.safe_load(text))


def test_single_tag_map_is_the_reference_only():
    m = TagMap.single(7)
    assert m.ids == [7] and m.reference_id == 7 and m.u[7] == 0.0
    assert m.spacing() == math.inf


def test_subset_keeps_reference(map_file):
    m = TagMap.load(map_file).subset([4])
    assert m.ids == [0, 4]
    assert m.sigma(4) == pytest.approx(0.002)


def test_perturbed_moves_only_non_reference_tags(map_file):
    m = TagMap.load(map_file)
    p1, p2 = m.perturbed(0.002, seed=3), m.perturbed(0.002, seed=3)
    assert p1.u[0] == 0.0
    assert all(p1.u[i] == p2.u[i] for i in m.ids)            # 可复现
    errs = [p1.u[i] - m.u[i] for i in m.ids if i != 0]
    assert all(e != 0.0 and abs(e) < 0.01 for e in errs)


def test_resolve_path():
    assert resolve_path('', '/cfg') is None
    assert resolve_path('/a/b.yaml', '/cfg') == '/a/b.yaml'
    assert resolve_path('tag_map.yaml', '/cfg') == '/cfg/tag_map.yaml'
    assert resolve_path('~/maps/a.yaml', '/cfg') == os.path.expanduser('~/maps/a.yaml')
