# -*- coding: utf-8 -*-
"""sdf_gen：墙面实体 / 演示相机 SDF."""
import math
import xml.etree.ElementTree as ET

import pytest

from scr_viz import sdf_gen
from scr_viz import spawn_wall_fixtures as swf
from scr_viz import wall_model as wm


def _floats(s):
    return [float(v) for v in s.split()]


def test_infer_kind():
    assert sdf_gen.infer_kind('配电箱') == 'panelboard'
    assert sdf_gen.infer_kind('灯开关') == 'switch'
    assert sdf_gen.infer_kind('已有插座A') == 'socket'
    assert sdf_gen.infer_kind('线盒') == 'junction_box'
    assert sdf_gen.infer_kind('???') == 'junction_box'
    assert sdf_gen.infer_kind('???', 'socket') == 'socket'


def test_fixtures_from_demo_task(demo_task_file):
    task = wm.load_yaml(demo_task_file)
    fx = sdf_gen.fixtures_from_task(task)
    kinds = {f.label: f.kind for f in fx}
    assert '黄色定位标记' not in kinds                 # world 里已有，跳过
    assert kinds['灯开关'] == 'switch' and kinds['配电箱'] == 'panelboard'
    assert kinds['已有插座A'] == 'socket' and kinds['新插座A'] == 'planned'
    assert kinds['书桌插座B'] == 'planned'
    names = [f.name for f in fx]
    assert len(names) == len(set(names))
    assert all(sdf_gen.safe_name(n) == n for n in names)
    box = [f for f in fx if f.kind == 'panelboard'][0]
    assert (box.u, box.v, box.w, box.h) == pytest.approx((3.55, 1.6, 0.3, 0.4))
    # 不要任务线盒
    assert len(sdf_gen.fixtures_from_task(task, include_endpoints=False)) == 2
    # 不跳过定位标记
    assert len(sdf_gen.fixtures_from_task(task, skip_labels=())) == len(fx) + 1


def test_fixture_pose_on_wall(demo_task_file):
    task = wm.load_yaml(demo_task_file)
    items, wall = swf.build(task)
    for f, xml in items:
        pose = sdf_gen.fixture_pose(wall, f)
        u, v, off = wall.from_map(*pose[:3])
        assert (u, v) == pytest.approx((f.u, f.v))
        assert off == pytest.approx(f.thickness / 2)    # 背面贴墙面，整体在房间一侧
        assert pose[5] == pytest.approx(math.pi)          # 南墙：模型 x 沿 t = -x
        root = ET.fromstring(xml.split('\n', 1)[1])
        model = root.find('model')
        assert model.get('name') == f.name
        assert model.find('static').text == 'true'
        assert _floats(model.find('pose').text) == pytest.approx(pose, abs=1e-5)
        link = model.find('link')
        assert link.find('collision') is None             # 默认无碰撞
        size = _floats(link.find('visual/geometry/box/size').text)
        assert size == pytest.approx((f.w, f.thickness, f.h))


def test_fixture_collision_flag(demo_task_file):
    items, _ = swf.build(wm.load_yaml(demo_task_file), collision=True)
    for _, xml in items:
        assert ET.fromstring(xml.split('\n', 1)[1]).find('model/link/collision') is not None


def test_socket_face_details_face_room(demo_task_file):
    """插座插孔画在面板朝房间一侧（模型 -y）."""
    items, wall = swf.build(wm.load_yaml(demo_task_file))
    f, xml = [(f, x) for f, x in items if f.kind == 'socket'][0]
    root = ET.fromstring(xml.split('\n', 1)[1])
    holes = [v for v in root.iter('visual') if v.get('name') == 'hole']
    assert len(holes) == 2
    for h in holes:
        assert _floats(h.find('pose').text)[1] < -f.thickness / 2


def test_write_out(tmp_path, demo_task_file):
    items, wall = swf.build(wm.load_yaml(demo_task_file))
    man = swf.write_out(items, wall, str(tmp_path))
    assert (tmp_path / 'fixtures.yaml').exists()
    assert len(man) == len(items)
    for m in man:
        assert (tmp_path / m['file']).exists()


def test_cli_offline(tmp_path, demo_task_file, capsys):
    assert swf.main([demo_task_file, '--out', str(tmp_path)]) == 0
    assert 'panelboard' in capsys.readouterr().out


@pytest.mark.parametrize('eye,target', [
    ((0, 0, 1), (1, 0, 1)), ((0, 0, 2), (1, 1, 0)), ((5, -2, 1.5), (4, -4, 0.3)),
    ((0, 0, 3), (0.001, 0, 0)),
])
def test_look_at(eye, target):
    rpy = sdf_gen.look_at_rpy(eye, target)
    d = [target[i] - eye[i] for i in range(3)]
    n = math.sqrt(sum(c * c for c in d))
    assert sdf_gen.rpy_forward(rpy) == pytest.approx([c / n for c in d], abs=1e-9)


def test_look_at_degenerate():
    with pytest.raises(ValueError):
        sdf_gen.look_at_rpy((1, 1, 1), (1, 1, 1))


def test_camera_presets_inside_room3():
    for name, (eye, target) in sdf_gen.CAMERA_PRESETS.items():
        assert 5.075 < eye[0] < 9.925 and -3.925 < eye[1] < 3.925, name
        assert sdf_gen.rpy_forward(sdf_gen.look_at_rpy(eye, target))[1] < 0, name  # 朝南墙


def test_camera_sdf():
    xml = sdf_gen.camera_sdf('cam1', (6, -2, 1.5), (8, -3.9, 0.5), 320, 240, 5)
    root = ET.fromstring(xml.split('\n', 1)[1])
    m = root.find('model')
    assert m.get('name') == 'cam1' and m.find('static').text == 'true'
    pose = _floats(m.find('pose').text)
    assert pose[:3] == [6, -2, 1.5]
    assert pose[3:] == pytest.approx(sdf_gen.look_at_rpy((6, -2, 1.5), (8, -3.9, 0.5)), abs=1e-5)
    sensor = m.find('link/sensor')
    assert sensor.get('type') == 'camera'
    assert sensor.find('camera/image/width').text == '320'
    assert sensor.find('update_rate').text == '5'
    plugin = sensor.find('plugin')
    assert plugin.get('filename') == 'libgazebo_ros_camera.so'
    assert plugin.find('ros/namespace').text == '/cam1'
    assert m.find('link/collision') is None
