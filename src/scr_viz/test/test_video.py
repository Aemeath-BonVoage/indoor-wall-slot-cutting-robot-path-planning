# -*- coding: utf-8 -*-
"""video：图像解码、抽帧、GIF / MP4 合成."""
import os

import numpy as np
import pytest

from scr_viz import video


def test_image_to_rgb_rgb8_with_padding():
    w, h, step = 3, 2, 12          # 每行 9 字节有效 + 3 字节填充
    rows = []
    for r in range(h):
        px = [(r, c, 100 + c) for c in range(w)]
        rows.append(bytes(sum(px, ())) + b'\xff' * 3)
    img = video.image_to_rgb(b''.join(rows), w, h, 'rgb8', step)
    assert img.shape == (2, 3, 3)
    assert tuple(img[1, 2]) == (1, 2, 102)


def test_image_to_rgb_bgr8_and_mono():
    data = bytes([10, 20, 30, 40, 50, 60])
    assert tuple(video.image_to_rgb(data, 2, 1, 'bgr8')[0, 0]) == (30, 20, 10)
    mono = video.image_to_rgb(bytes([7, 9]), 2, 1, 'mono8')
    assert mono.shape == (1, 2, 3) and tuple(mono[0, 1]) == (9, 9, 9)
    rgba = video.image_to_rgb(bytes([1, 2, 3, 4]), 1, 1, 'rgba8')
    assert tuple(rgba[0, 0]) == (1, 2, 3)


def test_image_to_rgb_errors():
    with pytest.raises(ValueError):
        video.image_to_rgb(b'\x00' * 3, 1, 1, '16UC1')
    with pytest.raises(ValueError):
        video.image_to_rgb(b'\x00' * 2, 1, 1, 'rgb8')


def test_frame_selector():
    sel = video.FrameSelector(0.5)
    ts = [k * 0.1 for k in range(21)]               # 10 Hz，2 s
    kept = [t for t in ts if sel.keep(t)]
    assert kept == pytest.approx([0.0, 0.5, 1.0, 1.5, 2.0])
    # 长时间没有图像后重新对齐
    assert sel.keep(10.03) and not sel.keep(10.3) and sel.keep(10.55)
    # 时间回跳（仿真重置）
    assert sel.keep(0.0) and not sel.keep(0.1)


def _write_frames(d, n=4, size=(64, 48)):
    Image = pytest.importorskip('PIL.Image')
    for k in range(n):
        arr = np.zeros((size[1], size[0], 3), np.uint8)
        arr[:, : (k + 1) * size[0] // n] = (255, 80, 0)
        Image.fromarray(arr).save(os.path.join(d, f'frame_{k:05d}.jpg'))


def test_list_frames(tmp_path):
    _write_frames(str(tmp_path), 6)
    (tmp_path / 'other.jpg').write_bytes(b'')
    fs = video.list_frames(str(tmp_path))
    assert [os.path.basename(f) for f in fs] == [f'frame_{k:05d}.jpg' for k in range(6)]
    assert len(video.list_frames(str(tmp_path), every=2)) == 3
    assert len(video.list_frames(str(tmp_path), start=1, end=4)) == 3


def test_make_gif(tmp_path):
    Image = pytest.importorskip('PIL.Image')
    _write_frames(str(tmp_path), 4, (100, 50))
    out = video.make_gif(video.list_frames(str(tmp_path)), str(tmp_path / 'a.gif'), 5, 40)
    with Image.open(out) as im:
        assert im.n_frames == 4 and im.size == (40, 20)
        assert im.info['duration'] == 200


def test_make_video_falls_back_to_gif(tmp_path, monkeypatch):
    _write_frames(str(tmp_path), 3)
    monkeypatch.setattr(video.shutil, 'which', lambda _: None)
    out = video.make_video(video.list_frames(str(tmp_path)), str(tmp_path / 'b.mp4'), 5)
    assert out.endswith('b.gif') and os.path.exists(out)


def test_ffmpeg_cmd_and_concat():
    cmd = video.ffmpeg_cmd('/x/list.txt', '/x/o.mp4', 12, 640)
    assert cmd[-1] == '/x/o.mp4' and 'libx264' in cmd and 'yuv420p' in cmd
    assert cmd[cmd.index('-vf') + 1] == 'scale=640:-2'
    txt = video.concat_list(['/a/f0.jpg', '/a/f1.jpg'], 4)
    assert txt.count("file '") == 3 and 'duration 0.250000' in txt


def test_make_mp4_if_ffmpeg(tmp_path):
    if video.shutil.which('ffmpeg') is None:
        pytest.skip('没有 ffmpeg')
    _write_frames(str(tmp_path), 4)
    out = video.make_mp4(video.list_frames(str(tmp_path)), str(tmp_path / 'c.mp4'), 4, 64)
    assert os.path.getsize(out) > 0
