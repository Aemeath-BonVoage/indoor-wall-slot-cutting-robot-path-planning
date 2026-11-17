# -*- coding: utf-8 -*-
"""
演示素材：图像消息解码、按时间抽帧、帧序列 → GIF / MP4（纯函数 + 命令行）.

  ros2 run scr_viz make_demo_video <frames_dir> <out.gif|out.mp4> [--fps 10] [--width 640]
      [--every N] [--start S] [--end E]

MP4 用 ffmpeg（libx264，yuv420p）；没有 ffmpeg 时自动改写 GIF（PIL）。
"""
import argparse
import glob
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

FRAME_GLOB = 'frame_*.jpg'

# encoding → (通道数, 是否 BGR 顺序)
_ENCODINGS = {
    'rgb8': (3, False), 'bgr8': (3, True), 'rgba8': (4, False), 'bgra8': (4, True),
    'mono8': (1, False),
}


def image_to_rgb(data, width, height, encoding, step=None):
    """sensor_msgs/Image 的字段 → H×W×3 uint8 RGB 数组（不依赖 cv_bridge）."""
    if encoding not in _ENCODINGS:
        raise ValueError(f'不支持的图像编码：{encoding}')
    ch, bgr = _ENCODINGS[encoding]
    step = step or width * ch
    buf = np.frombuffer(bytes(data), dtype=np.uint8)
    if buf.size < step * height:
        raise ValueError('图像数据长度不足')
    img = buf[:step * height].reshape(height, step)[:, :width * ch].reshape(height, width, ch)
    if ch == 1:
        return np.repeat(img, 3, axis=2)
    img = img[:, :, :3]
    return img[:, :, ::-1].copy() if bgr else img.copy()


class FrameSelector:
    """按消息时间戳每 period 秒保留一帧（仿真时间：慢于实时也按仿真进度抽帧）."""

    def __init__(self, period):
        self.period = float(period)
        self.next_t = None

    def keep(self, t):
        """时间戳 t（秒）这一帧是否保留."""
        if self.next_t is not None and t < self.next_t - 10 * self.period:
            self.next_t = None                       # 时间回跳（仿真重置）：重新开始
        if self.next_t is None or t >= self.next_t - 1e-9:
            if self.next_t is None or t - self.next_t > self.period:
                self.next_t = t                      # 首帧或长时间没有图像：重新对齐
            self.next_t += self.period               # 否则按网格推进，抖动不累积
            return True
        return False


def list_frames(frames_dir, every=1, start=0, end=None):
    """帧目录下按序号排序的帧文件，可抽稀 / 截取."""
    files = sorted(glob.glob(os.path.join(frames_dir, FRAME_GLOB)))
    return files[start:end][::max(1, int(every))]


def _resize(img, width):
    if not width or img.width <= width:
        return img
    h = int(round(img.height * width / img.width))
    h -= h % 2
    from PIL import Image
    return img.resize((width, h), Image.LANCZOS)


def make_gif(files, out, fps=10.0, width=480):
    """帧文件 → GIF（PIL，自适应调色板）."""
    from PIL import Image
    if not files:
        raise ValueError('没有帧')
    frames = []
    for f in files:
        with Image.open(f) as im:
            frames.append(_resize(im.convert('RGB'), width).quantize(
                colors=128, method=Image.MEDIANCUT, dither=Image.NONE))
    frames[0].save(out, save_all=True, append_images=frames[1:],
                   duration=int(round(1000.0 / fps)), loop=0, optimize=True)
    return out


def ffmpeg_cmd(list_file, out, fps=10.0, width=640, crf=23, ffmpeg='ffmpeg'):
    """MP4 编码命令（concat 列表 → libx264，宽度 2 的倍数）."""
    vf = f'scale={int(width)}:-2' if width else 'scale=trunc(iw/2)*2:trunc(ih/2)*2'
    return [ffmpeg, '-y', '-loglevel', 'error', '-r', f'{fps:g}', '-f', 'concat', '-safe', '0',
            '-i', list_file, '-vf', vf, '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
            '-crf', str(int(crf)), '-movflags', '+faststart', out]


def concat_list(files, fps):
    """生成 ffmpeg concat 列表文本."""
    d = 1.0 / fps
    lines = []
    for f in files:
        lines += [f"file '{os.path.abspath(f)}'", f'duration {d:.6f}']
    if files:
        lines.append(f"file '{os.path.abspath(files[-1])}'")
    return '\n'.join(lines) + '\n'


def make_mp4(files, out, fps=10.0, width=640, crf=23):
    """帧文件 → MP4；没有 ffmpeg 时返回 None."""
    exe = shutil.which('ffmpeg')
    if exe is None:
        return None
    if not files:
        raise ValueError('没有帧')
    with tempfile.NamedTemporaryFile('w', suffix='.txt', delete=False) as tf:
        tf.write(concat_list(files, fps))
        lst = tf.name
    try:
        subprocess.run(ffmpeg_cmd(lst, out, fps, width, crf, exe), check=True)
    finally:
        os.unlink(lst)
    return out


def make_video(files, out, fps=10.0, width=None):
    """按扩展名合成；.mp4 无 ffmpeg 时退回同名 .gif。返回实际输出路径."""
    if out.lower().endswith('.mp4'):
        res = make_mp4(files, out, fps, width or 640)
        if res:
            return res
        out = out[:-4] + '.gif'
        print('未找到 ffmpeg，改为输出 GIF：' + out)
    return make_gif(files, out, fps, width or 480)


def main(argv=None):
    """命令行入口."""
    ap = argparse.ArgumentParser(description='帧序列 → GIF / MP4')
    ap.add_argument('frames_dir')
    ap.add_argument('out')
    ap.add_argument('--fps', type=float, default=10.0)
    ap.add_argument('--width', type=int, default=None)
    ap.add_argument('--every', type=int, default=1)
    ap.add_argument('--start', type=int, default=0)
    ap.add_argument('--end', type=int, default=None)
    a = ap.parse_args(argv)
    files = list_frames(a.frames_dir, a.every, a.start, a.end)
    if not files:
        print(f'{a.frames_dir} 下没有 {FRAME_GLOB}')
        return 1
    out = make_video(files, a.out, a.fps, a.width)
    print(f'{len(files)} 帧 → {out}（{os.path.getsize(out) / 1e6:.2f} MB）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
