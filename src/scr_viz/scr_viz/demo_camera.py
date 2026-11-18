# -*- coding: utf-8 -*-
"""
演示相机：运行时在 Gazebo 里放一个固定视角相机，订阅图像存帧，结束后合成 GIF / MP4.

Wayland 下无法截屏，用 Gazebo 自己渲染的相机图像代替录屏。

  ros2 run scr_viz demo_record --out DIR [--preset oblique|front|side|top]
      [--eye X Y Z --target X Y Z] [--rate 2] [--duration 300] [--max-frames N]
      [--width 640 --height 360 --cam-fps 10] [--video demo.mp4 --video-fps 12]
      [--keep-camera] [--no-spawn --topic /<name>/<name>/image_raw] [--no-stamp]

- --rate：每仿真秒存几帧（按图像时间戳抽帧，仿真慢于实时也不影响节奏）；
- --duration：最长录制的墙钟秒数；Ctrl-C 随时结束；
- 结束时删除相机模型（--keep-camera 保留），给了 --video 时直接合成。
帧存为 DIR/frame_00000.jpg …，时间戳写在 DIR/frames.csv。
"""
import argparse
import csv
import os
import signal
import sys
import time

from scr_viz import sdf_gen
from scr_viz import video


def stamp_overlay(img, text):
    """在左上角画时间戳（PIL 默认字体，只用 ASCII）."""
    from PIL import ImageDraw
    d = ImageDraw.Draw(img)
    x, y = 8, 6
    d.rectangle((x - 4, y - 2, x + 7 * len(text) + 4, y + 14), fill=(0, 0, 0))
    d.text((x, y), text, fill=(255, 255, 255))
    return img


def parse_args(argv):
    """命令行参数."""
    ap = argparse.ArgumentParser(description='Gazebo 演示相机录制')
    ap.add_argument('--out', required=True)
    ap.add_argument('--name', default='demo_camera')
    ap.add_argument('--preset', default='oblique', choices=sorted(sdf_gen.CAMERA_PRESETS))
    ap.add_argument('--eye', type=float, nargs=3)
    ap.add_argument('--target', type=float, nargs=3)
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=360)
    ap.add_argument('--hfov', type=float, default=1.2)
    ap.add_argument('--cam-fps', type=float, default=10.0)
    ap.add_argument('--rate', type=float, default=2.0)
    ap.add_argument('--duration', type=float, default=600.0)
    ap.add_argument('--max-frames', type=int, default=0)
    ap.add_argument('--quality', type=int, default=90)
    ap.add_argument('--video')
    ap.add_argument('--video-fps', type=float, default=12.0)
    ap.add_argument('--keep-camera', action='store_true')
    ap.add_argument('--no-spawn', action='store_true')
    ap.add_argument('--topic')
    ap.add_argument('--no-stamp', action='store_true')
    ap.add_argument('--first-image-timeout', type=float, default=30.0)
    return ap.parse_known_args(argv)


def main(argv=None):
    """命令行入口."""
    a, ros_args = parse_args(argv)
    import rclpy
    from PIL import Image as PILImage
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Image

    from scr_viz import gazebo_io

    eye, target = sdf_gen.CAMERA_PRESETS[a.preset]
    eye, target = tuple(a.eye or eye), tuple(a.target or target)
    os.makedirs(a.out, exist_ok=True)
    # 自己处理 Ctrl-C / SIGTERM：先停止录制、删除相机，再关 rclpy
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('scr_demo_camera')
    log = node.get_logger()
    spawned = False
    stop = {'flag': False}

    def on_term(*_):
        stop['flag'] = True
    signal.signal(signal.SIGTERM, on_term)
    signal.signal(signal.SIGINT, on_term)

    sel = video.FrameSelector(1.0 / a.rate)
    count = {'n': 0, 'seen': 0}
    csv_f = open(os.path.join(a.out, 'frames.csv'), 'w', newline='')
    wr = csv.writer(csv_f)
    wr.writerow(['frame', 'stamp_s'])

    def on_image(msg):
        count['seen'] += 1
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not sel.keep(t):
            return
        try:
            rgb = video.image_to_rgb(msg.data, msg.width, msg.height, msg.encoding, msg.step)
        except ValueError as e:
            log.warn(str(e))
            return
        img = PILImage.fromarray(rgb)
        if not a.no_stamp:
            stamp_overlay(img, f't = {t:7.1f} s')
        k = count['n']
        img.save(os.path.join(a.out, f'frame_{k:05d}.jpg'), quality=a.quality)
        wr.writerow([k, f'{t:.3f}'])
        count['n'] += 1

    try:
        topic = a.topic or sdf_gen.camera_topic(a.name)
        if not a.no_spawn:
            xml = sdf_gen.camera_sdf(a.name, eye, target, a.width, a.height, a.cam_fps, a.hfov)
            with open(os.path.join(a.out, f'{a.name}.sdf'), 'w') as f:
                f.write(xml)
            gazebo_io.delete(node, a.name)        # 上次残留
            ok, msg = gazebo_io.spawn(node, a.name, xml)
            if not ok:
                log.error(f'相机生成失败：{msg}')
                return 1
            spawned = True
            log.info(f'相机 {a.name} 已生成：eye={eye} target={target}')
        node.create_subscription(Image, topic, on_image, qos_profile_sensor_data)
        log.info(f'订阅 {topic}，每仿真秒 {a.rate:g} 帧，存到 {a.out}')
        t0 = time.monotonic()
        while rclpy.ok() and not stop['flag']:
            rclpy.spin_once(node, timeout_sec=0.1)
            el = time.monotonic() - t0
            if count['seen'] == 0 and el > a.first_image_timeout:
                log.error(f'{a.first_image_timeout:g} s 内没有收到图像（话题 {topic}）')
                break
            if el > a.duration or (a.max_frames and count['n'] >= a.max_frames):
                break
    except KeyboardInterrupt:
        pass
    finally:
        csv_f.close()
        if spawned and not a.keep_camera and rclpy.ok():
            try:
                gazebo_io.delete(node, a.name, 5.0)
            except Exception as e:  # noqa: BLE001
                log.warn(f'删除相机失败：{e}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f'共存 {count["n"]} 帧（收到 {count["seen"]} 帧）→ {a.out}')
    if a.video and count['n']:
        out = video.make_video(video.list_frames(a.out), a.video, a.video_fps)
        print(f'已合成 {out}')
    return 0 if count['n'] else 1


if __name__ == '__main__':
    sys.exit(main())
