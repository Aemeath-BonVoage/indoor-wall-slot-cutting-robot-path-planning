# -*- coding: utf-8 -*-
"""
把墙面任务里的线盒 / 插座 / 开关 / 配电箱生成为贴墙薄盒 SDF，并可在 Gazebo 运行时生成.

  ros2 run scr_viz spawn_wall_fixtures <task.yaml> [--out DIR] [--spawn] [--delete]
      [--collision] [--no-endpoints] [--skip-label 定位标记 ...]

  --out DIR      把每个实体写成 DIR/<name>.sdf，并写 DIR/fixtures.yaml 清单
  --spawn        调 /spawn_entity 生成（同名实体已存在时先删除）
  --delete       调 /delete_entity 删除本任务对应的全部实体
  --collision    带 collision（默认只有 visual：激光与机械臂都不会碰到它）
  --no-endpoints 不生成开槽任务的起点 / 终点线盒，只生成 obstacles 中的 box

不修改 room.world；不带 --spawn/--delete 时完全离线。
"""
import argparse
import os
import sys

import yaml

from scr_viz import sdf_gen
from scr_viz import wall_model as wm


def build(task, collision=False, include_endpoints=True,
          skip_labels=sdf_gen.DEFAULT_SKIP_LABELS):
    """墙面任务 dict → [(Fixture, sdf_xml)]."""
    w = task['wall']
    wall = wm.Wall(tuple(w['left_end']), tuple(w['right_end']), float(w.get('height', 2.0)))
    fx = sdf_gen.fixtures_from_task(task, skip_labels, include_endpoints)
    return [(f, sdf_gen.fixture_sdf(wall, f, collision)) for f in fx], wall


def write_out(items, wall, out_dir):
    """写 SDF 文件和清单."""
    os.makedirs(out_dir, exist_ok=True)
    manifest = []
    for f, xml in items:
        path = os.path.join(out_dir, f'{f.name}.sdf')
        with open(path, 'w', encoding='utf-8') as fp:
            fp.write(xml)
        x, y, z, _, _, yaw = sdf_gen.fixture_pose(wall, f)
        manifest.append(dict(name=f.name, kind=f.kind, label=f.label, file=os.path.basename(path),
                             wall_uv=[round(f.u, 4), round(f.v, 4)],
                             size=[f.w, f.thickness, f.h],
                             pose=[round(x, 4), round(y, 4), round(z, 4), round(yaw, 4)]))
    with open(os.path.join(out_dir, 'fixtures.yaml'), 'w', encoding='utf-8') as fp:
        yaml.safe_dump(dict(fixtures=manifest), fp, allow_unicode=True, sort_keys=False)
    return manifest


def main(argv=None):
    """命令行入口."""
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('task')
    ap.add_argument('--out')
    ap.add_argument('--spawn', action='store_true')
    ap.add_argument('--delete', action='store_true')
    ap.add_argument('--collision', action='store_true')
    ap.add_argument('--no-endpoints', action='store_true')
    ap.add_argument('--skip-label', nargs='*', default=list(sdf_gen.DEFAULT_SKIP_LABELS))
    ap.add_argument('--timeout', type=float, default=10.0)
    args, ros_args = ap.parse_known_args(argv)

    task = wm.load_yaml(args.task)
    items, wall = build(task, args.collision, not args.no_endpoints, tuple(args.skip_label))
    for f, _ in items:
        x, y, z, *_ = sdf_gen.fixture_pose(wall, f)
        print(f'{f.name:40s} {f.kind:13s} u={f.u:.3f} v={f.v:.3f} → '
              f'({x:.3f}, {y:.3f}, {z:.3f})  {f.label}')
    if args.out:
        write_out(items, wall, args.out)
        print(f'已写出 {len(items)} 个 SDF 到 {args.out}')
    if not (args.spawn or args.delete):
        return 0

    import rclpy
    from scr_viz import gazebo_io
    rclpy.init(args=ros_args)
    node = rclpy.create_node('scr_wall_fixtures')
    ok = True
    try:
        for f, xml in items:
            if args.delete or args.spawn:
                gazebo_io.delete(node, f.name, args.timeout)   # 不存在时返回失败，忽略
            if args.spawn:
                s, msg = gazebo_io.spawn(node, f.name, xml, args.timeout)
                ok &= s
                print(f'spawn {f.name}: {"OK" if s else "失败 " + msg}')
        if args.delete and not args.spawn:
            print(f'已删除 {len(items)} 个实体')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
