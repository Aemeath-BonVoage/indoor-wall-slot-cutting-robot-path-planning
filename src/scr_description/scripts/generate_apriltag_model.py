#!/usr/bin/env python3
"""
生成 Gazebo 用的 AprilTag 定位标记模型（贴图由 OpenCV 生成）.

  python3 generate_apriltag_model.py [--id 0] [--size 0.10] [--out ../models]

输出 models/scr_apriltag_<family>_<id>/：model.config、model.sdf、Ogre 材质脚本与 PNG 贴图。
size 为黑色外框的边长（米，即检测与 PnP 使用的标记边长）；贴图四周再留 1 格白边，
整块板的边长 = size × 10 / 8。板厚 0.5 mm，模型原点在板的背面中心，摆放时让背面贴墙。

贴图朝向：Gazebo 盒子 +z 面的贴图 u 沿模型 +x、v 沿模型 −y。世界文件中模型姿态取
roll = π/2、yaw = π（贴在法向 +y 的墙面上）时，从房间内看图案为正、不镜像。
"""
import argparse
import os

import cv2
import numpy as np

FAMILIES = {'tag36h11': cv2.aruco.DICT_APRILTAG_36h11}


def tag_image(family, tag_id, cell_px):
    d = cv2.aruco.getPredefinedDictionary(FAMILIES[family])
    side = (d.markerSize + 2) * cell_px
    if hasattr(cv2.aruco, 'generateImageMarker'):
        img = cv2.aruco.generateImageMarker(d, tag_id, side, borderBits=1)
    else:
        img = cv2.aruco.drawMarker(d, tag_id, side, borderBits=1)
    return cv2.copyMakeBorder(img, cell_px, cell_px, cell_px, cell_px,
                              cv2.BORDER_CONSTANT, value=255)


MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description>AprilTag {family} id {tag_id}, black square {size_mm:.0f} mm (texture generated
  with OpenCV by scr_description/scripts/generate_apriltag_model.py).</description>
</model>
"""

MODEL_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <pose>0 0 {half_t} 0 0 0</pose>
        <geometry><box><size>{plate} {plate} {thick}</size></box></geometry>
        <material>
          <script>
            <uri>model://{name}/materials/scripts</uri>
            <uri>model://{name}/materials/textures</uri>
            <name>{material}</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

MATERIAL = """material {material}
{{
  technique
  {{
    pass
    {{
      ambient 1 1 1 1
      diffuse 1 1 1 1
      specular 0 0 0 1 1
      texture_unit
      {{
        texture {texture}
        filtering trilinear
      }}
    }}
  }}
}}
"""


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('--family', default='tag36h11', choices=sorted(FAMILIES))
    ap.add_argument('--id', type=int, default=0)
    ap.add_argument('--size', type=float, default=0.10, help='黑色外框边长（米）')
    ap.add_argument('--cell-px', type=int, default=64, help='贴图中每格的像素数')
    ap.add_argument('--out', default=os.path.join(here, '..', 'models'))
    args = ap.parse_args()

    name = f'scr_apriltag_{args.family}_{args.id}'
    root = os.path.join(args.out, name)
    os.makedirs(os.path.join(root, 'materials', 'scripts'), exist_ok=True)
    os.makedirs(os.path.join(root, 'materials', 'textures'), exist_ok=True)
    texture = f'{args.family}_{args.id}.png'
    material = f'ScrAprilTag/{args.family}_{args.id}'
    img = tag_image(args.family, args.id, args.cell_px)
    cv2.imwrite(os.path.join(root, 'materials', 'textures', texture), img)
    cells = img.shape[0] // args.cell_px
    plate = args.size * cells / (cells - 2)
    thick = 0.0005
    with open(os.path.join(root, 'model.config'), 'w') as f:
        f.write(MODEL_CONFIG.format(name=name, family=args.family, tag_id=args.id,
                                    size_mm=args.size * 1000))
    with open(os.path.join(root, 'model.sdf'), 'w') as f:
        f.write(MODEL_SDF.format(name=name, plate=f'{plate:.6f}', thick=thick,
                                 half_t=thick / 2, material=material))
    with open(os.path.join(root, 'materials', 'scripts', 'apriltag.material'), 'w') as f:
        f.write(MATERIAL.format(material=material, texture=texture))
    print(f'{root}: 贴图 {img.shape[1]}×{img.shape[0]} px，板边长 {plate * 1000:.1f} mm，'
          f'黑框 {args.size * 1000:.1f} mm，均值灰度 {np.mean(img):.0f}')


if __name__ == '__main__':
    main()
