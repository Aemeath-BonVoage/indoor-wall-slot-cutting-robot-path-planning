#!/usr/bin/env python3
"""
自定义切割控制 - 通过YAML配置文件指定切割任务

用法: ros2 run scr_arm_controller custom_cutting <config_file.yaml>
"""

import sys
import os
import math
import rclpy
import yaml
from scr_arm_controller.cartesian_cutting import CartesianCuttingController


def parse_rotation(rotation_str):
    """
    解析旋转角度，支持多种格式：

    - "90deg" 或 "90" -> 转换为弧度
    - "1.57rad" -> 直接使用弧度
    - 1.57 (数字) -> 直接使用弧度
    """
    if isinstance(rotation_str, (int, float)):
        return float(rotation_str)

    rotation_str = str(rotation_str).strip().lower()

    if rotation_str.endswith('deg'):
        # 角度格式
        degrees = float(rotation_str[:-3])
        return math.radians(degrees)
    elif rotation_str.endswith('rad'):
        # 弧度格式
        return float(rotation_str[:-3])
    else:
        # 纯数字，假设是弧度
        return float(rotation_str)


def load_config(config_file):
    """加载YAML配置文件"""
    if not os.path.exists(config_file):
        print(f"错误: 配置文件不存在: {config_file}")
        return None

    try:
        with open(config_file, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        return config
    except Exception as e:
        print(f"错误: 无法解析配置文件: {e}")
        return None


def validate_config(config):
    """验证配置文件格式"""
    if 'points' not in config:
        print("错误: 配置文件缺少 'points' 字段")
        return False

    if len(config['points']) < 2:
        print(f"错误: 至少需要2个点，但配置了{len(config['points'])}个")
        return False

    for i, point in enumerate(config['points']):
        if 'x' not in point or 'y' not in point or 'z' not in point:
            print(f"错误: 点{i+1}缺少x/y/z坐标")
            return False
        if 'rotation' not in point:
            print(f"错误: 点{i+1}缺少rotation角度")
            return False

    return True


def print_usage():
    """打印使用说明"""
    print("用法: ros2 run scr_arm_controller custom_cutting <config_file.yaml>")
    print("\n示例:")
    print("  # 使用预定义的配置文件")
    print("  ros2 run scr_arm_controller custom_cutting wall_cutting.yaml")
    print("  ros2 run scr_arm_controller custom_cutting horizontal_cutting.yaml")
    print("")
    print("  # 使用完整路径")
    print("  ros2 run scr_arm_controller custom_cutting /path/to/my_task.yaml")
    print("")
    print("配置文件格式:")
    print("  points:")
    print("    - {x: 0.537, y: -0.025, z: 0.482, rotation: 0}")
    print("    - {x: 0.537, y: 0.025, z: 0.482, rotation: 0}")
    print("    - {x: 0.537, y: 0.025, z: 0.532, rotation: 0}")
    print("    - {x: 0.537, y: -0.025, z: 0.532, rotation: 0}")
    print("  motion:")
    print("    duration: 3.0")
    print("    return_home: true")
    print("")
    print("注意: rotation 单位为度，控制 joint4 旋转角度")
    print("")
    print("工作空间范围（相对于base_link）:")
    print("  初始位置: X=0.537m, Y≈0m, Z=0.507m")
    print("  X范围: [0.492, 0.582] m")
    print("  Y范围: [-0.080, 0.060] m")
    print("  Z范围: [0.157, 0.707] m")


def main(args=None):
    # 检查命令行参数
    if len(sys.argv) < 2:
        print_usage()
        return

    config_file = sys.argv[1]

    # 如果是相对路径，尝试在多个位置查找
    if not os.path.isabs(config_file) and not os.path.exists(config_file):
        # 尝试1: 在源码config目录中查找
        src_config = os.path.join('/workspace/src/scr_arm_controller/config', config_file)
        if os.path.exists(src_config):
            config_file = src_config
        else:
            # 尝试2: 在安装目录中查找
            install_config = os.path.join(
                '/workspace/install/scr_arm_controller/share/scr_arm_controller/config',
                config_file
            )
            if os.path.exists(install_config):
                config_file = install_config
            else:
                # 尝试3: 使用ament_index查找（如果可用）
                try:
                    from ament_index_python.packages import get_package_share_directory
                    pkg_share = get_package_share_directory('scr_arm_controller')
                    ament_config = os.path.join(pkg_share, 'config', config_file)
                    if os.path.exists(ament_config):
                        config_file = ament_config
                except Exception:
                    pass

    # 加载配置
    config = load_config(config_file)
    if config is None:
        return

    # 验证配置
    if not validate_config(config):
        return

    # 解析配置
    points = [(p['x'], p['y'], p['z'], math.radians(p['rotation'])) for p in config['points']]
    duration = config.get('motion', {}).get('duration', 3.0)
    return_home = config.get('motion', {}).get('return_home', True)

    # 初始化ROS2
    rclpy.init(args=args)

    controller = CartesianCuttingController()

    # 显示工作空间信息
    controller.print_workspace_info()

    controller.get_logger().info('='*60)
    controller.get_logger().info('自定义切割任务')
    controller.get_logger().info('='*60)
    controller.get_logger().info(f'\n配置文件: {config_file}')
    controller.get_logger().info('\n输入的四个点:')
    for i, (x, y, z, rotation) in enumerate(points):
        controller.get_logger().info(f'  点{i+1}: X={x:.4f}, Y={y:.4f}, Z={z:.4f}, '
                                     f'R={math.degrees(rotation):.1f}°')
    controller.get_logger().info('\n运动参数:')
    controller.get_logger().info(f'  运动时长: {duration:.1f}s')
    controller.get_logger().info(f'  完成后回原点: {return_home}')

    # 执行切割（使用配置的参数）
    success = execute_cutting(controller, points, duration, return_home)

    if success:
        controller.get_logger().info('任务成功完成！')
    else:
        controller.get_logger().error('任务失败！')

    controller.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def execute_cutting(controller, points, duration, return_home):
    """使用配置的参数执行切割"""
    if len(points) < 2:
        controller.get_logger().error(f'至少需要2个点，但提供了{len(points)}个')
        return False

    controller.get_logger().info('='*60)
    controller.get_logger().info('开始轨迹执行任务')
    controller.get_logger().info('='*60)

    # 检查所有点是否可达
    controller.get_logger().info('\n检查所有点的可达性...')
    for i, (x, y, z, rotation) in enumerate(points):
        reachable, joints = controller.check_reachability(x, y, z, rotation)
        if not reachable:
            controller.get_logger().error(f'点{i+1} ({x}, {y}, {z}, '
                                          f'{math.degrees(rotation):.1f}°) 不可达！')
            return False
        controller.get_logger().info(f'点{i+1}: X={x:.4f}, Y={y:.4f}, Z={z:.4f}, '
                                     f'R={math.degrees(rotation):.1f}° ✓')

    controller.get_logger().info('\n所有点都可达，开始执行...\n')

    # 1. 回到初始位置（使用第一个点的旋转角度）
    first_rotation = points[0][3]
    controller.get_logger().info('步骤1: 回到初始位置')
    if not controller.move_to_joints([0.0, 0.0, 0.0, first_rotation], duration=duration):
        return False

    # 2. 依次移动到所有点
    for i, (x, y, z, rotation) in enumerate(points):
        controller.get_logger().info(f'\n步骤{i+2}: 移动到点{i+1}')
        if not controller.move_to_cartesian(x, y, z, rotation, duration=duration):
            return False

    # 3. 回到初始位置（可选）
    if return_home:
        controller.get_logger().info(f'\n步骤{len(points)+2}: 回到初始位置')
        if not controller.move_to_joints([0.0, 0.0, 0.0, first_rotation], duration=duration):
            return False
    else:
        controller.get_logger().info('\n跳过回到初始位置（配置: return_home=false）')

    controller.get_logger().info('\n' + '='*60)
    controller.get_logger().info('轨迹执行任务完成！')
    controller.get_logger().info('='*60)

    return True


if __name__ == '__main__':
    main()
