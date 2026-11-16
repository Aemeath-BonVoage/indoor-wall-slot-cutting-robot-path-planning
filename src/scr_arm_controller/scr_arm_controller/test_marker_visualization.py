#!/usr/bin/env python3
"""
测试Gazebo轨迹可视化
单独测试红色边框在Gazebo中的显示效果
"""

import rclpy
from scr_arm_controller.trajectory_visualizer import TrajectoryVisualizer


def main(args=None):
    rclpy.init(args=args)

    print('\n' + '='*60)
    print('测试Gazebo轨迹可视化')
    print('='*60)
    print('\n前提条件:')
    print('1. 确保Gazebo正在运行:')
    print('   ros2 launch scr_description gazebo.launch.py')
    print('\n测试内容:')
    print('- 在黄色方块周围生成红色边框')
    print('- 边框位置：方块外扩1cm')
    print('- 边框尺寸：7cm × 7cm')
    print('='*60 + '\n')

    # 创建可视化器
    visualizer = TrajectoryVisualizer()

    # 方块位置（从world文件）
    marker_x = 8.42
    marker_y = -3.9
    marker_z = 0.6

    # 方块大小：5cm x 5cm
    half_size = 0.025
    offset = 0.01  # 外扩1cm
    size_with_offset = half_size + offset

    # 生成边框的4个顶点（X-Z平面，贴在墙上）
    points = [
        (marker_x - size_with_offset, marker_y, marker_z - size_with_offset, 0),  # 左下
        (marker_x + size_with_offset, marker_y, marker_z - size_with_offset, 0),  # 右下
        (marker_x + size_with_offset, marker_y, marker_z + size_with_offset, 0),  # 右上
        (marker_x - size_with_offset, marker_y, marker_z + size_with_offset, 0),  # 左上
    ]

    print('生成红色边框...')
    print(f'方块位置: X={marker_x:.2f}, Y={marker_y:.2f}, Z={marker_z:.2f}')
    print(f'边框尺寸: {(size_with_offset*2)*100:.1f}cm × {(size_with_offset*2)*100:.1f}cm')
    print(f'外扩距离: {offset*100:.1f}cm\n')

    # 在Gazebo中生成红色边框
    visualizer.visualize_in_map_frame(points, color='red', frame_id='map')

    print('\n' + '='*60)
    print('✅ 边框已生成！')
    print('='*60)
    print('\n查看结果:')
    print('1. 在Gazebo窗口中查看南墙（Y=-3.9）')
    print('2. 应该看到黄色方块周围有红色边框')
    print('3. 边框由4条红色线段组成')
    print('\n清除边框:')
    print('- 重启Gazebo')
    print('- 或运行: visualizer.clear_markers()')
    print('\n模型名称:')
    print('- cutting_border_bottom')
    print('- cutting_border_right')
    print('- cutting_border_top')
    print('- cutting_border_left')
    print('\n查看模型列表:')
    print('gz model --list')
    print('='*60 + '\n')

    # 保持节点运行一小段时间以确保服务调用完成
    print('等待服务调用完成...')
    for _ in range(10):
        rclpy.spin_once(visualizer, timeout_sec=0.5)

    print('测试完成！\n')

    visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
