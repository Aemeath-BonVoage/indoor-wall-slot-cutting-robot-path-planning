"""一键启动：Gazebo 仿真 + Nav2 导航 + RViz.

用法:
  ros2 launch scr_bringup sim_nav.launch.py                    # 完整界面
  ros2 launch scr_bringup sim_nav.launch.py gui:=false         # 不开 Gazebo 窗口
  ros2 launch scr_bringup sim_nav.launch.py gui:=false rviz:=false   # 纯后台（最省内存）
  ros2 launch scr_bringup sim_nav.launch.py \
      params_file:=$(ros2 pkg prefix scr_navigation)/share/scr_navigation/config/nav2_params_scr_planner.yaml
                                                               # 换用 C++ 增强 A* 全局规划器
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    desc_share = get_package_share_directory('scr_description')
    nav_share = get_package_share_directory('scr_navigation')

    args = [
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start Gazebo GUI (gzclient)'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz'),
        DeclareLaunchArgument('world',
                              default_value=os.path.join(desc_share, 'worlds', 'room.world'),
                              description='Gazebo world file'),
        DeclareLaunchArgument('map',
                              default_value=os.path.join(nav_share, 'maps', 'my_map.yaml'),
                              description='Map yaml for Nav2'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(nav_share, 'config', 'nav2_params.yaml'),
                              description='Nav2 params file (e.g. nav2_params_scr_planner.yaml)'),
        DeclareLaunchArgument('nav_delay', default_value='8.0',
                              description='Seconds to wait for Gazebo before starting Nav2'),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(desc_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'rviz': LaunchConfiguration('rviz'),
            'use_sim_time': 'true',
        }.items(),
    )

    # 等 Gazebo 起来、机器人生成并发布 /clock 与 TF 后再启动导航
    delayed_navigation = TimerAction(period=LaunchConfiguration('nav_delay'),
                                     actions=[navigation])

    return LaunchDescription(args + [gazebo, delayed_navigation])
