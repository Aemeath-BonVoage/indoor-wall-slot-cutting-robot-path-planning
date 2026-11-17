# -*- coding: utf-8 -*-
"""
墙面开槽规划可视化：plan_viz 节点 + RViz（可选静态 TF，离线查看）.

  ros2 launch scr_viz plan_viz.launch.py [task_file:=...] [plan_file:=...]
      [rviz:=true] [static_tf:=false] [use_sim_time:=false]

- 离线（没有 Gazebo / 定位）：static_tf:=true 发布 map → odom 单位变换，RViz 固定坐标系 map；
- 与仿真一起用：use_sim_time:=true，刀尖轨迹自动从 /gazebo/link_states 推算。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('scr_viz')
    cfg = os.path.join(share, 'config')
    args = [
        DeclareLaunchArgument('task_file',
                              default_value=os.path.join(cfg, 'wall_task_room3_south_demo.yaml')),
        DeclareLaunchArgument('plan_file',
                              default_value=os.path.join(cfg, 'plan_room3_south_demo.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config',
                              default_value=os.path.join(share, 'rviz', 'scr_viz.rviz')),
        DeclareLaunchArgument('static_tf', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_link_states', default_value='true'),
        DeclareLaunchArgument('ascii_labels', default_value='true'),
    ]
    sim = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    viz = Node(package='scr_viz', executable='plan_viz', name='scr_plan_viz', output='screen',
               parameters=[{'task_file': LaunchConfiguration('task_file'),
                            'plan_file': LaunchConfiguration('plan_file'),
                            'use_link_states': LaunchConfiguration('use_link_states'),
                            'ascii_labels': LaunchConfiguration('ascii_labels')}, sim])
    tf = Node(package='tf2_ros', executable='static_transform_publisher', name='scr_viz_map_odom',
              arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
              condition=IfCondition(LaunchConfiguration('static_tf')), parameters=[sim])
    rviz = Node(package='rviz2', executable='rviz2', name='scr_viz_rviz', output='log',
                arguments=['-d', LaunchConfiguration('rviz_config')], parameters=[sim],
                condition=IfCondition(LaunchConfiguration('rviz')))
    return LaunchDescription(args + [viz, tf, rviz])
