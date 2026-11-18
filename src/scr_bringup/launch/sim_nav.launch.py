"""一键启动：Gazebo 仿真 + Nav2 导航 + 任务层（技能服务器 + 行为树执行器）+ RViz.

用法:
  ros2 launch scr_bringup sim_nav.launch.py                    # 完整界面
  ros2 launch scr_bringup sim_nav.launch.py gui:=false         # 不开 Gazebo 窗口
  ros2 launch scr_bringup sim_nav.launch.py gui:=false rviz:=false   # 纯后台（最省内存）
  ros2 launch scr_bringup sim_nav.launch.py params_file:=/path/to/nav2_params_xxx.yaml
                                                               # 换用其他 Nav2 参数文件
  ros2 launch scr_bringup sim_nav.launch.py task:=false        # 不启动任务层（只做导航实验）
  ros2 launch scr_bringup sim_nav.launch.py localization:=wheel  # 只用轮式里程计（不融合 IMU）
  ros2 launch scr_bringup sim_nav.launch.py odom_error:=false     # 理想轮式里程计
  ros2 launch scr_bringup sim_nav.launch.py plant:=wheels        # 轮子驱动底盘（实验）
  ros2 launch scr_bringup sim_nav.launch.py base_control:=plugin # 早期力控插件（/odom 为真值）

任务层就绪后执行任务：
  ros2 run scr_arm_controller navigation_cutting_task task_at_target.yaml
  ros2 run scr_arm_controller navigation_cutting_task <plan.yaml> --dry-run-gap=0.01
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    desc_share = get_package_share_directory('scr_description')
    nav_share = get_package_share_directory('scr_navigation')
    arm_share = get_package_share_directory('scr_arm_controller')
    task_share = get_package_share_directory('scr_task')

    args = [
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start Gazebo GUI (gzclient)'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz'),
        DeclareLaunchArgument('rtf', default_value='1.0',
                              description='Max real-time factor (1.0 = real time, 0 = unlimited)'),
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
        DeclareLaunchArgument('base_control', default_value='ros2_control',
                              description='ros2_control | plugin (early force plugin, '
                                          'ground-truth odometry)'),
        DeclareLaunchArgument('plant', default_value='force',
                              description='ros2_control base plant: force | wheels'),
        DeclareLaunchArgument('odom_error', default_value='true',
                              description='Wheel encoder error model on (force plant)'),
        DeclareLaunchArgument('odom_seed', default_value='1',
                              description='Seed of the encoder slip process'),
        DeclareLaunchArgument('localization', default_value='ekf',
                              description='ekf (wheel odometry + IMU) | wheel'),
        DeclareLaunchArgument('use_composition', default_value='True',
                              description='Nav2 servers in one container (False: one process '
                                          'per server)'),
        DeclareLaunchArgument('task', default_value='true',
                              description='Start the task layer (skill servers + BT executor)'),
        DeclareLaunchArgument('alignment_params',
                              default_value=os.path.join(arm_share, 'config',
                                                         'fine_alignment.yaml'),
                              description='Fine-alignment parameters of the skill server '
                                          '(default: multi-tag map + online lateral odometry '
                                          'scale, see docs/experiments/along_wall_accuracy)'),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(desc_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'rtf': LaunchConfiguration('rtf'),
            'base_control': LaunchConfiguration('base_control'),
            'plant': LaunchConfiguration('plant'),
            'odom_error': LaunchConfiguration('odom_error'),
            'odom_seed': LaunchConfiguration('odom_seed'),
            'localization': LaunchConfiguration('localization'),
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'rviz': LaunchConfiguration('rviz'),
            'use_sim_time': 'true',
            'use_composition': LaunchConfiguration('use_composition'),
        }.items(),
    )

    # 等 Gazebo 起来、机器人生成并发布 /clock 与 TF 后再启动导航
    delayed_navigation = TimerAction(period=LaunchConfiguration('nav_delay'),
                                     actions=[navigation])

    # 任务层：技能服务器需要机械臂控制器（Gazebo 起来后约 5 s 加载），执行器自己等待
    # Nav2 激活与技能服务器就绪后再加载行为树
    task = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(task_share, 'launch', 'task.launch.py')),
        launch_arguments={
            'alignment_params': LaunchConfiguration('alignment_params'),
            # 技能的局部闭环：融合定位时用轮式里程计（见 scr_arm_controller/task_skills.py）
            'odom_topic': PythonExpression([
                "'/wheel/odom' if '", LaunchConfiguration('base_control'),
                "' == 'ros2_control' and '", LaunchConfiguration('localization'),
                "' == 'ekf' else '/odom'"]),
        }.items(),
        condition=IfCondition(LaunchConfiguration('task')),
    )
    delayed_task = TimerAction(period=LaunchConfiguration('nav_delay'), actions=[task])

    return LaunchDescription(args + [gazebo, delayed_navigation, delayed_task])
