import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('scr_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    rviz_config = os.path.join(pkg_share, 'config', 'nav2_simple.rviz')
    
    # 初始位姿参数
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_map_file = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_share, 'maps', 'map.yaml'),
        description='Full path to map yaml file'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Full path to nav2 params file'
    )
    
    declare_initial_pose_x = DeclareLaunchArgument(
        'initial_pose_x',
        default_value='0.0',
        description='Initial pose X coordinate'
    )
    
    declare_initial_pose_y = DeclareLaunchArgument(
        'initial_pose_y',
        default_value='0.0',
        description='Initial pose Y coordinate'
    )
    
    declare_initial_pose_yaw = DeclareLaunchArgument(
        'initial_pose_yaw',
        default_value='0.0',
        description='Initial pose yaw angle (radians)'
    )

    # Nav2 bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': params_file
        }.items()
    )

    # RViz with navigation config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 自动设置初始位姿节点
    initial_pose_node = Node(
        package='scr_navigation',
        executable='set_initial_pose.py',
        name='initial_pose_setter',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'x': initial_pose_x},
            {'y': initial_pose_y},
            {'yaw': initial_pose_yaw}
        ],
        output='screen'
    )
    
    # 延迟启动初始位姿设置（等待Nav2和TF完全启动）
    delayed_initial_pose = TimerAction(
        period=6.0,
        actions=[initial_pose_node]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_file,
        declare_params_file,
        declare_initial_pose_x,
        declare_initial_pose_y,
        declare_initial_pose_yaw,
        nav2_bringup,
        rviz_node,
        delayed_initial_pose
    ])
