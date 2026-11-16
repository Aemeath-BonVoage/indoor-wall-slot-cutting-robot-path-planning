import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    pkg_path = get_package_share_directory('scr_description')
    
    # URDF文件路径
    urdf_file = os.path.join(pkg_path, 'urdf', 'scr_robot.urdf')
    
    # 读取URDF文件内容
    with open(urdf_file, 'r', encoding='utf-8') as infp:
        robot_desc = infp.read()
    
    # RViz配置文件路径
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'scr_view.rviz')
    
    # 声明launch参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Robot State Publisher节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )
    
    # Joint State Publisher GUI节点
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz2节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
