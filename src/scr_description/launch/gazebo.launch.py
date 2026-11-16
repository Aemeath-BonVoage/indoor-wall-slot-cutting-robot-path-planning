import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    robot_name_in_model = "scr_robot_gazebo"
    urdf_tutorial_path = get_package_share_directory('scr_description')
    default_model_path = os.path.join(
        urdf_tutorial_path, 'urdf', 'scr_robot_gazebo.urdf')
    
    # 默认世界文件
    default_world_path = os.path.join(
        urdf_tutorial_path, 'worlds', 'room.world')

    # Read URDF file content
    with open(default_model_path, 'r') as urdf_file:
        robot_description = urdf_file.read()
    
    # 替换 URDF 中的 $(find scr_description) 为实际路径
    robot_description = robot_description.replace(
        '$(find scr_description)', 
        urdf_tutorial_path
    )

    # 声明world参数
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='Gazebo world file'
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # 直接启动 Gazebo，但需要加载 ROS 2 插件
    # 使用环境变量加载必要的 Gazebo ROS 插件
    
    # 设置 Gazebo 插件路径
    gazebo_plugin_path = os.path.join(get_package_prefix('gazebo_ros'), 'lib')
    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + os.pathsep + gazebo_plugin_path
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = gazebo_plugin_path
    
    # 启动 gzserver，加载 ROS 工厂插件
    gzserver = ExecuteProcess(
        cmd=['gzserver', 
             '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             LaunchConfiguration('world')],
        output='screen'
    )
    
    # 启动 gzclient（不带 EOL GUI 插件）
    gzclient = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen'
    )

    # Spawn robot
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name_in_model])

    # Set GAZEBO_MODEL_PATH
    package_name = 'scr_description'
    pkg_share = os.pathsep + os.path.join(get_package_prefix(package_name), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += pkg_share
    else:
        os.environ['GAZEBO_MODEL_PATH'] = "/usr/share/gazebo-11/models" + pkg_share

    # 麦克纳姆轮控制器
    # Python 脚本安装在 lib 目录下，不是 share 目录
    mecanum_controller_script = os.path.join(
        get_package_prefix('scr_description'), 'lib', 'scr_description', 'mecanum_controller.py')
    
    mecanum_controller_node = ExecuteProcess(
        cmd=['python3', mecanum_controller_script],
        output='screen'
    )

    # 启动 joint_state_broadcaster
    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # 启动 arm_controller
    arm_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )

    # 启动 wheel_controller
    wheel_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_controller'],
        output='screen'
    )

    # 延迟启动控制器
    delayed_controllers = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner, arm_controller_spawner, wheel_controller_spawner]
    )

    return launch.LaunchDescription([
        declare_world,
        robot_state_publisher_node,
        gzserver,
        gzclient,
        spawn_entity_node,
        mecanum_controller_node,
        delayed_controllers
    ])

