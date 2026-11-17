import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction, ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
import math
import os
import re
import tempfile


def inject_camera_yaw_error(urdf, yaw_deg):
    """
    在 Gazebo 相机上叠加绕竖直轴的安装偏角（实验用，模拟相机外参误差）.

    改的是生成到 Gazebo 的描述中 camera_joint 的 origin；robot_state_publisher 仍用名义
    URDF，发布的 TF 不变，即软件“不知道”这个误差——与真机上外参标定不准的情形相同。
    不能用传感器的 <pose>：URDF → SDF 合并固定关节（camera_link 并入 link1）时，
    传感器位姿被关节偏移整体覆盖，<pose> 中的旋转会丢失（实测相机朝向不变）。
    """
    pat = (r'(<joint name="camera_joint" type="fixed">\s*<parent link="link1"/>\s*'
           r'<child link="camera_link"/>\s*<origin xyz="[^"]*" rpy=")0 0 0(")')
    out, n = re.subn(pat, r'\g<1>0 0 ' + f'{math.radians(yaw_deg):.8f}' + r'\g<2>', urdf)
    if n != 1:
        raise RuntimeError('URDF 中找不到 camera_joint，无法注入相机安装偏角')
    return out


def spawn_robot(context, entity, robot_description):
    """生成机器人：camera_yaw_error_deg 非 0 时用注入了相机偏角的描述生成（TF 不变）."""
    yaw = float(context.perform_substitution(LaunchConfiguration('camera_yaw_error_deg')))
    args = ['-topic', '/robot_description', '-entity', entity]
    if yaw != 0.0:
        f = tempfile.NamedTemporaryFile('w', suffix='.urdf', prefix='scr_robot_', delete=False)
        f.write(inject_camera_yaw_error(robot_description, yaw))
        f.close()
        args = ['-file', f.name, '-entity', entity]
    return [launch_ros.actions.Node(package='gazebo_ros', executable='spawn_entity.py',
                                    arguments=args)]


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
    # 本包的 Gazebo 插件（libscr_planar_move.so）
    gazebo_plugin_path += os.pathsep + os.path.join(get_package_prefix('scr_description'), 'lib')
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
    
    # gui:=false 时不启动 Gazebo 窗口（gzclient），可显著降低内存/显卡占用
    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI (gzclient)'
    )

    # rtf：仿真速度上限（相对实时的倍数）。1.0 = 实时（默认）；2.5 = 最快 2.5 倍；0 = 不限速。
    # 控制器均按仿真时钟计时，因此加速不改变控制律；但感知 / 控制代码的计算延迟按仿真时间
    # 计会放大 rtf 倍，批量实验前应先与 rtf=1 对比确认结果一致。
    declare_rtf = DeclareLaunchArgument(
        'rtf',
        default_value='1.0',
        description='Max real-time factor (1.0 = real time, 0 = unlimited)'
    )
    unthrottle = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=['gz', 'physics', '-u',
                 PythonExpression(['str(int(1000 * float("', LaunchConfiguration('rtf'), '")))'])],
            output='screen')],
        condition=IfCondition(PythonExpression(['float("', LaunchConfiguration('rtf'), '") != 1.0']))
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # 相机安装偏角（度，绕竖直轴）：0 = 名义安装（默认）；用于验证零位标定
    declare_camera_yaw = DeclareLaunchArgument(
        'camera_yaw_error_deg',
        default_value='0.0',
        description='Camera mount yaw error injected into the Gazebo sensor only (deg)'
    )

    # Spawn robot
    spawn_entity_node = OpaqueFunction(
        function=spawn_robot, args=[robot_name_in_model, robot_description])

    # Set GAZEBO_MODEL_PATH
    package_name = 'scr_description'
    pkg_share = os.pathsep + os.path.join(get_package_prefix(package_name), 'share')
    # 本包的 Gazebo 模型（AprilTag 定位标记，model://scr_apriltag_*）
    pkg_share += os.pathsep + os.path.join(urdf_tutorial_path, 'models')
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
        declare_gui,
        declare_rtf,
        declare_camera_yaw,
        unthrottle,
        robot_state_publisher_node,
        gzserver,
        gzclient,
        spawn_entity_node,
        mecanum_controller_node,
        delayed_controllers
    ])

