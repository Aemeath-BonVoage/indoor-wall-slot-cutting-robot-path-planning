"""Gazebo 仿真：机器人模型、控制器与（可选）底盘定位融合.

底盘的两种实现（base_control）：
  ros2_control（默认）  仿真硬件接口 scr_base_control/ScrMecanumBaseSystem + 麦克纳姆控制器
                        base_controller（cmd_vel → 轮速，轮速 → 轮式里程计）。
                        plant:=force   底盘由力 / 力矩驱动（与 scr_planar_move 相同），编码器
                                       读数由真实运动加误差模型得到（odom_error 控制误差开关）；
                        plant:=wheels  轮子电机驱动，底盘只受插件计算的轮–地接触力（辊子轴向
                                       滑移摩擦），编码器为轮子实际转速。
  plugin                早期实现：scr_planar_move 力控插件 + mecanum_controller.py，/odom 为
                        真值位姿（仿真配置 v2，保留用于复现早期结果）。

定位（localization，仅 ros2_control）：
  ekf（默认）  轮式里程计 /wheel/odom + 陀螺 /imu（底盘硬件接口模拟）→ robot_localization
               → /odom 与 TF
  wheel        控制器直接发布 /odom 与 TF（只用轮式里程计）

真值始终发布在 /ground_truth/odom（gazebo_ros_p3d）与 /gazebo/model_states，只用于评估。

切割力（cutting_force，默认 false）：为 true 时在模型中加入 libscr_cutting_force.so，按切割力
模型在刀具上施加力旋量（见 src/scr_cutting_force.cpp）；cutting_force_params 以
"键=值;键=值" 的形式给出插件参数（例如 "specific_energy=1.5e9;apply_to=base"）。
base_max_accel / base_max_ang_accel 非空时改写 ros2_control 底盘（force）的驱动力 / 力矩上限。
"""
import math
import os
import re
import tempfile

import launch
import launch_ros
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

ENTITY = 'scr_robot_gazebo'
ODOM_ERROR_PARAMS = ('wheel_scale_errors', 'slip_sigma', 'lateral_slip', 'yaw_slip')


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


def select_base_control(urdf, mode):
    """保留 base_control=mode 的块，删除其他实现的块."""
    def keep(m):
        return m.group(2) if m.group(1) == mode else ''
    pat = (r'[ \t]*<!-- BEGIN base_control=(\w+) -->\n(.*?)'
           r'[ \t]*<!-- END base_control=\1 -->\n?')
    out = re.sub(pat, keep, urdf, flags=re.S)
    if 'base_control=' in out:
        raise RuntimeError('URDF 中的 base_control 标记不成对')
    return out


def set_hw_param(urdf, name, value):
    out, n = re.subn(rf'(<param name="{name}">)[^<]*(</param>)', rf'\g<1>{value}\g<2>', urdf)
    if n != 1:
        raise RuntimeError(f'URDF 中找不到硬件参数 {name}')
    return out


def configure_base(urdf, plant, odom_error, seed):
    """ros2_control 底盘：被控对象与编码器误差模型（只作用于 force）."""
    urdf = set_hw_param(urdf, 'plant', plant)
    urdf = set_hw_param(urdf, 'seed', str(int(seed)))
    if not odom_error:
        urdf = set_hw_param(urdf, 'wheel_scale_errors', '0,0,0,0')
        for k in ODOM_ERROR_PARAMS[1:]:
            urdf = set_hw_param(urdf, k, '0.0')
    return urdf


def set_base_limits(urdf, max_accel, max_ang_accel):
    """ros2_control 底盘（force）的驱动力 / 力矩上限：M·max_accel、I_z·max_ang_accel."""
    extra = ''.join(f'\n      <param name="{k}">{float(v)}</param>'
                    for k, v in (('max_accel', max_accel), ('max_ang_accel', max_ang_accel))
                    if v != '')
    if not extra:
        return urdf
    out, n = re.subn(r'(<param name="plant">[^<]*</param>)', r'\g<1>' + extra, urdf)
    if n != 1:
        raise RuntimeError('URDF 中找不到 ros2_control 底盘的硬件参数块')
    return out


def add_cutting_force(urdf, params):
    """在模型末尾加入切割力插件；params 为 "键=值;键=值"（值中可含空格，如向量）."""
    elems = ['<enabled>true</enabled>']
    for item in filter(None, (p.strip() for p in params.split(';'))):
        key, sep, value = item.partition('=')
        if not sep or not re.fullmatch(r'[a-z_]+', key.strip()):
            raise RuntimeError(f'无效的切割力插件参数：{item}')
        elems.append(f'<{key.strip()}>{value.strip()}</{key.strip()}>')
    block = ('  <gazebo>\n    <plugin name="scr_cutting_force" '
             'filename="libscr_cutting_force.so">\n      ' + '\n      '.join(elems) +
             '\n    </plugin>\n  </gazebo>\n')
    out, n = re.subn(r'</robot>\s*$', block + '</robot>\n', urdf)
    if n != 1:
        raise RuntimeError('URDF 末尾找不到 </robot>')
    return out


def launch_robot(context, robot_description):
    def arg(name):
        return context.perform_substitution(LaunchConfiguration(name))

    base = arg('base_control')
    plant = arg('plant')
    localization = arg('localization')
    odom_error = arg('odom_error').lower() in ('1', 'true', 'yes')
    if base not in ('ros2_control', 'plugin') or plant not in ('force', 'wheels') or \
            localization not in ('ekf', 'wheel'):
        raise RuntimeError(f'无效参数 base_control={base} plant={plant} '
                           f'localization={localization}')
    urdf = select_base_control(robot_description, base)
    if base == 'ros2_control':
        urdf = configure_base(urdf, plant, odom_error, int(arg('odom_seed')))
        urdf = set_base_limits(urdf, arg('base_max_accel'), arg('base_max_ang_accel'))
    if arg('cutting_force').lower() in ('1', 'true', 'yes'):
        urdf = add_cutting_force(urdf, arg('cutting_force_params'))
    actions = [launch_ros.actions.Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': urdf}])]

    # 生成机器人：相机偏角只注入 Gazebo 中的模型（TF 不变）
    gz_urdf = urdf
    yaw = float(arg('camera_yaw_error_deg'))
    if yaw != 0.0:
        gz_urdf = inject_camera_yaw_error(gz_urdf, yaw)
    if gz_urdf is not urdf:
        f = tempfile.NamedTemporaryFile('w', suffix='.urdf', prefix='scr_robot_', delete=False)
        f.write(gz_urdf)
        f.close()
        spawn_args = ['-file', f.name, '-entity', ENTITY]
    else:
        spawn_args = ['-topic', '/robot_description', '-entity', ENTITY]
    actions.append(launch_ros.actions.Node(package='gazebo_ros', executable='spawn_entity.py',
                                           arguments=spawn_args))

    def spawner(name, *extra):
        return launch_ros.actions.Node(package='controller_manager', executable='spawner',
                                       arguments=[name, *extra], output='screen')

    controllers = [spawner('joint_state_broadcaster'), spawner('arm_controller')]
    if base == 'plugin':
        script = os.path.join(get_package_prefix('scr_description'), 'lib', 'scr_description',
                              'mecanum_controller.py')
        actions.append(ExecuteProcess(cmd=['python3', script], output='screen'))
        controllers.append(spawner('wheel_controller'))
    else:
        extra = []
        if localization == 'ekf':
            extra = ['--param-file', os.path.join(
                get_package_share_directory('scr_base_control'), 'config',
                'base_controller_ekf.yaml')]
            actions.append(launch_ros.actions.Node(
                package='robot_localization', executable='ekf_node', name='ekf_filter_node',
                output='screen',
                parameters=[os.path.join(get_package_share_directory('scr_navigation'),
                                         'config', 'ekf.yaml')],
                remappings=[('odometry/filtered', '/odom')]))
        controllers.append(spawner('base_controller', *extra))
    actions.append(TimerAction(period=5.0, actions=controllers))
    return actions


def generate_launch_description():
    share = get_package_share_directory('scr_description')
    with open(os.path.join(share, 'urdf', 'scr_robot_gazebo.urdf')) as f:
        robot_description = f.read().replace('$(find scr_description)', share)

    # Gazebo 插件路径：gazebo_ros、本包（libscr_planar_move.so）
    plugin_path = os.pathsep.join([os.path.join(get_package_prefix('gazebo_ros'), 'lib'),
                                   os.path.join(get_package_prefix('scr_description'), 'lib')])
    os.environ['GAZEBO_PLUGIN_PATH'] = (os.environ['GAZEBO_PLUGIN_PATH'] + os.pathsep + plugin_path
                                        if 'GAZEBO_PLUGIN_PATH' in os.environ else plugin_path)
    # 模型路径：本包（含 AprilTag 定位标记 model://scr_apriltag_*）
    model_path = os.pathsep + os.path.join(get_package_prefix('scr_description'), 'share') + \
        os.pathsep + os.path.join(share, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = '/usr/share/gazebo-11/models' + model_path

    args = [
        DeclareLaunchArgument('world', default_value=os.path.join(share, 'worlds', 'room.world'),
                              description='Gazebo world file'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start Gazebo GUI (gzclient)'),
        # rtf：仿真速度上限（相对实时的倍数）。控制器按仿真时钟计时，加速不改变控制律；但
        # 感知 / 控制代码的计算延迟按仿真时间计会放大 rtf 倍，批量实验前应先与 rtf=1 对比
        DeclareLaunchArgument('rtf', default_value='1.0',
                              description='Max real-time factor (1.0 = real time, 0 = unlimited)'),
        DeclareLaunchArgument('camera_yaw_error_deg', default_value='0.0',
                              description='Camera mount yaw error injected into the Gazebo '
                                          'sensor only (deg)'),
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
        DeclareLaunchArgument('cutting_force', default_value='false',
                              description='Load the cutting-force plugin (scr_cutting_force)'),
        DeclareLaunchArgument('cutting_force_params', default_value='',
                              description='Plugin parameters as "key=value;key=value"'),
        DeclareLaunchArgument('base_max_accel', default_value='',
                              description='Drive force limit of the force plant as an '
                                          'acceleration (m/s^2); empty = plugin default 5'),
        DeclareLaunchArgument('base_max_ang_accel', default_value='',
                              description='Drive torque limit of the force plant as an angular '
                                          'acceleration (rad/s^2); empty = plugin default 10'),
    ]

    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', LaunchConfiguration('world')],
        output='screen')
    gzclient = ExecuteProcess(cmd=['gzclient', '--verbose'], output='screen',
                              condition=IfCondition(LaunchConfiguration('gui')))
    unthrottle = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=['gz', 'physics', '-u',
                 PythonExpression(['str(int(1000 * float("', LaunchConfiguration('rtf'),
                                   '")))'])],
            output='screen')],
        condition=IfCondition(PythonExpression(['float("', LaunchConfiguration('rtf'),
                                                '") != 1.0'])))
    robot = OpaqueFunction(function=launch_robot, args=[robot_description])
    return launch.LaunchDescription(args + [unthrottle, gzserver, gzclient, robot])
