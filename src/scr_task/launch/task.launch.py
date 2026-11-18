"""任务层：技能 action 服务器（scr_arm_controller）+ 行为树执行器（scr_task）.

  ros2 launch scr_task task.launch.py
  ros2 launch scr_task task.launch.py alignment_params:=/path/to/calib.yaml

执行器启动后等待 Nav2（bt_navigator 激活）与技能服务器就绪，再加载行为树并激活。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    task_share = get_package_share_directory('scr_task')
    arm_share = get_package_share_directory('scr_arm_controller')
    args = [
        DeclareLaunchArgument(
            'alignment_params',
            default_value=os.path.join(arm_share, 'config', 'fine_alignment.yaml'),
            description='Fine-alignment parameters (alignment.*) for the skill server'),
        DeclareLaunchArgument(
            'executor_params',
            default_value=os.path.join(task_share, 'config', 'task_executor.yaml'),
            description='Behaviour-tree executor parameters'),
    ]
    skills = Node(
        package='scr_arm_controller', executable='task_skill_server', name='task_skills',
        output='screen', parameters=[LaunchConfiguration('alignment_params'),
                                     {'use_sim_time': True}])
    executor = Node(
        package='scr_task', executable='task_executor', output='screen',
        parameters=[LaunchConfiguration('executor_params')])
    return LaunchDescription(args + [skills, executor])
