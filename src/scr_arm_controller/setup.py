from setuptools import find_packages, setup
from glob import glob

package_name = 'scr_arm_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SCR Team',
    maintainer_email='scr@robot.com',
    description='SCR robot arm controller package for kinematics and control algorithms',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'square_demo = scr_arm_controller.square_demo:main',
            'cartesian_cutting = scr_arm_controller.cartesian_cutting:main',
            'custom_cutting = scr_arm_controller.custom_cutting:main',
            'navigation_cutting_task = scr_arm_controller.navigation_cutting_task:main',
            'marker_scanner = scr_arm_controller.marker_scanner:main',
            'trajectory_visualizer = scr_arm_controller.trajectory_visualizer:main',
            'test_marker_visualization = scr_arm_controller.test_marker_visualization:main',
        ],
    },
)
