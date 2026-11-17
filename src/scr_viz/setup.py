from glob import glob

from setuptools import find_packages, setup

package_name = 'scr_viz'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SCR Team',
    maintainer_email='scr@robot.com',
    description='SCR wall slot visualization (RViz markers, Gazebo wall fixtures, demo camera)',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'plan_viz = scr_viz.plan_viz_node:main',
            'plan_preview = scr_viz.preview:main',
            'spawn_wall_fixtures = scr_viz.spawn_wall_fixtures:main',
            'demo_record = scr_viz.demo_camera:main',
            'make_demo_video = scr_viz.video:main',
        ],
    },
)
