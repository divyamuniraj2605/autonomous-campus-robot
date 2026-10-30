import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'alliance_campus_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.pgm') + glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'models', 'central_library'),
         glob('models/central_library/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lt108-05',
    maintainer_email='lt108-05@todo.todo',
    description='Campus robot: teleop, LiDAR obstacle avoidance and SLAM mapping',
    license='MIT',
    entry_points={
        'console_scripts': [
            'obstacle_avoidance = alliance_campus_robot.obstacle_avoidance:main',
            'road_patrol = alliance_campus_robot.road_patrol:main',
            'known_pose_mapper = alliance_campus_robot.known_pose_mapper:main',
            'ground_truth_localizer = alliance_campus_robot.ground_truth_localizer:main',
            'goto_destination = alliance_campus_robot.goto_destination:main',
            'gui_bridge = alliance_campus_robot.gui_bridge:main',
        ],
    },
)
