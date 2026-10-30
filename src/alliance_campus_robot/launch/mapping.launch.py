"""SLAM Toolbox (online async) + RViz2. Run AFTER sim.launch.py."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('alliance_campus_robot')
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'slam_params.yaml'),
                        {'use_sim_time': True}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg, 'rviz', 'campus_mapping.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
