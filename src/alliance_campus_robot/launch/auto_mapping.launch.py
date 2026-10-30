"""Known-pose mapper + RViz + automatic road patrol. Run AFTER sim.launch.py.
Do NOT run teleop or obstacle_avoidance at the same time.
(Use mapping.launch.py instead if you want SLAM Toolbox.)"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('alliance_campus_robot')
    mapper = Node(package='alliance_campus_robot', executable='known_pose_mapper',
                  output='screen', parameters=[{'use_sim_time': True}])
    rviz = Node(package='rviz2', executable='rviz2', output='screen',
                arguments=['-d', os.path.join(pkg, 'rviz', 'campus_mapping.rviz')],
                parameters=[{'use_sim_time': True}])
    patrol = Node(package='alliance_campus_robot', executable='road_patrol',
                  output='screen', parameters=[{'use_sim_time': True}])
    return LaunchDescription([mapper, rviz, TimerAction(period=6.0, actions=[patrol])])
