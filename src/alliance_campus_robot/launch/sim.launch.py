"""Gazebo Classic 11 + campus world + robot. Use this for teleoperation."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('alliance_campus_robot')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    world = os.path.join(pkg, 'worlds', 'alliance_campus_gazebo11.sdf')
    urdf = os.path.join(pkg, 'urdf', 'campus_robot.urdf')
    with open(urdf, 'r') as f:
        robot_description = f.read()

    model_path = os.path.join(pkg, 'models')
    old = os.environ.get('GAZEBO_MODEL_PATH', '')

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='-30.0'),   # near Main Gate, on the main road
        DeclareLaunchArgument('yaw', default_value='1.5708'),  # facing +y (up the road)

        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path + (':' + old if old else '')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world}.items(),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': True}],
            output='screen',
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'campus_ugv', '-topic', 'robot_description',
                       '-x', LaunchConfiguration('x'),
                       '-y', LaunchConfiguration('y'),
                       '-z', '0.2',
                       '-Y', LaunchConfiguration('yaw')],
            output='screen',
        ),
    ])
