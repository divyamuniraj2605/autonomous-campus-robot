"""Nav2 on the road-only campus map. Run AFTER sim.launch.py.
Do NOT run auto_mapping / mapping launches at the same time."""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from alliance_campus_robot.campus_nav_utils import tune_nav2_params


def patched_nav2_params():
    """Take Nav2's own default params and adapt them to this robot (see campus_nav_utils).
    Obstacle detection distance (metres): set env CAMPUS_OBSTACLE_RANGE (default 2.5)."""
    src = os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml')
    p = tune_nav2_params(yaml.safe_load(open(src)),
                         float(os.environ.get('CAMPUS_OBSTACLE_RANGE', '2.5')))
    out = os.path.join(tempfile.gettempdir(), 'campus_nav2_params.yaml')
    yaml.safe_dump(p, open(out, 'w'))
    return out


def generate_launch_description():
    pkg = get_package_share_directory('alliance_campus_robot')
    nav2 = get_package_share_directory('nav2_bringup')
    sim = {'use_sim_time': True}

    localizer = Node(package='alliance_campus_robot', executable='ground_truth_localizer',
                     output='screen', parameters=[sim])
    map_server = Node(package='nav2_map_server', executable='map_server', output='screen',
                      parameters=[{'yaml_filename': os.path.join(pkg, 'maps', 'campus_nav_map.yaml'),
                                   'use_sim_time': True}])
    map_lifecycle = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                         name='lifecycle_manager_map', output='screen',
                         parameters=[{'use_sim_time': True, 'autostart': True,
                                      'node_names': ['map_server']}])
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'true', 'autostart': 'true',
                          'params_file': patched_nav2_params()}.items())
    rviz = Node(package='rviz2', executable='rviz2', output='screen',
                arguments=['-d', os.path.join(pkg, 'rviz', 'campus_nav.rviz')],
                parameters=[sim])
    return LaunchDescription([localizer, map_server, map_lifecycle, navigation, rviz])
