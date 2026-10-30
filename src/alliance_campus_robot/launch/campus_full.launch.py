"""One command for the whole demo.

  ros2 launch alliance_campus_robot campus_full.launch.py              # mode:=nav (default)
  ros2 launch alliance_campus_robot campus_full.launch.py mode:=auto_map
  ros2 launch alliance_campus_robot campus_full.launch.py mode:=slam
  ros2 launch alliance_campus_robot campus_full.launch.py mode:=sim_only

Starts Gazebo (campus world, robot, 3D labels + destination panel), then:
  nav      Nav2 on the road map + RViz + GUI bridge (click a place -> robot drives)
  auto_map robot patrols the roads and builds the map (known-pose mapper) + RViz
  slam     SLAM Toolbox + RViz (drive with teleop in another terminal)
  sim_only just the simulation (teleop / obstacle avoidance)
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('alliance_campus_robot')
    plugin_dir = os.path.join(get_package_prefix('campus_gui_plugin'), 'lib')
    old = os.environ.get('GAZEBO_PLUGIN_PATH', '')

    def include(name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', name)))

    def when(mode):
        return IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == '", mode, "'"]))

    sim_time = {'use_sim_time': True}
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='nav',
                              description='nav | auto_map | slam | sim_only'),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', plugin_dir + (':' + old if old else '')),

        include('sim.launch.py'),

        # Give Gazebo + the robot time to come up before starting the rest.
        TimerAction(period=12.0, actions=[
            GroupAction(condition=when('nav'), actions=[
                include('navigation.launch.py'),
                Node(package='alliance_campus_robot', executable='gui_bridge',
                     output='screen', parameters=[sim_time]),
            ]),
            GroupAction(condition=when('auto_map'), actions=[include('auto_mapping.launch.py')]),
            GroupAction(condition=when('slam'), actions=[include('mapping.launch.py')]),
        ]),
    ])
