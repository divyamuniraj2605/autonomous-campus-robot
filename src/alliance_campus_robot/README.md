# Alliance Campus Robot (ROS 2 Humble + Gazebo Classic)

A 4-wheel robot with LiDAR in a simulated Alliance University campus.
Features: keyboard teleoperation, automatic road-patrol mapping, an exact road-only
map generated from the world file, and Nav2 navigation to named places
("library", "food court", ...).

**Needs:** Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11.

## 1. Install dependencies (once)
    sudo apt update
    sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-slam-toolbox \
      ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-map-server \
      ros-humble-rviz2 ros-humble-robot-state-publisher \
      ros-humble-teleop-twist-keyboard python3-colcon-common-extensions

## 2. Build
    mkdir -p ~/campus_ws/src
    cp -r alliance_campus_robot ~/campus_ws/src/
    cd ~/campus_ws
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source install/setup.bash

In every new terminal run:
    source /opt/ros/humble/setup.bash && source ~/campus_ws/install/setup.bash

## 3. Run (Terminal 1 is always the simulation)
    ros2 launch alliance_campus_robot sim.launch.py

### A) Teleoperation
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.3 -p turn:=0.4

### B) Automatic mapping (robot patrols the roads; map saved to ~/campus_map)
    ros2 launch alliance_campus_robot auto_mapping.launch.py
(Do not run teleop at the same time. Alternative SLAM version: `mapping.launch.py` + teleop.)

### C) Navigation to named places
    ros2 launch alliance_campus_robot navigation.launch.py     # wait ~15 s
    ros2 run alliance_campus_robot goto_destination            # type: library, food court, main gate ...
Or click a goal in RViz with the "2D Goal Pose" tool.

## Notes
- Gazebo only gives the LiDAR collision objects (buildings, library, walls, gate pillars);
  roads and trees are paint only. `maps/campus_map` is the exact obstacle map and
  `maps/campus_nav_map` is the road-only map used for navigation. Rebuild them from any
  world with: `python3 tools/generate_maps_from_world.py worlds/alliance_campus_gazebo11.sdf maps config/locations.yaml`
- Navigation and the road patrol use Gazebo's exact robot pose (/ground_truth/odom), so
  they work in simulation only, not on a real robot.
- Place names and coordinates live in `config/locations.yaml`.
