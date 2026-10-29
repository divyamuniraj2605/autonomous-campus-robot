# campus-ros-final - Alliance Campus Robot

Simulated campus robot (Gazebo Classic 11, ROS 2 Humble) with 3D building labels, a
destination panel, SLAM / mapping, Nav2 navigation and obstacle avoidance.
Runs in Docker, so it works on Ubuntu 24.04 / Jazzy hosts too.

## From scratch (host = Ubuntu with a screen)
    unzip campus-ros-final.zip -d ~ && cd ~/campus-ros-final
    ./install_docker.sh          # once; then LOG OUT and back in
    cd ~/campus-ros-final
    ./run_demo.sh                # first run builds the image (5-10 min) + the workspace

Wait ~25 s after Gazebo opens, then click a place in the CAMPUS LOCATIONS panel
(right side). The robot drives there on the roads and goes around obstacles.
"STOP ROBOT" cancels the goal.

## Other modes
    ./run_demo.sh mode:=auto_map     # robot patrols roads and builds the map
    ./run_demo.sh mode:=slam         # SLAM Toolbox + RViz
    ./run_demo.sh mode:=sim_only     # Gazebo + robot only
    CAMPUS_OBSTACLE_RANGE=2.0 ./run_demo.sh   # obstacle detection distance (m), default 2.5

## Second terminal (teleop, debugging)
    ./second_terminal.sh
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ros2 topic echo /campus/status
    ros2 topic pub --once /campus/destination std_msgs/msg/String "{data: library}"

## How it works
Panel click -> UDP 127.0.0.1:5005 -> `gui_bridge` node -> Nav2 `navigate_to_pose`.
Packages: `src/alliance_campus_robot` (robot, world, maps, nodes, launch), `src/campus_gui_plugin`
(Gazebo overlay). Edit place names/coordinates in `src/alliance_campus_robot/config/locations.yaml`.

## Notes
- Navigation uses Gazebo's ground-truth pose: simulation only.
- The LiDAR is a 2D scan 0.52 m above ground, so obstacles lower than that are not seen.
- Only run from this folder. Do not keep other copies of `alliance_campus_robot` under the same
  folder tree (colcon refuses duplicate package names).
- Old backups are in `src/alliance_campus_robot/_backups`.
