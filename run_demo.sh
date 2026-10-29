#!/usr/bin/env bash
# Builds (first time only) and launches the whole demo inside a ROS 2 Humble container.
#   ./run_demo.sh                 -> mode:=nav  (click a place, robot drives there)
#   ./run_demo.sh mode:=auto_map  -> robot patrols roads and builds the map
#   ./run_demo.sh mode:=slam      -> SLAM Toolbox (drive with ./second_terminal.sh + teleop)
#   ./run_demo.sh mode:=sim_only  -> just Gazebo + robot
# Optional: CAMPUS_OBSTACLE_RANGE=2.0 ./run_demo.sh   (obstacle detection distance, metres)
cd "$(dirname "$0")" || exit 1
D=docker
docker ps >/dev/null 2>&1 || D="sudo docker"

if ! $D image inspect campus-humble >/dev/null 2>&1; then
  echo ">>> Building the Humble image (first time only, 5-10 min)..."
  $D build -t campus-humble . || exit 1
fi

xhost +local:docker >/dev/null 2>&1
$D rm -f campus >/dev/null 2>&1

$D run -it --rm --name campus --net=host --device /dev/dri \
  -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 \
  -e CAMPUS_OBSTACLE_RANGE="${CAMPUS_OBSTACLE_RANGE:-2.5}" \
  -v "$PWD":/ws -w /ws campus-humble bash -c \
  "source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash && ros2 launch alliance_campus_robot campus_full.launch.py $*"
