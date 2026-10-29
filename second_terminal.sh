#!/usr/bin/env bash
# Opens a shell inside the running demo container (for teleop, topic echo, etc.)
D=docker
docker ps >/dev/null 2>&1 || D="sudo docker"
$D exec -it campus bash -c "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; exec bash"
