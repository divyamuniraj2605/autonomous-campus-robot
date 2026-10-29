FROM osrf/ros:humble-desktop
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs ros-humble-slam-toolbox \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-teleop-twist-keyboard qtbase5-dev \
    python3-colcon-common-extensions mesa-utils \
    && rm -rf /var/lib/apt/lists/*
