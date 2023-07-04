#!/bin/bash
set -o pipefail

# Set up a ROS workspace
mkdir -p $NVBLOX_DEP_WS/src
cd $NVBLOX_DEP_WS
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

# Install ROS packages from apt
apt-get install -y \
	ros-noetic-tf-conversions \
	ros-noetic-rviz-visual-tools

# Install all the other dependencies in the NVBLOX_DEP_WS
cd $NVBLOX_DEP_WS/src || exit 1
vcs import --recursive --input $SCRIPTS_PATH/nvblox_ros_deps.repos

# Clear cache to keep layer size down
rm -rf /var/lib/apt/lists/*
