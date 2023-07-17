#!/bin/bash
set -o pipefail

# This script builds all the installed ROS packages and sets up the bashrc.
cd $NVBLOX_DEP_WS
catkin build -c

# Add sourcing of the repo to the ~/.bashrc
echo 'source $NVBLOX_DEP_WS/devel/setup.bash' >> ~/.bashrc
