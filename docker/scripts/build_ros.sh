#!/bin/bash
set -o pipefail

# This script builds all the installed ROS packages and sets up the bashrc.
cd $NVBLOX_DEP_WS
catkin build -c

# Add sourcing of the repo to the ~/.bashrc
echo 'source $NVBLOX_DEP_WS/devel/setup.bash' >> ~/.bashrc

# Add sourcing of the repo to the ~/.bashrc
echo "source $NVBLOX_DEP_WS/devel/setup.bash" >> ~/.bashrc
echo 'ROSBASH=/root/nvblox_ws/devel/setup.bash && [ -e "$ROSBASH" ] && source $ROSBASH' >> ~/.bashrc