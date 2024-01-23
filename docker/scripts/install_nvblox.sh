#!/bin/bash
set -o pipefail

if [ "$1" = true ] ; then
  # Also install a copy of nvblox in the deps directory so it works "out of the box"
  cd $NVBLOX_DEP_WS/src || exit 1
  git clone https://github.com/ethz-asl/nvblox_ros1.git
fi

