# nvblox_ROS1

This is the ROS1 wrapper of nvblox, intended to make nvblox usable for the wider robotics community (especially for research).
This is a work in progress by [Turcan Tuna](https://github.com/tutunarsl) and [Helen Oleynikova](https://github.com/helenol).

See the [TODO](TODO.md) list for a complete list.

<div align="center"><img src="resources/isaac_sim_nvblox_humans.gif" width=400px/></div>

## Overview

Isaac ROS Nvblox contains ROS 1 packages for 3D reconstruction. The mapping pipeline is mainly designed to work with stereo cameras, which provide a depth image, and the corresponding pose uses GPU acceleration to compute 3D reconstruction and 2D costmaps using [nvblox](https://github.com/nvidia-isaac/nvblox). However, this ROS 1 wrapper also supports LiDAR point cloud input.

`nvblox_ros1` builds the reconstructed map in the form of a TSDF (Truncated Signed Distance Function) stored in a 3D voxel grid. This approach is similar to 3D occupancy grid mapping approaches in which occupancy probabilities are stored at each voxel. However, TSDF-based approaches like nvblox store the (signed) distance to the closest surface at each voxel. The surface of the environment can then be extracted as the zero-level set of this voxelized function. Typically, TSDF-based reconstructions provide higher quality surface reconstructions.

In addition to their use in reconstruction, distance fields are also useful for path planning because they provide an immediate means of checking whether potential future robot positions are in collision.

People are common obstacles for mobile robots, and while part of a costmap, people should not be part of the 3D reconstruction.  Planners that provide behavioral awareness by navigating differently depending on their proximity to people, benefit from a costmap for people. Person segmentation is computed using the color image, with the resulting mask applied to the depth image separating depth into scene depth and person depth images. The scene depth image is forwarded to TSDF mapping as explained above, the depth image for people is processed to an occupancy grid map.

To relax the assumption that occupancy grid maps only capture static objects, Nvblox applies an occupancy decay step. At a fixed frequency, all voxel occupancy probabilities are decayed towards 0.5 over time. This means that the state of the map (occupied or free) becomes less certain after it has fallen out of the field of view, until it becomes unknown (0.5 occupancy probability).


## Table of Contents
- [nvblox_ROS1](#nvblox_ros1)
  * [Overview](#overview)
  * [Table of Contents](#table-of-contents)
  * [Latest Update](#latest-update)
  * [Supported Platforms](#supported-platforms)
- [Installation](#installation)
  * [Docker](#docker)
  * [Native Install](#native-install)
- [Details ](#details)
  * [Packages Overview](#packages-overview)
  * [ROS 1 Parameters](#ros-1-parameters)
  * [ROS 1 Topics and Services](#ros-1-topics-and-services)
  * [Troubleshooting](#troubleshooting)
  * [Updates](#updates)

## Latest Update

Update 2023-07-01: Adaptations from ROS 2 to ROS 1.

## Supported Platforms

This package is designed and tested to be compatible with ROS 1 Noetic running on [Jetson](https://developer.nvidia.com/embedded-computing) or an x86_64 system with an NVIDIA GPU.

> **Note**: Versions of ROS 1 earlier than Noetic are **not** tested.

| Platform | Hardware   | Software  | Notes   |
| -------- | -------    | -------  | ------------ |
| Jetson   | [Jetson Orin](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/) <br> [Jetson Xavier](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-agx-xavier/) | [JetPack 5.1.1](https://developer.nvidia.com/embedded/jetpack) | For best performance, ensure that [power settings](https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/text/SD/PlatformPowerAndPerformance.html) are configured appropriately. |
| x86_64   | NVIDIA GPU   | [Ubuntu 20.04+](https://releases.ubuntu.com/20.04/) <br> [CUDA 11.8+](https://developer.nvidia.com/cuda-downloads) |                                                                                                                                                                                         |

# Installation
There's 2 options: docker
## Docker

To simplify development, we strongly recommend using the docker images in [the docker subfolder](./docker/run_docker.sh)

First check out this repo into `~/nvblox_ws/src/` and create `~/data` to store data:
```
mkdir -p ~/nvblox_ws/src/
mkdir -p ~/data
cd ~/nvblox_ws/src/
git clone https://github.com/ethz-asl/nvblox_ros1.git
cd nvblox_ros1
git submodule update --init --recursive
```

Then build the docker using:
```
cd ~/nvblox_ws/src/nvblox_ros1/docker
./run_docker.sh -b
```

Within the docker, make sure to set up the workspace with:
```
cd ~/nvblox_ws/
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
echo "source ~/nvblox_ws/devel/setup.bash" >> ~/.bashrc
```

For future iterations you can run it with just:
```
./run_docker.sh
```
(Check out `./run_docker.sh -h` for more settings, such as the docker name).

To launch additional terminal sessions into the docker, use:
```
docker exec -it nvblox bash
```

## Native Install
This is meant for Ubuntu 20.04 and will not work on other versions. Use the docker instead.

First, install ROS1 noetic, preferably desktop-full, following instructions here: [ROS installation](https://wiki.ros.org/noetic/Installation/Ubuntu)

Then install CUDA, preferably 11.8, but most versions after 11.0 should work: [CUDA installation](https://developer.nvidia.com/cuda-11-8-0-download-archive)

Install additional dependenices:
```
apt-get install python3-catkin-tools python3-vcstool python3-pip qtbase5-dev
apt-get -qq update &&  apt-get install -y libgoogle-glog-dev libgtest-dev libgflags-dev python3-dev libsqlite3-dev
pip install --upgrade cmake
```

Note that the above upgrades your version of CMake as CUDA support before 3.18 was seriously missing.

Then check out the necessary repos:
```
mkdir -p ~/nvblox_ws/src/
cd ~/nvblox_ws/src/
git clone https://github.com/ethz-asl/nvblox_ros1.git
cd nvblox_ros1
git submodule update --init --recursive
cd ~/nvblox_ws/src/
vcs import --recursive --input nvblox_ros1/docker/scripts/nvblox_ros_deps.repos
cd ~/nvblox_ws/
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
echo "source ~/nvblox_ws/devel/setup.bash" >> ~/.bashrc
```

## Quickstart
Download a rosbag version of the [Panoptic Mapping](https://github.com/ethz-asl/panoptic_mapping/) synthetic dataset: [run1.bag](http://robotics.ethz.ch/~asl-datasets/2023_nvblox_datasets/panopt/run1.bag) and/or [run2.bag](http://robotics.ethz.ch/~asl-datasets/2023_nvblox_datasets/panopt/run2.bag). These bags contain synthetic depth and color images and ground truth poses.

Put it in `~/data` so that it gets mapped in the docker (if using a docker).

In one terminal, start up nvblox with:
```
roslaunch nvblox_ros nvblox_panopt.launch rviz:=true
```

In another terminal (if in docker, start with `docker exec -it nvblox bash`):
```
rosbag play --clock ~/data/run1.bag
```

<div align="center"><img src="resources/nvblox_panopt_quick.gif" width=621px/></div>

# Details
A better overview of the technical details is available [here](./docs/technical-details.md).

## Packages Overview
* **nvblox_msgs**: Custom messages for transmitting the output distance map slice and mesh over ROS 1.
* **nvblox_ros**: The ROS 1 wrapper for the core reconstruction library and the nvblox node.
* **nvblox_rviz_plugin**: A plugin for displaying nvblox's (custom) mesh type in RVIZ.
* **\[submodule\] nvblox**: The core (ROS independent) reconstruction library.

## ROS 1 Parameters

Find all available ROS 1 parameters [here](./docs/parameters.md).

## ROS 1 Topics and Services

Find all ROS 1 subscribers, publishers and services [here](./docs/topics-and-services.md).

## Troubleshooting
Currently, the nvblox_ros1 package is only tested on Ubuntu 20.04 with ROS 1 Noetic.

## Updates

| Date       | Changes    |
| ---------- | ------------ |
| 2023-07-19 | ROS2 to ROS1 port is done and tested. |
| 2023-07-01 | ROS 2 to ROS 1 port is started. |
