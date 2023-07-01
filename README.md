# nvblox_ROS1

This is the ROS1 wrapper of nvblox, intended to make nvblox usable for the wider robotics community (especially for research).
This is a work in progress by [Turcan Tuna](https://github.com/tutunarsl) and [Helen Oleynikova](https://github.com/helenol).

Everything below this line still needs to be cleaned up/edited.

See the [TODO](TODO.md) list for a complete list.

---

<div align="center"><img src="resources/isaac_sim_nvblox_humans.gif" width=400px/></div>

## Overview

Isaac ROS Nvblox contains ROS 1 packages for 3D reconstruction. The mapping pipeline is mainly designed to work with stereo cameras, which provide a depth image, and the corresponding pose uses GPU acceleration to compute 3D reconstruction and 2D costmaps using [nvblox](https://github.com/nvidia-isaac/nvblox). However, this ROS 1 wrapper also supports LiDAR point cloud input.

`nvblox_ros1` builds the reconstructed map in the form of a TSDF (Truncated Signed Distance Function) stored in a 3D voxel grid. This approach is similar to 3D occupancy grid mapping approaches in which occupancy probabilities are stored at each voxel. However, TSDF-based approaches like nvblox store the (signed) distance to the closest surface at each voxel. The surface of the environment can then be extracted as the zero-level set of this voxelized function. Typically, TSDF-based reconstructions provide higher quality surface reconstructions.

In addition to their use in reconstruction, distance fields are also useful for path planning because they provide an immediate means of checking whether potential future robot positions are in collision. 

People are common obstacles for mobile robots, and while part of a costmap, people should not be part of the 3D reconstruction.  Planners that provide behavioral awareness by navigating differently depending on their proximity to people, benefit from a costmap for people. Person segmentation is computed using the color image, with the resulting mask applied to the depth image separating depth into scene depth and person depth images. The scene depth image is forwarded to TSDF mapping as explained above, the depth image for people is processed to an occupancy grid map.

To relax the assumption that occupancy grid maps only capture static objects, Nvblox applies an occupancy decay step. At a fixed frequency, all voxel occupancy probabilities are decayed towards 0.5 over time. This means that the state of the map (occupied or free) becomes less certain after it has fallen out of the field of view, until it becomes unknown (0.5 occupancy probability).


## Table of Contents

- [Isaac ROS Nvblox](#isaac-ros-nvblox)
  - [Overview](#overview)
  - [Table of Contents](#table-of-contents)
  - [Latest Update](#latest-update)
  - [Supported Platforms](#supported-platforms)
    - [Docker](#docker)
  - [Quickstart](#quickstart)
  - [Next Steps](#next-steps)
    - [Try More Examples](#try-more-examples)
    - [Customize your Dev Environment](#customize-your-dev-environment)
  - [Packages Overview](#packages-overview)
  - [ROS 1 Parameters](#ros-2-parameters)
  - [ROS 1 Topics and Services](#ros-2-topics-and-services)
  - [Troubleshooting](#troubleshooting)
  - [Updates](#updates)

## Latest Update

Update 2023-07-01: Adaptations from ROS 2 to ROS 1.

## Supported Platforms

This package is designed and tested to be compatible with ROS 1 Noetic running on [Jetson](https://developer.nvidia.com/embedded-computing) or an x86_64 system with an NVIDIA GPU.

> **Note**: Versions of ROS 1 earlier than Noetic are **not** tested.

| Platform | Hardware                                                                                                                                                                                                 | Software                                                                                                           | Notes                                                                                                                                                                                   |
| -------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Jetson   | [Jetson Orin](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/) <br> [Jetson Xavier](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-agx-xavier/) | [JetPack 5.1.1](https://developer.nvidia.com/embedded/jetpack)                                                     | For best performance, ensure that [power settings](https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/text/SD/PlatformPowerAndPerformance.html) are configured appropriately. |
| x86_64   | NVIDIA GPU                                                                                                                                                                                               | [Ubuntu 20.04+](https://releases.ubuntu.com/20.04/) <br> [CUDA 11.8+](https://developer.nvidia.com/cuda-downloads) |                                                                                                                                                                                         |

### Docker

To simplify development, we strongly recommend leveraging the Isaac ROS Dev Docker images by following [these steps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/dev-env-setup.md). This will streamline your development environment setup with the correct versions of dependencies on both Jetson and x86_64 platforms.

> **Note**: All Isaac ROS Quickstarts, tutorials, and examples have been designed with the Isaac ROS Docker images as a prerequisite.

## Quickstart

0. nvblox_ros1 requies an up-to-date compiler. Make sure your system has one. For example, on Ubuntu 20.04, you can install gcc-9 and gcc-11 with:
  
      ```bash
      sudo apt update && sudo apt install -y build-essential
      sudo apt install gcc-11
      ```
   Furthermore, CMake version >3.20. You can install it with:
      1. Download the latest source distribution tar archive from https://cmake.org/download/ for your operating system (e.g. cmake-3.24.1.tar.gz).
      2. Extract it by doing 'tar -xf cmake-<version>.tar.gz', where <version> can for example be 3.24.1.
      3. Then install this cmake version by doing:
   
      ```bash
      cd cmake-<version>-rc4.tar.gz
      ./configure
      make -j$(nproc)
      sudo make install
      ```

1. Set up your development environment by following the instructions [here](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/dev-env-setup.md).

2. Clone this repository and its dependencies under `~/workspaces/isaac_ros-dev/src`.

    ```bash
    cd ~/workspaces/isaac_ros-dev/src
    ```

    ```bash
    git clone --recurse-submodules https://github.com/ethz-asl/nvblox_ros1.git && \
        cd nvblox_ros1 && git lfs pull
    ```

3. Pull down a ROS Bag of sample data:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/nvblox_ros1 && \ 
      git lfs pull -X "" -I "nvblox_ros/test/test_cases/rosbags/nvblox_pol"
    ```

4. Inside the container, install package-specific dependencies via `rosdep`:

    ```bash
    cd /workspaces/isaac_ros-dev/ && \
        rosdep install -i -r --from-paths src --rosdistro noetic -y --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv nvblox"
    ```

5. Build and source the workspace:  

    ```bash
    cd /workspaces/isaac_ros-dev && \
      catkin build && \
      source devel/setup.bash
    ```

6. (Optional) Run tests to verify complete and correct installation: (TODO[TT] Convert to cmake tests or remove.)  

    ```bash
    catkin test --executor sequential
    ```

7. In a **current terminal** inside the Docker container, run the launch file for Nvblox:

    ```bash
    source /workspaces/isaac_ros-dev/devel/setup.bash && \
        roslaunch nvblox_ros nvblox_ros.launch
    ```

8. Open a **second terminal** inside the docker container:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

9. In the **second terminal**, play the ROS Bag:

    ```bash
    rosbag play --pause --clock -s 0 -r 1 <your-favorite-rosbag>.bag
    ```

You should see the robot reconstructing a mesh, with the 2d esdf slice overlaid on top.

<div align="center"><img src="resources/basic_example_rviz.png" width=500px/></div>

## Next Ste                     |
### Customize your Dev Environment

To customize your development environment, reference [this guide](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/modify-dockerfile.md).

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
The LiDAR depth input known to be unstable in terms of robustness.

### Isaac ROS Troubleshooting

For solutions to problems with Isaac ROS, please check [here](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/troubleshooting.md).

## Updates

| Date       | Changes                                                                                                                                    |
| ---------- | ------------------------------------------------------------------------------------------------------------------------------------------ |
| 2023-07-01 | ROS 2 to ROS 1 port is done.                                                                                                               |
