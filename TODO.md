# TODO List
Please try to update this in the PR where the appropriate changes are made. :)
- [ ] Example unfinished task.
- [x] Example finished task.

## Porting ROS2 -> ROS1
- [x] nvblox_msgs
- [x] nvblox_ros
  - [x] nvblox_node_main
  - [x] nvblox_human_node_main (low priority)
- [x] nvblox_rviz_plugin
- [ ] check install targets so downstream packages can depend on this
- [ ] switch to catkin simple?

## General
- [x] Run clang-format with standard Google C++ on everything (should be done after code compiles) or diffs will be hard.
- [x] Add a Dockerfile for ROS1
- [ ] Add a CI job

## Porting Documentation
- [ ] Update README to actually reflect repo status
- [ ] Update/remove docs

## Testing
- [x] Realsense Bags
- [ ] LIDAR Bags
- [ ] Jetson
