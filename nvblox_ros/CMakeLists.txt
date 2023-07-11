# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20)
include(ExternalProject)

project(nvblox_ros LANGUAGES CXX CUDA)
add_compile_options(-Wall -Wextra -Wpedantic -O3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Default to release build
if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()
message( STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}" )

################
# DEPENDENCIES #
################
find_package(Eigen3 3.3.7 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_eigen
  nvblox_msgs
  cv_bridge
  message_filters
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    std_msgs
    std_srvs
    sensor_msgs
    geometry_msgs
    tf2_ros
    tf2_eigen
    nvblox_msgs
    message_filters
    visualization_msgs
)

# Set up correct include directories
include_directories(AFTER include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

##########
# NVBLOX #
##########
message(STATUS "Installing nvblox.")
include(cmake/nvblox.cmake)

find_package(nvblox REQUIRED)

########
# CUDA #
########
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr -Xcudafe --display_error_number --disable-warnings ")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} --compiler-options -fPIC")
include_directories("${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}")

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}_lib SHARED
  src/lib/conversions/image_conversions.cu
  src/lib/conversions/layer_conversions.cu
  src/lib/conversions/mesh_conversions.cpp
  src/lib/conversions/pointcloud_conversions.cu
  src/lib/conversions/esdf_slice_conversions.cu
  src/lib/visualization.cpp
  src/lib/transformer.cpp
  src/lib/mapper_initialization.cpp
  src/lib/nvblox_node.cpp
  src/lib/nvblox_human_node.cpp
)

target_link_libraries(${PROJECT_NAME}_lib 
  nvblox::nvblox_lib
  nvblox::nvblox_eigen
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

set_target_properties(${PROJECT_NAME}_lib PROPERTIES CUDA_ARCHITECTURES "62;72;75;80;86;87") 

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${catkin_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME}_lib SYSTEM BEFORE PRIVATE
  $<TARGET_PROPERTY:nvblox::nvblox_eigen,INTERFACE_INCLUDE_DIRECTORIES>)


add_dependencies(${PROJECT_NAME}_lib
  ${catkin_EXPORTED_TARGETS}
  nvblox_interface
)

############
# BINARIES #
############
add_executable(nvblox_node
  src/nvblox_node_main.cpp
)
target_link_libraries(nvblox_node ${PROJECT_NAME}_lib)

add_dependencies(nvblox_node
  ${catkin_EXPORTED_TARGETS}
)

add_executable(nvblox_human_node
  src/nvblox_human_node_main.cpp
)
target_link_libraries(nvblox_human_node ${PROJECT_NAME}_lib)

add_dependencies(nvblox_human_node
  ${catkin_EXPORTED_TARGETS}
)

# TODO(helenol): Make sure we're properly setting up the install targets.

# #get_target_property(nvblox_INCLUDES nvblox_interface INTERFACE_INCLUDE_DIRECTORIES)
# ###########
# # INSTALL #
# ###########

# # Install includes.
# install(
#   DIRECTORY 
#   include/${PROJECT_NAME}/
#     include/
#     config
#     launch
#     rviz
#   DESTINATION 
#     include
#     #${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# # Install the library target.
# install(
#   TARGETS ${PROJECT_NAME}_lib nvblox_lib nvblox_gpu_hash nvblox_eigen
#   EXPORT ${PROJECT_NAME}Targets
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

# # Install nodes which live in lib for some reason.
# install(
#   TARGETS nvblox_node nvblox_human_node
#   DESTINATION lib/${PROJECT_NAME}
# )