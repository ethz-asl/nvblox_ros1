include(ExternalProject)

set(NVBLOX_DIR ${PROJECT_SOURCE_DIR}/../nvblox/nvblox)

# Disable tests and experiments
set(BUILD_EXPERIMENTS OFF)
set(BUILD_TESTS OFF)

set(NVBLOX_INCLUDE_DIR ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
set(NVBLOX_BUILD_DIR ${CMAKE_BINARY_DIR}/build)
set(NVBLOX_INCLUDE_OUTPUT_DIR ${CATKIN_DEVEL_PREFIX}/${NVBLOX_INCLUDE_DIR})

ExternalProject_Add(nvblox
  PREFIX nvblox
  SOURCE_DIR ${NVBLOX_DIR}
  UPDATE_COMMAND ""
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> -DCMAKE_INSTALL_INCLUDEDIR=${NVBLOX_INCLUDE_DIR}
  INSTALL_DIR "${CATKIN_DEVEL_PREFIX}"
)

# Set up correct include directories
include_directories(BEFORE include ${NVBLOX_INCLUDE_OUTPUT_DIR} ${NVBLOX_INCLUDE_OUTPUT_DIR}/eigen3)

add_library(nvblox_interface INTERFACE)
add_dependencies(nvblox_interface nvblox)
