#include <glog/logging.h>
#include <memory>

#include <ros/ros.h>

#include <nvblox/core/internal/warmup_cuda.h>

#include <nvblox_ros/nvblox_node.hpp>

using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  google::InstallFailureSignalHandler();
  ros::init(argc, argv, "nvblox_node_test");
  ros::NodeHandle nh, nh_private("~");

  // Warmup CUDA so it doesn't affect our timings *as* much for the first
  // CUDA call.
  nvblox::warmupCuda();

  nvblox::NvbloxNode node(nh, nh_private);

  ros::spin();

  ros::shutdown();
  return 0;
}