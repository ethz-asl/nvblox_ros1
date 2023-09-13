// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <nvblox/io/mesh_io.h>
#include <nvblox/io/pointcloud_io.h>
#include <nvblox/utils/timing.h>

#include "nvblox_ros/transformer.hpp"
#include "nvblox_ros/visualization.hpp"

#include "nvblox_ros/nvblox_node.hpp"

namespace nvblox {

NvbloxNode::NvbloxNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      processing_spinner_(1, &processing_queue_),
      transformer_(nh) {
  // Get parameters first (stuff below depends on parameters)
  getParameters();

  // Set the transformer settings.
  transformer_.set_global_frame(global_frame_);
  transformer_.set_pose_frame(pose_frame_);

  // Initialize the mapper (the interface to the underlying nvblox library)
  // Note: This needs to be called after getParameters()
  // The mapper includes:
  // - Map layers
  // - Integrators
  const std::string mapper_name = "mapper";

  mapper_ = std::make_shared<Mapper>(voxel_size_, MemoryType::kDevice,
                                     static_projective_layer_type_);

  initializeMapper(mapper_.get(), nh_private_);

  // Setup interactions with ROS
  subscribeToTopics();
  setupTimers();
  advertiseTopics();
  advertiseServices();

  ROS_INFO_STREAM("Started up nvblox node in frame "
                  << global_frame_ << " and voxel size " << voxel_size_);

  // Set state.
  last_depth_update_time_ = ros::Time(0.0);
  last_color_update_time_ = ros::Time(0.0);
  last_lidar_update_time_ = ros::Time(0.0);

  // Start the processing spinner now that everything is set up.
  processing_spinner_.start();
}

NvbloxNode::~NvbloxNode() {
  // Need to explicitly delete these or there's a segfault on exit :(
  timesync_depth_.reset();
  timesync_color_.reset();
}

void NvbloxNode::getParameters() {
  ROS_INFO_STREAM("Getting parameters from parameter server.");

  // Get the type of layer to use.
  bool is_occupancy = false;
  nh_private_.param("use_static_occupancy_layer", is_occupancy, is_occupancy);

  if (is_occupancy) {
    static_projective_layer_type_ = ProjectiveLayerType::kOccupancy;
    ROS_INFO_STREAM(
        "static_projective_layer_type: occupancy (Attention: ESDF and Mesh "
        "integration is not yet implemented for occupancy.)");
  } else {
    static_projective_layer_type_ = ProjectiveLayerType::kTsdf;
    ROS_INFO_STREAM(
        "static_projective_layer_type: TSDF (for occupancy set the "
        "use_static_occupancy_layer parameter)");
  }

  // Declare & initialize the parameters.
  nh_private_.param("voxel_size", voxel_size_, voxel_size_);
  nh_private_.param("global_frame", global_frame_, global_frame_);
  nh_private_.param("pose_frame", pose_frame_, pose_frame_);
  nh_private_.param("is_realsense_data", is_realsense_data_,
                    is_realsense_data_);
  nh_private_.param("compute_mesh", compute_mesh_, compute_mesh_);
  nh_private_.param("compute_esdf", compute_esdf_, compute_esdf_);
  nh_private_.param("esdf_2d", esdf_2d_, esdf_2d_);
  nh_private_.param("esdf_distance_slice", esdf_distance_slice_,
                    esdf_distance_slice_);
  nh_private_.param("use_color", use_color_, use_color_);
  nh_private_.param("use_depth", use_depth_, use_depth_);
  nh_private_.param("use_lidar", use_lidar_, use_lidar_);
  nh_private_.param("esdf_slice_height", esdf_slice_height_,
                    esdf_slice_height_);
  nh_private_.param("esdf_2d_min_height", esdf_2d_min_height_,
                    esdf_2d_min_height_);
  nh_private_.param("esdf_2d_max_height", esdf_2d_max_height_,
                    esdf_2d_max_height_);
  nh_private_.param("lidar_width", lidar_width_, lidar_width_);
  nh_private_.param("lidar_height", lidar_height_, lidar_height_);
  nh_private_.param("lidar_vertical_fov_deg", lidar_vertical_fov_deg_,
                    lidar_vertical_fov_deg_);
  nh_private_.param("slice_visualization_attachment_frame_id",
                    slice_visualization_attachment_frame_id_,
                    slice_visualization_attachment_frame_id_);
  nh_private_.param("slice_visualization_side_length",
                    slice_visualization_side_length_,
                    slice_visualization_side_length_);
  nh_private_.param("max_depth_update_hz", max_depth_update_hz_,
                    max_depth_update_hz_);
  nh_private_.param("max_color_update_hz", max_color_update_hz_,
                    max_color_update_hz_);
  nh_private_.param("max_lidar_update_hz", max_lidar_update_hz_,
                    max_lidar_update_hz_);
  nh_private_.param("mesh_update_rate_hz", mesh_update_rate_hz_,
                    mesh_update_rate_hz_);
  nh_private_.param("esdf_update_rate_hz", esdf_update_rate_hz_,
                    esdf_update_rate_hz_);
  nh_private_.param("occupancy_publication_rate_hz",
                    occupancy_publication_rate_hz_,
                    occupancy_publication_rate_hz_);
  nh_private_.param("max_poll_rate_hz", max_poll_rate_hz_, max_poll_rate_hz_);
  nh_private_.param("maximum_sensor_message_queue_length",
                    maximum_sensor_message_queue_length_,
                    maximum_sensor_message_queue_length_);
  nh_private_.param("map_clearing_radius_m", map_clearing_radius_m_,
                    map_clearing_radius_m_);
  nh_private_.param("map_clearing_frame_id", map_clearing_frame_id_,
                    map_clearing_frame_id_);
  nh_private_.param("clear_outside_radius_rate_hz",
                    clear_outside_radius_rate_hz_,
                    clear_outside_radius_rate_hz_);
}

void NvbloxNode::subscribeToTopics() {
  ROS_INFO_STREAM("Subscribing to topics.");

  if (!use_depth_ && !use_lidar_) {
    ROS_WARN(
        "Nvblox is running without depth or lidar input, the cost maps and "
        "reconstructions will not update");
  }

  if (use_depth_) {
    // Subscribe to synchronized depth + cam_info topics
    depth_sub_.subscribe(nh_, "depth/image",
                         maximum_sensor_message_queue_length_);
    depth_camera_info_sub_.subscribe(nh_, "depth/camera_info",
                                     maximum_sensor_message_queue_length_);

    timesync_depth_.reset(new message_filters::Synchronizer<time_policy_t>(
        time_policy_t(maximum_sensor_message_queue_length_), depth_sub_,
        depth_camera_info_sub_));
    timesync_depth_->registerCallback(std::bind(&NvbloxNode::depthImageCallback,
                                                this, std::placeholders::_1,
                                                std::placeholders::_2));
  }

  if (use_color_) {
    // Subscribe to synchronized color + cam_info topics
    color_sub_.subscribe(nh_, "color/image",
                         maximum_sensor_message_queue_length_);
    color_camera_info_sub_.subscribe(nh_, "color/camera_info",
                                     maximum_sensor_message_queue_length_);

    timesync_color_.reset(new message_filters::Synchronizer<time_policy_t>(
        time_policy_t(maximum_sensor_message_queue_length_), color_sub_,
        color_camera_info_sub_));
    timesync_color_->registerCallback(std::bind(&NvbloxNode::colorImageCallback,
                                                this, std::placeholders::_1,
                                                std::placeholders::_2));
  }

  if (use_lidar_) {
    // Subscribe to pointclouds.
    pointcloud_sub_ =
        nh_.subscribe("pointcloud", 10, &NvbloxNode::pointcloudCallback, this);
  }

  // Subscribe to transforms.
  transform_sub_ =
      nh_.subscribe("transform", 10, &NvbloxNode::transformCallback, this);
  pose_sub_ = nh_.subscribe("pose", 10, &NvbloxNode::poseCallback, this);
}

void NvbloxNode::advertiseTopics() {
  mesh_publisher_ = nh_private_.advertise<nvblox_msgs::Mesh>("mesh", 1, false);
  esdf_pointcloud_publisher_ = nh_private_.advertise<sensor_msgs::PointCloud2>(
      "esdf_pointcloud", 1, false);
  map_slice_publisher_ = nh_private_.advertise<nvblox_msgs::DistanceMapSlice>(
      "map_slice", 1, false);
  mesh_marker_publisher_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("mesh_marker", 1,
                                                             false);
  slice_bounds_publisher_ = nh_private_.advertise<visualization_msgs::Marker>(
      "map_slice_bounds", 1, true);
  occupancy_publisher_ =
      nh_private_.advertise<sensor_msgs::PointCloud2>("occupancy", 1, false);
}

void NvbloxNode::advertiseServices() {
  save_ply_service_ =
      nh_private_.advertiseService("save_ply", &NvbloxNode::savePly, this);
  save_map_service_ =
      nh_private_.advertiseService("save_map", &NvbloxNode::saveMap, this);
  load_map_service_ =
      nh_private_.advertiseService("load_map", &NvbloxNode::loadMap, this);
}

void NvbloxNode::setupTimers() {
  if (use_depth_) {
    ros::TimerOptions timer_options(
        ros::Duration(1.0 / max_poll_rate_hz_),
        boost::bind(&NvbloxNode::processDepthQueue, this, _1),
        &processing_queue_);

    depth_processing_timer_ = nh_private_.createTimer(timer_options);
  }
  if (use_color_) {
    ros::TimerOptions timer_options(
        ros::Duration(1.0 / max_poll_rate_hz_),
        boost::bind(&NvbloxNode::processColorQueue, this, _1),
        &processing_queue_);
    color_processing_timer_ = nh_private_.createTimer(timer_options);
  }
  if (use_lidar_) {
    ros::TimerOptions timer_options(
        ros::Duration(1.0 / max_poll_rate_hz_),
        boost::bind(&NvbloxNode::processPointcloudQueue, this, _1),
        &processing_queue_);
    pointcloud_processing_timer_ = nh_private_.createTimer(timer_options);
  }
  if (compute_esdf_) {
    ros::TimerOptions timer_options(
        ros::Duration(1.0 / esdf_update_rate_hz_),
        boost::bind(&NvbloxNode::processEsdf, this, _1), &processing_queue_);
    esdf_processing_timer_ = nh_private_.createTimer(timer_options);
  }
  if (compute_mesh_) {
    ros::TimerOptions timer_options(
        ros::Duration(1.0 / mesh_update_rate_hz_),
        boost::bind(&NvbloxNode::processMesh, this, _1), &processing_queue_);
    mesh_processing_timer_ =
        nh_private_.createTimer(ros::Duration(1.0 / mesh_update_rate_hz_),
                                &NvbloxNode::processMesh, this);
  }
  if (static_projective_layer_type_ == ProjectiveLayerType::kOccupancy) {
    ros::TimerOptions timer_options(
        ros::Duration(1.0 / occupancy_publication_rate_hz_),
        boost::bind(&NvbloxNode::publishOccupancyPointcloud, this, _1),
        &processing_queue_);
    occupancy_publishing_timer_ = nh_private_.createTimer(timer_options);
  }
  if (map_clearing_radius_m_ > 0.0f) {
    ros::TimerOptions timer_options(
        ros::Duration(1.0 / clear_outside_radius_rate_hz_),
        boost::bind(&NvbloxNode::clearMapOutsideOfRadiusOfLastKnownPose, this,
                    _1),
        &processing_queue_);
    clear_outside_radius_timer_ = nh_private_.createTimer(timer_options);
  }
}

void NvbloxNode::transformCallback(
    const geometry_msgs::TransformStampedConstPtr& transform_msg) {
  transformer_.transformCallback(transform_msg);
}

void NvbloxNode::poseCallback(
    const geometry_msgs::PoseStampedConstPtr& transform_msg) {
  transformer_.poseCallback(transform_msg);
}

void NvbloxNode::depthImageCallback(
    const sensor_msgs::ImageConstPtr& depth_img_ptr,
    const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg) {
  pushMessageOntoQueue({depth_img_ptr, camera_info_msg}, &depth_image_queue_,
                       &depth_queue_mutex_);
}

void NvbloxNode::colorImageCallback(
    const sensor_msgs::ImageConstPtr& color_image_ptr,
    const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg) {
  pushMessageOntoQueue({color_image_ptr, camera_info_msg}, &color_image_queue_,
                       &color_queue_mutex_);
}

void NvbloxNode::pointcloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr pointcloud) {
  pushMessageOntoQueue(pointcloud, &pointcloud_queue_,
                       &pointcloud_queue_mutex_);
}

void NvbloxNode::processDepthQueue(const ros::TimerEvent& /*event*/) {
  using ImageInfoMsgPair =
      std::pair<sensor_msgs::ImageConstPtr, sensor_msgs::CameraInfo::ConstPtr>;
  auto message_ready = [this](const ImageInfoMsgPair& msg) {
    return this->canTransform(msg.first->header);
  };

  processMessageQueue<ImageInfoMsgPair>(
      &depth_image_queue_,  // NOLINT
      &depth_queue_mutex_,  // NOLINT
      message_ready,        // NOLINT
      std::bind(&NvbloxNode::processDepthImage, this, std::placeholders::_1));

  limitQueueSizeByDeletingOldestMessages(maximum_sensor_message_queue_length_,
                                         "depth", &depth_image_queue_,
                                         &depth_queue_mutex_);
}

void NvbloxNode::processColorQueue(const ros::TimerEvent& /*event*/) {
  using ImageInfoMsgPair =
      std::pair<sensor_msgs::ImageConstPtr, sensor_msgs::CameraInfo::ConstPtr>;
  auto message_ready = [this](const ImageInfoMsgPair& msg) {
    return this->canTransform(msg.first->header);
  };

  processMessageQueue<ImageInfoMsgPair>(
      &color_image_queue_,  // NOLINT
      &color_queue_mutex_,  // NOLINT
      message_ready,        // NOLINT
      std::bind(&NvbloxNode::processColorImage, this, std::placeholders::_1));

  limitQueueSizeByDeletingOldestMessages(maximum_sensor_message_queue_length_,
                                         "color", &color_image_queue_,
                                         &color_queue_mutex_);
}

void NvbloxNode::processPointcloudQueue(const ros::TimerEvent& /*event*/) {
  using PointcloudMsg = sensor_msgs::PointCloud2::ConstPtr;
  auto message_ready = [this](const PointcloudMsg& msg) {
    return this->canTransform(msg->header);
  };
  processMessageQueue<PointcloudMsg>(
      &pointcloud_queue_,        // NOLINT
      &pointcloud_queue_mutex_,  // NOLINT
      message_ready,             // NOLINT
      std::bind(&NvbloxNode::processLidarPointcloud, this,
                std::placeholders::_1));

  limitQueueSizeByDeletingOldestMessages(maximum_sensor_message_queue_length_,
                                         "pointcloud", &pointcloud_queue_,
                                         &pointcloud_queue_mutex_);
}

void NvbloxNode::processEsdf(const ros::TimerEvent& /*event*/) {
  std::unique_lock<std::mutex> lock(map_mutex_);

  const ros::Time timestamp = ros::Time::now();
  timing::Timer ros_total_timer("ros/total");
  timing::Timer ros_esdf_timer("ros/esdf");

  timing::Timer esdf_integration_timer("ros/esdf/integrate");
  std::vector<Index3D> updated_blocks;
  if (esdf_2d_) {
    updated_blocks = mapper_->updateEsdfSlice(
        esdf_2d_min_height_, esdf_2d_max_height_, esdf_slice_height_);
  } else {
    updated_blocks = mapper_->updateEsdf();
  }
  esdf_integration_timer.Stop();

  if (updated_blocks.empty()) {
    return;
  }

  timing::Timer esdf_output_timer("ros/esdf/output");

  // If anyone wants a slice
  if (esdf_distance_slice_ &&
      (esdf_pointcloud_publisher_.getNumSubscribers() > 0 ||
       map_slice_publisher_.getNumSubscribers() > 0)) {
    // Get the slice as an image
    timing::Timer esdf_slice_compute_timer("ros/esdf/output/compute");
    AxisAlignedBoundingBox aabb;
    Image<float> map_slice_image;
    esdf_slice_converter_.distanceMapSliceImageFromLayer(
        mapper_->esdf_layer(), esdf_slice_height_, &map_slice_image, &aabb);
    esdf_slice_compute_timer.Stop();

    // Slice pointcloud for RVIZ
    if (esdf_pointcloud_publisher_.getNumSubscribers() > 0) {
      timing::Timer esdf_output_pointcloud_timer("ros/esdf/output/pointcloud");
      sensor_msgs::PointCloud2 pointcloud_msg;
      esdf_slice_converter_.sliceImageToPointcloud(
          map_slice_image, aabb, esdf_slice_height_,
          mapper_->esdf_layer().voxel_size(), &pointcloud_msg);
      pointcloud_msg.header.frame_id = global_frame_;
      pointcloud_msg.header.stamp = ros::Time::now();
      esdf_pointcloud_publisher_.publish(pointcloud_msg);
    }

    // Also publish the map slice (costmap for nav2).
    if (map_slice_publisher_.getNumSubscribers() > 0) {
      timing::Timer esdf_output_esdf_slice_timer("ros/esdf/output/slice");
      nvblox_msgs::DistanceMapSlice map_slice_msg;
      esdf_slice_converter_.distanceMapSliceImageToMsg(
          map_slice_image, aabb, esdf_slice_height_, mapper_->voxel_size_m(),
          &map_slice_msg);
      map_slice_msg.header.frame_id = global_frame_;
      map_slice_msg.header.stamp = ros::Time::now();
      map_slice_publisher_.publish(map_slice_msg);
    }
  }

  // Also publish the slice bounds (showing esdf max/min 2d height)
  if (slice_bounds_publisher_.getNumSubscribers() > 0) {
    // The frame to which the slice limits visualization is attached.
    // We get the transform from the plane-body (PB) frame, to the scene (S).
    Transform T_S_PB;
    if (transformer_.lookupTransformToGlobalFrame(
            slice_visualization_attachment_frame_id_, ros::Time(0), &T_S_PB)) {
      // Get and publish the planes representing the slice bounds in z.
      const visualization_msgs::Marker marker = sliceLimitsToMarker(
          T_S_PB, slice_visualization_side_length_, timestamp, global_frame_,
          esdf_2d_min_height_, esdf_2d_max_height_);
      slice_bounds_publisher_.publish(marker);
    } else {
      constexpr float kTimeBetweenDebugMessages = 1.0;
      ROS_INFO_STREAM_THROTTLE(
          kTimeBetweenDebugMessages,
          "Tried to publish slice bounds but couldn't look up frame: "
              << slice_visualization_attachment_frame_id_);
    }
  }

  // If we don't want the slice, output the full map.
  if (!esdf_distance_slice_ &&
      esdf_pointcloud_publisher_.getNumSubscribers() > 0) {
    timing::Timer esdf_output_esdf_full_map_timer("ros/esdf/output/full_cloud");
    sensor_msgs::PointCloud2 pointcloud_msg;
    layer_converter_.pointcloudMsgFromLayer(mapper_->esdf_layer(),
                                            &pointcloud_msg);
    pointcloud_msg.header.frame_id = global_frame_;
    pointcloud_msg.header.stamp = ros::Time::now();
    esdf_pointcloud_publisher_.publish(pointcloud_msg);
  }
}

void NvbloxNode::processMesh(const ros::TimerEvent& /*event*/) {
  std::unique_lock<std::mutex> lock(map_mutex_);

  const ros::Time timestamp = ros::Time::now();
  timing::Timer ros_total_timer("ros/total");
  timing::Timer ros_mesh_timer("ros/mesh");

  timing::Timer mesh_integration_timer("ros/mesh/integrate_and_color");
  const std::vector<Index3D> mesh_updated_list = mapper_->updateMesh();
  mesh_integration_timer.Stop();

  // In the case that some mesh blocks have been re-added after deletion, remove
  // them from the deleted list.
  for (const Index3D& idx : mesh_updated_list) {
    mesh_blocks_deleted_.erase(idx);
  }
  // Make a list to be published to rviz of blocks to be removed from the viz
  const std::vector<Index3D> mesh_blocks_to_delete(mesh_blocks_deleted_.begin(),
                                                   mesh_blocks_deleted_.end());
  mesh_blocks_deleted_.clear();

  bool should_publish = !mesh_updated_list.empty();

  // Publish the mesh updates.
  timing::Timer mesh_output_timer("ros/mesh/output");
  size_t new_subscriber_count = mesh_publisher_.getNumSubscribers();
  if (new_subscriber_count > 0) {
    nvblox_msgs::Mesh mesh_msg;
    // In case we have new subscribers, publish the ENTIRE map once.
    if (new_subscriber_count > mesh_subscriber_count_) {
      ROS_INFO("Got a new subscriber, sending entire map.");
      conversions::meshMessageFromMeshLayer(mapper_->mesh_layer(), &mesh_msg);
      mesh_msg.clear = true;
      should_publish = true;
    } else {
      conversions::meshMessageFromMeshBlocks(mapper_->mesh_layer(),
                                             mesh_updated_list, &mesh_msg,
                                             mesh_blocks_to_delete);
    }
    mesh_msg.header.frame_id = global_frame_;
    mesh_msg.header.stamp = timestamp;
    if (should_publish) {
      mesh_publisher_.publish(mesh_msg);
    }
  }
  mesh_subscriber_count_ = new_subscriber_count;

  // optionally publish the markers.
  if (mesh_marker_publisher_.getNumSubscribers() > 0) {
    visualization_msgs::MarkerArray marker_msg;
    conversions::markerMessageFromMeshLayer(mapper_->mesh_layer(),
                                            global_frame_, &marker_msg);
    mesh_marker_publisher_.publish(marker_msg);
  }

  mesh_output_timer.Stop();
}

bool NvbloxNode::canTransform(const std_msgs::Header& header) {
  Transform T_L_C;
  return transformer_.lookupTransformToGlobalFrame(header.frame_id,
                                                   header.stamp, &T_L_C);
}

bool NvbloxNode::isUpdateTooFrequent(const ros::Time& current_stamp,
                                     const ros::Time& last_update_stamp,
                                     float max_update_rate_hz) {
  if (max_update_rate_hz > 0.0f &&
      (current_stamp - last_update_stamp).toSec() < 1.0f / max_update_rate_hz) {
    return true;
  }
  return false;
}

bool NvbloxNode::processDepthImage(
    const std::pair<sensor_msgs::ImageConstPtr,
                    sensor_msgs::CameraInfo::ConstPtr>& depth_camera_pair) {
  ROS_DEBUG("Depth Image processing has started");
  timing::Timer ros_depth_timer("ros/depth");
  timing::Timer transform_timer("ros/depth/transform");

  // Message parts
  const sensor_msgs::ImageConstPtr& depth_img_ptr = depth_camera_pair.first;
  const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg =
      depth_camera_pair.second;

  // Check that we're not updating more quickly than we should.
  if (isUpdateTooFrequent(depth_img_ptr->header.stamp, last_depth_update_time_,
                          max_depth_update_hz_)) {
    return true;
  }
  last_depth_update_time_ = depth_img_ptr->header.stamp;

  // Get the TF for this image.
  Transform T_L_C;
  std::string target_frame = depth_img_ptr->header.frame_id;

  if (!transformer_.lookupTransformToGlobalFrame(
          target_frame, depth_img_ptr->header.stamp, &T_L_C)) {
    ROS_ERROR("Could not get transform from %s to %s", target_frame.c_str(),
              global_frame_.c_str());
    return false;
  }
  transform_timer.Stop();

  timing::Timer conversions_timer("ros/depth/conversions");
  // Convert camera info message to camera object.
  Camera camera = conversions::cameraFromMessage(*camera_info_msg);

  // Convert the depth image.
  if (!conversions::depthImageFromImageMessage(depth_img_ptr, &depth_image_)) {
    ROS_ERROR("Failed to transform depth image.");
    return false;
  }
  conversions_timer.Stop();

  // Integrate
  std::unique_lock<std::mutex> lock(map_mutex_);
  timing::Timer integration_timer("ros/depth/integrate");
  mapper_->integrateDepth(depth_image_, T_L_C, camera);
  ROS_DEBUG("Depth Camera based depth integration is done.");
  integration_timer.Stop();
  return true;
}

bool NvbloxNode::processColorImage(
    const std::pair<sensor_msgs::ImageConstPtr,
                    sensor_msgs::CameraInfo::ConstPtr>& color_camera_pair) {
  timing::Timer ros_color_timer("ros/color");
  timing::Timer transform_timer("ros/color/transform");

  const sensor_msgs::ImageConstPtr& color_img_ptr = color_camera_pair.first;
  const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg =
      color_camera_pair.second;

  // Check that we're not updating more quickly than we should.
  if (isUpdateTooFrequent(color_img_ptr->header.stamp, last_color_update_time_,
                          max_color_update_hz_)) {
    return true;
  }
  last_color_update_time_ = color_img_ptr->header.stamp;

  // Get the TF for this image.
  const std::string target_frame = color_img_ptr->header.frame_id;
  Transform T_L_C;

  if (!transformer_.lookupTransformToGlobalFrame(
          target_frame, color_img_ptr->header.stamp, &T_L_C)) {
    return false;
  }

  transform_timer.Stop();

  timing::Timer color_convert_timer("ros/color/conversion");

  // Convert camera info message to camera object.
  Camera camera = conversions::cameraFromMessage(*camera_info_msg);

  // Convert the color image.
  if (!conversions::colorImageFromImageMessage(color_img_ptr, &color_image_)) {
    ROS_ERROR("Failed to transform color image.");
    return false;
  }
  color_convert_timer.Stop();

  // Integrate.
  std::unique_lock<std::mutex> lock(map_mutex_);
  timing::Timer color_integrate_timer("ros/color/integrate");
  mapper_->integrateColor(color_image_, T_L_C, camera);
  color_integrate_timer.Stop();
  return true;
}

bool NvbloxNode::processLidarPointcloud(
    const sensor_msgs::PointCloud2::ConstPtr& pointcloud_ptr) {
  timing::Timer ros_lidar_timer("ros/lidar");
  timing::Timer transform_timer("ros/lidar/transform");

  // Check that we're not updating more quickly than we should.
  if (isUpdateTooFrequent(pointcloud_ptr->header.stamp, last_lidar_update_time_,
                          max_lidar_update_hz_)) {
    return true;
  }
  last_lidar_update_time_ = pointcloud_ptr->header.stamp;

  // Get the TF for this image.
  const std::string target_frame = pointcloud_ptr->header.frame_id;
  Transform T_L_C;

  if (!transformer_.lookupTransformToGlobalFrame(
          target_frame, pointcloud_ptr->header.stamp, &T_L_C)) {
    return false;
  }

  transform_timer.Stop();

  // LiDAR intrinsics model
  Lidar lidar(lidar_width_, lidar_height_,
              lidar_vertical_fov_deg_ * deg_to_rad);

  // We check that the pointcloud is consistent with this LiDAR model
  // NOTE(alexmillane): If the check fails we return true which indicates that
  // this pointcloud can be removed from the queue even though it wasn't
  // integrated (because the intrisics model is messed up).
  // NOTE(alexmillane): Note that internally we cache checks, so each LiDAR
  // intrisics model is only tested against a single pointcloud. This is because
  // the check is expensive to perform.
  if (!pointcloud_converter_.checkLidarPointcloud(pointcloud_ptr, lidar)) {
    ROS_ERROR("LiDAR intrinsics are inconsistent with the received pointcloud");
    return true;
  }

  timing::Timer lidar_conversion_timer("ros/lidar/conversion");
  pointcloud_converter_.depthImageFromPointcloudGPU(pointcloud_ptr, lidar,
                                                    &pointcloud_image_);

  lidar_conversion_timer.Stop();

  std::unique_lock<std::mutex> lock(map_mutex_);

  timing::Timer lidar_integration_timer("ros/lidar/integration");
  mapper_->integrateLidarDepth(pointcloud_image_, T_L_C, lidar);
  lidar_integration_timer.Stop();

  return true;
}

void NvbloxNode::publishOccupancyPointcloud(const ros::TimerEvent& /*event*/) {
  timing::Timer ros_total_timer("ros/total");
  timing::Timer esdf_output_timer("ros/occupancy/output");

  if (occupancy_publisher_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 pointcloud_msg;
    std::unique_lock<std::mutex> lock(map_mutex_);
    layer_converter_.pointcloudMsgFromLayer(mapper_->occupancy_layer(),
                                            &pointcloud_msg);
    pointcloud_msg.header.frame_id = global_frame_;
    pointcloud_msg.header.stamp = ros::Time::now();
    occupancy_publisher_.publish(pointcloud_msg);
  }
}

void NvbloxNode::clearMapOutsideOfRadiusOfLastKnownPose(
    const ros::TimerEvent& /*event*/) {
  if (map_clearing_radius_m_ > 0.0f) {
    timing::Timer("ros/clear_outside_radius");
    Transform T_L_MC;  // MC = map clearing frame
    if (transformer_.lookupTransformToGlobalFrame(map_clearing_frame_id_,
                                                  ros::Time(0), &T_L_MC)) {
      std::unique_lock<std::mutex> lock(map_mutex_);
      const std::vector<Index3D> blocks_cleared = mapper_->clearOutsideRadius(
          T_L_MC.translation(), map_clearing_radius_m_);
      // We keep track of the deleted blocks for publishing later.
      mesh_blocks_deleted_.insert(blocks_cleared.begin(), blocks_cleared.end());
    } else {
      constexpr float kTimeBetweenDebugMessages = 1.0;
      ROS_INFO_STREAM_THROTTLE(
          kTimeBetweenDebugMessages,
          "Tried to clear map outside of radius but couldn't look up frame: "
              << map_clearing_frame_id_);
    }
  }
}

// Helper function for ends with. :)
bool ends_with(const std::string& value, const std::string& ending) {
  if (ending.size() > value.size()) {
    return false;
  }
  return std::equal(ending.crbegin(), ending.crend(), value.crbegin(),
                    [](const unsigned char a, const unsigned char b) {
                      return std::tolower(a) == std::tolower(b);
                    });
}

bool NvbloxNode::savePly(nvblox_msgs::FilePath::Request& request,
                         nvblox_msgs::FilePath::Response& response) {
  // If we get a full path, then write to that path.
  bool success = false;
  if (ends_with(request.file_path, ".ply")) {
    std::unique_lock<std::mutex> lock(map_mutex_);
    success =
        io::outputMeshLayerToPly(mapper_->mesh_layer(), request.file_path);
  } else {
    // If we get a partial path then output a bunch of stuff to a folder.
    std::unique_lock<std::mutex> lock(map_mutex_);
    io::outputVoxelLayerToPly(mapper_->tsdf_layer(),
                              request.file_path + "/ros2_tsdf.ply");
    io::outputVoxelLayerToPly(mapper_->esdf_layer(),
                              request.file_path + "/ros2_esdf.ply");
    success = io::outputMeshLayerToPly(mapper_->mesh_layer(),
                                       request.file_path + "/ros2_mesh.ply");
  }
  if (success) {
    ROS_INFO_STREAM("Output PLY file(s) to " << request.file_path);
    response.success = true;
  } else {
    ROS_WARN_STREAM("Failed to write PLY file(s) to " << request.file_path);
    response.success = false;
  }
  return true;
}

bool NvbloxNode::saveMap(nvblox_msgs::FilePath::Request& request,
                         nvblox_msgs::FilePath::Response& response) {
  std::unique_lock<std::mutex> lock(map_mutex_);

  std::string filename = request.file_path;
  if (!ends_with(request.file_path, ".nvblx")) {
    filename += ".nvblx";
  }

  response.success = mapper_->saveMap(filename);
  if (response.success) {
    ROS_INFO_STREAM("Output map to file to " << filename);
  } else {
    ROS_WARN_STREAM("Failed to write file to " << filename);
  }
  return true;
}

bool NvbloxNode::loadMap(nvblox_msgs::FilePath::Request& request,
                         nvblox_msgs::FilePath::Response& response) {
  std::unique_lock<std::mutex> lock(map_mutex_);

  std::string filename = request.file_path;
  if (!ends_with(request.file_path, ".nvblx")) {
    filename += ".nvblx";
  }

  response.success = mapper_->loadMap(filename);
  if (response.success) {
    ROS_INFO_STREAM("Loaded map to file from " << filename);
  } else {
    ROS_WARN_STREAM("Failed to load map file from " << filename);
  }
  return true;
}

}  // namespace nvblox
