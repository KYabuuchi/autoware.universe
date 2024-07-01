#pragma once
#include "offline_nvtl_tool/rosbag_handler.hpp"

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>

#include <optional>
#include <vector>

struct SynchronizedData
{
  sensor_msgs::msg::PointCloud2 pointcloud;

  autoware_perception_msgs::msg::DetectedObjects objects;

  tier4_debug_msgs::msg::Float32Stamped score;

  // EKF pose's stamp is not synchronized with other topics.
  // It is just nearest pose to the other topics stamp.
  geometry_msgs::msg::Pose ekf_pose;

  // NDT pose sometimes is not available due to low score
  std::optional<geometry_msgs::msg::Pose> ndt_pose = std::nullopt;
};

std::vector<SynchronizedData> extract_synchronized_data(const RosbagReader & reader);