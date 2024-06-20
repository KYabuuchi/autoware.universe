#pragma once
#include "offline_nvtl_tool/rosbag_handler.hpp"

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <vector>

struct PointCloudWithPose
{
  sensor_msgs::msg::PointCloud2 pointcloud;
  geometry_msgs::msg::Pose ndt_pose;
  geometry_msgs::msg::Pose ekf_pose;
  autoware_perception_msgs::msg::DetectedObjects objects;
};

std::vector<PointCloudWithPose> extract_pointcloud_with_pose(const RosbagReader & reader);