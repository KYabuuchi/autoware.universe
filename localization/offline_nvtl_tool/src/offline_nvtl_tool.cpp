// Copyright 2024 TIER IV
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

#include "offline_nvtl_tool/offline_nvtl_tool.hpp"

#include "offline_nvtl_tool/ndt_interface.hpp"

#include <localization_util/util_func.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <pcl/filters/crop_box.h>

#include <iostream>
#include <map>
#include <memory>

OfflineNvtlTool::OfflineNvtlTool()
: Node("offline_nvtl_tool"), tf2_broadcaster_(*this), margin_(declare_parameter<double>("margin"))
{
  // Create publisher
  map_points_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("map", rclcpp::QoS(1).transient_local());
  lidar_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar", 10);
  static_lidar_points_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("static_lidar", 10);
  objects_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("objects", 10);
  objects_marker_pub_ = this->create_publisher<MarkerArray>("objects_marker", 10);

  const std::string input_rosbag_path = this->declare_parameter<std::string>("input_rosbag_path");

  //
  const auto reader = RosbagReader(input_rosbag_path);
  //
  nvtl_file_ = std::ofstream("nvtl.csv");

  // Create NDT
  const auto map_msg = extract_map_pointcloud(reader);
  // const auto map_msg = load_map_as_msg(declare_parameter<std::string>("pcd_path"));
  NdtInterface ndt{this};
  ndt.set_pointcloud_map(map_msg);
  {
    map_points_pub_->publish(map_msg);
  }

  // Iterate all pose & point cloud
  const auto associated_sensor_and_pose = extract_pointcloud_with_pose(reader);

  RCLCPP_INFO_STREAM(this->get_logger(), "Start processing: " << associated_sensor_and_pose.size());

  // Publish tf: map -> viewer
  {
    const auto msg = associated_sensor_and_pose.front();
    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header.stamp = this->get_clock()->now();
    pose_stamped_msg.header.frame_id = "map";
    pose_stamped_msg.pose = msg.pose;
    tf2_broadcaster_.sendTransform(
      tier4_autoware_utils::pose2transform(pose_stamped_msg, "viewer"));
  }

  rclcpp::Rate loop_rate(10);  // 10 fps

  // Compute normal NVTL
  for (const auto & sensor_and_pose : associated_sensor_and_pose) {
    const PointCloud2 static_pointcloud_msg = exclude_object_points(sensor_and_pose);

    // Publish tf
    {
      geometry_msgs::msg::PoseStamped pose_stamped_msg;
      pose_stamped_msg.header.stamp = this->get_clock()->now();
      pose_stamped_msg.header.frame_id = "map";
      pose_stamped_msg.pose = sensor_and_pose.pose;
      tf2_broadcaster_.sendTransform(
        tier4_autoware_utils::pose2transform(pose_stamped_msg, "base_link"));
    }
    // Publish lidar pointcloud
    {
      PointCloud2 msg_with_now_stamp = sensor_and_pose.pointcloud;
      msg_with_now_stamp.header.stamp = this->get_clock()->now();
      lidar_points_pub_->publish(msg_with_now_stamp);
    }
    // Publish static lidar pointcloud
    {
      PointCloud2 msg_with_now_stamp = static_pointcloud_msg;
      msg_with_now_stamp.header.stamp = this->get_clock()->now();
      static_lidar_points_pub_->publish(msg_with_now_stamp);
    }
    // Publish objects
    publish_objects(sensor_and_pose.objects, "base_link");

    const double raw_nvtl = ndt.get_nvtl(sensor_and_pose.pointcloud, sensor_and_pose.pose);
    const double static_nvtl = ndt.get_nvtl(static_pointcloud_msg, sensor_and_pose.pose);

    const auto position = sensor_and_pose.pose.position;
    RCLCPP_INFO_STREAM(
      this->get_logger(), " raw NVTL: " << raw_nvtl << " static NVTL: " << static_nvtl);

    // Write to file
    {
      const auto stamp = rclcpp::Time(sensor_and_pose.pointcloud.header.stamp);
      nvtl_file_ << stamp.nanoseconds() << "," << position.x << "," << position.y << ","
                 << position.z << "," << raw_nvtl << "," << static_nvtl << std::endl;
    }

    if (!rclcpp::ok()) {
      break;
    }
    loop_rate.sleep();
  }
}

OfflineNvtlTool::PointCloud2 OfflineNvtlTool::exclude_object_points(
  const PointCloudWithPose & point_cloud_with_pose) const
{
  const auto & objects = point_cloud_with_pose.objects;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_base_frame =
    pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(point_cloud_with_pose.pointcloud, *cloud_in_base_frame);

  for (const auto & object : objects.objects) {
    pcl::CropBox<pcl::PointXYZ> crop_box_filter;
    crop_box_filter.setInputCloud(cloud_in_base_frame);

    if (object.shape.type == object.shape.CYLINDER) {
      RCLCPP_ERROR_STREAM(get_logger(), "CYLINDER is not supported yet");
      continue;
    }
    if (object.shape.type == object.shape.POLYGON) {
      RCLCPP_ERROR_STREAM(get_logger(), "POLYGON is not supported yet");
      continue;
    }
    if (object.shape.type != object.shape.BOUNDING_BOX) {
      throw std::runtime_error("unsupported shape");
    }

    // Set the bounding box
    const auto dimensions = object.shape.dimensions;
    const Eigen::Vector4f max_with_margin(
      dimensions.x * margin_ * 0.5, dimensions.y * margin_ * 0.5, dimensions.z * margin_ * 0.5,
      1.0);
    crop_box_filter.setMin(-max_with_margin);
    crop_box_filter.setMax(max_with_margin);

    crop_box_filter.setTransform(
      Eigen::Affine3f(pose_to_matrix4f(object.kinematics.pose_with_covariance.pose)).inverse());
    crop_box_filter.setNegative(true);

    crop_box_filter.filter(*cloud_in_base_frame);
  }

  PointCloud2 filtered_pointcloud_msg;
  pcl::toROSMsg(*cloud_in_base_frame, filtered_pointcloud_msg);
  return filtered_pointcloud_msg;
}

OfflineNvtlTool::PointCloud2 OfflineNvtlTool::load_map_as_msg(const std::string & pcd_path) const
{
  RCLCPP_INFO_STREAM(get_logger(), "start to load pcd: " << pcd_path);
  pcl::PointCloud<pcl::PointXYZ> map_pointcloud;
  pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, map_pointcloud);

  PointCloud2 map_pointcloud_msg;
  pcl::toROSMsg(map_pointcloud, map_pointcloud_msg);
  map_pointcloud_msg.header.frame_id = "map";

  RCLCPP_INFO_STREAM(get_logger(), "finish to load pcd");
  return map_pointcloud_msg;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::make_shared<OfflineNvtlTool>();
  return 0;
}