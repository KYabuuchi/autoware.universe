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

#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <iostream>
#include <map>
#include <memory>

OfflineNvtlTool::OfflineNvtlTool() : Node("offline_nvtl_tool"), tf2_broadcaster_(*this)
{
  map_points_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("map", rclcpp::QoS(1).transient_local());
  lidar_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar", 10);
  objects_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("objects", 10);

  const std::string input_rosbag_path = this->declare_parameter<std::string>("input_rosbag_path");

  const auto reader = RosbagReader(input_rosbag_path);

  // Create NDT
  const auto map_msg = extract_map_pointcloud(reader);
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
    // Publish objects
    publish_objects(sensor_and_pose.objects, "base_link");

    const double nvtl = ndt.get_nvtl(sensor_and_pose.pointcloud, sensor_and_pose.pose);

    RCLCPP_INFO_STREAM(this->get_logger(), "NVTL: " << nvtl);

    if (!rclcpp::ok()) {
      break;
    }
    loop_rate.sleep();
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::make_shared<OfflineNvtlTool>();
  return 0;
}