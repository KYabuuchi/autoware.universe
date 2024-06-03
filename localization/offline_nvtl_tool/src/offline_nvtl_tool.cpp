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

#include "offline_nvtl_tool/ndt.hpp"
#include "offline_nvtl_tool/rosbag_handler.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <iostream>
#include <map>
#include <memory>

class OfflineNvtlTool : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Pose = geometry_msgs::msg::Pose;

  OfflineNvtlTool() : Node("offline_nvtl_tool")
  {
    sensor_in_pose_frame_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points", 10);

    const std::string input_rosbag_path = this->declare_parameter<std::string>("input_rosbag_path");

    const auto reader = RosbagReader(input_rosbag_path);

    // Create NDT
    const auto map_msg = extract_map_pointcloud(reader);
    NdtInterface ndt{this};
    ndt.set_pointcloud_map(map_msg);

    // Iterate all pose & point cloud
    const auto associated_sensor_and_pose = extract_pointcloud_with_pose(reader);

    RCLCPP_INFO_STREAM(
      this->get_logger(), "Start processing: " << associated_sensor_and_pose.size());

    // Compute normatl NVTL
    // TODO:
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_in_pose_frame_pub_;

  struct PointCloudWithPose
  {
    PointCloud2 pointcloud;
    Pose pose;
  };

  static PointCloud2 extract_map_pointcloud(const RosbagReader & reader)
  {
    while (reader.has_next()) {
      const std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg = reader.read_next();
      if (msg->topic_name == "/map/pointcloud_map") {
        return decode_with_type<PointCloud2>(msg);
      }
    }
    throw std::runtime_error("No map pointcloud found in rosbag");
  }

  std::vector<PointCloudWithPose> extract_pointcloud_with_pose(const RosbagReader & reader)
  {
    // header.stamp.nanosec -> msg
    std::unordered_map<uint32_t, PointCloud2> point_cloud_map;
    std::unordered_map<uint32_t, Pose> pose_map;

    // Extract all pose messages
    while (reader.has_next()) {
      const std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg = reader.read_next();

      if (msg->topic_name == "/localization/pose_estimator/pose_with_covariance") {
        const PoseCovStamped pose_cov_stamped = decode_with_type<PoseCovStamped>(msg);
        pose_map.emplace(pose_cov_stamped.header.stamp.nanosec, pose_cov_stamped.pose.pose);
      }
      if (msg->topic_name == "/localization/util/downsample/pointcloud") {
        const PointCloud2 point_cloud = decode_with_type<PointCloud2>(msg);
        point_cloud_map.emplace(point_cloud.header.stamp.nanosec, point_cloud);
      }
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Extracted " << pose_map.size() << " poses");
    RCLCPP_INFO_STREAM(this->get_logger(), "Extracted " << point_cloud_map.size() << " pointcloud");

    // Associated point cloud with pose
    std::vector<PointCloudWithPose> point_cloud_with_pose_array;

    for (const auto [stamp, pose] : pose_map) {
      if (point_cloud_map.find(stamp) != point_cloud_map.end()) {
        point_cloud_with_pose_array.push_back({point_cloud_map[stamp], pose});
      }
    }

    return point_cloud_with_pose_array;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OfflineNvtlTool>();
  rclcpp::spin(node);

  return 0;
}