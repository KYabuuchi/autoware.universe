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
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <iostream>
#include <map>
#include <memory>

class OfflineNvtlTool : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Pose = geometry_msgs::msg::Pose;

  OfflineNvtlTool() : Node("offline_nvtl_tool"), tf2_broadcaster_(*this)
  {
    map_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "map", rclcpp::QoS(1).transient_local());
    lidar_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar", 10);

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

    RCLCPP_INFO_STREAM(
      this->get_logger(), "Start processing: " << associated_sensor_and_pose.size());

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
    for (const auto & lidar_and_pose : associated_sensor_and_pose) {
      // Publish tf
      {
        geometry_msgs::msg::PoseStamped pose_stamped_msg;
        pose_stamped_msg.header.stamp = this->get_clock()->now();
        pose_stamped_msg.header.frame_id = "map";
        pose_stamped_msg.pose = lidar_and_pose.pose;
        tf2_broadcaster_.sendTransform(
          tier4_autoware_utils::pose2transform(pose_stamped_msg, "base_link"));
      }
      // // Publish lidar pointcloud
      // {
      //   PointCloud2 msg_with_now_stamp = lidar_and_pose.pointcloud;
      //   msg_with_now_stamp.header.stamp = this->get_clock()->now();
      //   lidar_points_pub_->publish(msg_with_now_stamp);
      // }

      const auto [nvtl, msg_in_map_frame] =
        ndt.get_nvtl(lidar_and_pose.pointcloud, lidar_and_pose.pose);

      // Publish lidar pointcloud
      {
        PointCloud2 msg_with_now_stamp = msg_in_map_frame;
        msg_with_now_stamp.header.stamp = this->get_clock()->now();
        msg_with_now_stamp.header.frame_id = "map";
        lidar_points_pub_->publish(msg_with_now_stamp);
      }

      RCLCPP_INFO_STREAM(this->get_logger(), "NVTL: " << nvtl);

      if (!rclcpp::ok()) {
        break;
      }
      loop_rate.sleep();
    }
  }

private:
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
    std::vector<PointCloud2> point_cloud_vector;
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
        point_cloud_vector.push_back(point_cloud);
      }
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Extracted " << pose_map.size() << " poses");
    RCLCPP_INFO_STREAM(
      this->get_logger(), "Extracted " << point_cloud_vector.size() << " pointcloud");

    // Associated point cloud with pose
    std::vector<PointCloudWithPose> point_cloud_with_pose_array;

    for (const auto & point_cloud : point_cloud_vector) {
      const uint32_t nanosec = point_cloud.header.stamp.nanosec;
      if (pose_map.find(nanosec) != pose_map.end()) {
        point_cloud_with_pose_array.push_back({point_cloud, pose_map.at(nanosec)});
      }
    }

    return point_cloud_with_pose_array;
  }

private:
  rclcpp::Publisher<PointCloud2>::SharedPtr map_points_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr lidar_points_pub_;
  tf2_ros::TransformBroadcaster tf2_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::make_shared<OfflineNvtlTool>();
  return 0;
}