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
  no_ground_points_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("no_ground_points", 10);
  objects_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("objects", 10);
  objects_marker_pub_ = this->create_publisher<MarkerArray>("objects_marker", 10);
  around_nvtl_pub_ = this->create_publisher<MarkerArray>("around_nvtl", 10);

  const std::string input_rosbag_path = this->declare_parameter<std::string>("input_rosbag_path");

  //
  const auto reader = RosbagReader(input_rosbag_path);
  //
  nvtl_file_ = std::ofstream("nvtl.csv");

  // Create NDT
  const std::string pcd_path = declare_parameter<std::string>("pcd_path");
  PointCloud2 map_msg;
  if (pcd_path != "") {
    map_msg = load_map_as_msg(pcd_path);
  } else {
    map_msg = extract_map_pointcloud(reader);
  }

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
    pcl::PointCloud<pcl::PointXYZ> cloud_in_base_frame;
    pcl::fromROSMsg(sensor_and_pose.pointcloud, cloud_in_base_frame);

    const pcl::PointCloud<pcl::PointXYZ> no_dynamic_pointcloud =
      exclude_object_points(sensor_and_pose);
    const auto no_ground_pointcloud = extract_no_ground(no_dynamic_pointcloud);

    const double raw_nvtl = ndt.get_nvtl(cloud_in_base_frame, sensor_and_pose.pose);
    const double no_ground_nvtl = ndt.get_nvtl(no_ground_pointcloud, sensor_and_pose.pose);
    const double no_dynamic_nvtl = ndt.get_nvtl(no_dynamic_pointcloud, sensor_and_pose.pose);

    const auto position = sensor_and_pose.pose.position;
    RCLCPP_INFO_STREAM(
      this->get_logger(), " raw NVTL: " << raw_nvtl << " no-dynamic NVTL: " << no_dynamic_nvtl
                                        << " no-ground NVTL: " << no_ground_nvtl);

    // Compute nvtl around ego position
    std::unordered_map<std::pair<int, int>, double> around_nvtl;
    for (int i = -6; i <= 6; i++) {
      for (int j = -6; j <= 6; j++) {
        auto offsetted_pose = sensor_and_pose.pose;
        offsetted_pose.position.x += i * offset_interval_;
        offsetted_pose.position.y += j * offset_interval_;
        const double offsetted_nvtl = ndt.get_nvtl(no_dynamic_pointcloud, offsetted_pose);
        around_nvtl.emplace(std::make_pair(i, j), offsetted_nvtl);
        if (i == 0 && j == 0) {
          RCLCPP_INFO_STREAM(this->get_logger(), "offsetted NVTL: " << offsetted_nvtl);
        }
      }
    }
    publish_around_nvtl(around_nvtl);

    // Write to file
    {
      const auto stamp = rclcpp::Time(sensor_and_pose.pointcloud.header.stamp);
      nvtl_file_ << stamp.nanoseconds() << "," << position.x << "," << position.y << ","
                 << position.z << "," << raw_nvtl << "," << no_dynamic_nvtl << "," << no_ground_nvtl
                 << std::endl;
    }

    // Publish for visualization
    {
      // Publish tf
      {
        geometry_msgs::msg::PoseStamped pose_stamped_msg;
        pose_stamped_msg.header.stamp = this->get_clock()->now();
        pose_stamped_msg.header.frame_id = "map";
        pose_stamped_msg.pose = sensor_and_pose.pose;
        tf2_broadcaster_.sendTransform(
          tier4_autoware_utils::pose2transform(pose_stamped_msg, "base_link"));
      }
      auto publish_point_cloud = [this](
                                   rclcpp::Publisher<PointCloud2>::SharedPtr publisher,
                                   pcl::PointCloud<pcl::PointXYZ> pointcloud) {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(pointcloud, cloud_msg);
        cloud_msg.header.stamp = this->get_clock()->now();
        cloud_msg.header.frame_id = "base_link";
        publisher->publish(cloud_msg);
      };
      // Publish raw pointcloud
      publish_point_cloud(lidar_points_pub_, cloud_in_base_frame);
      // Publish no-dynamic pointcloud
      publish_point_cloud(static_lidar_points_pub_, no_dynamic_pointcloud);
      // Publish no-ground-dynamic points
      publish_point_cloud(no_ground_points_pub_, no_ground_pointcloud);
      // Publish objects
      publish_objects(sensor_and_pose.objects, "base_link");
    }

    if (!rclcpp::ok()) {
      break;
    }
    loop_rate.sleep();
  }
}

pcl::PointCloud<pcl::PointXYZ> OfflineNvtlTool::exclude_object_points(
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

  return *cloud_in_base_frame;
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

pcl::PointCloud<pcl::PointXYZ> OfflineNvtlTool::extract_no_ground(
  const pcl::PointCloud<pcl::PointXYZ> & pointcloud)
{
  // whether use no ground points to calculate score
  // remove ground
  pcl::PointCloud<pcl::PointXYZ> no_ground_points_in_base;

  constexpr double z_margin_for_ground_removal = 0.8;

  for (std::size_t i = 0; i < pointcloud.size(); i++) {
    const float point_z = pointcloud.points[i].z;  // NOLINT
    if (point_z > z_margin_for_ground_removal) {
      no_ground_points_in_base.push_back(pointcloud.points[i]);
    }
  }

  return no_ground_points_in_base;
}

void OfflineNvtlTool::publish_around_nvtl(
  std::unordered_map<std::pair<int, int>, double> around_nvtl)
{
  MarkerArray marker_array;
  int marker_id = 0;

  auto color_scale = [](double value) -> std_msgs::msg::ColorRGBA {
    // clang-format off
    double r = 1.0, g = 1.0, b = 1.0;
    value = std::clamp(value, 0.0, 1.0);
    if (value < 0.25) {
      r = 0; g = 4 * (value);
    } else if (value < 0.5) {
      r = 0; b = 1 + 4 * (0.25 - value);
    } else if (value < 0.75) {
      r = 4 * (value - 0.5); b = 0;
    } else {
      g = 1 + 4 * (0.75 - value); b = 0;
    }
    // clang-format on
    return std_msgs::msg::ColorRGBA().set__r(r).set__g(g).set__b(b).set__a(0.8);
  };

  auto normalize = [color_scale](double score) -> double {
    constexpr double max_score = 3.0;
    constexpr double min_score = 2.3;
    const double normalized_score = (score - min_score) / (max_score - min_score);
    return std::clamp(normalized_score, 0.0, 1.0);
  };

  for (const auto & [offset, nvtl] : around_nvtl) {
    const double normalized_score = normalize(nvtl);

    Marker marker;
    marker.id = marker_id++;
    marker.header.frame_id = "base_link";
    marker.type = Marker::CUBE;
    marker.pose.position.set__x(offset.first * offset_interval_)
      .set__y(offset.second * offset_interval_)
      .set__z(0.0);
    marker.scale.set__x(0.5).set__y(0.5).set__z(normalized_score + 0.1);
    marker.color = color_scale(normalized_score);

    // append
    marker_array.markers.push_back(marker);
  }
  around_nvtl_pub_->publish(marker_array);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::make_shared<OfflineNvtlTool>();
  return 0;
}