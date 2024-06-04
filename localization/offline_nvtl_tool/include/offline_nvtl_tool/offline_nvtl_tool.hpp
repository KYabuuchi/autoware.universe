#pragma once
#include "offline_nvtl_tool/rosbag_handler.hpp"

#include <rclcpp/node.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>

class OfflineNvtlTool : public rclcpp::Node
{
public:
  using DetectedObjects = autoware_auto_perception_msgs::msg::DetectedObjects;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Pose = geometry_msgs::msg::Pose;

  OfflineNvtlTool();

private:
  struct PointCloudWithPose
  {
    PointCloud2 pointcloud;
    Pose pose;
    DetectedObjects objects;
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
    std::unordered_map<uint32_t, DetectedObjects> objects_map;

    // Extract all pose messages
    while (reader.has_next()) {
      const std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg = reader.read_next();

      if (msg->topic_name == "/perception/object_recognition/detection/centerpoint/objects") {
        const DetectedObjects objects = decode_with_type<DetectedObjects>(msg);
        objects_map.emplace(objects.header.stamp.nanosec, objects);
      }
      if (msg->topic_name == "/localization/pose_estimator/pose_with_covariance") {
        const PoseCovStamped pose_cov_stamped = decode_with_type<PoseCovStamped>(msg);
        pose_map.emplace(pose_cov_stamped.header.stamp.nanosec, pose_cov_stamped.pose.pose);
      }
      if (msg->topic_name == "/localization/util/downsample/pointcloud") {
        const PointCloud2 point_cloud = decode_with_type<PointCloud2>(msg);
        point_cloud_vector.push_back(point_cloud);
      }
    }

    // Associated point cloud with pose
    std::vector<PointCloudWithPose> point_cloud_with_pose_array;

    for (const auto & point_cloud : point_cloud_vector) {
      const uint32_t nanosec = point_cloud.header.stamp.nanosec;
      if (pose_map.find(nanosec) == pose_map.end()) {
        continue;
      }
      if (objects_map.find(nanosec) == objects_map.end()) {
        continue;
      }

      point_cloud_with_pose_array.push_back(
        {point_cloud, pose_map.at(nanosec), objects_map.at(nanosec)});
    }

    RCLCPP_INFO_STREAM(
      this->get_logger(), "Extracted " << point_cloud_with_pose_array.size() << " dataset");

    return point_cloud_with_pose_array;
  }

  void publish_objects(const DetectedObjects & objects, const std::string & frame_id)
  {
    pcl::PointCloud<pcl::PointXYZ> object_point_cloud;
    for (const auto & object : objects.objects) {
      const auto position = object.kinematics.pose_with_covariance.pose.position;
      object_point_cloud.push_back(pcl::PointXYZ(position.x, position.y, position.z));
    }

    sensor_msgs::msg::PointCloud2 object_points_msg;
    pcl::toROSMsg(object_point_cloud, object_points_msg);
    object_points_msg.header.stamp = this->get_clock()->now();
    object_points_msg.header.frame_id = frame_id;
    objects_pub_->publish(object_points_msg);
  }

private:
  rclcpp::Publisher<PointCloud2>::SharedPtr map_points_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr lidar_points_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr objects_pub_;
  tf2_ros::TransformBroadcaster tf2_broadcaster_;
};