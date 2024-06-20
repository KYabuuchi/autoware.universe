#include "offline_nvtl_tool/topic_synchronizer.hpp"

#include <rclcpp/time.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <iostream>
#include <map>

namespace std
{
template <>
class hash<rclcpp::Time>
{
public:
  size_t operator()(const rclcpp::Time & time) const
  {
    return std::hash<int64_t>{}(time.nanoseconds());
  }
};
}  // namespace std

namespace
{
using Odometry = nav_msgs::msg::Odometry;
using DetectedObjects = autoware_perception_msgs::msg::DetectedObjects;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using Pose = geometry_msgs::msg::Pose;
using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
}  // namespace

Pose search_nearest_odometry(
  const std::map<rclcpp::Time, Pose> & odometry_map, const rclcpp::Time & query)
{
  const auto it = odometry_map.lower_bound(query);
  if (it == odometry_map.begin()) {
    return it->second;
  }
  if (it == odometry_map.end()) {
    return std::prev(it)->second;
  }
  // TODO: pick the nearest one from it and std::prev(it)
  // const auto prev_it = std::prev(it);

  // TODO: check whether the time difference is within a certain threshold
  return it->second;
}

std::vector<PointCloudWithPose> extract_pointcloud_with_pose(const RosbagReader & reader)
{
  // header.stamp.nanosec -> msg
  std::vector<PointCloud2> point_cloud_vector;
  std::unordered_map<rclcpp::Time, Pose> pose_map;
  std::unordered_map<rclcpp::Time, DetectedObjects> objects_map;
  std::map<rclcpp::Time, geometry_msgs::msg::Pose> odometry_map;

  // Extract all pose messages
  while (reader.has_next()) {
    const std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg = reader.read_next();

    if (msg->topic_name == "/perception/object_recognition/detection/centerpoint/objects") {
      const DetectedObjects objects = decode_with_type<DetectedObjects>(msg);
      objects_map.emplace(objects.header.stamp, objects);
    }
    if (msg->topic_name == "/localization/pose_estimator/pose_with_covariance") {
      const PoseCovStamped pose_cov_stamped = decode_with_type<PoseCovStamped>(msg);
      pose_map.emplace(pose_cov_stamped.header.stamp, pose_cov_stamped.pose.pose);
    }
    if (msg->topic_name == "/localization/kinematic_state") {
      const Odometry odometry = decode_with_type<Odometry>(msg);
      odometry_map.emplace(odometry.header.stamp, odometry.pose.pose);
    }
    if (msg->topic_name == "/localization/util/downsample/pointcloud") {
      const PointCloud2 point_cloud = decode_with_type<PointCloud2>(msg);
      point_cloud_vector.push_back(point_cloud);
    }
  }

  // Associated point cloud with pose
  std::vector<PointCloudWithPose> point_cloud_with_pose_array;

  for (const auto & point_cloud : point_cloud_vector) {
    const auto query_stamp = rclcpp::Time(point_cloud.header.stamp);

    if (pose_map.find(query_stamp) == pose_map.end()) {
      continue;
    }
    if (objects_map.find(query_stamp) == objects_map.end()) {
      continue;
    }
    const Pose & odometry_pose = search_nearest_odometry(odometry_map, query_stamp);

    point_cloud_with_pose_array.push_back(
      {point_cloud, pose_map.at(query_stamp), odometry_pose, objects_map.at(query_stamp)});
  }

  return point_cloud_with_pose_array;
}
