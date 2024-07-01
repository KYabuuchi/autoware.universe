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
using Float32Stamped = tier4_debug_msgs::msg::Float32Stamped;
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

std::vector<SynchronizedData> extract_synchronized_data(const RosbagReader & reader)
{
  // header.stamp.nanosec -> msg
  std::vector<PointCloud2> point_cloud_vector;
  std::unordered_map<rclcpp::Time, Pose> pose_map;
  std::unordered_map<rclcpp::Time, DetectedObjects> objects_map;
  std::unordered_map<rclcpp::Time, Float32Stamped> score_map;
  std::map<rclcpp::Time, geometry_msgs::msg::Pose> odometry_ordered_map;

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
    if (msg->topic_name == "/localization/pose_estimator/nearest_voxel_transformation_likelihood") {
      const Float32Stamped score = decode_with_type<Float32Stamped>(msg);
      score_map.emplace(score.stamp, score);
    }
    if (msg->topic_name == "/localization/kinematic_state") {
      const Odometry odometry = decode_with_type<Odometry>(msg);
      odometry_ordered_map.emplace(odometry.header.stamp, odometry.pose.pose);
    }
    if (msg->topic_name == "/localization/util/downsample/pointcloud") {
      const PointCloud2 point_cloud = decode_with_type<PointCloud2>(msg);
      point_cloud_vector.push_back(point_cloud);
    }
  }

  // Associated point cloud with pose
  std::vector<SynchronizedData> synchronized_data;

  for (const auto & point_cloud : point_cloud_vector) {
    const auto query_stamp = rclcpp::Time(point_cloud.header.stamp);
    const Pose & odometry_pose = search_nearest_odometry(odometry_ordered_map, query_stamp);

    if (objects_map.find(query_stamp) == objects_map.end()) {
      continue;
    }

    if (score_map.find(query_stamp) == score_map.end()) {
      continue;
    }

    std::optional<Pose> ndt_pose = std::nullopt;
    if (pose_map.find(query_stamp) != pose_map.end()) {
      ndt_pose = pose_map.at(query_stamp);
    }

    synchronized_data.push_back(
      {point_cloud, objects_map.at(query_stamp), score_map.at(query_stamp), odometry_pose,
       ndt_pose});
  }

  return synchronized_data;
}
