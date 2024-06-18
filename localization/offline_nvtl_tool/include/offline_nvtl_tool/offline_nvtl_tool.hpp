#pragma once
#include "offline_nvtl_tool/rosbag_handler.hpp"

#include <rclcpp/node.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>

#include <fstream>
namespace std
{
template <>
class hash<std::pair<int, int>>
{
public:
  size_t operator()(const std::pair<int, int> & p) const
  {
    return (p.first * 73856093 ^ p.second * 19349663);
  }
};
}  // namespace std

struct MyTime
{
  int32_t sec;
  uint32_t nanosec;
  MyTime(const builtin_interfaces::msg::Time & time) : sec(time.sec), nanosec(time.nanosec) {}

  bool operator<(const MyTime & other) const
  {
    if (sec == other.sec) {
      return nanosec < other.nanosec;
    }
    return sec < other.sec;
  }
};
using OdometryMap = std::map<MyTime, geometry_msgs::msg::Pose>;

geometry_msgs::msg::Pose search_nearest_odometry(
  const OdometryMap & odometry_map, const MyTime query)
{
  const auto it = odometry_map.lower_bound(query);
  if (it == odometry_map.begin()) {
    return it->second;
  }
  if (it == odometry_map.end()) {
    return std::prev(it)->second;
  }
  // const auto prev_it = std::prev(it);
  // prev_it->second is fine too
  return it->second;
}

class OfflineNvtlTool : public rclcpp::Node
{
public:
  using DetectedObjects = autoware_perception_msgs::msg::DetectedObjects;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Pose = geometry_msgs::msg::Pose;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Odometry = nav_msgs::msg::Odometry;

  OfflineNvtlTool();

private:
  struct PointCloudWithPose
  {
    PointCloud2 pointcloud;
    Pose pose;
    DetectedObjects objects;
  };

  static pcl::PointCloud<pcl::PointXYZ> extract_map_pointcloud(const RosbagReader & reader)
  {
    while (reader.has_next()) {
      const std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg = reader.read_next();
      if (msg->topic_name == "/map/pointcloud_map") {
        const PointCloud2 pointcloud_msg = decode_with_type<PointCloud2>(msg);
        pcl::PointCloud<pcl::PointXYZ> pointcloud;
        pcl::fromROSMsg(pointcloud_msg, pointcloud);
        return pointcloud;
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
    OdometryMap odometry_map;

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
      const uint32_t nanosec = point_cloud.header.stamp.nanosec;
      // if (pose_map.find(nanosec) == pose_map.end()) {
      //   continue;
      // }
      if (objects_map.find(nanosec) == objects_map.end()) {
        continue;
      }

      const Pose odometry_pose =
        search_nearest_odometry(odometry_map, MyTime{point_cloud.header.stamp});

      point_cloud_with_pose_array.push_back({point_cloud, odometry_pose, objects_map.at(nanosec)});
    }

    RCLCPP_INFO_STREAM(
      this->get_logger(), "Extracted " << point_cloud_with_pose_array.size() << " dataset");

    return point_cloud_with_pose_array;
  }

  void publish_objects(const DetectedObjects & objects, const std::string & frame_id)
  {
    // Publish objects as point cloud
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

    // Publish objects as marker array
    MarkerArray marker_array;
    marker_array.markers.push_back(Marker()
                                     .set__type(Marker::DELETEALL)
                                     .set__header(std_msgs::msg::Header().set__frame_id(
                                       "base_link")));  // delete all previous markers
    int marker_id = 1;
    for (const auto & object : objects.objects) {
      Marker marker;
      marker.id = marker_id++;
      marker.header.frame_id = "base_link";
      marker.pose = object.kinematics.pose_with_covariance.pose;

      if (object.shape.type == object.shape.CYLINDER) {
        marker.type = Marker::CYLINDER;
      }
      if (object.shape.type == object.shape.BOUNDING_BOX) {
        marker.type = Marker::CUBE;
      }
      if (object.shape.type == object.shape.POLYGON) {
        std::cout << "POLYGON" << std::endl;
        throw std::runtime_error("POLYGON is not supported yet.");
      }
      marker.scale = object.shape.dimensions;
      marker.color.set__a(0.2).set__r(1.0).set__g(1.0).set__b(1.0);
      marker.lifetime = rclcpp::Duration(0, 0.3 * 1e9);  // 0.3 sec

      // append
      marker_array.markers.push_back(marker);
    }
    objects_marker_pub_->publish(marker_array);
  }

  pcl::PointCloud<pcl::PointXYZ> exclude_object_points(const PointCloudWithPose &) const;

  PointCloud2 load_map_as_msg(const std::string & pcd_path) const;

  pcl::PointCloud<pcl::PointXYZ> extract_no_ground(
    const pcl::PointCloud<pcl::PointXYZ> & pointcloud);

  void publish_around_nvtl(std::unordered_map<std::pair<int, int>, double> around_nvtl);

private:
  rclcpp::Publisher<PointCloud2>::SharedPtr map_points_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr lidar_points_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr static_lidar_points_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr objects_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr no_ground_points_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr objects_marker_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr around_nvtl_pub_;
  tf2_ros::TransformBroadcaster tf2_broadcaster_;

  std::ofstream nvtl_file_;
  const double margin_;
  const double offset_interval_{0.5};
};