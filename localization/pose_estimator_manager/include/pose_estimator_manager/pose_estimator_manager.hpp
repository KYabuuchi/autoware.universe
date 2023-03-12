#ifndef POSE_ESTIMATOR_MANAGER_
#define POSE_ESTIMATOR_MANAGER_

#include "pose_estimator_manager/manager_client.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace multi_pose_estimator
{
class PoseEstimatorManager : public rclcpp::Node
{
public:
  using SetBool = std_srvs::srv::SetBool;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  PoseEstimatorManager();

private:
  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_pose_cov_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_map_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Service<SetBool>::SharedPtr switch_service_server_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unordered_map<std::string, ManagerClient::SharedPtr> clients_;
  std::optional<PoseCovStamped> latest_pose_{std::nullopt};

  pcl::PointCloud<pcl::PointXYZ> occupied_areas_;

  bool toggle_mode(bool enable_ndt);
  void publish_occupied_area();

  void on_service(SetBool::Request::ConstSharedPtr request, SetBool::Response::SharedPtr response);
  void on_map(PointCloud2::ConstSharedPtr msg);
  void on_timer();
  void on_pose_cov(PoseCovStamped::ConstSharedPtr msg);
};
}  // namespace multi_pose_estimator

#endif /* POSE_ESTIMATOR_MANAGER_ */
