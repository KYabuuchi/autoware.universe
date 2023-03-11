#include "pose_estimator_manager/pose_estimator_manager.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
namespace multi_pose_estimator
{
PoseEstimatorManager::PoseEstimatorManager() : Node("pose_estimator_manager")
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  const auto map_qos = rclcpp::QoS(1).transient_local().reliable();

  auto on_map = std::bind(&PoseEstimatorManager::on_map, this, _1);
  sub_map_ = create_subscription<PointCloud2>("/map/pointcloud", map_qos, on_map);
  auto on_pose_cov = std::bind(&PoseEstimatorManager::on_pose_cov, this, _1);
  sub_pose_cov_ =
    create_subscription<PoseCovStamped>("/localization/pose_with_covariance", 10, on_pose_cov);

  // Service server
  auto on_service = std::bind(&PoseEstimatorManager::on_service, this, _1, _2);
  switch_service_server_ = create_service<SetBool>("/toggle_ndt", on_service);

  // Service client
  service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  clients_.emplace(
    "ndt", std::make_shared<ManagerClient>(this, "/ndt_enable_srv", service_callback_group_));
  clients_.emplace(
    "pcdless",
    std::make_shared<ManagerClient>(this, "/pcdless_enable_srv", service_callback_group_));

  // Timer callback
  auto on_timer = std::bind(&PoseEstimatorManager::on_timer, this);
  timer_ =
    rclcpp::create_timer(this, this->get_clock(), rclcpp::Rate(1).period(), std::move(on_timer));

  toggle_mode(true);
}

bool PoseEstimatorManager::toggle_mode(bool enable_ndt)
{
  RCLCPP_INFO_STREAM(get_logger(), "ENTER: toggle pose estimator");
  if (enable_ndt) {
    clients_.at("ndt")->enable();
    clients_.at("pcdless")->disable();
  } else {
    clients_.at("ndt")->disable();
    clients_.at("pcdless")->enable();
  }
  RCLCPP_INFO_STREAM(get_logger(), "EXIT: toggle pose estimator");
  return true;
}

void PoseEstimatorManager::on_service(
  SetBool::Request::ConstSharedPtr request, SetBool::Response::SharedPtr response)
{
  toggle_mode(request->data);
  response->success = true;

  RCLCPP_INFO_STREAM(get_logger(), "override mode and disable timer callback");
  timer_->reset();
}

void PoseEstimatorManager::on_map(PointCloud2::ConstSharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);

  RCLCPP_INFO_STREAM(get_logger(), "on_map: PCD map initialization is done");
}

void PoseEstimatorManager::on_timer()
{
  RCLCPP_INFO_STREAM(get_logger(), "on_timer");
  toggle_mode(true);
}

void PoseEstimatorManager::on_pose_cov(PoseCovStamped::ConstSharedPtr msg) { latest_pose_ = *msg; }

}  // namespace multi_pose_estimator
