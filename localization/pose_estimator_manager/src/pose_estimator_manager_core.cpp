#include "pose_estimator_manager/grid_info.hpp"
#include "pose_estimator_manager/pose_estimator_manager.hpp"

#include <pcl_conversions/pcl_conversions.h>
namespace multi_pose_estimator
{
PoseEstimatorManager::PoseEstimatorManager() : Node("pose_estimator_manager")
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  const auto map_qos = rclcpp::QoS(1).transient_local().reliable();

  // Publisher
  pub_marker_ =
    create_publisher<MarkerArray>("/pcd_occupied_area", rclcpp::QoS(1).transient_local());

  // Subscriber
  auto on_map = std::bind(&PoseEstimatorManager::on_map, this, _1);
  sub_map_ = create_subscription<PointCloud2>("/map/pointcloud_map", map_qos, on_map);
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

  std::unordered_map<GridInfo, size_t> grid_point_count;
  for (pcl::PointXYZ xyz : cloud) {
    grid_point_count[GridInfo(xyz.x, xyz.y)] += 1;
  }

  occupied_areas_.clear();
  for (const auto [grid, count] : grid_point_count) {
    if (count > 50) {
      occupied_areas_.push_back(grid.get_center_point());
    }
  }

  publish_occupied_area();
  RCLCPP_INFO_STREAM(
    get_logger(), "on_map: PCD map initialization is done " << occupied_areas_.size());
}

void PoseEstimatorManager::publish_occupied_area()
{
  MarkerArray msg;
  {
    Marker marker;
    marker.header.frame_id = "map";
    marker.type = Marker::POINTS;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    marker.scale.x = 2.f;
    marker.scale.y = 2.f;
    marker.scale.z = 2.f;
    marker.id = 0;
    for (const auto xyz : occupied_areas_) {
      geometry_msgs::msg::Point p;
      p.x = xyz.x;
      p.y = xyz.y;
      p.z = xyz.z;
      marker.points.push_back(p);
    }
    msg.markers.push_back(marker);
  }
  pub_marker_->publish(msg);
}

void PoseEstimatorManager::on_timer()
{
  RCLCPP_INFO_STREAM(get_logger(), "on_timer");
  toggle_mode(true);
}

void PoseEstimatorManager::on_pose_cov(PoseCovStamped::ConstSharedPtr msg) { latest_pose_ = *msg; }

}  // namespace multi_pose_estimator
