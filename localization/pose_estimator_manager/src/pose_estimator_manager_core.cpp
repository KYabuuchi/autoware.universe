#include "pose_estimator_manager/pose_estimator_manager.hpp"

namespace multi_pose_estimator
{
PoseEstimatorManager::PoseEstimatorManager() : Node("pose_estimator_manager")
{
  using std::placeholders::_1;
  using std::placeholders::_2;

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
}

void PoseEstimatorManager::on_service(
  SetBool::Request::ConstSharedPtr request, SetBool::Response::SharedPtr response)
{
  RCLCPP_INFO_STREAM(get_logger(), "ENTER: toggle pose estimator");
  if (request->data) {
    clients_.at("ndt")->enable();
    clients_.at("pcdless")->disable();
  } else {
    clients_.at("ndt")->disable();
    clients_.at("pcdless")->enable();
  }
  response->success = true;
  RCLCPP_INFO_STREAM(get_logger(), "EXIT: toggle pose estimator");
}

}  // namespace multi_pose_estimator
