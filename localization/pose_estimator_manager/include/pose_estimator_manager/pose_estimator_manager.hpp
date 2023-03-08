#ifndef POSE_ESTIMATOR_MANAGER_
#define POSE_ESTIMATOR_MANAGER_

#include "pose_estimator_manager/manager_client.hpp"

#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/set_bool.hpp>

namespace multi_pose_estimator
{
class PoseEstimatorManager : public rclcpp::Node
{
public:
  using SetBool = std_srvs::srv::SetBool;
  PoseEstimatorManager();

private:
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Service<SetBool>::SharedPtr switch_service_server_;

  std::unordered_map<std::string, ManagerClient::SharedPtr> clients_;

  void on_service(SetBool::Request::ConstSharedPtr request, SetBool::Response::SharedPtr response);
};
}  // namespace multi_pose_estimator

#endif /* POSE_ESTIMATOR_MANAGER_ */
