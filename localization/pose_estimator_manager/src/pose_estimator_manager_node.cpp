#include "pose_estimator_manager.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pose_estimator::PoseEstimatorManager>());
  rclcpp::shutdown();
  return 0;
}
