#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

namespace pose_estimator
{
class PoseEstimatorManager : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using CompressedImage = sensor_msgs::msg::CompressedImage;
  using NavPVT = ublox_msgs::msg::NavPVT;
  using SetBool = std_srvs::srv::SetBool;

  PoseEstimatorManager();

private:
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_pointcloud_;
  rclcpp::Subscription<CompressedImage>::SharedPtr sub_compressed_image_;
  rclcpp::Subscription<NavPVT>::SharedPtr sub_navpvt_;

  rclcpp::Publisher<PointCloud2>::SharedPtr pub_pointcloud_;
  rclcpp::Publisher<CompressedImage>::SharedPtr pub_compressed_image_;
  rclcpp::Publisher<NavPVT>::SharedPtr pub_navpvt_;

  rclcpp::Service<SetBool>::SharedPtr switch_service_;

  bool publish_ndt_;

  void on_service(SetBool::Request::ConstSharedPtr request, SetBool::Response::SharedPtr response);

  void on_pointcloud(PointCloud2::ConstSharedPtr msg);
  void on_compressed_image(CompressedImage::ConstSharedPtr msg);
  void on_navpvt(NavPVT::ConstSharedPtr msg);
};
}  // namespace pose_estimator