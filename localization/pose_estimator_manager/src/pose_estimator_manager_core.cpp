#include "pose_estimator_manager.hpp"

namespace pose_estimator
{
PoseEstimatorManager::PoseEstimatorManager() : Node("pose_estimator_manager"), publish_ndt_(true)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  auto on_pointcloud = std::bind(&PoseEstimatorManager::on_pointcloud, this, _1);
  auto on_compressed_image = std::bind(&PoseEstimatorManager::on_compressed_image, this, _1);
  auto on_navpvt = std::bind(&PoseEstimatorManager::on_navpvt, this, _1);

  sub_pointcloud_ = create_subscription<PointCloud2>("/input/pointcloud", 10, on_pointcloud);
  sub_compressed_image_ =
    create_subscription<CompressedImage>("/input/compressed_image", 10, on_compressed_image);
  sub_navpvt_ = create_subscription<NavPVT>("/input/navpvt", 10, on_navpvt);

  pub_pointcloud_ = create_publisher<PointCloud2>("/output/pointcloud", 10);
  pub_compressed_image_ = create_publisher<CompressedImage>("/output/compressed_image", 10);
  pub_navpvt_ = create_publisher<NavPVT>("/output/navpvt", 10);

  // Service definition
  auto on_service = std::bind(&PoseEstimatorManager::on_service, this, _1, _2);
  switch_service_ = create_service<SetBool>("/toggle_ndt", on_service);
}

void PoseEstimatorManager::on_navpvt(NavPVT::ConstSharedPtr msg)
{
  if (!publish_ndt_) {
    pub_navpvt_->publish(*msg);
  }
}

void PoseEstimatorManager::on_pointcloud(PointCloud2::ConstSharedPtr msg)
{
  if (publish_ndt_) {
    pub_pointcloud_->publish(*msg);
  }
}

void PoseEstimatorManager::on_compressed_image(CompressedImage::ConstSharedPtr msg)
{
  if (!publish_ndt_) {
    pub_compressed_image_->publish(*msg);
  }
}

void PoseEstimatorManager::on_service(
  SetBool::Request::ConstSharedPtr request, SetBool::Response::SharedPtr response)
{
  response->success = true;

  publish_ndt_ = request->data;
  if (publish_ndt_)
    RCLCPP_INFO_STREAM(get_logger(), "NDT mode is enabled");
  else
    RCLCPP_INFO_STREAM(get_logger(), "PCD-less mode is enabled");
}

}  // namespace pose_estimator
