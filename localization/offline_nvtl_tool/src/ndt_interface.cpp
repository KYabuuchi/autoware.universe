#include "offline_nvtl_tool/ndt_interface.hpp"

#include <localization_util/util_func.hpp>
#include <ndt_scan_matcher/hyper_parameters.hpp>
#include <tier4_autoware_utils/transform/transforms.hpp>

#include <pcl_conversions/pcl_conversions.h>

NdtInterface::NdtInterface(rclcpp::Node * node) : ndt_ptr_(new NdtType)
{
  const HyperParameters parameters{node};
  ndt_ptr_->setParams(parameters.ndt);
}

void NdtInterface::set_pointcloud_map(const sensor_msgs::msg::PointCloud2 & map_points_msg)
{
  pcl::shared_ptr<pcl::PointCloud<PointTarget>> map_points(new pcl::PointCloud<PointTarget>);

  pcl::fromROSMsg(map_points_msg, *map_points);

  ndt_ptr_->setInputTarget(map_points);
}

double NdtInterface::get_nvtl(
  const sensor_msgs::msg::PointCloud2 & source_cloud_msg,
  const geometry_msgs::msg::Pose & pose_msg) const
{
  pcl::PointCloud<PointSource> cloud_in_base_frame;
  pcl::fromROSMsg(source_cloud_msg, cloud_in_base_frame);

  const Eigen::Matrix4f map_to_base_matrix = pose_to_matrix4f(pose_msg);

  // NOTE: We need to call align() once before calling
  // calculateNearestVoxelTransformationLikelihood() to initialize the some internal variables.
  static bool first_ndt = true;
  if (first_ndt) {
    first_ndt = false;
    pcl::PointCloud<PointSource> output_cloud;
    ndt_ptr_->setInputSource(cloud_in_base_frame.makeShared());
    ndt_ptr_->align(output_cloud, map_to_base_matrix);
  }

  pcl::PointCloud<PointSource> cloud_in_map_frame;
  tier4_autoware_utils::transformPointCloud(
    cloud_in_base_frame, cloud_in_map_frame, map_to_base_matrix);

  const double nvtl = ndt_ptr_->calculateNearestVoxelTransformationLikelihood(cloud_in_map_frame);

  return nvtl;
}
