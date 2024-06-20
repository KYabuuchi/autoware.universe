#include "offline_nvtl_tool/ndt_interface.hpp"

#include <autoware/universe_utils/transform/transforms.hpp>
#include <localization_util/util_func.hpp>
#include <ndt_scan_matcher/hyper_parameters.hpp>

#include <pcl_conversions/pcl_conversions.h>

NdtInterface::NdtInterface(rclcpp::Node * node) : ndt_ptr_(new NdtType)
{
  const HyperParameters parameters{node};
  ndt_ptr_->setParams(parameters.ndt);
}

void NdtInterface::set_pointcloud_map(const pcl::PointCloud<pcl::PointXYZ> & map_points)
{
  pcl::PointCloud<PointSource>::Ptr map_points_ptr = map_points.makeShared();
  ndt_ptr_->setInputTarget(map_points_ptr);

  // NOTE: We need to call align() once before calling
  // calculateNearestVoxelTransformationLikelihood() to initialize the some internal variables.
  {
    pcl::PointCloud<PointSource> output_cloud;
    ndt_ptr_->setInputSource(map_points_ptr);
    ndt_ptr_->align(output_cloud, Eigen::Matrix4f::Identity());
  }
}

double NdtInterface::get_nvtl(
  const pcl::PointCloud<pcl::PointXYZ> & cloud_in_base_frame,
  const geometry_msgs::msg::Pose & pose_msg) const
{
  const Eigen::Matrix4f map_to_base_matrix = pose_to_matrix4f(pose_msg);

  pcl::PointCloud<PointSource> cloud_in_map_frame;
  autoware::universe_utils::transformPointCloud(
    cloud_in_base_frame, cloud_in_map_frame, map_to_base_matrix);

  const double nvtl = ndt_ptr_->calculateNearestVoxelTransformationLikelihood(cloud_in_map_frame);

  return nvtl;
}
