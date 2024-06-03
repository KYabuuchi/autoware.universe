#pragma once

#include <ndt_scan_matcher/hyper_parameters.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <multigrid_pclomp/multigrid_ndt_omp.h>
#include <pcl_conversions/pcl_conversions.h>

class NdtInterface
{
public:
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using NdtType = pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>;
  using NdtPtrType = std::shared_ptr<NdtType>;

  NdtInterface(rclcpp::Node * node) : ndt_ptr_(new NdtType)
  {
    const HyperParameters parameters{node};
    ndt_ptr_->setParams(parameters.ndt);
  }

  void set_pointcloud_map(const sensor_msgs::msg::PointCloud2 & map_points_msg)
  {
    pcl::shared_ptr<pcl::PointCloud<PointTarget>> map_points(new pcl::PointCloud<PointTarget>);

    pcl::fromROSMsg(map_points_msg, *map_points);

    ndt_ptr_->setInputTarget(map_points);

    std::cout << "NDT target size: " << ndt_ptr_->getInputTarget()->size() << std::endl;
  }

  double get_nvtl(
    const sensor_msgs::msg::PointCloud2 &,
    const geometry_msgs::msg::PoseWithCovarianceStamped &) const
  {
    pcl::PointCloud<PointSource> source_cloud;
    return ndt_ptr_->calculateNearestVoxelTransformationLikelihood(source_cloud);
  }

private:
  NdtPtrType ndt_ptr_;
};