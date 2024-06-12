#pragma once

#include <rclcpp/node.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <multigrid_pclomp/multigrid_ndt_omp.h>

class NdtInterface
{
public:
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using NdtType = pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>;
  using NdtPtrType = std::shared_ptr<NdtType>;

  NdtInterface(rclcpp::Node * node);

  void set_pointcloud_map(const pcl::PointCloud<pcl::PointXYZ> & map_points);

  double get_nvtl(
    const pcl::PointCloud<PointSource> & cloud_in_base_frame,
    const geometry_msgs::msg::Pose & pose_msg) const;

private:
  NdtPtrType ndt_ptr_;
};