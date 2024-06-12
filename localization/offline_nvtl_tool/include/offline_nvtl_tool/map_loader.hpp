#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

pcl::PointCloud<pcl::PointXYZ> load_pcd_files(const std::string & pcd_path);