#include "offline_nvtl_tool/map_loader.hpp"

#include <pcl/io/pcd_io.h>

#include <filesystem>

pcl::PointCloud<pcl::PointXYZ> load_pcd_files(const std::string & pcd_path)
{
  pcl::PointCloud<pcl::PointXYZ> whole_point_cloud;

  if (!std::filesystem::exists(pcd_path)) {
    std::cerr << "can not open directory: " << pcd_path.c_str() << std::endl;
  }

  if (std::filesystem::is_regular_file(pcd_path)) {
    pcl::PointCloud<pcl::PointXYZ> partial_pointcloud;

    if (pcl::io::loadPCDFile(pcd_path, partial_pointcloud) == -1) {
      std::cerr << "PCD load failed: " << pcd_path.c_str() << std::endl;
    }
    return partial_pointcloud;
  }

  for (const auto & file : std::filesystem::directory_iterator(pcd_path)) {
    const std::string filename = file.path().c_str();
    const std::string extension = file.path().extension().string();

    if (extension != ".pcd" && extension != ".PCD") {
      std::cerr << "ignore files: " << extension.c_str() << std::endl;
      continue;
    }

    pcl::PointCloud<pcl::PointXYZ> partial_pointcloud;

    if (pcl::io::loadPCDFile(filename, partial_pointcloud) == -1) {
      std::cerr << "PCD load failed: " << filename.c_str() << std::endl;
      continue;
    }

    whole_point_cloud += partial_pointcloud;
  }

  return whole_point_cloud;
}
