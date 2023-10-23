// Copyright 2023 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pose_estimator_manager/rule_helper/pcd_occupancy.hpp"

#include "pose_estimator_manager/rule_helper/grid_info.hpp"

#include <tier4_autoware_utils/ros/parameter.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace pose_estimator_manager::rule_helper
{
using tier4_autoware_utils::getOrDeclareParameter;

PcdOccupancy::PcdOccupancy(rclcpp::Node * node)
: pcd_density_upper_threshold_(
    getOrDeclareParameter<int>(*node, "pcd_occupancy_rule/pcd_density_upper_threshold")),
  pcd_density_lower_threshold_(
    getOrDeclareParameter<int>(*node, "pcd_occupancy_rule/pcd_density_lower_threshold"))
{
}

bool PcdOccupancy::ndt_can_operate(
  const geometry_msgs::msg::Point & position, std::string * optional_message) const
{
  if (!kdtree_) {
    if (optional_message) {
      *optional_message = "enable YabLoc\npcd is not subscribed yet";
    }
    return false;
  }

  const pcl::PointXYZ query{
    static_cast<float>(position.x), static_cast<float>(position.y), static_cast<float>(position.z)};
  std::vector<int> indices;
  std::vector<float> distances;
  const int count = kdtree_->radiusSearch(query, 50, indices, distances, 0);

  static bool last_is_ndt_mode = true;
  const bool is_ndt_mode = (last_is_ndt_mode) ? (count > pcd_density_lower_threshold_)
                                              : (count > pcd_density_upper_threshold_);
  last_is_ndt_mode = is_ndt_mode;

  std::stringstream ss;
  if (is_ndt_mode) {
    ss << "enable NDT";
  } else {
    ss << "enable YabLoc";
  }
  ss << "\npcd occupancy: " << count << " > "
     << (last_is_ndt_mode ? pcd_density_lower_threshold_ : pcd_density_upper_threshold_);
  *optional_message = ss.str();

  return is_ndt_mode;
}

visualization_msgs::msg::MarkerArray PcdOccupancy::debug_marker_array() const
{
  visualization_msgs::msg::Marker msg;
  msg.ns = "pcd_occupancy";
  msg.id = 0;
  msg.header.frame_id = "map";
  msg.scale.set__x(3.0f).set__y(3.0f).set__z(3.f);
  msg.color.set__r(1.0f).set__g(1.0f).set__b(0.2f).set__a(1.0f);

  if (occupied_areas_) {
    for (auto p : occupied_areas_->points) {
      geometry_msgs::msg::Point geometry_point{};
      geometry_point.set__x(p.x).set__y(p.y).set__z(p.z);
      msg.points.push_back(geometry_point);
    }
  }

  visualization_msgs::msg::MarkerArray msg_array;
  msg_array.markers.push_back(msg);

  return msg_array;
}

void PcdOccupancy::on_point_cloud_map(PointCloud2::ConstSharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);

  std::unordered_map<GridInfo, size_t> grid_point_count;
  for (pcl::PointXYZ xyz : cloud) {
    grid_point_count[GridInfo(xyz.x, xyz.y)] += 1;
  }

  occupied_areas_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (const auto [grid, count] : grid_point_count) {
    if (count > 50) {
      occupied_areas_->push_back(grid.get_center_point());
    }
  }

  kdtree_ = pcl::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();
  kdtree_->setInputCloud(occupied_areas_);
}

}  // namespace pose_estimator_manager::rule_helper
