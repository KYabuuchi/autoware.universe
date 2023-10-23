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

#ifndef POSE_ESTIMATOR_MANAGER__RULE_HELPER__GRID_INFO_HPP_
#define POSE_ESTIMATOR_MANAGER__RULE_HELPER__GRID_INFO_HPP_
#include <boost/functional/hash.hpp>

#include <pcl/point_types.h>

#include <functional>

namespace pose_estimator_manager::rule_helper
{
struct GridInfo
{
  const float unit_length = 10.f;
  int x, y;

  GridInfo() : x(0), y(0) {}
  GridInfo(float x, float y) : x(std::floor(x / unit_length)), y(std::floor(y / unit_length)) {}

  pcl::PointXYZ get_center_point() const
  {
    pcl::PointXYZ xyz;
    xyz.x = unit_length * (static_cast<float>(x) + 0.5f);
    xyz.y = unit_length * (static_cast<float>(y) + 0.5f);
    xyz.z = 0.f;
    return xyz;
  }

  friend bool operator==(const GridInfo & one, const GridInfo & other)
  {
    return one.x == other.x && one.y == other.y;
  }
  friend bool operator!=(const GridInfo & one, const GridInfo & other) { return !(one == other); }
};

}  // namespace pose_estimator_manager::rule_helper

// This is for unordered_map and unodered_set
namespace std
{
template <>
struct hash<pose_estimator_manager::rule_helper::GridInfo>
{
public:
  size_t operator()(const pose_estimator_manager::rule_helper::GridInfo & grid) const
  {
    std::size_t seed = 0;
    boost::hash_combine(seed, grid.x);
    boost::hash_combine(seed, grid.y);
    return seed;
  }
};
}  // namespace std

#endif  // POSE_ESTIMATOR_MANAGER__RULE_HELPER__GRID_INFO_HPP_
