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

#ifndef POSE_ESTIMATOR_MANAGER__SWITCH_RULE__MAP_BASED_RULE_HPP_
#define POSE_ESTIMATOR_MANAGER__SWITCH_RULE__MAP_BASED_RULE_HPP_

#include "pose_estimator_manager/pose_estimator_name.hpp"
#include "pose_estimator_manager/rule_helper/ar_tag_position.hpp"
#include "pose_estimator_manager/rule_helper/eagleye_area.hpp"
#include "pose_estimator_manager/rule_helper/pcd_occupancy.hpp"
#include "pose_estimator_manager/shared_data.hpp"
#include "pose_estimator_manager/switch_rule/base_switch_rule.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace pose_estimator_manager::switch_rule
{
class MapBasedRule : public BaseSwitchRule
{
public:
  MapBasedRule(
    rclcpp::Node & node, const std::unordered_set<PoseEstimatorName> & running_estimator_list,
    const std::shared_ptr<const SharedData> shared_data);

  std::unordered_map<PoseEstimatorName, bool> update() override;

  std::string debug_string() override;

  MarkerArray debug_marker_array() override;

protected:
  const double ar_marker_available_distance_;
  const std::unordered_set<PoseEstimatorName> running_estimator_list_;
  std::shared_ptr<const SharedData> shared_data_{nullptr};

  std::unique_ptr<rule_helper::ArTagPosition> ar_tag_position_{nullptr};
  std::unique_ptr<rule_helper::PcdOccupancy> pcd_occupancy_{nullptr};
  std::unique_ptr<rule_helper::EagleyeArea> eagleye_area_{nullptr};

  std::string debug_string_;

  bool eagleye_is_available() const;
  bool yabloc_is_available() const;
  bool ndt_is_available() const;
  bool artag_is_available() const;
  bool ndt_is_more_suitable_than_yabloc(std::string * optional_message = nullptr) const;
};
}  // namespace pose_estimator_manager::switch_rule

#endif  // POSE_ESTIMATOR_MANAGER__SWITCH_RULE__MAP_BASED_RULE_HPP_
