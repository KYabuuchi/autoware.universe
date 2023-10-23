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

#include "pose_estimator_manager/pose_estimator_manager.hpp"
#include "pose_estimator_manager/pose_estimator_name.hpp"
#include "pose_estimator_manager/sub_manager/sub_manager_artag.hpp"
#include "pose_estimator_manager/sub_manager/sub_manager_eagleye.hpp"
#include "pose_estimator_manager/sub_manager/sub_manager_ndt.hpp"
#include "pose_estimator_manager/sub_manager/sub_manager_yabloc.hpp"
#include "pose_estimator_manager/switch_rule/map_based_rule.hpp"

#include <magic_enum.hpp>

#include <sstream>

namespace pose_estimator_manager
{

static std::unordered_set<PoseEstimatorName> parse_estimator_name_args(
  const std::vector<std::string> & arg)
{
  std::unordered_set<PoseEstimatorName> running_estimator_list;
  for (const auto & estimator_name : arg) {
    auto estimator = magic_enum::enum_cast<PoseEstimatorName>(estimator_name);
    if (estimator.has_value()) {
      running_estimator_list.insert(estimator.value());
    } else {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("pose_estimator_manager"),
        "invalid pose_estimator_name is spciefied: " << estimator_name);
    }
  }

  return running_estimator_list;
}

PoseEstimatorManager::PoseEstimatorManager()
: Node("pose_estimator_manager"),
  running_estimator_list_(
    parse_estimator_name_args(declare_parameter<std::vector<std::string>>("pose_sources")))
{
  // Publisher
  pub_debug_string_ = create_publisher<String>("~/debug/string", 10);
  pub_debug_marker_array_ = create_publisher<MarkerArray>("~/debug/marker_array", 10);

  // sub-managers
  for (auto pose_estimator_name : running_estimator_list_) {
    using namespace sub_manager;
    switch (pose_estimator_name) {
      case PoseEstimatorName::ndt:
        sub_managers_.emplace(pose_estimator_name, std::make_shared<SubManagerNdt>(this));
        break;
      case PoseEstimatorName::yabloc:
        sub_managers_.emplace(pose_estimator_name, std::make_shared<SubManagerYabLoc>(this));
        break;
      case PoseEstimatorName::eagleye:
        sub_managers_.emplace(pose_estimator_name, std::make_shared<SubManagerEagleye>(this));
        break;
      case PoseEstimatorName::artag:
        sub_managers_.emplace(pose_estimator_name, std::make_shared<SubManagerArTag>(this));
        break;
      default:
        RCLCPP_ERROR_STREAM(get_logger(), "invalid pose_estimator is specified");
    }
  }

  // Load switching rule
  load_switch_rule();

  // Timer callback
  auto on_timer = std::bind(&PoseEstimatorManager::on_timer, this);
  timer_ =
    rclcpp::create_timer(this, this->get_clock(), rclcpp::Rate(1).period(), std::move(on_timer));

  // Enable all pose estimators at the first
  toggle_all(true);
}

void PoseEstimatorManager::load_switch_rule()
{
  // NOTE: In the future, some rule will be laid below
  RCLCPP_INFO_STREAM(get_logger(), "load default switching rule");
  switch_rule_ = std::make_shared<switch_rule::MapBasedRule>(*this, running_estimator_list_);
}

void PoseEstimatorManager::toggle_each(
  const std::unordered_map<PoseEstimatorName, bool> & toggle_list)
{
  for (auto s : sub_managers_) {
    if (toggle_list.at(s.first)) {
      s.second->enable();
    } else {
      s.second->disable();
    }
  }
}

void PoseEstimatorManager::toggle_all(bool enabled)
{
  RCLCPP_INFO_STREAM(get_logger(), (enabled ? "Enable" : "Disable") << " all pose estimators");

  std::unordered_map<PoseEstimatorName, bool> toggle_list;
  for (auto s : sub_managers_) {
    toggle_list.emplace(s.first, enabled);
  }
  toggle_each(toggle_list);
}

void PoseEstimatorManager::on_timer()
{
  if (switch_rule_) {
    auto toggle_list = switch_rule_->update();
    toggle_each(toggle_list);

    {
      String msg;
      msg.data = switch_rule_->debug_string();
      pub_debug_string_->publish(msg);
    }
    {
      MarkerArray msg = switch_rule_->debug_marker_array();
      pub_debug_marker_array_->publish(msg);
    }

  } else {
    RCLCPP_WARN_STREAM(
      get_logger(), "swtich_rule is not activated. Therefore, enable all pose_estimators");
    toggle_all(true);
  }
}

}  // namespace pose_estimator_manager
