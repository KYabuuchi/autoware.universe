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

  shared_data_ = std::make_shared<SharedData>();

  // sub-managers
  for (auto pose_estimator_name : running_estimator_list_) {
    switch (pose_estimator_name) {
      case PoseEstimatorName::ndt:
        sub_managers_.emplace(
          pose_estimator_name, std::make_shared<sub_manager::SubManagerNdt>(this, shared_data_));
        break;
      case PoseEstimatorName::yabloc:
        sub_managers_.emplace(
          pose_estimator_name, std::make_shared<sub_manager::SubManagerYabLoc>(this, shared_data_));
        break;
      case PoseEstimatorName::eagleye:
        sub_managers_.emplace(
          pose_estimator_name,
          std::make_shared<sub_manager::SubManagerEagleye>(this, shared_data_));
        break;
      case PoseEstimatorName::artag:
        sub_managers_.emplace(
          pose_estimator_name, std::make_shared<sub_manager::SubManagerArTag>(this, shared_data_));
        break;
      default:
        RCLCPP_ERROR_STREAM(get_logger(), "invalid pose_estimator is specified");
    }
  }
  {
    using std::placeholders::_1;
    const rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
    const rclcpp::QoS latch_qos = rclcpp::QoS(1).transient_local().reliable();

    // subscriber for sub manager
    auto on_artag_input = std::bind(&PoseEstimatorManager::on_artag_input, this, _1);
    sub_artag_input_ =
      create_subscription<Image>("~/input/artag/image", sensor_qos, on_artag_input);
    auto on_yabloc_input = std::bind(&PoseEstimatorManager::on_yabloc_input, this, _1);
    sub_yabloc_input_ = create_subscription<Image>("~/input/yabloc/image", 5, on_yabloc_input);
    auto on_ndt_input = std::bind(&PoseEstimatorManager::on_ndt_input, this, _1);
    sub_ndt_input_ =
      create_subscription<PointCloud2>("~/input/ndt/pointcloud", sensor_qos, on_ndt_input);
    auto on_eagleye_output = std::bind(&PoseEstimatorManager::on_eagleye_output, this, _1);
    sub_eagleye_output_ = create_subscription<PoseCovStamped>(
      "~/input/eagleye/pose_with_covariance", 5, on_eagleye_output);

    // subscriber for switch rule
    auto on_vector_map = [this](HADMapBin::ConstSharedPtr msg) -> void {
      shared_data_->vector_map.set(msg);
    };
    auto on_point_cloud_map = [this](PointCloud2::ConstSharedPtr msg) -> void {
      shared_data_->point_cloud_map.set(msg);
    };
    auto on_localization_pose_cov = [this](PoseCovStamped::ConstSharedPtr msg) -> void {
      shared_data_->localization_pose_cov.set(msg);
    };
    auto on_initialization_state = [this](InitializationState::ConstSharedPtr msg) -> void {
      shared_data_->initialization_state.set(msg);
    };
    sub_localization_pose_cov_ = create_subscription<PoseCovStamped>(
      "~/input/pose_with_covariance", 5, on_localization_pose_cov);
    sub_point_cloud_map_ =
      create_subscription<PointCloud2>("~/input/pointcloud_map", latch_qos, on_point_cloud_map);
    sub_vector_map_ =
      create_subscription<HADMapBin>("~/input/vector_map", latch_qos, on_vector_map);
    sub_initialization_state_ = create_subscription<InitializationState>(
      "~/input/initialization_state", latch_qos, on_initialization_state);
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
  switch_rule_ =
    std::make_shared<switch_rule::MapBasedRule>(*this, running_estimator_list_, shared_data_);
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
  auto now = rclcpp::Clock().now();
  RCLCPP_WARN_STREAM(get_logger(), "debug " << now.seconds() << " " << now.nanoseconds());

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

void PoseEstimatorManager::call_all_callback()
{
  for (auto & [name, manager] : sub_managers_) {
    manager->callback();
  }
}

void PoseEstimatorManager::on_yabloc_input(Image::ConstSharedPtr msg)
{
  shared_data_->yabloc_input_image.set(msg);
  call_all_callback();
  shared_data_->reset_update_flag();
}
void PoseEstimatorManager::on_artag_input(Image::ConstSharedPtr msg)
{
  shared_data_->artag_input_image.set(msg);
  call_all_callback();
  shared_data_->reset_update_flag();
}
void PoseEstimatorManager::on_ndt_input(PointCloud2::ConstSharedPtr msg)
{
  shared_data_->ndt_input_points.set(msg);
  call_all_callback();
  shared_data_->reset_update_flag();
}
void PoseEstimatorManager::on_eagleye_output(PoseCovStamped::ConstSharedPtr msg)
{
  shared_data_->eagleye_output_pose_cov.set(msg);
  call_all_callback();
  shared_data_->reset_update_flag();
}

}  // namespace pose_estimator_manager
