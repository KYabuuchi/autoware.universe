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

#include "pose_estimator_arbiter/pose_estimator_arbiter.hpp"
#include "pose_estimator_arbiter/pose_estimator_name.hpp"
#include "pose_estimator_arbiter/stopper/stopper_artag.hpp"
#include "pose_estimator_arbiter/stopper/stopper_eagleye.hpp"
#include "pose_estimator_arbiter/stopper/stopper_ndt.hpp"
#include "pose_estimator_arbiter/stopper/stopper_yabloc.hpp"
#include "pose_estimator_arbiter/switch_rule/map_based_rule.hpp"

#include <magic_enum.hpp>

namespace pose_estimator_arbiter
{

static std::unordered_set<PoseEstimatorName> parse_estimator_name_args(
  const std::vector<std::string> & arg, const rclcpp::Logger & logger)
{
  std::unordered_set<PoseEstimatorName> running_estimator_list;
  for (const auto & estimator_name : arg) {
    auto estimator = magic_enum::enum_cast<PoseEstimatorName>(estimator_name);
    if (estimator.has_value()) {
      running_estimator_list.insert(estimator.value());
    } else {
      RCLCPP_ERROR_STREAM(logger, "invalid pose_estimator_name is specified: " << estimator_name);
    }
  }

  return running_estimator_list;
}

PoseEstimatorArbiter::PoseEstimatorArbiter()
: Node("pose_estimator_arbiter"),
  running_estimator_list_(parse_estimator_name_args(
    declare_parameter<std::vector<std::string>>("pose_sources"), get_logger())),
  logger_configure_(std::make_unique<tier4_autoware_utils::LoggerLevelConfigure>(this))
{
  // Shared data
  shared_data_ = std::make_shared<SharedData>();

  // Publisher
  pub_diag_ = create_publisher<DiagnosticArray>("/diagnostics", 10);
  pub_debug_string_ = create_publisher<String>("~/debug/string", 10);
  pub_debug_marker_array_ = create_publisher<MarkerArray>("~/debug/marker_array", 10);

  // Stoppers
  for (auto pose_estimator_name : running_estimator_list_) {
    switch (pose_estimator_name) {
      case PoseEstimatorName::ndt:
        stoppers_.emplace(
          pose_estimator_name, std::make_shared<stopper::StopperNdt>(this, shared_data_));
        break;
      case PoseEstimatorName::yabloc:
        stoppers_.emplace(
          pose_estimator_name, std::make_shared<stopper::StopperYabLoc>(this, shared_data_));
        break;
      case PoseEstimatorName::eagleye:
        stoppers_.emplace(
          pose_estimator_name, std::make_shared<stopper::StopperEagleye>(this, shared_data_));
        break;
      case PoseEstimatorName::artag:
        stoppers_.emplace(
          pose_estimator_name, std::make_shared<stopper::StopperArTag>(this, shared_data_));
        break;
      default:
        RCLCPP_ERROR_STREAM(get_logger(), "invalid pose_estimator is specified");
    }
  }
  {
    using std::placeholders::_1;
    const rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
    const rclcpp::QoS latch_qos = rclcpp::QoS(1).transient_local().reliable();

    // Subscriber for stoppers
    sub_artag_input_ = create_subscription<Image>(
      "~/input/artag/image", sensor_qos, shared_data_->artag_input_image.create_callback());
    sub_yabloc_input_ = create_subscription<Image>(
      "~/input/yabloc/image", sensor_qos, shared_data_->yabloc_input_image.create_callback());
    sub_ndt_input_ = create_subscription<PointCloud2>(
      "~/input/ndt/pointcloud", sensor_qos, shared_data_->ndt_input_points.create_callback());
    sub_eagleye_output_ = create_subscription<PoseCovStamped>(
      "~/input/eagleye/pose_with_covariance", 5,
      shared_data_->eagleye_output_pose_cov.create_callback());

    // Subscriber for switch rule
    sub_localization_pose_cov_ = create_subscription<PoseCovStamped>(
      "~/input/pose_with_covariance", 5, shared_data_->localization_pose_cov.create_callback());
    sub_point_cloud_map_ = create_subscription<PointCloud2>(
      "~/input/pointcloud_map", latch_qos, shared_data_->point_cloud_map.create_callback());
    sub_vector_map_ = create_subscription<HADMapBin>(
      "~/input/vector_map", latch_qos, shared_data_->vector_map.create_callback());
    sub_initialization_state_ = create_subscription<InitializationState>(
      "~/input/initialization_state", latch_qos,
      shared_data_->initialization_state.create_callback());
  }

  // Load switching rule
  load_switch_rule();

  // Timer callback
  auto on_timer = std::bind(&PoseEstimatorArbiter::on_timer, this);
  timer_ =
    rclcpp::create_timer(this, this->get_clock(), rclcpp::Rate(1).period(), std::move(on_timer));

  // Enable all pose estimators at the first
  toggle_all(true);
}

void PoseEstimatorArbiter::load_switch_rule()
{
  // NOTE: In the future, some rule will be laid below
  RCLCPP_INFO_STREAM(get_logger(), "load default switching rule");
  switch_rule_ =
    std::make_shared<switch_rule::MapBasedRule>(*this, running_estimator_list_, shared_data_);
}

void PoseEstimatorArbiter::toggle_each(
  const std::unordered_map<PoseEstimatorName, bool> & toggle_list)
{
  for (auto s : stoppers_) {
    if (toggle_list.at(s.first)) {
      s.second->enable();
    } else {
      s.second->disable();
    }
  }
}

void PoseEstimatorArbiter::toggle_all(bool enabled)
{
  std::unordered_map<PoseEstimatorName, bool> toggle_list;
  for (auto s : stoppers_) {
    toggle_list.emplace(s.first, enabled);
  }
  toggle_each(toggle_list);
}

void PoseEstimatorArbiter::publish_diagnostics() const
{
  diagnostic_msgs::msg::DiagnosticStatus diag_status;

  // Temporary implementation
  {
    diag_status.name = "localization: " + std::string(this->get_name());
    diag_status.hardware_id = this->get_name();

    diag_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diag_status.message = "OK";

    diagnostic_msgs::msg::KeyValue key_value_msg;
    key_value_msg.key = "state";
    key_value_msg.value = "Diagnostics is not implemented.";
    diag_status.values.push_back(key_value_msg);
  }

  DiagnosticArray diag_msg;
  diag_msg.header.stamp = this->now();
  diag_msg.status.push_back(diag_status);

  pub_diag_->publish(diag_msg);
}

void PoseEstimatorArbiter::on_timer()
{
  auto now = rclcpp::Clock().now();

  if (switch_rule_) {
    auto toggle_list = switch_rule_->update();
    toggle_each(toggle_list);

    {
      const auto msg = String().set__data(switch_rule_->debug_string());
      pub_debug_string_->publish(msg);
    }
    {
      const MarkerArray msg = switch_rule_->debug_marker_array();
      pub_debug_marker_array_->publish(msg);
    }

  } else {
    RCLCPP_WARN_STREAM(
      get_logger(), "switch_rule is not activated. Therefore, enable all pose_estimators");
    toggle_all(true);
  }

  //
  publish_diagnostics();
}

}  // namespace pose_estimator_arbiter
