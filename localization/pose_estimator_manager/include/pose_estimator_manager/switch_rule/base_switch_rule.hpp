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

#pragma once

#include "pose_estimator_manager/pose_estimator_name.hpp"

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <unordered_map>

namespace multi_pose_estimator
{

class BaseSwitchRule
{
public:
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  BaseSwitchRule(rclcpp::Node & node)
  : logger_ptr_(std::make_shared<rclcpp::Logger>(node.get_logger()))
  {
  }

  virtual ~BaseSwitchRule() = default;
  virtual std::unordered_map<PoseEstimatorName, bool> update() = 0;
  virtual std::vector<PoseEstimatorName> supporting_pose_estimators() = 0;

  virtual std::string debug_string() { return std::string{}; }
  virtual MarkerArray debug_marker_array() { return MarkerArray{}; }

protected:
  rclcpp::Logger get_logger() { return *logger_ptr_; }
  std::shared_ptr<rclcpp::Logger> logger_ptr_{nullptr};
};

}  // namespace multi_pose_estimator
