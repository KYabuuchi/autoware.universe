// Copyright 2024 TIER IV
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

#include "offline_nvtl_tool/rosbag_handler.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <iostream>

int main(int argc, char * argv[])
{
  if (argc != 2) {
    std::cerr << "rosbag path is required" << std::endl;
    return 1;
  }

  const auto reader = RosbagReader(argv[1]);

  using PoseStamped = geometry_msgs::msg::PoseStamped;

  while (reader.has_next()) {
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg = reader.read_next();

    if (msg->topic_name == "/localization/pose_estimator/pose_with_covariance") {
      const PoseStamped pose = decode_with_type<PoseStamped>(msg);
      const auto position = pose.pose.position;

      std::cout << pose.header.stamp.sec << ": " << position.x << " " << position.y << " "
                << position.z << std::endl;
    }
  }
}