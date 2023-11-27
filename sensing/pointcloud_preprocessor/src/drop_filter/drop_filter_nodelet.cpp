// Copyright 2023 Tier IV, Inc.
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

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <vector>

namespace pointcloud_preprocessor
{
class DropFilterComponent : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  DropFilterComponent(const rclcpp::NodeOptions & options)
  : Node("drop_filter", options), drop_rate_(declare_parameter("drop_rate", 3))
  {
    using std::placeholders::_1;

    // Subscriber
    auto on_points = std::bind(&DropFilterComponent::on_points, this, _1);
    sub_points_ = create_subscription<PointCloud2>(
      "input", rclcpp::QoS(10).durability_volatile().best_effort(), on_points);
    // Publisher
    pub_points_ = create_publisher<PointCloud2>("output", 10);
  }

  void on_points(const PointCloud2::ConstSharedPtr & input)
  {
    static int count = 0;
    if ((count++) % drop_rate_ == 0) {
      return;
    }
    pub_points_->publish(*input);
  }

private:
  const int drop_rate_;

  rclcpp::Subscription<PointCloud2>::SharedPtr sub_points_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_points_;
};

}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::DropFilterComponent)