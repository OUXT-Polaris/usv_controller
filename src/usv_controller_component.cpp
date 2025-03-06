// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <usv_controller/usv_controller_component.hpp>

// Headers in ROS2
#include <rclcpp_components/register_node_macro.hpp>

namespace usv_controller
{
UsvControllerComponent::UsvControllerComponent(const rclcpp::NodeOptions & options)
: Node("usv_controller", options)
{
  get_parameter("linear_pid_gain.kp", linear_pid_gain_.p_gain_);
  get_parameter("linear_pid_gain.ki", linear_pid_gain_.i_gain_);
  get_parameter("linear_pid_gain.kd", linear_pid_gain_.d_gain_);
  get_parameter("linear_pid_gain.i_min", linear_pid_gain_.i_min_);
  get_parameter("linear_pid_gain.i_max", linear_pid_gain_.i_max_);
  get_parameter("linear_pid_gain.antiwindup", linear_pid_gain_.antiwindup_);
  get_parameter("anguler_pid_gain.kp", anguler_pid_gain_.p_gain_);
  get_parameter("anguler_pid_gain.ki", anguler_pid_gain_.i_gain_);
  get_parameter("anguler_pid_gain.kd", anguler_pid_gain_.d_gain_);
  get_parameter("anguler_pid_gain.i_min", anguler_pid_gain_.i_min_);
  get_parameter("anguler_pid_gain.i_max", anguler_pid_gain_.i_max_);
  get_parameter("anguler_pid_gain.antiwindup", anguler_pid_gain_.antiwindup_);
  get_parameter("hull_width", hull_width_);
  target_twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "target_twist", 1, [this](const geometry_msgs::msg::Twist & msg) { target_twist_ = msg; });
  current_twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "current_twist", 1, [this](const geometry_msgs::msg::Twist & msg) { current_twist_ = msg; });
  using namespace std::chrono_literals;
  timer_ = create_wall_timer(10ms, [this]() {
    if (!target_twist_sub_ || !current_twist_sub_) {
      return;
    }
  });
}
}  // namespace usv_controller

RCLCPP_COMPONENTS_REGISTER_NODE(usv_controller::UsvControllerComponent)
