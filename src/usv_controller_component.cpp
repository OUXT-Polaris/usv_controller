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

  linear_pid_ = std::make_shared<control_toolbox::Pid>();
  linear_pid_->setGains(linear_pid_gain_);

  anguler_pid_ = std::make_shared<control_toolbox::Pid>();
  anguler_pid_->setGains(linear_pid_gain_);

  target_twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "target_twist", 1, [this](const geometry_msgs::msg::Twist & msg) { target_twist_ = msg; });
  current_twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "current_twist", 1, [this](const geometry_msgs::msg::Twist & msg) { current_twist_ = msg; });
  using namespace std::chrono_literals;
  timer_ = create_wall_timer(10ms, [this]() {
    if (!target_twist_sub_ || !current_twist_sub_) {
      return;
    }
    double target_twist_x = target_twist_.value().linear.x;
    double target_twist_omega = target_twist_.value().angular.z;
    double current_twist_x = current_twist_.value().linear.x;
    double current_twist_omega = current_twist_.value().angular.z;
    const uint64_t dt_ns = 10UL * 1000 * 1000;  // 10ms

    // Calculate force
    double linear_force = linear_pid_->computeCommand(target_twist_x - current_twist_x, dt_ns);
    double turning_force =
      anguler_pid_->computeCommand(target_twist_omega - current_twist_omega, dt_ns);

    left_motor_pub_ =
      create_publisher<hardware_communication_msgs::msg::MotorControl>("left_motor/cmd", 1);
    right_motor_pub_ =
      create_publisher<hardware_communication_msgs::msg::MotorControl>("right_motor/cmd", 1);

    left_motor_pub_->publish(
      hardware_communication_msgs::build<hardware_communication_msgs::msg::MotorControl>()
        .motor_enable(true)
        .motor_speed(linear_force + 0.5 * hull_width_ * turning_force));
    right_motor_pub_->publish(
      hardware_communication_msgs::build<hardware_communication_msgs::msg::MotorControl>()
        .motor_enable(true)
        .motor_speed(linear_force - 0.5 * hull_width_ * turning_force));
  });
}
}  // namespace usv_controller

RCLCPP_COMPONENTS_REGISTER_NODE(usv_controller::UsvControllerComponent)
