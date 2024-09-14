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

#include <std_msgs/msg/header.hpp>
#include <usv_controller/usv_controller_component.hpp>

namespace usv_controller
{
UsvControllerComponent::UsvControllerComponent(const rclcpp::NodeOptions & options)
: Node("usv_controller_node", options),
  parameters_(usv_controller_node::ParamListener(get_node_parameters_interface()).get_params()),
  left_thruster_client_(parameters_.thrusters.left.ip, parameters_.thrusters.left.port),
  joy_interface_(p9n_interface::getHwType(parameters_.joystick_type)),
  control_mode_(ControlMode::MANUAL),
  last_joy_timestamp_(get_clock()->now())
{
  assert(isManual());
  if (parameters_.auto_start) {
    becomeAutonomous();
  }
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, [this](const sensor_msgs::msg::Joy::ConstSharedPtr & msg) {
      std::lock_guard<std::mutex> lock(mtx_);
      last_joy_timestamp_ = msg->header.stamp;
      joy_interface_.setJoyMsg(msg);
    });
  using namespace std::chrono_literals;
  watchdog_timer_ = create_wall_timer(30ms, [this]() { watchDogFunction(); });
}

void UsvControllerComponent::watchDogFunction()
{
  std::lock_guard<std::mutex> lock(mtx_);
  const auto is_joystick_alive = [this]() {
    constexpr double e = std::numeric_limits<double>::epsilon();
    if (std::abs(parameters_.joystick_connection_timeout) <= e) {
      return true;
    }
    return get_clock()->now() - last_joy_timestamp_ >=
           rclcpp::Duration::from_seconds(parameters_.joystick_connection_timeout);
  };
  const auto is_manual_requested = [this]() {
    return isAutonomous() && joy_interface_.pressedCross();
  };
  if (!isEmergencyStop() && is_joystick_alive()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Joystick disconeected, wamv enters emergency stop mode.");
    becomeEmergency();
    return;
  }
  if (is_manual_requested()) {
    becomeManual();
    return;
  }
}
}  // namespace usv_controller

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(usv_controller::UsvControllerComponent)
