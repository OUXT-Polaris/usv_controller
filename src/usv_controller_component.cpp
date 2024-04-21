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

namespace usv_controller
{
UsvControllerComponent::UsvControllerComponent(const rclcpp::NodeOptions & options)
: Node("usv_controller_node", options),
  parameters_(usv_controller_node::ParamListener(get_node_parameters_interface()).get_params()),
  joy_interface_(p9n_interface::getHwType(parameters_.joystick_type)),
  control_mode_(ControlMode::MANUAL)
{
  if (parameters_.auto_start) {
    control_mode_ = ControlMode::AUTONOMOUS;
  }
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10,
    [this](const sensor_msgs::msg::Joy::ConstSharedPtr & msg) { joy_interface_.setJoyMsg(msg); });
}
}  // namespace usv_controller

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(usv_controller::UsvControllerComponent)
