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

#ifndef USV_CONTROLLER__USV_CONTROLLER_COMPONENT_HPP_
#define USV_CONTROLLER__USV_CONTROLLER_COMPONENT_HPP_

#include <p9n_interface/p9n_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <usv_controller/visibility_control.hpp>
#include <usv_controller_component_parameters.hpp>

namespace usv_controller
{
class UsvControllerComponent : public rclcpp::Node
{
  enum class ControlMode { AUTONOMOUS, MANUAL, EMERGENCY_STOP };

public:
  USV_CONTROLLER_PUBLIC
  explicit UsvControllerComponent(const rclcpp::NodeOptions & options);

private:
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_sub_;
  const usv_controller_node::Params parameters_;
  p9n_interface::PlayStationInterface joy_interface_;
  ControlMode control_mode_;
};
}  // namespace usv_controller

#endif  // USV_CONTROLLER__USV_CONTROLLER_COMPONENT_HPP_
