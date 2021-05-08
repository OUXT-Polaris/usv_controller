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

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef USV_CONTROLLER__USV_VELOCITY_CONTROLLER_HPP_
#define USV_CONTROLLER__USV_VELOCITY_CONTROLLER_HPP_

#include <usv_controller/visibility_control.hpp>
#include <usv_controller/usv_controller_base.hpp>

#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/subscription.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <controller_interface/controller_interface.hpp>

namespace usv_controller
{
class UsvVelocityController : public usv_controller::UsvVelocityControllerBase<geometry_msgs::msg::Twist>
{
public:
  USV_CONTROLLER_PUBLIC
  UsvVelocityController();

  USV_CONTROLLER_PUBLIC
  controller_interface::return_type update() override;

protected:
  std::string left_azimuth_joint_;
  std::string right_azimuth_joint_;
  std::string left_thruster_joint_;
  std::string right_thruster_joint_;
  std::string logger_name_;
};

}  // namespace usv_controller

#endif  // USV_CONTROLLER__USV_VELOCITY_CONTROLLER_HPP_
