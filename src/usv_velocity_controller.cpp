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

#include <usv_controller/usv_velocity_controller.hpp>

#include <hardware_interface/loaned_command_interface.hpp>

namespace usv_controller
{
using hardware_interface::LoanedCommandInterface;

UsvVelocityController::UsvVelocityController()
: usv_controller::UsvVelocityControllerBase<geometry_msgs::msg::Twist>()
{}
}  // namespace usv_velocity_controller
