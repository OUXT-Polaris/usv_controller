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

#include <usv_velocity_controller/usv_velocity_controller.hpp>

#include <hardware_interface/loaned_command_interface.hpp>

namespace usv_velocity_controller
{
using hardware_interface::LoanedCommandInterface;

UsvVelocityController::UsvVelocityController()
: controller_interface::ControllerInterface(),
  rt_command_ptr_(nullptr),
  joints_command_subscriber_(nullptr)
{}

controller_interface::return_type
UsvVelocityController::init(const std::string & controller_name)
{
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }
  try {
    auto node = get_node();
    node->declare_parameter<std::string>("left_azimuth_joint", "left_azimuth_joint");
    node->declare_parameter<std::string>("left_azimuth_joint", "right_azimuth_joint");
    node->declare_parameter<std::string>("left_thruster_joint", "left_thruster_joint");
    node->declare_parameter<std::string>("left_thruster_joint", "right_thruster_joint");
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration UsvVelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names.push_back(left_azimuth_joint_ + "/position");
  command_interfaces_config.names.push_back(right_azimuth_joint_ + "/position");
  command_interfaces_config.names.push_back(left_thruster_joint_ + "/velocity");
  command_interfaces_config.names.push_back(right_thruster_joint_ + "/velocity");
  return command_interfaces_config;
}

}  // namespace usv_velocity_controller
