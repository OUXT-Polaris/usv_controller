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

#include <usv_controller/usv_joy_controller.hpp>

#include <hardware_interface/loaned_command_interface.hpp>

namespace usv_controller
{
using hardware_interface::LoanedCommandInterface;

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn UsvJoyController::
on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  node->get_parameter("left_azimuth_joint", left_azimuth_joint_);
  node->get_parameter("right_azimuth_joint", right_azimuth_joint_);
  node->get_parameter("left_thruster_joint", left_thruster_joint_);
  node->get_parameter("right_thruster_joint", right_thruster_joint_);
  node->get_parameter("joy_topic", joy_topic_);
  node->create_subscription<sensor_msgs::msg::Joy>(
    joy_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      rt_command_ptr_.writeFromNonRT(msg);
    });
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
}  // namespace usv_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  usv_controller::UsvJoyController,
  controller_interface::ControllerInterface)
