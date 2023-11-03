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

#include <hardware_interface/loaned_command_interface.hpp>
#include <string>
#include <usv_controller/usv_joy_controller.hpp>

namespace usv_controller
{
using hardware_interface::LoanedCommandInterface;

UsvJoyController::UsvJoyController()
: rt_command_ptr_(nullptr), sub_(nullptr) {}

controller_interface::return_type UsvJoyController::init(const std::string & controller_name)
{
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }
  try {
    auto node = get_node();
    if (!node->has_parameter("left_azimuth_joint")) {
      node->declare_parameter<std::string>("left_azimuth_joint", "left_azimuth_joint");
    }
    if (!node->has_parameter("right_azimuth_joint")) {
      node->declare_parameter<std::string>("right_azimuth_joint", "right_azimuth_joint");
    }
    if (!node->has_parameter("left_thruster_joint")) {
      node->declare_parameter<std::string>("left_thruster_joint", "left_thruster_joint");
    }
    if (!node->has_parameter("right_thruster_joint")) {
      node->declare_parameter<std::string>("right_thruster_joint", "right_thruster_joint");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration UsvJoyController::command_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names.push_back(left_azimuth_joint_ + "/position");
  command_interfaces_config.names.push_back(right_azimuth_joint_ + "/position");
  command_interfaces_config.names.push_back(left_thruster_joint_ + "/velocity");
  command_interfaces_config.names.push_back(right_thruster_joint_ + "/velocity");
  return command_interfaces_config;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UsvJoyController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  node->get_parameter("left_azimuth_joint", left_azimuth_joint_);
  node->get_parameter("right_azimuth_joint", right_azimuth_joint_);
  node->get_parameter("left_thruster_joint", left_thruster_joint_);
  node->get_parameter("right_thruster_joint", right_thruster_joint_);
  node->get_parameter("joy_topic", joy_topic_);
  sub_ = node->create_subscription<sensor_msgs::msg::Joy>(
    joy_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const sensor_msgs::msg::Joy::SharedPtr msg) {rt_command_ptr_.writeFromNonRT(msg);});
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

#if defined(GALACTIC) || defined(HUMBLE)
controller_interface::return_type UsvJoyController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
#else
controller_interface::return_type UsvJoyController::update()
#endif
{
  auto joy = rt_command_ptr_.readFromRT();
  if (joy && joy->get()) {
    float left_azimuth = joy->get()->axes[0] * M_PI * 0.5;
    float right_azimuth = joy->get()->axes[2] * -1 * M_PI * 0.5;
    float left_thrust = joy->get()->axes[1];
    float right_thrust = joy->get()->axes[5];
    command_interfaces_[0].set_value(left_azimuth);
    command_interfaces_[1].set_value(right_azimuth);
    command_interfaces_[2].set_value(left_thrust);
    command_interfaces_[3].set_value(right_thrust);
  }
  return controller_interface::return_type::OK;
}
}  // namespace usv_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(usv_controller::UsvJoyController, controller_interface::ControllerInterface)
