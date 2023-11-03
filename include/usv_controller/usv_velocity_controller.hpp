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

#include <realtime_tools/realtime_buffer.h>

#include <control_toolbox/pid.hpp>
#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <usv_controller/visibility_control.hpp>

namespace usv_controller
{
class UsvVelocityController : public controller_interface::ControllerInterface
{
public:
  UsvVelocityController();

  controller_interface::return_type init(const std::string & controller_name);

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

#if defined(GALACTIC) || defined(HUMBLE)
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init();
#endif

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

#if defined(GALACTIC) || defined(HUMBLE)
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &);
#else
  controller_interface::return_type update() override;
#endif

protected:
  std::string left_azimuth_joint_;
  std::string right_azimuth_joint_;
  std::string left_thruster_joint_;
  std::string right_thruster_joint_;
  std::string logger_name_;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist::SharedPtr> target_twist_ptr_{nullptr};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr target_twist_sub_;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist::SharedPtr> current_twist_ptr_{nullptr};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr current_twist_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thruster_left_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thruster_left_agl_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thruster_right_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thruster_right_agl_pub_;
  using PidSharedPtr = std::shared_ptr<control_toolbox::Pid>;
  control_toolbox::Pid::Gains linear_pid_gain_;
  PidSharedPtr linear_pid_ = {nullptr};
  control_toolbox::Pid::Gains anguler_pid_gain_;
  PidSharedPtr anguler_pid_ = {nullptr};
  double hull_width_;
};

}  // namespace usv_controller

#endif  // USV_CONTROLLER__USV_VELOCITY_CONTROLLER_HPP_
