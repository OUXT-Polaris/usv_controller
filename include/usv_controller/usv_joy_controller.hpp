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
#ifndef USV_CONTROLLER__USV_JOY_CONTROLLER_HPP_
#define USV_CONTROLLER__USV_JOY_CONTROLLER_HPP_

#include <realtime_tools/realtime_buffer.h>

#include <controller_interface/controller_interface.hpp>
#include <memory>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <usv_controller/visibility_control.hpp>

namespace usv_controller
{
class UsvJoyController : public controller_interface::ControllerInterface
{
public:
  UsvJoyController();
  controller_interface::return_type init(const std::string & controller_name);

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
  }

#if defined(GALACTIC) || defined(HUMBLE)
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init()
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
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
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
#else
  controller_interface::return_type update() override;
#endif

protected:
  realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::Joy>> rt_command_ptr_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  std::string joy_topic_;
  std::string left_azimuth_joint_;
  std::string right_azimuth_joint_;
  std::string left_thruster_joint_;
  std::string right_thruster_joint_;
  std::string logger_name_;
};

}  // namespace usv_controller

#endif  // USV_CONTROLLER__USV_JOY_CONTROLLER_HPP_
