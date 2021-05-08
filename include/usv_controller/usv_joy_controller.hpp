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

#include <usv_controller/visibility_control.hpp>

#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/subscription.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <controller_interface/controller_interface.hpp>

namespace usv_controller
{
class UsvJoyController : public controller_interface::ControllerInterface
{
public:
  UsvJoyController()
  : rt_command_ptr_(nullptr)//, command_subscriber_(nullptr)
  {}
  controller_interface::return_type init(const std::string & controller_name) override
  {
    auto ret = ControllerInterface::init(controller_name);
    if (ret != controller_interface::return_type::OK) {
      return ret;
    }
    try {
      auto node = get_node();
      node->declare_parameter<std::string>("left_azimuth_joint", "left_azimuth_joint");
      node->declare_parameter<std::string>("right_azimuth_joint", "right_azimuth_joint");
      node->declare_parameter<std::string>("left_thruster_joint", "left_thruster_joint");
      node->declare_parameter<std::string>("right_thruster_joint", "right_thruster_joint");
      node->declare_parameter<std::string>("joy_topic", "/joy");
    } catch (const std::exception & e) {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return controller_interface::return_type::ERROR;
    }
    return controller_interface::return_type::OK;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    command_interfaces_config.names.push_back(left_azimuth_joint_ + "/position");
    command_interfaces_config.names.push_back(right_azimuth_joint_ + "/position");
    command_interfaces_config.names.push_back(left_thruster_joint_ + "/velocity");
    command_interfaces_config.names.push_back(right_thruster_joint_ + "/velocity");
    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) override
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

  controller_interface::return_type update() override
  {
    return controller_interface::return_type::OK;
  }

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
