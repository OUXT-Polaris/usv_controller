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

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <control_toolbox/pid.hpp>
#include <string>
#include <usv_controller/visibility_control.hpp>

namespace usv_controller
{
class UsvVelocityController : public controller_interface::ControllerInterface
{
public:
  UsvVelocityController(){}
  controller_interface::return_type init(const std::string & controller_name) override
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
        get_node()->get_logger(), "Exception thrown during init stage with message: %s \n",
        e.what());
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

#if GALACTIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init()
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
#endif

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    auto node = get_node();
    node->get_parameter("left_azimuth_joint", left_azimuth_joint_);
    node->get_parameter("right_azimuth_joint", right_azimuth_joint_);
    node->get_parameter("left_thruster_joint", left_thruster_joint_);
    node->get_parameter("right_thruster_joint", right_thruster_joint_);
    node->get_parameter("linear_pid_gain.kp", linear_pid_gain_.p_gain_);
    node->get_parameter("linear_pid_gain.ki", linear_pid_gain_.i_gain_);
    node->get_parameter("linear_pid_gain.kd", linear_pid_gain_.d_gain_);
    node->get_parameter("linear_pid_gain.i_min", linear_pid_gain_.i_min_);
    node->get_parameter("linear_pid_gain.i_max", linear_pid_gain_.i_max_);
    node->get_parameter("linear_pid_gain.antiwindup", linear_pid_gain_.antiwindup_);
    node->get_parameter("anguler_pid_gain.kp", anguler_pid_gain_.p_gain_);
    node->get_parameter("anguler_pid_gain.ki", anguler_pid_gain_.i_gain_);
    node->get_parameter("anguler_pid_gain.kd", anguler_pid_gain_.d_gain_);
    node->get_parameter("anguler_pid_gain.i_min", anguler_pid_gain_.i_min_);
    node->get_parameter("anguler_pid_gain.i_max", anguler_pid_gain_.i_max_);
    node->get_parameter("anguler_pid_gain.antiwindup", anguler_pid_gain_.antiwindup_);
    node->get_parameter("hull_width", hull_width_);

    linear_pid_ = std::make_shared<control_toolbox::Pid>();
    linear_pid_->setGains(linear_pid_gain_);

    anguler_pid_ = std::make_shared<control_toolbox::Pid>();
    anguler_pid_->setGains(linear_pid_gain_);

    target_twist_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "target_twist", 1,
      [&](geometry_msgs::msg::Twist::SharedPtr msg){ target_twist_ptr_.writeFromNonRT(msg); });

    current_twist_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "current_twist", 1,
      [&](geometry_msgs::msg::Twist::SharedPtr msg){ current_twist_ptr_.writeFromNonRT(msg); });

    debug_cmd_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("/usv_force_cmd", 1);

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

#if GALACTIC
  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period)
#else
  controller_interface::return_type update() override
#endif
  {
#if GALACTIC
    (void) time;
    const uint64_t dt_ns = (uint64_t)period.nanoseconds();
#else
    const uint64_t dt_ns = 10UL * 1000 * 1000; // 10ms
#endif
    auto target_twist = target_twist_ptr_.readFromRT();
    auto current_twist = current_twist_ptr_.readFromRT();
    if(target_twist && target_twist->get() && current_twist && current_twist->get())
    {
      double target_twist_x = target_twist->get()->linear.x;
      double target_twist_omega = target_twist->get()->angular.z;
      double current_twist_x = current_twist->get()->linear.x;
      double current_twist_omega = current_twist->get()->angular.z;

      // Calculate force
      double linear_force = linear_pid_->computeCommand(target_twist_x-current_twist_x, dt_ns);
      double turning_force = anguler_pid_->computeCommand(target_twist_omega-current_twist_omega, dt_ns);

      // Calculate motor thrust
      std::array<double, 2> fb_mthrust;
      fb_mthrust[0] = linear_force+0.5*hull_width_*turning_force;
      fb_mthrust[1] = linear_force-0.5*hull_width_*turning_force;

      // Calculate motor command
      float left_azimuth = 0;
      float right_azimuth = 0;
      float left_thrust = fb_mthrust[1];
      float right_thrust = fb_mthrust[0];
      command_interfaces_[0].set_value(left_azimuth);
      command_interfaces_[1].set_value(right_azimuth);
      command_interfaces_[2].set_value(left_thrust);
      command_interfaces_[3].set_value(right_thrust);

      std_msgs::msg::Float64MultiArray debug_msg;
      debug_msg.data.resize(4);
      debug_msg.data[0] = left_azimuth;
      debug_msg.data[1] = right_azimuth;
      debug_msg.data[2] = left_thrust;
      debug_msg.data[3] = right_thrust;
      
      debug_cmd_pub_->publish(debug_msg);
    }

    return controller_interface::return_type::OK;
  }

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
  using PidSharedPtr = std::shared_ptr<control_toolbox::Pid>;
  control_toolbox::Pid::Gains linear_pid_gain_;
  PidSharedPtr linear_pid_ = {nullptr};
  control_toolbox::Pid::Gains anguler_pid_gain_;
  PidSharedPtr anguler_pid_ = {nullptr};
  double hull_width_;
};

}  // namespace usv_controller

#endif  // USV_CONTROLLER__USV_VELOCITY_CONTROLLER_HPP_
