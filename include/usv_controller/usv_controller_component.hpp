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

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <usv_controller/visibility_control.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <control_toolbox/pid.hpp>
#include <controller_interface/controller_interface.hpp>

namespace usv_controller
{
class UsvControllerComponent : public rclcpp::Node
{
public:
  USV_CONTROLLER_PUBLIC
  explicit UsvControllerComponent(const rclcpp::NodeOptions & options);

private:
  // std::shared_ptr<std_msgs::msgs::String> display_color_pub_;
  std::string left_azimuth_joint_;
  std::string right_azimuth_joint_;
  std::string left_thruster_joint_;
  std::string right_thruster_joint_;
  std::string logger_name_;
  
  //publisher
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thruster_left_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thruster_left_agl_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thruster_right_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thruster_right_agl_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_cmd_pub_;
  //subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr target_twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr current_twist_sub_;
  //timer
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<geometry_msgs::msg::Twist> target_twist_;
  std::optional<geometry_msgs::msg::Twist> current_twist_;
  
  using PidSharedPtr = std::shared_ptr<control_toolbox::Pid>;
  control_toolbox::Pid::Gains linear_pid_gain_;
  PidSharedPtr linear_pid_ = {nullptr};
  control_toolbox::Pid::Gains anguler_pid_gain_;
  PidSharedPtr anguler_pid_ = {nullptr};
  double hull_width_;
  void target_twist_cb(const geometry_msgs::msg::Twist & msg);
  void update();
};
}  // namespace usv_controller

#endif  // USV_CONTROLLER__USV_CONTROLLER_COMPONENT_HPP_
