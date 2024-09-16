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

#include <thruster_command.pb.h>

#include <p9n_interface/p9n_interface.hpp>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <usv_controller/visibility_control.hpp>
#include <usv_controller_component_parameters.hpp>

namespace usv_controller
{
class UsvControllerComponent : public rclcpp::Node
{
  enum class ControlMode { AUTONOMOUS, MANUAL, EMERGENCY_STOP };

public:
  USV_CONTROLLER_PUBLIC
  explicit UsvControllerComponent(const rclcpp::NodeOptions & options);

private:
  void watchDogFunction();
  void controlFunction();
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_sub_;

  const usv_controller_node::Params parameters_;
  bool joy_subscribed_ = false;
  p9n_interface::PlayStationInterface joy_interface_;
  protolink::udp::Client left_thruster_client_;
  protolink::udp::Client right_thruster_client_;
  std::mutex mtx_;
  ControlMode control_mode_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Time last_joy_timestamp_;
#define DEFINE_IS_FUNCTION(StateName, StateEnumName) \
  bool is##StateName() const { return control_mode_ == ControlMode::StateEnumName; }

  DEFINE_IS_FUNCTION(Autonomous, AUTONOMOUS)
  DEFINE_IS_FUNCTION(Manual, MANUAL)
  DEFINE_IS_FUNCTION(EmergencyStop, EMERGENCY_STOP)

  bool becomeAutonomous()
  {
    if (isManual()) {
      RCLCPP_INFO_STREAM(get_logger(), "wamv become autonomous state.");
      control_mode_ = ControlMode::AUTONOMOUS;
      return true;
    }
    if (isEmergencyStop()) {
      RCLCPP_ERROR_STREAM(get_logger(), "wamv is in emergency stop state.");
    }
    return false;
  }

  bool becomeManual()
  {
    if (isAutonomous()) {
      control_mode_ = ControlMode::MANUAL;
      RCLCPP_INFO_STREAM(get_logger(), "wamv become manual state.");
      return true;
    }
    if (isEmergencyStop()) {
      RCLCPP_ERROR_STREAM(get_logger(), "wamv is in emergency stop state.");
    }
    return false;
  }

  bool becomeEmergency()
  {
    RCLCPP_INFO_STREAM(get_logger(), "wamv become emergency state.");
    control_mode_ = ControlMode::EMERGENCY_STOP;
    return true;
  }
};  // namespace usv_controller
}  // namespace usv_controller

#endif  // USV_CONTROLLER__USV_CONTROLLER_COMPONENT_HPP_
