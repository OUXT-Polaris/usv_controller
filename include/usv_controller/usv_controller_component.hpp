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

#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <p9n_interface/p9n_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <udp_driver/udp_driver.hpp>
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
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_sub_;
  std::shared_ptr<rclcpp::Publisher<udp_msgs::msg::UdpPacket>> left_thruster_cmd_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>>
    left_thruster_change_state_client_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> left_thruster_get_state_client_;
  std::shared_ptr<rclcpp::Publisher<udp_msgs::msg::UdpPacket>> right_thruster_cmd_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>>
    right_thruster_change_state_client_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> right_thruster_get_state_client_;

  const usv_controller_node::Params parameters_;
  p9n_interface::PlayStationInterface joy_interface_;
  std::mutex mtx_;
  ControlMode control_mode_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Time last_joy_timestamp_;
#define DEFINE_IS_FUNCTION(StateName, StateEnumName) \
  bool is##StateName() const { return control_mode_ == ControlMode::StateEnumName; }

  bool activateThrusterDrivers()
  {
    using namespace std::chrono_literals;
    if (
      !left_thruster_change_state_client_->wait_for_service(1s) &&
      !left_thruster_get_state_client_->wait_for_service(1s) &&
      !right_thruster_change_state_client_->wait_for_service(1s) &&
      !right_thruster_get_state_client_->wait_for_service(1s)) {
      return false;
    }
    auto request_get_state = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto result = left_thruster_get_state_client_->async_send_request(request_get_state);
    return true;
  }

  DEFINE_IS_FUNCTION(Autonomous, AUTONOMOUS)
  DEFINE_IS_FUNCTION(Manual, MANUAL)
  DEFINE_IS_FUNCTION(EmergencyStop, EMERGENCY_STOP)

  bool becomeAutonomous()
  {
    if (isManual()) {
      RCLCPP_INFO_STREAM(get_logger(), "wamv is becom automous.");
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
      return true;
    }
    if (isEmergencyStop()) {
      RCLCPP_ERROR_STREAM(get_logger(), "wamv is in emergency stop state.");
    }
    return false;
  }

  bool becomeEmergency()
  {
    control_mode_ = ControlMode::EMERGENCY_STOP;
    return true;
  }

#define DEFINE_IS_STATE_FUNCTION(STATE_NAME)                                                  \
  bool is##STATE_NAME##State(                                                                 \
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> & client)                  \
  {                                                                                           \
    using namespace std::chrono_literals;                                                     \
    if (!client->wait_for_service(1s)) {                                                      \
      return false;                                                                           \
    }                                                                                         \
    auto result =                                                                             \
      client->async_send_request(std::make_shared<lifecycle_msgs::srv::GetState::Request>()); \
    auto return_code = rclcpp::spin_until_future_complete(get_node_base_interface(), result); \
    if (                                                                                      \
      return_code == rclcpp::FutureReturnCode::SUCCESS &&                                     \
      result.get()->current_state.id == lifecycle_msgs::msg::State::STATE_NAME) {             \
      return true;                                                                            \
    }                                                                                         \
    return false;                                                                             \
  }

  DEFINE_IS_STATE_FUNCTION(PRIMARY_STATE_UNKNOWN);
  DEFINE_IS_STATE_FUNCTION(PRIMARY_STATE_UNCONFIGURED);
  DEFINE_IS_STATE_FUNCTION(PRIMARY_STATE_INACTIVE);

#define DEFINE_TRANSITION_FUNCTION(TRANSITION_NAME)                                                \
  bool TRANSITION_NAME(std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> & client) \
  {                                                                                                \
    using namespace std::chrono_literals;                                                          \
    if (!client->wait_for_service(1s)) {                                                           \
      return false;                                                                                \
    }                                                                                              \
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();                  \
    request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_NAME;                     \
    auto result = client->async_send_request(request);                                             \
    auto return_code = rclcpp::spin_until_future_complete(get_node_base_interface(), result);      \
    if (return_code == rclcpp::FutureReturnCode::SUCCESS && result.get()->success) {               \
      return true;                                                                                 \
    }                                                                                              \
    return false;                                                                                  \
  }

  DEFINE_TRANSITION_FUNCTION(TRANSITION_CONFIGURE)
  DEFINE_TRANSITION_FUNCTION(TRANSITION_ACTIVATE)
};  // namespace usv_controller
}  // namespace usv_controller

#endif  // USV_CONTROLLER__USV_CONTROLLER_COMPONENT_HPP_
