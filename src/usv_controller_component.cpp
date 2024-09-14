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

#include <std_msgs/msg/header.hpp>
#include <usv_controller/usv_controller_component.hpp>

namespace usv_controller
{
udp_msgs::msg::UdpPacket buildPacketMessage(
  const rclcpp::Time & stamp, const std::string & ip_address, const std::int16_t port,
  const float value)
{
  const auto float_to_uint8_vector = [](float value) {
    uint8_t * pointer = reinterpret_cast<uint8_t *>(&value);
    /// @note Size of float is 32 and size of uint8 is 8. So 32/8 = 4
    return std::vector<uint8_t>(pointer, pointer + 4);
  };
  return udp_msgs::build<udp_msgs::msg::UdpPacket>()
    .header(std_msgs::build<std_msgs::msg::Header>().stamp(stamp).frame_id(""))
    .address(ip_address)
    .src_port(port)
    .data(float_to_uint8_vector(value));
}

UsvControllerComponent::UsvControllerComponent(const rclcpp::NodeOptions & options)
: Node("usv_controller_node", options),
  parameters_(usv_controller_node::ParamListener(get_node_parameters_interface()).get_params()),
  joy_interface_(p9n_interface::getHwType(parameters_.joystick_type)),
  control_mode_(ControlMode::MANUAL),
  last_joy_timestamp_(get_clock()->now())
{
  assert(isManual());
  if (parameters_.auto_start) {
    becomeAutonomous();
  }
  left_thruster_change_state_client_ =
    create_client<lifecycle_msgs::srv::ChangeState>("left_thruster_controller_node/change_state");
  left_thruster_get_state_client_ =
    create_client<lifecycle_msgs::srv::GetState>("left_thruster_controller_node/get_state");
  right_thruster_change_state_client_ =
    create_client<lifecycle_msgs::srv::ChangeState>("right_thruster_controller_node/change_state");
  right_thruster_get_state_client_ =
    create_client<lifecycle_msgs::srv::GetState>("right_thruster_controller_node/get_state");

  using namespace std::chrono_literals;
  while (!left_thruster_change_state_client_->wait_for_service(1s) &&
         !right_thruster_change_state_client_->wait_for_service(1s)) {
    RCLCPP_INFO_STREAM(get_logger(), "Waiting for service is ready...");
  }
  /// @sa https://github.com/ros-drivers/transport_drivers/blob/9fff59f66e4e0f9296501b3f671adc6543509996/udp_driver/src/udp_sender_node.cpp#L72C14-L72C62
  left_thruster_cmd_ = create_publisher<udp_msgs::msg::UdpPacket>(
    "left_thruster_controller_node/udp_write", rclcpp::QoS(rclcpp::KeepLast(32)).best_effort());
  right_thruster_cmd_ = create_publisher<udp_msgs::msg::UdpPacket>(
    "right_thruster_controller_node/udp_write", rclcpp::QoS(rclcpp::KeepLast(32)).best_effort());
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, [this](const sensor_msgs::msg::Joy::ConstSharedPtr & msg) {
      std::lock_guard<std::mutex> lock(mtx_);
      last_joy_timestamp_ = msg->header.stamp;
      joy_interface_.setJoyMsg(msg);
    });
  using namespace std::chrono_literals;
  watchdog_timer_ = create_wall_timer(30ms, [this]() { watchDogFunction(); });
}

void UsvControllerComponent::watchDogFunction()
{
  std::lock_guard<std::mutex> lock(mtx_);
  const auto is_joystick_alive = [this]() {
    constexpr double e = std::numeric_limits<double>::epsilon();
    if (std::abs(parameters_.joystick_connection_timeout) <= e) {
      return true;
    }
    return get_clock()->now() - last_joy_timestamp_ >=
           rclcpp::Duration::from_seconds(parameters_.joystick_connection_timeout);
  };
  const auto is_manual_requested = [this]() {
    return isAutonomous() && joy_interface_.pressedCross();
  };
  if (!isEmergencyStop() && is_joystick_alive()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Joystick disconeected, wamv enters emergency stop mode.");
    becomeEmergency();
    return;
  }
  if (is_manual_requested()) {
    becomeManual();
    return;
  }
}
}  // namespace usv_controller

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(usv_controller::UsvControllerComponent)
