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


#include <usv_controller/usv_controller_component.hpp>
#include <hardware_interface/loaned_command_interface.hpp>

namespace usv_controller
{
UsvControllerComponent::UsvControllerComponent(const rclcpp::NodeOptions & options)
: Node("usv_controller", options)
{
  if (!has_parameter("left_azimuth_joint")) {
    declare_parameter<std::string>("left_azimuth_joint", "left_azimuth_joint");
  }
  if (!has_parameter("right_azimuth_joint")) {
    declare_parameter<std::string>("right_azimuth_joint", "right_azimuth_joint");
  }
  if (!has_parameter("left_thruster_joint")) {
    declare_parameter<std::string>("left_thruster_joint", "left_thruster_joint");
  }
  if (!has_parameter("right_thruster_joint")) {
    declare_parameter<std::string>("right_thruster_joint", "right_thruster_joint");
  }
  get_parameter("left_azimuth_joint", left_azimuth_joint_);
  get_parameter("right_azimuth_joint", right_azimuth_joint_);
  get_parameter("left_thruster_joint", left_thruster_joint_);
  get_parameter("right_thruster_joint", right_thruster_joint_);
  get_parameter("linear_pid_gain.kp", linear_pid_gain_.p_gain_);
  get_parameter("linear_pid_gain.ki", linear_pid_gain_.i_gain_);
  get_parameter("linear_pid_gain.kd", linear_pid_gain_.d_gain_);
  get_parameter("linear_pid_gain.i_min", linear_pid_gain_.i_min_);
  get_parameter("linear_pid_gain.i_max", linear_pid_gain_.i_max_);
  get_parameter("linear_pid_gain.antiwindup", linear_pid_gain_.antiwindup_);
  get_parameter("anguler_pid_gain.kp", anguler_pid_gain_.p_gain_);
  get_parameter("anguler_pid_gain.ki", anguler_pid_gain_.i_gain_);
  get_parameter("anguler_pid_gain.kd", anguler_pid_gain_.d_gain_);
  get_parameter("anguler_pid_gain.i_min", anguler_pid_gain_.i_min_);
  get_parameter("anguler_pid_gain.i_max", anguler_pid_gain_.i_max_);
  get_parameter("anguler_pid_gain.antiwindup", anguler_pid_gain_.antiwindup_);
  get_parameter("hull_width", hull_width_);

  linear_pid_ = std::make_shared<control_toolbox::Pid>();
  linear_pid_->setGains(linear_pid_gain_);

  anguler_pid_ = std::make_shared<control_toolbox::Pid>();
  anguler_pid_->setGains(linear_pid_gain_);

  target_twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/target_twist", 1,std::bind(&UsvControllerComponent::target_twist_cb, this, std::placeholders::_1));

  thruster_left_cmd_pub_ = create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/thrust", 1);
  thruster_left_agl_pub_ = create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/pos", 1);
  thruster_right_cmd_pub_ = create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/thrust", 1);
  thruster_right_agl_pub_ = create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/pos", 1);
  debug_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/usv_force_cmd", 1);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&UsvControllerComponent::update, this));
}


void UsvControllerComponent::target_twist_cb(const geometry_msgs::msg::Twist & msg)
{
  target_twist_ = msg;
}


void UsvControllerComponent::update()
{
  const uint64_t dt_ns = 10UL * 1000 * 1000;  // 10ms

  if (target_twist_ && current_twist_ ) {
    double target_twist_x = target_twist_->linear.x;
    double target_twist_omega = target_twist_->angular.z;
    double current_twist_x = current_twist_->linear.x;
    double current_twist_omega = current_twist_->angular.z;
    
    current_twist_ = target_twist_;

    // Calculate force
    double linear_force = linear_pid_->computeCommand(target_twist_x - current_twist_x, dt_ns);
    double turning_force =
      anguler_pid_->computeCommand(target_twist_omega - current_twist_omega, dt_ns);

       // Calculate motor thrust
    std::array<float, 2> fb_mthrust;
    fb_mthrust[0] = linear_force + 0.5 * hull_width_ * turning_force;
    fb_mthrust[1] = linear_force - 0.5 * hull_width_ * turning_force;

    // Calculate motor command
    float left_azimuth = 0;
    float right_azimuth = 0;
    float left_thrust = fb_mthrust[1];
    float right_thrust = fb_mthrust[0];

    std_msgs::msg::Float64MultiArray debug_msg;
    debug_msg.data.resize(4);
    debug_msg.data[0] = left_azimuth;
    debug_msg.data[1] = right_azimuth;
    debug_msg.data[2] = left_thrust;
    debug_msg.data[3] = right_thrust;

    debug_cmd_pub_->publish(debug_msg);

    // gazebo

    std_msgs::msg::Float64 wamv_left_agl, wamv_right_agl, wamv_left_thrust, wamv_right_thrust;
    wamv_left_agl.data = left_azimuth;
    wamv_right_agl.data = right_azimuth;

    wamv_left_thrust.data = 100000 * left_thrust;
    wamv_right_thrust.data = 100000 * right_thrust;
    if (wamv_left_thrust.data < 500 && 10 < wamv_left_thrust.data) {
      wamv_left_thrust.data = 500;
    } else if (-500 < wamv_left_thrust.data && wamv_left_thrust.data < 10) {
      wamv_left_thrust.data = -500;
    } else if (wamv_left_thrust.data < -2000) {
      wamv_left_thrust.data = -2000;
    } else if (2000 < wamv_left_thrust.data) {
      wamv_left_thrust.data = 2000;
    }

    if (wamv_right_thrust.data < 500 && 10 < wamv_right_thrust.data) {
      wamv_right_thrust.data = 500;
    } else if (-500 < wamv_right_thrust.data && wamv_right_thrust.data < 10) {
      wamv_right_thrust.data = -500;
    } else if (wamv_right_thrust.data < -2000) {
      wamv_right_thrust.data = -2000;
    } else if (2000 < wamv_right_thrust.data) {
      wamv_right_thrust.data = 2000;
    }

    // wamv_left_thrust.data = 1000;
    // wamv_right_thrust.data = 1000;

    thruster_left_agl_pub_->publish(wamv_left_agl);
    thruster_right_agl_pub_->publish(wamv_right_agl);
    thruster_left_cmd_pub_->publish(wamv_left_thrust);
    thruster_right_cmd_pub_->publish(wamv_right_thrust);
  }
} 
}  // namespace usv_controller

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(usv_controller::UsvControllerComponent)

