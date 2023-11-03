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
#include <usv_controller/usv_velocity_controller.hpp>

namespace usv_controller
{
using hardware_interface::LoanedCommandInterface;
UsvVelocityController::UsvVelocityController()
: target_twist_ptr_(nullptr),
  target_twist_sub_(nullptr),
  current_twist_ptr_(nullptr),
  current_twist_sub_(nullptr)
{
}

controller_interface::return_type UsvVelocityController::init(const std::string & controller_name)
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

controller_interface::InterfaceConfiguration
UsvVelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names.push_back(left_azimuth_joint_ + "/position");
  command_interfaces_config.names.push_back(right_azimuth_joint_ + "/position");
  command_interfaces_config.names.push_back(left_thruster_joint_ + "/velocity");
  command_interfaces_config.names.push_back(right_thruster_joint_ + "/velocity");
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration UsvVelocityController::state_interface_configuration()
  const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

#if defined(GALACTIC) || defined(HUMBLE)
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UsvVelocityController::on_init()
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
#endif

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UsvVelocityController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
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
    [&](geometry_msgs::msg::Twist::SharedPtr msg) { target_twist_ptr_.writeFromNonRT(msg); });

  current_twist_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    "current_twist", 1,
    [&](geometry_msgs::msg::Twist::SharedPtr msg) { current_twist_ptr_.writeFromNonRT(msg); });

  debug_cmd_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("/usv_force_cmd", 1);

  thruster_left_cmd_pub_ = node->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/thrust", 1);
  thruster_left_agl_pub_ = node->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/pos", 1);
  thruster_right_cmd_pub_ = node->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/thrust", 1);
  thruster_right_agl_pub_ = node->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/pos", 1);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

#if defined(GALACTIC) || defined(HUMBLE)
controller_interface::return_type UsvVelocityController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
#else
controller_interface::return_type UsvVelocityController::update() override
#endif
{
#if GALACTIC
  (void)time;
  const uint64_t dt_ns = (uint64_t)period.nanoseconds();
#else
  const uint64_t dt_ns = 10UL * 1000 * 1000;  // 10ms
#endif
  auto target_twist = target_twist_ptr_.readFromRT();
  auto current_twist = current_twist_ptr_.readFromRT();
  if (target_twist && target_twist->get() && current_twist && current_twist->get()) {
    double target_twist_x = target_twist->get()->linear.x;
    double target_twist_omega = target_twist->get()->angular.z;
    double current_twist_x = current_twist->get()->linear.x;
    double current_twist_omega = current_twist->get()->angular.z;

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

    // gazebo 
    
    std_msgs::msg::Float64 wamv_left_agl, wamv_right_agl, wamv_left_thrust, wamv_right_thrust;
    wamv_left_agl.data = left_azimuth;
    wamv_right_agl.data = right_azimuth;

    wamv_left_thrust.data = 100000*left_thrust;
    wamv_right_thrust.data = 100000*right_thrust;
    if (wamv_left_thrust.data<500 && 10 < wamv_left_thrust.data)
      wamv_left_thrust.data = 500;
    else if (-500 < wamv_left_thrust.data && wamv_left_thrust.data < 10)
      wamv_left_thrust.data = -500;
    else if (wamv_left_thrust.data < -2000)
      wamv_left_thrust.data = -20000;
    else if (2000 < wamv_left_thrust.data )
      wamv_left_thrust.data = 20000;

    
    if (wamv_right_thrust.data<500 && 10 < wamv_right_thrust.data)
      wamv_right_thrust.data = 500;
    else if (-500 < wamv_right_thrust.data && wamv_right_thrust.data < 10)
      wamv_right_thrust.data = -500;
    else if (wamv_right_thrust.data < -2000)
      wamv_right_thrust.data = -20000;
    else if (2000 < wamv_right_thrust.data )
      wamv_right_thrust.data = 20000;

    
    // wamv_left_thrust.data = 1000;
    // wamv_right_thrust.data = 1000;
    

    thruster_left_agl_pub_->publish(wamv_left_agl);
    thruster_right_agl_pub_->publish(wamv_right_agl);
    thruster_left_cmd_pub_->publish(wamv_left_thrust);
    thruster_right_cmd_pub_->publish(wamv_right_thrust);
    
  }

  return controller_interface::return_type::OK;
}

}  // namespace usv_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  usv_controller::UsvVelocityController, controller_interface::ControllerInterface)
