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
#include <usv_controller/usv_controller_base.hpp>

#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/subscription.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <controller_interface/controller_interface.hpp>

namespace usv_controller
{
class UsvJoyController : public usv_controller::UsvControllerBase<sensor_msgs::msg::Joy>
{
public:
  USV_CONTROLLER_PUBLIC
  UsvJoyController();

  USV_CONTROLLER_PUBLIC
  controller_interface::return_type update() override;
};

}  // namespace usv_controller

#endif  // USV_CONTROLLER__USV_JOY_CONTROLLER_HPP_
