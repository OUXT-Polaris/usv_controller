cmake_minimum_required(VERSION 3.5)
project(usv_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)

add_library(usv_velocity_controller SHARED
  src/usv_velocity_controller.cpp
)
target_include_directories(
  usv_velocity_controller
  PRIVATE
  include
)
ament_target_dependencies(
  usv_velocity_controller
  pluginlib
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  realtime_tools
)

add_library(usv_joy_controller SHARED
  src/usv_joy_controller.cpp
)
target_include_directories(
  usv_joy_controller
  PRIVATE
  include
)
ament_target_dependencies(
  usv_joy_controller
  pluginlib
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  realtime_tools
)

pluginlib_export_plugin_description_file(usv_controller usv_controller.xml)

install(
  TARGETS usv_velocity_controller usv_joy_controller
  DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
