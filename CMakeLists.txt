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

add_library(usv_controller SHARED
  src/usv_velocity_controller.cpp
  src/usv_joy_controller.cpp
)
target_include_directories(
  usv_controller
  PRIVATE
  include
)
ament_target_dependencies(
  usv_controller
  builtin_interfaces
  pluginlib
  rcutils
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  realtime_tools
  sensor_msgs
  geometry_msgs
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(usv_controller PRIVATE "USV_CONTROLLER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(usv_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface usv_controller.xml)

install(
  TARGETS usv_controller
  DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  builtin_interfaces
  pluginlib
  rcutils
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  realtime_tools
  sensor_msgs
  geometry_msgs
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  usv_controller
)

ament_package()
