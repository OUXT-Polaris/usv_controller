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

if($ENV{ROS_DISTRO} STREQUAL "galactic")
  add_definitions(-DGALACTIC)
endif()

if($ENV{ROS_DISTRO} STREQUAL "humble")
  add_definitions(-DHUMBLE)
endif()


# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(usv_controller SHARED
  src/usv_velocity_controller.cpp
  src/usv_joy_controller.cpp
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(usv_controller PRIVATE "USV_CONTROLLER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(usv_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface usv_controller.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(
  usv_controller
)

ament_auto_package()
