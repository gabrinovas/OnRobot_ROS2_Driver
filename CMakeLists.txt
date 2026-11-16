# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

cmake_minimum_required(VERSION 3.12)  # object libraries that link to other targets
project(onrobot_driver)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(controller_manager REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

# ROS packages the hardware plugins are built against.
set(ONROBOT_HARDWARE_DEPENDENCIES
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)

# The bundled Modbus library, with its serial transport.
set(MODBUS_SERIAL_COMMUNICATION ON CACHE BOOL "Enable Modbus Serial Communication" FORCE)
add_subdirectory(include/onrobot_driver/Modbus)

include(cmake/onrobot_objects.cmake)
include(cmake/onrobot_plugins.cmake)
include(cmake/onrobot_monitor.cmake)
include(cmake/onrobot_python.cmake)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config examples launch urdf DESTINATION share/${PROJECT_NAME})
install(
  FILES twofg_hardware_interface.xml threefg_hardware_interface.xml vgc10_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${ONROBOT_PLUGIN_TARGETS})
ament_export_dependencies(${ONROBOT_HARDWARE_DEPENDENCIES})
pluginlib_export_plugin_description_file(hardware_interface twofg_hardware_interface.xml)
pluginlib_export_plugin_description_file(hardware_interface threefg_hardware_interface.xml)
pluginlib_export_plugin_description_file(hardware_interface vgc10_hardware_interface.xml)

if(BUILD_TESTING)
  include(cmake/onrobot_tests.cmake)
endif()

ament_package()
