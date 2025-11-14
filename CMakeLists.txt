cmake_minimum_required(VERSION 3.5)
project(onrobot_driver)

# Enable PIC for all targets
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_SOURCE_DIR}/include/onrobot_driver
)

# List of dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_manager
  controller_manager_msgs
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
  tf2_geometry_msgs
)

# Enable serial communication for the Modbus library
set(MODBUS_SERIAL_COMMUNICATION ON CACHE BOOL "Enable Modbus Serial Communication" FORCE)

# Add the Modbus library as a subdirectory
add_subdirectory(include/onrobot_driver/Modbus)

# =============================================================================
# Build the RG hardware interface as a shared library
# =============================================================================
add_library(onrobot_rg_driver_plugin
  SHARED
  src/rg_hardware_interface.cpp
  src/RG.cpp
  include/onrobot_driver/TCPConnectionWrapper.hpp
  include/onrobot_driver/SerialConnectionWrapper.hpp
  include/onrobot_driver/IModbusConnection.hpp
)

target_link_libraries(onrobot_rg_driver_plugin
  Modbus
  pluginlib::pluginlib  # NECESARIO para PLUGINLIB_EXPORT_CLASS
)

target_include_directories(onrobot_rg_driver_plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    include
)

ament_target_dependencies(onrobot_rg_driver_plugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# =============================================================================
# Build the 2FG hardware interface as a shared library
# =============================================================================
add_library(onrobot_twofg_driver_plugin
  SHARED
  src/twofg_hardware_interface.cpp
  src/TwoFG.cpp
  include/onrobot_driver/TCPConnectionWrapper.hpp
  include/onrobot_driver/SerialConnectionWrapper.hpp
  include/onrobot_driver/IModbusConnection.hpp
)

target_link_libraries(onrobot_twofg_driver_plugin
  Modbus
  pluginlib::pluginlib
)

target_include_directories(onrobot_twofg_driver_plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    include
)

ament_target_dependencies(onrobot_twofg_driver_plugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# =============================================================================
# Build the 3FG hardware interface as a shared library
# =============================================================================
add_library(onrobot_threefg_driver_plugin
  SHARED
  src/threefg_hardware_interface.cpp
  src/ThreeFG.cpp
  include/onrobot_driver/TCPConnectionWrapper.hpp
  include/onrobot_driver/SerialConnectionWrapper.hpp
  include/onrobot_driver/IModbusConnection.hpp
)

target_link_libraries(onrobot_threefg_driver_plugin
  Modbus
  pluginlib::pluginlib
)

target_include_directories(onrobot_threefg_driver_plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    include
)

ament_target_dependencies(onrobot_threefg_driver_plugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# =============================================================================
# Build the gripper status monitor node
# =============================================================================
add_executable(gripper_status_monitor
  src/gripper_status_monitor.cpp
)

ament_target_dependencies(gripper_status_monitor
  rclcpp
  std_msgs
)

# =============================================================================
# INSTALL
# =============================================================================
install(
  TARGETS
    onrobot_rg_driver_plugin
    onrobot_twofg_driver_plugin
    onrobot_threefg_driver_plugin
    gripper_status_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch and config files
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install the plugin description files
install(
  FILES
    rg_hardware_interface.xml
    twofg_hardware_interface.xml
    threefg_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# =============================================================================
# EXPORTS
# =============================================================================
ament_export_include_directories(include)
ament_export_libraries(
  onrobot_rg_driver_plugin
  onrobot_twofg_driver_plugin
  onrobot_threefg_driver_plugin
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# =============================================================================
# Export hardware plugins (pluginlib registration)
# =============================================================================
pluginlib_export_plugin_description_file(hardware_interface twofg_hardware_interface.xml)
pluginlib_export_plugin_description_file(hardware_interface threefg_hardware_interface.xml)
pluginlib_export_plugin_description_file(hardware_interface rg_hardware_interface.xml)

# =============================================================================
# Testing (optional)
# =============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()