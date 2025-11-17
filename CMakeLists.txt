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
  ${CMAKE_SOURCE_DIR}/include/onrobot_driver/common
  ${CMAKE_SOURCE_DIR}/include/onrobot_driver/threefg
  ${CMAKE_SOURCE_DIR}/include/onrobot_driver/twofg
  ${CMAKE_SOURCE_DIR}/include/onrobot_driver/rg
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
  src/RG.cpp
  src/rg_hardware_interface.cpp
)
target_link_libraries(onrobot_rg_driver_plugin
  Modbus
  pluginlib::pluginlib
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
# Build the TwoFG hardware interface as a shared library
# =============================================================================
add_library(onrobot_twofg_driver_plugin
  SHARED
  src/TwoFG.cpp
  src/twofg_hardware_interface.cpp
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
# Build the ThreeFG hardware interface as a shared library
# =============================================================================
add_library(onrobot_threefg_driver_plugin
  SHARED
  src/ThreeFG.cpp
  src/threefg_hardware_interface.cpp
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
# PYBIND11: Python bindings for grippers
# =============================================================================
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# DEBUG: Print versions
message(STATUS "pybind11 found: ${pybind11_VERSION}")
message(STATUS "Python3 found: ${Python3_VERSION}")
message(STATUS "Python3 interpreter: ${Python3_EXECUTABLE}")

# Create TwoFG Python module
pybind11_add_module(TwoFG
  src/TwoFG.cpp
  src/TwoFG_binding.cpp
)
target_include_directories(TwoFG
  PRIVATE
    include
    ${CMAKE_SOURCE_DIR}/include
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(TwoFG
  PRIVATE
    Modbus
)

# Create ThreeFG Python module
pybind11_add_module(ThreeFG
  src/ThreeFG.cpp
  src/ThreeFG_binding.cpp
)
target_include_directories(ThreeFG
  PRIVATE
    include
    ${CMAKE_SOURCE_DIR}/include
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(ThreeFG
  PRIVATE
    Modbus
)

# Create RG Python module (optional - add if you want Python support for RG)
pybind11_add_module(RG
  src/RG.cpp
  src/RG_binding.cpp
)
target_include_directories(RG
  PRIVATE
    include
    ${CMAKE_SOURCE_DIR}/include
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(RG
  PRIVATE
    Modbus
)

# Install Python modules
install(
  TARGETS TwoFG ThreeFG RG
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install Python nodes
install(
  PROGRAMS
    src/test_traj_with_gripper.py
  DESTINATION lib/${PROJECT_NAME}
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
    TwoFG
    ThreeFG
    RG
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

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
# Export hardware plugins
# =============================================================================
pluginlib_export_plugin_description_file(hardware_interface twofg_hardware_interface.xml)
pluginlib_export_plugin_description_file(hardware_interface threefg_hardware_interface.xml)
pluginlib_export_plugin_description_file(hardware_interface rg_hardware_interface.xml)

# =============================================================================
# Testing
# =============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()