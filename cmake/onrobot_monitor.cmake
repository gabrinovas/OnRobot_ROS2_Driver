# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

# Node that publishes the state of the gripper as diagnostics and offers the tool power reset.
add_executable(gripper_status_monitor
  $<TARGET_OBJECTS:onrobot_gripper_objects>
  src/monitor/main.cpp
  src/monitor/gripper_status_monitor.cpp
  src/monitor/diagnostics.cpp
  src/monitor/power_reset.cpp
)
target_link_libraries(gripper_status_monitor Modbus)
ament_target_dependencies(gripper_status_monitor
  diagnostic_msgs
  diagnostic_updater
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)
install(TARGETS gripper_status_monitor DESTINATION lib/${PROJECT_NAME})
