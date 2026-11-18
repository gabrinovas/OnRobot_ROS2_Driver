# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

# Sources shared by several targets, compiled once as object libraries. The protocol classes (plain C++ over
# Modbus) go into the hardware plugins, the Python modules and the tests; the plugin code shared by the grippers
# goes into every plugin.

# onrobot_add_objects(<name> [ROS] <source>...)
#   Creates the object library onrobot_<name>_objects. ROS: the sources use the ROS hardware packages.
function(onrobot_add_objects name)
  cmake_parse_arguments(ARG "ROS" "" "" ${ARGN})
  set(target onrobot_${name}_objects)
  add_library(${target} OBJECT ${ARG_UNPARSED_ARGUMENTS})
  if(ARG_ROS)
    target_link_libraries(${target} Modbus pluginlib::pluginlib)
    ament_target_dependencies(${target} ${ONROBOT_HARDWARE_DEPENDENCIES})
  else()
    target_link_libraries(${target} PRIVATE Modbus)
  endif()
endfunction()

onrobot_add_objects(gripper src/gripper/connection.cpp src/gripper/requests.cpp)
onrobot_add_objects(twofg src/gripper/TwoFG.cpp)
onrobot_add_objects(threefg src/gripper/ThreeFG.cpp)
onrobot_add_objects(vgc10 src/gripper/VGC10.cpp)

onrobot_add_objects(hardware_base ROS src/hardware/gripper_hardware_base.cpp)
onrobot_add_objects(finger_hardware ROS
  src/hardware/finger_configuration.cpp
  src/hardware/finger_communication.cpp
)
