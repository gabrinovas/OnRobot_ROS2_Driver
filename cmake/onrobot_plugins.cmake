# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

# The ros2_control hardware plugins, one shared library per gripper.

# onrobot_add_plugin(<name> OBJECTS <object library name>... SOURCES <source>...)
#   Creates the plugin library onrobot_<name>_driver_plugin from its own sources and the objects of the
#   object libraries onrobot_<object library name>_objects.
function(onrobot_add_plugin name)
  cmake_parse_arguments(ARG "" "" "OBJECTS;SOURCES" ${ARGN})
  set(target onrobot_${name}_driver_plugin)
  set(objects)
  foreach(object_library ${ARG_OBJECTS})
    list(APPEND objects $<TARGET_OBJECTS:onrobot_${object_library}_objects>)
  endforeach()
  add_library(${target} SHARED ${objects} ${ARG_SOURCES})
  target_link_libraries(${target} Modbus pluginlib::pluginlib)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${target} ${ONROBOT_HARDWARE_DEPENDENCIES})
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
  set(ONROBOT_PLUGIN_TARGETS ${ONROBOT_PLUGIN_TARGETS} ${target} PARENT_SCOPE)
endfunction()

onrobot_add_plugin(twofg
  OBJECTS gripper twofg hardware_base finger_hardware
  SOURCES src/hardware/twofg_hardware_interface.cpp
)
onrobot_add_plugin(threefg
  OBJECTS gripper threefg hardware_base finger_hardware
  SOURCES src/hardware/threefg_hardware_interface.cpp
)
onrobot_add_plugin(vgc10
  OBJECTS gripper vgc10 hardware_base
  SOURCES src/hardware/vgc10_configuration.cpp src/hardware/vgc10_communication.cpp
)
