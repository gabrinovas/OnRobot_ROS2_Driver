# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

# Tests: protocol classes against a mock Modbus connection, the hardware plugins against the description this
# package generates, the Python configuration helpers, the source rules and the linters.
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(ament_index_cpp REQUIRED)
include(cmake/onrobot_linters.cmake)

ament_add_gtest(test_modbus_crc_frames test/test_modbus_crc_frames.cpp)
target_link_libraries(test_modbus_crc_frames Modbus)

# onrobot_add_register_test(<gripper object library name>)
#   Test of one protocol class: test/test_<name>_registers.cpp with the protocol objects.
function(onrobot_add_register_test name)
  ament_add_gtest(test_${name}_registers
    test/test_${name}_registers.cpp
    $<TARGET_OBJECTS:onrobot_gripper_objects>
    $<TARGET_OBJECTS:onrobot_${name}_objects>
  )
  target_link_libraries(test_${name}_registers Modbus)
endfunction()

onrobot_add_register_test(twofg)
onrobot_add_register_test(threefg)
onrobot_add_register_test(vgc10)

ament_add_gtest(test_hardware_plugins test/test_hardware_plugins.cpp)
target_link_libraries(test_hardware_plugins ${ONROBOT_PLUGIN_TARGETS} Modbus)
ament_target_dependencies(test_hardware_plugins ament_index_cpp ${ONROBOT_HARDWARE_DEPENDENCIES})

ament_add_pytest_test(test_configuration test/test_configuration.py)
ament_add_pytest_test(test_source_rules test/test_source_rules.py)
