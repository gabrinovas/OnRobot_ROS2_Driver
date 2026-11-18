# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

# Python modules (pybind11) of the protocol classes, installed inside the onrobot_driver Python package.
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME})

# onrobot_add_python_module(<module> <object library name>)
#   Creates the Python module <module> from src/bindings/<module>_binding.cpp and the protocol objects.
function(onrobot_add_python_module module object_library)
  pybind11_add_module(${module}
    $<TARGET_OBJECTS:onrobot_gripper_objects>
    $<TARGET_OBJECTS:onrobot_${object_library}_objects>
    src/bindings/${module}_binding.cpp
  )
  target_link_libraries(${module} PRIVATE Modbus)
  install(TARGETS ${module} DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")
endfunction()

onrobot_add_python_module(TwoFG twofg)
onrobot_add_python_module(ThreeFG threefg)
onrobot_add_python_module(VGC10 vgc10)
