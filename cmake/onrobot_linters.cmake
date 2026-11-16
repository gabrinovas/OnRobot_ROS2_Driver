# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

# The linters of ament_lint_common, run on the files of this package only. The bundled Modbus library
# (include/onrobot_driver/Modbus) is a git submodule with a style of its own, so the directories are listed
# here instead of letting ament_lint_auto scan the whole package.
find_package(ament_cmake_copyright REQUIRED)
find_package(ament_cmake_cppcheck REQUIRED)
find_package(ament_cmake_cpplint REQUIRED)
find_package(ament_cmake_flake8 REQUIRED)
find_package(ament_cmake_lint_cmake REQUIRED)
find_package(ament_cmake_pep257 REQUIRED)
find_package(ament_cmake_uncrustify REQUIRED)
find_package(ament_cmake_xmllint REQUIRED)

set(ONROBOT_CPP_DIRECTORIES
  include/onrobot_driver/common
  include/onrobot_driver/hardware
  include/onrobot_driver/monitor
  include/onrobot_driver/threefg
  include/onrobot_driver/twofg
  include/onrobot_driver/vgc10
  src
  test
)
set(ONROBOT_PYTHON_DIRECTORIES examples launch onrobot_driver test)

ament_copyright(${ONROBOT_CPP_DIRECTORIES} ${ONROBOT_PYTHON_DIRECTORIES} cmake CMakeLists.txt)
ament_cppcheck(${ONROBOT_CPP_DIRECTORIES})
ament_cpplint(${ONROBOT_CPP_DIRECTORIES})
ament_uncrustify(${ONROBOT_CPP_DIRECTORIES})
ament_flake8(${ONROBOT_PYTHON_DIRECTORIES})
ament_pep257(${ONROBOT_PYTHON_DIRECTORIES})
ament_lint_cmake(cmake CMakeLists.txt)
ament_xmllint(
  package.xml twofg_hardware_interface.xml threefg_hardware_interface.xml vgc10_hardware_interface.xml)
