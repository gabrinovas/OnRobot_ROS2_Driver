// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "binding_helpers.hpp"
#include "onrobot_driver/twofg/TwoFG.hpp"

namespace py = pybind11;

PYBIND11_MODULE(TwoFG, m) {
  py::class_<TwoFG> gripper(m, "TwoFG");
  gripper
  .def(
    onrobot_driver::finger_constructor<TwoFG>(), py::arg("device_address"), py::arg("min_width"),
    py::arg("max_width"), py::arg("max_force"), py::arg("default_force"), py::arg("default_speed"))
  .def("getWidth", &TwoFG::getWidth)
  .def("getForce", &TwoFG::getForce)
  .def("getStatus", &TwoFG::getStatus)
  .def("getStatusRaw", &TwoFG::getStatusRaw)
  .def("setTargetWidth", &TwoFG::setTargetWidth)
  .def("setTargetForce", &TwoFG::setTargetForce)
  .def("setTargetSpeed", &TwoFG::setTargetSpeed)
  .def("gripExternal", &TwoFG::gripExternal)
  .def("gripInternal", &TwoFG::gripInternal)
  .def("stop", &TwoFG::stop)
  .def(
    "moveGripper", static_cast<void (TwoFG::*)(float, bool)>(&TwoFG::moveGripper),
    py::arg("width_val"),
    py::arg("external_grip"))
  .def("getMinWidth", &TwoFG::getMinWidth)
  .def("getMaxWidth", &TwoFG::getMaxWidth)
  .def("resetToolPower", &TwoFG::resetToolPower, py::arg("compute_box_address"));
  onrobot_driver::bind_connection(gripper);
}
