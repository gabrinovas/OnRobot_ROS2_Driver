// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "binding_helpers.hpp"
#include "onrobot_driver/threefg/ThreeFG.hpp"

namespace py = pybind11;

PYBIND11_MODULE(ThreeFG, m) {
  py::class_<ThreeFG> gripper(m, "ThreeFG");
  gripper
  .def(
    onrobot_driver::finger_constructor<ThreeFG>(), py::arg("device_address"), py::arg("min_width"),
    py::arg("max_width"), py::arg("max_force"), py::arg("default_force"), py::arg("default_speed"))
  .def("getWidth", &ThreeFG::getWidth)
  .def("getCurrentDiameter", &ThreeFG::getCurrentDiameter)
  .def("getDiameterWithOffset", &ThreeFG::getDiameterWithOffset)
  .def("getAppliedForce", &ThreeFG::getAppliedForce)
  .def("getStatus", &ThreeFG::getStatus)
  .def("getStatusRaw", &ThreeFG::getStatusRaw)
  .def(
    "getDetailedStatus", [](ThreeFG & self) {
      const auto status = self.getDetailedStatus();
      py::dict result;
      result["busy"] = status.busy;
      result["grip_detected"] = status.grip_detected;
      result["force_grip_detected"] = status.force_grip_detected;
      result["calibration_ok"] = status.calibration_ok;
      return result;
    })
  .def("setTargetWidth", &ThreeFG::setTargetWidth)
  .def("setTargetForce", &ThreeFG::setTargetForce)
  .def("setTargetSpeed", &ThreeFG::setTargetSpeed)
  .def("gripInternal", &ThreeFG::gripInternal)
  .def("gripExternal", &ThreeFG::gripExternal)
  .def("stop", &ThreeFG::stop)
  .def("moveGripper", &ThreeFG::moveGripper)
  .def("getMinWidth", &ThreeFG::getMinWidth)
  .def("getMaxWidth", &ThreeFG::getMaxWidth)
  .def("getMinDiameter", &ThreeFG::getMinDiameter)
  .def("getMaxDiameter", &ThreeFG::getMaxDiameter)
  .def("getFingerLength", &ThreeFG::getFingerLength)
  .def("getFingerPosition", &ThreeFG::getFingerPosition)
  .def("getFingertipOffset", &ThreeFG::getFingertipOffset)
  .def("setGripType", &ThreeFG::setGripType)
  .def("setFingerLength", &ThreeFG::setFingerLength)
  .def("setFingerPosition", &ThreeFG::setFingerPosition)
  .def("setFingertipOffset", &ThreeFG::setFingertipOffset)
  .def("resetToolPower", &ThreeFG::resetToolPower, py::arg("compute_box_address"));
  onrobot_driver::bind_connection(gripper);
}
