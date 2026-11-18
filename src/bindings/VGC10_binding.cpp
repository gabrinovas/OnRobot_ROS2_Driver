// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "binding_helpers.hpp"
#include "onrobot_driver/vgc10/VGC10.hpp"

namespace py = pybind11;
using onrobot_driver::VGC10;

PYBIND11_MODULE(VGC10, m) {
  py::class_<VGC10> gripper(m, "VGC10");
  gripper
  .def(
    py::init(
      [](int device_address, uint8_t max_vacuum_percent, uint16_t max_current_ma,
      uint16_t initial_current_ma,
      float grip_threshold) {
        return new VGC10(
          device_address,
          onrobot_driver::VacuumSettings{max_vacuum_percent, max_current_ma, initial_current_ma,
            grip_threshold});
      }),
    py::arg("device_address"), py::arg("max_vacuum_percent"), py::arg("max_current_ma"),
    py::arg("initial_current_ma"), py::arg("grip_threshold"))
  .def("grip_channel_a", &VGC10::gripChannelA, py::arg("vacuum_pct"))
  .def("release_channel_a", &VGC10::releaseChannelA)
  .def("idle_channel_a", &VGC10::idleChannelA)
  .def("get_vacuum_channel_a", &VGC10::getVacuumChannelA)
  .def("get_raw_vacuum_channel_a", &VGC10::getRawVacuumChannelA)
  .def("grip_channel_b", &VGC10::gripChannelB, py::arg("vacuum_pct"))
  .def("release_channel_b", &VGC10::releaseChannelB)
  .def("idle_channel_b", &VGC10::idleChannelB)
  .def("get_vacuum_channel_b", &VGC10::getVacuumChannelB)
  .def("get_raw_vacuum_channel_b", &VGC10::getRawVacuumChannelB)
  .def("grip_all", &VGC10::gripAll, py::arg("vacuum_pct"))
  .def("release_all", &VGC10::releaseAll)
  .def("idle_all", &VGC10::idleAll)
  .def("stop", &VGC10::stop)
  .def("move_gripper", &VGC10::moveGripper, py::arg("width_val"))
  .def("get_width", &VGC10::getWidth)
  .def("get_force", &VGC10::getForce)
  .def("get_status", &VGC10::getStatus)
  .def("set_current_limit", &VGC10::setCurrentLimit, py::arg("current_ma"))
  .def("reset_tool_power", &VGC10::resetToolPower, py::arg("compute_box_address"))
  .def(
    "read_both_vacuums", [](VGC10 & self) {
      float vacuum_a = 0.0f;
      float vacuum_b = 0.0f;
      const bool ok = self.readBothVacuums(vacuum_a, vacuum_b);
      return py::make_tuple(ok, vacuum_a, vacuum_b);
    });
  onrobot_driver::bind_connection(gripper);
}
