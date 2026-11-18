// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef BINDINGS__BINDING_HELPERS_HPP_
#define BINDINGS__BINDING_HELPERS_HPP_

#include <pybind11/pybind11.h>

#include <string>

#include "onrobot_driver/common/OnRobotGripperBase.hpp"

namespace onrobot_driver
{

/// Methods every gripper class offers to Python: opening the connection and looking at its health.
/// No argument has a value of its own: onrobot_driver.create_gripper() fills them from the
/// configuration.
template<typename Gripper>
void bind_connection(pybind11::class_<Gripper> & gripper)
{
  namespace py = pybind11;
  gripper
  .def(
    "connect_tcp",
    [](Gripper & self, const std::string & ip_address, int port, int timeout_ms,
    int connect_retry_ms) {
      ConnectionSettings settings{};
      settings.type = "tcp";
      settings.ip_address = ip_address;
      settings.port = port;
      settings.timeout_ms = timeout_ms;
      settings.connect_retry_ms = connect_retry_ms;
      self.connect(settings);
    },
    py::arg("ip_address"), py::arg("port"), py::arg("timeout_ms"), py::arg("connect_retry_ms"))
  .def(
    "connect_serial",
    [](Gripper & self, const std::string & device, int serial_timeout_ms, int connect_retry_ms) {
      ConnectionSettings settings{};
      settings.type = "serial";
      settings.device = device;
      settings.serial_timeout_ms = serial_timeout_ms;
      settings.connect_retry_ms = connect_retry_ms;
      self.connect(settings);
    },
    py::arg("device"), py::arg("serial_timeout_ms"), py::arg("connect_retry_ms"))
  .def("is_connected", &Gripper::isConnected)
  .def("reconnect", &Gripper::reconnect)
  .def("get_reconnect_count", &Gripper::getReconnectCount)
  .def("get_total_requests", &Gripper::getTotalRequests)
  .def("get_failed_requests", &Gripper::getFailedRequests)
  .def("get_last_roundtrip_ms", &Gripper::getLastRoundtripMs);
}

/// Constructor of a finger gripper from plain keyword arguments.
template<typename Gripper>
auto finger_constructor()
{
  namespace py = pybind11;
  return py::init(
    [](int device_address, float min_width, float max_width, float max_force, float default_force,
    float default_speed) {
      return new Gripper(
        device_address,
        FingerSettings{min_width, max_width, max_force, default_force, default_speed});
    });
}

}  // namespace onrobot_driver

#endif  // BINDINGS__BINDING_HELPERS_HPP_
