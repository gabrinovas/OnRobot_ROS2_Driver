#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "onrobot_driver/twofg/TwoFG.hpp"

namespace py = pybind11;

PYBIND11_MODULE(TwoFG, m) {
    py::class_<TwoFG>(m, "TwoFG")
        .def(py::init<const std::string&, const std::string&, int, int>(),
             py::arg("type"), py::arg("ip"), py::arg("port"), py::arg("device_address"))
        .def(py::init<const std::string&, const std::string&, int>(),
             py::arg("type"), py::arg("device"), py::arg("device_address"))
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
        .def("moveGripper", static_cast<void (TwoFG::*)(float, bool)>(&TwoFG::moveGripper),
             py::arg("width_val"), py::arg("external_grip") = true)
        .def("getMinWidth", &TwoFG::getMinWidth)
        .def("getMaxWidth", &TwoFG::getMaxWidth)
        .def("resetToolPower", &TwoFG::resetToolPower,
             py::arg("compute_box_address") = 63)
        // Diagnostics and connection health
        .def("is_connected", &TwoFG::isConnected)
        .def("reconnect", &TwoFG::reconnect, py::arg("timeout_ms") = 1000)
        .def("get_reconnect_count", &TwoFG::getReconnectCount)
        .def("get_total_requests", &TwoFG::getTotalRequests)
        .def("get_failed_requests", &TwoFG::getFailedRequests)
        .def("get_last_roundtrip_ms", &TwoFG::getLastRoundtripMs);
}