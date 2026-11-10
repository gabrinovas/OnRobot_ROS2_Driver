#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "onrobot_driver/vgc10/VGC10.hpp"

namespace py = pybind11;

PYBIND11_MODULE(VGC10, m) {
    py::class_<onrobot_driver::VGC10>(m, "VGC10")
        .def(py::init<const std::string&, const std::string&, int, int>(),
             py::arg("type"), py::arg("ip"), py::arg("port") = 502, py::arg("device_address") = 65)
        .def(py::init<const std::string&, const std::string&, int>(),
             py::arg("type"), py::arg("device"), py::arg("device_address") = 65)
        
        // Channel A methods
        .def("grip_channel_a", &onrobot_driver::VGC10::gripChannelA, py::arg("vacuum_pct") = 60)
        .def("release_channel_a", &onrobot_driver::VGC10::releaseChannelA)
        .def("idle_channel_a", &onrobot_driver::VGC10::idleChannelA)
        .def("get_vacuum_channel_a", &onrobot_driver::VGC10::getVacuumChannelA)
        .def("get_raw_vacuum_channel_a", &onrobot_driver::VGC10::getRawVacuumChannelA)

        // Channel B methods
        .def("grip_channel_b", &onrobot_driver::VGC10::gripChannelB, py::arg("vacuum_pct") = 60)
        .def("release_channel_b", &onrobot_driver::VGC10::releaseChannelB)
        .def("idle_channel_b", &onrobot_driver::VGC10::idleChannelB)
        .def("get_vacuum_channel_b", &onrobot_driver::VGC10::getVacuumChannelB)
        .def("get_raw_vacuum_channel_b", &onrobot_driver::VGC10::getRawVacuumChannelB)

        // Coordinated / All channels
        .def("grip_all", &onrobot_driver::VGC10::gripAll, py::arg("vacuum_pct") = 60)
        .def("release_all", &onrobot_driver::VGC10::releaseAll)
        .def("idle_all", &onrobot_driver::VGC10::idleAll)
        .def("stop", &onrobot_driver::VGC10::stop)
        .def("move_gripper", &onrobot_driver::VGC10::moveGripper, py::arg("width_val"))

        // Common getters/setters
        .def("get_width", &onrobot_driver::VGC10::getWidth)
        .def("get_force", &onrobot_driver::VGC10::getForce)
        .def("get_status", &onrobot_driver::VGC10::getStatus)
        .def("set_current_limit", &onrobot_driver::VGC10::setCurrentLimit, py::arg("current_ma"))
        .def("reset_tool_power", &onrobot_driver::VGC10::resetToolPower, py::arg("compute_box_address") = 63)

        // Diagnostics and connection health
        .def("is_connected", &onrobot_driver::VGC10::isConnected)
        .def("reconnect", &onrobot_driver::VGC10::reconnect, py::arg("timeout_ms") = 1000)
        .def("get_reconnect_count", &onrobot_driver::VGC10::getReconnectCount)
        .def("get_total_requests", &onrobot_driver::VGC10::getTotalRequests)
        .def("get_failed_requests", &onrobot_driver::VGC10::getFailedRequests)
        .def("get_last_roundtrip_ms", &onrobot_driver::VGC10::getLastRoundtripMs)
        .def("read_both_vacuums", [](onrobot_driver::VGC10 &self) {
            float vac_a = 0.0f, vac_b = 0.0f;
            bool ok = self.readBothVacuums(vac_a, vac_b);
            return py::make_tuple(ok, vac_a, vac_b);
        });
}
