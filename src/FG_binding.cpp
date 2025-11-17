// src/FG_binding.cpp
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "onrobot_driver/FG.hpp"

namespace py = pybind11;

PYBIND11_MODULE(FG, m) {
    py::class_<FG>(m, "FG")
        // Solo el constructor que YA existe
        .def(py::init<const std::string&, const std::string&, int>(),
             py::arg("type"), py::arg("ip"), py::arg("port"))

        // Métodos públicos
        .def("getCurrentDiameter", &FG::getCurrentDiameter)
        .def("getAppliedForce", &FG::getAppliedForce)
        .def("getDetailedStatus", [](FG& self) {
            auto s = self.getDetailedStatus();
            py::dict result;
            result["busy"] = s.busy;
            result["grip_detected"] = s.grip_detected;
            result["force_grip_detected"] = s.force_grip_detected;
            result["calibration_ok"] = s.calibration_ok;
            return result;
        })
        .def("setTargetWidth", &FG::setTargetWidth)
        .def("gripInternal", &FG::gripInternal)
        .def("gripExternal", &FG::gripExternal)
        .def("stop", &FG::stop)
        .def("moveGripper", &FG::moveGripper,
             py::arg("width_val"), py::arg("external_grip") = true);
}