#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "onrobot_driver/threefg/ThreeFG.hpp"

namespace py = pybind11;

PYBIND11_MODULE(ThreeFG, m) {
    py::class_<ThreeFG>(m, "ThreeFG")
        .def(py::init<const std::string&, int, int>(),
             py::arg("ip"), py::arg("port"), py::arg("device_address") = 65)
        .def(py::init<const std::string&, int>(),
             py::arg("device"), py::arg("device_address") = 65)
        .def("getWidth", &ThreeFG::getWidth)
        .def("getCurrentDiameter", &ThreeFG::getCurrentDiameter)
        .def("getDiameterWithOffset", &ThreeFG::getDiameterWithOffset)
        .def("getAppliedForce", &ThreeFG::getAppliedForce)
        .def("getStatus", &ThreeFG::getStatus)
        .def("getStatusRaw", &ThreeFG::getStatusRaw)
        .def("getDetailedStatus", [](ThreeFG& self) {
            auto s = self.getDetailedStatus();
            py::dict result;
            result["busy"] = s.busy;
            result["grip_detected"] = s.grip_detected;
            result["force_grip_detected"] = s.force_grip_detected;
            result["calibration_ok"] = s.calibration_ok;
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
        .def("setFingertipOffset", &ThreeFG::setFingertipOffset);
}