#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "onrobot_driver/rg/RG.hpp"

namespace py = pybind11;

PYBIND11_MODULE(RG, m) {
    py::class_<RG>(m, "RG")
        .def(py::init<const std::string&, const std::string&, int, int>(),
             py::arg("type"), py::arg("ip"), py::arg("port"), py::arg("device_address"))
        .def(py::init<const std::string&, const std::string&, int>(),
             py::arg("type"), py::arg("device"), py::arg("device_address"))
        .def("getWidth", &RG::getWidth)
        .def("getWidthWithOffset", &RG::getWidthWithOffset)
        .def("getFingertipOffset", &RG::getFingertipOffset)
        .def("getStatus", &RG::getStatus)
        .def("getStatusAndPrint", &RG::getStatusAndPrint)
        .def("setTargetWidth", &RG::setTargetWidth)
        .def("setTargetForce", &RG::setTargetForce)
        .def("setFingertipOffset", &RG::setFingertipOffset)
        .def("closeGripper", &RG::closeGripper)
        .def("openGripper", &RG::openGripper)
        .def("moveGripper", &RG::moveGripper)
        .def("getMinWidth", &RG::getMinWidth)
        .def("getMaxWidth", &RG::getMaxWidth);
}