#include "pybind11/pybind11.h"
#include "pybind11/eigen.h"

#include "frankpiv/controller.hpp"

namespace py = pybind11;
using namespace frankpiv;

PYBIND11_MODULE(frankpyv, module) {
    py::class_<Controller>(module, "Controller")
        .def(py::init<const std::string &>())
        .def("start", &Controller::start)
        .def("stop", &Controller::stop)
        .def("move_to_point", &Controller::moveToPoint)
        .def("move_pyrz", &Controller::movePYRZ)
        .def("move_pyrz_relative", &Controller::movePYRZRelative);
}