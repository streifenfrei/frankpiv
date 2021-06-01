#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "frankpiv/controller.hpp"

namespace py = pybind11;
using namespace pybind11::literals;
using namespace frankpiv;
using namespace Eigen;

PYBIND11_MODULE(frankpyv, module) {
    py::class_<Controller>(module, "Controller")
            .def(py::init<const std::string &>())
            .def("start", &Controller::start)
            .def("stop", &Controller::stop)
            .def("move_to_point", py::overload_cast<std::array<double, 3>, double, std::optional<std::array<double, 6>>>
                    (&Controller::moveToPoint, py::const_),
                 "point"_a, "roll"_a, "frame"_a = nullptr)
            .def("move_pyrz", py::overload_cast<std::array<double, 4>, bool>(&Controller::movePYRZ, py::const_),
                 "pyrz"_a, "degrees"_a = false)
            .def("move_pyrz_relative",
                 py::overload_cast<std::array<double, 4>, bool>(&Controller::movePYRZRelative, py::const_),
                 "pyrz"_a, "degrees"_a = false);
}