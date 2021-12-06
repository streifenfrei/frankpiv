#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "frankpiv/controller.hpp"

namespace py = pybind11;
using namespace pybind11::literals;
using namespace frankpiv;

PYBIND11_MODULE(frankpyv, module) {
    py::class_<Controller>(module, "Controller")
            .def(py::init<std::string>())
            .def("start", py::overload_cast<std::optional<std::array<double, 6>>>(&Controller::start, py::const_), "reference_frame"_a = nullptr)
            .def("stop", &Controller::stop)
            .def("move_to_point", py::overload_cast<std::array<double, 3>, double, std::optional<std::array<double, 6>>>(&Controller::moveToPoint, py::const_), "point"_a, "roll"_a, "frame"_a = nullptr)
            .def("move_pyrz", py::overload_cast<std::array<double, 4>, bool>(&Controller::movePYRZ, py::const_), "pyrz"_a, "degrees"_a = false)
            .def("move_pyrz_relative", py::overload_cast<std::array<double, 4>, bool>(&Controller::movePYRZRelative, py::const_), "pyrz"_a, "degrees"_a = false)
            .def_property("initial_eef_ppoint_distance", py::overload_cast<>(&Controller::initial_eef_ppoint_distance, py::const_), py::overload_cast<double>(&Controller::initial_eef_ppoint_distance))
            .def_property("tool_length", py::overload_cast<>(&Controller::tool_length, py::const_), py::overload_cast<double>(&Controller::tool_length))
            .def_property("max_angle", py::overload_cast<>(&Controller::max_angle, py::const_), py::overload_cast<double>(&Controller::max_angle))
            .def_property("roll_boundaries", &Controller::roll_boundaries_as_array, py::overload_cast<std::array<double, 2>>(&Controller::roll_boundaries))
            .def_property("z_translation", &Controller::z_translation_boundaries_as_array, py::overload_cast<std::array<double, 2>>(&Controller::z_translation_boundaries))
            .def_property("clip_boundaries", py::overload_cast<>(&Controller::clip_to_boundaries, py::const_), py::overload_cast<bool>(&Controller::clip_to_boundaries))
            .def_property("move_directly", py::overload_cast<>(&Controller::move_directly, py::const_), py::overload_cast<bool>(&Controller::move_directly))
            .def_property("reference_frame", &Controller::reference_frame, py::overload_cast<std::array<double, 6>>(&Controller::reference_frame))
            .def_property_readonly("current_pyrz", &Controller::getCurrentPYRZAsArray);
}