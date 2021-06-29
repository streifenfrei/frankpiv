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
            .def("start", py::overload_cast<std::optional<std::array<double, 6>>>(&Controller::start, py::const_),
                 "reference_frame"_a = nullptr)
            .def("stop", &Controller::stop)
            .def("move_to_point", py::overload_cast<std::array<double, 3>, double, std::optional<std::array<double, 6>>>
                         (&Controller::moveToPoint, py::const_),
                 "point"_a, "roll"_a, "frame"_a = nullptr)
            .def("move_pyrz", py::overload_cast<std::array<double, 4>, bool>(&Controller::movePYRZ, py::const_),
                 "pyrz"_a, "degrees"_a = false)
            .def("move_pyrz_relative",
                 py::overload_cast<std::array<double, 4>, bool>(&Controller::movePYRZRelative, py::const_),
                 "pyrz"_a, "degrees"_a = false)
            .def_property("initial_eef_ppoint_distance", &Controller::getInitialEefPpointDistance, &Controller::setInitialEefPpointDistance)
            .def_property("tool_length", &Controller::getToolLength, &Controller::setToolLength)
            .def_property("max_angle", &Controller::getMaxAngle, &Controller::setMaxAngle)
            .def_property("roll_boundaries", &Controller::getRollBoundariesAsArray, py::overload_cast<const std::array<double, 2>&>(&Controller::setRollBoundaries))
            .def_property("z_translation", &Controller::getZTranslationBoundariesAsArray, py::overload_cast<const std::array<double, 2>&>(&Controller::setZTranslationBoundaries))
            .def_property("clip_boundaries", &Controller::isClipToBoundaries, &Controller::setClipToBoundaries)
            .def_property("move_directly", &Controller::isMoveDirectly, &Controller::setMoveDirectly)
            .def_property("reference_frame", &Controller::getReferenceFrameAsArray, py::overload_cast<const std::array<double, 6>&>(&Controller::setReferenceFrame))
            .def_property_readonly("current_pyrz", &Controller::getCurrentPyrzAsArray)
            .def_property_readonly("pivot_error", &Controller::getPivotError);
}