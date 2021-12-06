#ifndef NDEBUG
#define ROS_LOG_LEVEL Debug
#else
#define ROS_LOG_LEVEL Info
#endif

#include "yaml-cpp/yaml.h"

#include "frankpiv/controller.hpp"
#include "frankpiv/exceptions.hpp"
#include "frankpiv/moveit_backend.hpp"

#ifdef FRANKR
#include "frankpiv/frankr_backend.hpp"
#define LOAD_FRANKR else if (backend_name == "frankr") { this->backend = std::make_unique<backend::FrankrBackend>(config); }
#else
#define LOAD_FRANKR
#endif
#ifdef FRANKX
#include "frankpiv/frankx_backend.hpp"
#define LOAD_FRANKX else if (backend_name == "frankx") { this->backend = std::make_unique<backend::FrankxBackend>(config); }
#else
#define LOAD_FRANKX
#endif


using namespace Eigen;
using namespace frankpiv::exceptions;

namespace frankpiv {
    Controller::Controller(const std::string &config_file) {
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::ROS_LOG_LEVEL))
            ros::console::notifyLoggerLevelsChanged();
        YAML::Node config;
        try {
            config = YAML::LoadFile(config_file);
        } catch (const YAML::BadFile &exception) {
            throw ConfigError("Config file not found or corrupt: " + config_file);
        }
        try {
            if (getConfigValue<float>(config, "eef_ppoint_distance")[0] >
                getConfigValue<float>(config, "tool_length")[0]) {
                throw ConfigError("'eef_ppoint_distance' has to be smaller than 'tool_length'");
            }
            config["pivot_error_tolerance"] = std::max(getConfigValue<double>(config, "pivot_error_tolerance")[0], 0.);
            config["max_angle"] = std::min(getConfigValue<double>(config, "max_angle")[0], 90.);
            config["roll_boundaries"][0] = std::max(getConfigValue<double>(config, "roll_boundaries")[0], -360.);
            config["roll_boundaries"][1] = std::min(getConfigValue<double>(config, "roll_boundaries")[1], 360.);
            config["z_translation_boundaries"][0] = std::max(getConfigValue<double>(config, "z_translation_boundaries")[0],
                                                             getConfigValue<double>(config, "eef_ppoint_distance")[0] -
                                                             getConfigValue<double>(config, "tool_length")[0]);
            config["z_translation_boundaries"][1] = std::min(getConfigValue<double>(config, "z_translation_boundaries")[1],
                                                             getConfigValue<double>(config, "eef_ppoint_distance")[0]);
            std::string backend_name = getConfigValue<std::string>(config, "backend")[0];
            if (backend_name == "moveit") {
                this->backend = std::make_unique<backend::MoveitBackend>(config);
            }LOAD_FRANKR
                    LOAD_FRANKX
            else {
                throw ConfigError("No such backend: " + backend_name);
            }
        } catch (const YAML::BadConversion &exception) {
            throw ConfigError(exception.msg);
        }
    }

    Controller::~Controller() {
        this->stop();
    }

    void Controller::start(Affine3d *reference_frame) const {
        (*this->backend).start(reference_frame);
    }

    void Controller::start(const std::optional <std::array<double, 6>> reference_frame) const {
        Affine3d *frame_affine = nullptr;
        if (reference_frame.has_value()) {
            frame_affine = new Affine3d();
            *frame_affine = toAffine(*reference_frame);
        }
        (*this->backend).start(frame_affine);
    }

    void Controller::stop() const {
        (*this->backend).stop();
    }

    void Controller::moveToPoint(Vector3d point, double roll, const Affine3d *frame) const {
        (*this->backend).moveToPoint(point, roll, frame);
    }

    void Controller::moveToPoint(std::array<double, 3> point, double roll, std::optional <std::array<double, 6>> frame) const {
        const Vector3d point_vector = Vector3d(point[0], point[1], point[2]);
        Affine3d *frame_affine = nullptr;
        if (frame.has_value()) {
            frame_affine = new Affine3d();
            *frame_affine = toAffine(*frame);
        }
        this->moveToPoint(point_vector, roll, frame_affine);
    }

    void Controller::movePYRZ(Vector4d pyrz, bool degrees) const {
        (*this->backend).movePYRZ(pyrz, degrees);
    }

    void Controller::movePYRZ(std::array<double, 4> pyrz, bool degrees) const {
        Vector4d pyrz_vector = Vector4d(pyrz[0], pyrz[1], pyrz[2], pyrz[3]);
        this->movePYRZ(pyrz_vector, degrees);
    }

    void Controller::movePYRZRelative(Vector4d pyrz, bool degrees) const {
        (*this->backend).movePYRZRelative(pyrz, degrees);
    }

    void Controller::movePYRZRelative(std::array<double, 4> pyrz, bool degrees) const {
        Vector4d pyrz_vector = Vector4d(pyrz[0], pyrz[1], pyrz[2], pyrz[3]);
        this->movePYRZ(pyrz_vector, degrees);
    }

    Eigen::Vector4d Controller::getCurrentPYRZ() const {
        return (*this->backend).getCurrentPYRZ();
    }

    std::array<double, 4> Controller::getCurrentPYRZAsArray() const {
        Eigen::Vector4d pyrz = this->getCurrentPYRZ();
        return {pyrz[0], pyrz[1], pyrz[2], pyrz[3]};
    }
}

