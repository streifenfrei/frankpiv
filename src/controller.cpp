#include "yaml-cpp/yaml.h"

#include "frankpiv/controller.hpp"
#include "frankpiv/utilities.hpp"
#include "frankpiv/moveit_backend.hpp"
#ifdef FRANKR
#include "frankpiv/frankr_backend.hpp"
#endif
#ifdef FRANKX
#include "frankpiv/frankx_backend.hpp"
#endif



using namespace Eigen;
using namespace frankpiv::util;

namespace frankpiv {
    Controller::Controller(const std::string &config_file) {
        YAML::Node config;
        try {
            config = YAML::LoadFile(config_file);
        } catch (const YAML::BadFile &exception) {
            throw ConfigError("Config file not found or corrupt: " + config_file);
        }
        try {
            if (get_config_value<float>(config, "eef_ppoint_distance")[0] >
                get_config_value<float>(config, "tool_length")[0]) {
                throw ConfigError("'eef_ppoint_distance' has to be smaller than 'tool_length'");
            }
            config["max_angle"] = std::min(get_config_value<double>(config, "max_angle")[0], 90.);
            config["roll_boundaries"][0] = std::max(get_config_value<double>(config, "roll_boundaries")[0], -360.);
            config["roll_boundaries"][1] = std::min(get_config_value<double>(config, "roll_boundaries")[1], 360.);
            config["z_translation_boundaries"][0] = std::max(
                    get_config_value<double>(config, "z_translation_boundaries")[0],
                    get_config_value<double>(config, "eef_ppoint_distance")[0] -
                    get_config_value<double>(config, "tool_length")[0]);
            config["z_translation_boundaries"][1] = std::min(
                    get_config_value<double>(config, "z_translation_boundaries")[1],
                    get_config_value<double>(config, "eef_ppoint_distance")[0]);
            std::string backend_name = get_config_value<std::string>(config, "backend")[0];
            if (backend_name == "moveit") {
                this->backend = new backend::MoveitBackend(config);
            }
#ifdef FRANKR
                else if (backend_name == "frankr") {
                    this->backend = new backend::FrankrBackend(config);
                }
#endif
#ifdef FRANKX
            else if (backend_name == "frankx") {
                this->backend = new backend::FrankxBackend(config);
            }
#endif
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

    void Controller::start() const {
        (*this->backend).start();
    }

    void Controller::stop() const {
        (*this->backend).stop();
    }

    void Controller::moveToPoint(const Vector3d &point, double roll, const Affine3d *frame) const {
        (*this->backend).moveToPoint(point, roll, frame);
    }

    void Controller::movePYRZ(const Vector4d &pyrz, bool degrees) const {
        (*this->backend).movePYRZ(pyrz, degrees);
    }

    void Controller::movePYRZRelative(const Vector4d &pyrz, bool degrees) const {
        (*this->backend).movePYRZRelative(pyrz, degrees);
    }
}

