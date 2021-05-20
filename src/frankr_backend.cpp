#include <utility>

#include "frankpiv/frankr_backend.hpp"
#include "frankpiv/utilities.hpp"

using namespace Eigen;
using namespace frankpiv::util;

namespace frankpiv::backend {
    FrankrBackend::FrankrBackend(const YAML::Node &config, std::string node_name) : GeneralBackend(config, std::move(
            node_name)) {
        YAML::Node frankx_config = config["frankr"];
        this->robot_name = get_config_value<std::string>(frankx_config, "robot_name")[0];
        this->dynamic_rel = get_config_value<double>(frankx_config, "dynamic_rel")[0];
        this->robot = nullptr;
        this->motion_data = nullptr;
    }

    FrankrBackend::~FrankrBackend() {
        this->stop();
    }

    void FrankrBackend::initialize() {
        this->robot = new Robot(this->robot_name, this->dynamic_rel);
        MotionData _motion_data;
        this->motion_data = &_motion_data;
    }

    void FrankrBackend::finish() {
        this->robot = nullptr;
        this->motion_data = nullptr;
    }

    Affine3d FrankrBackend::currentPose() {
        return this->robot->currentPose(Affine()).data;
    }

    void FrankrBackend::moveRobotCartesian(const Affine3d &target_pose) {
        this->robot->moveCartesian(Affine(), target_pose, *this->motion_data);
    }
}