#include <utility>

#include "frankpiv/frankr_backend.hpp"
#include "frankpiv/utilities.hpp"

namespace frankpiv::backend {
    FrankrBackend::FrankrBackend(const YAML::Node &config, std::string node_name) : GeneralBackend(config, std::move(
            node_name)) {
        YAML::Node frankx_config = config["frankr"];
        this->robot_name = frankx_config["robot_name"].as<std::string>();
        this->dynamic_rel = frankx_config["dynamic_rel"].as<double>();
        this->robot = nullptr;
        this->motion_data = nullptr;
    }

    void FrankrBackend::initialize() {
        Robot _robot = Robot(this->robot_name, this->dynamic_rel);
        this->robot = &_robot;
        MotionData _motion_data;
        this->motion_data = &_motion_data;
    }

    void FrankrBackend::finish() {
        this->robot = nullptr;
        this->motion_data = nullptr;
    }

    Eigen::Affine3d FrankrBackend::currentPose() {
        return this->robot->currentPose(Affine()).data;
    }

    void FrankrBackend::moveRobotCartesian(const Eigen::Affine3d &target_pose) {
        this->robot->moveCartesian(Affine(), target_pose, *this->motion_data);
    }
}