#include <utility>

#include "frankpiv/frankx_backend.hpp"
#include "frankpiv/utilities.hpp"

using namespace Eigen;

namespace frankpiv::backend {
    FrankxBackend::FrankxBackend(const YAML::Node &config, std::string node_name) : GeneralBackend(config, std::move(
            node_name)) {
        YAML::Node frankx_config = config["frankx"];
        this->fci_ip = frankx_config["fci_ip"].as<std::string>();
        this->dynamic_rel = frankx_config["dynamic_rel"].as<double>();
        this->robot = nullptr;
    }

    void FrankxBackend::initialize() {
        frankx::Robot _robot = frankx::Robot(this->fci_ip, this->dynamic_rel);
        this->robot = &_robot;
    }

    void FrankxBackend::finish() {
        this->robot = nullptr;
    }

    Eigen::Affine3d FrankxBackend::currentPose() {
        return this->robot->currentPose(frankx::Affine()).data;
    }

    void FrankxBackend::moveRobotCartesian(const Eigen::Affine3d &target_pose) {
        frankx::LinearMotion motion = frankx::LinearMotion(movex::WaypointMotion::Affine(target_pose));
        this->robot->move(motion);
    }
}