#include "frankpiv/frankx_backend.hpp"
#include "frankpiv/utilities.hpp"

using namespace Eigen;
using namespace frankpiv::util;

namespace frankpiv::backend {
    FrankxBackend::FrankxBackend(const YAML::Node &config, const std::string& node_name) : GeneralBackend(config,
                                                                                                         node_name) {
        YAML::Node frankx_config = config["frankx"];
        this->fci_ip = get_config_value<std::string>(frankx_config, "fci_ip")[0];
        this->dynamic_rel = get_config_value<double>(frankx_config, "dynamic_rel")[0];
        this->robot = nullptr;
    }

    FrankxBackend::~FrankxBackend() {
        this->stop();
    }

    void FrankxBackend::initialize() {
        this->robot = new frankx::Robot(this->fci_ip, this->dynamic_rel);
    }

    void FrankxBackend::finish() {
        this->robot = nullptr;
    }

    Affine3d FrankxBackend::currentPose() {
        return this->robot->currentPose(frankx::Affine()).data;
    }

    bool FrankxBackend::moveRobotCartesian(const Affine3d &target_pose) {
        frankx::LinearMotion motion = frankx::LinearMotion(movex::WaypointMotion::Affine(target_pose));
        return this->robot->move(motion);
    }
}