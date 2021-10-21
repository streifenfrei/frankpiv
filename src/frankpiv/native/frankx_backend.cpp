#include "frankpiv/frankx_backend.hpp"
#include "frankpiv/utilities.hpp"

using namespace Eigen;
using namespace frankpiv::util;

namespace frankpiv::backend {
    FrankxBackend::FrankxBackend(const YAML::Node &config, std::string node_name) :
            GeneralBackend(config, node_name),
            fci_ip(getConfigValue<std::string>(config["frankx"], "fci_ip")[0]),
            dynamic_rel(getConfigValue<double>(config["frankx"], "dynamic_rel")[0]) {}

    FrankxBackend::~FrankxBackend() {
        this->stop();
    }

    void FrankxBackend::initialize() {
        this->robot = std::make_unique<frankx::Robot>(this->fci_ip, this->dynamic_rel);
    }

    void FrankxBackend::finish() {
        this->robot.release();
    }

    Affine3d FrankxBackend::currentPose() {
        return this->robot->currentPose(frankx::Affine()).data;
    }

    bool FrankxBackend::moveRobotCartesian(Affine3d target_pose) {
        auto motion = frankx::LinearMotion(movex::WaypointMotion::Affine(target_pose));
        return this->robot->move(motion);
    }
}