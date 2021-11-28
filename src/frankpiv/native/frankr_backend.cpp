#include <utility>

#include "frankpiv/frankr_backend.hpp"
#include "frankpiv/moveit_backend.hpp"
#include "frankpiv/utilities.hpp"

using namespace Eigen;
using namespace frankpiv::util;

namespace frankpiv::backend {
    FrankrBackend::FrankrBackend(const YAML::Node &config, std::string node_name) :
            GeneralBackend(config, node_name),
            dynamic_rel(getConfigValue<double>(config["frankr"], "dynamic_rel")[0]) {}

    FrankrBackend::~FrankrBackend() {
        this->stop();
    }

    void FrankrBackend::initialize() {
        this->robot = std::make_unique<Robot>(MoveitBackend::MOVE_GROUP_NAME, this->dynamic_rel);
        this->motion_data = std::make_unique<MotionData>();
    }

    void FrankrBackend::finish() {
        this->robot.release();
        this->motion_data.release();
    }

    Affine3d FrankrBackend::currentPose() {
        return this->robot->currentPose(Affine()).data;
    }

    bool FrankrBackend::moveRobotPYRZ(Vector4d pyrz) {
        return this->robot->moveCartesian(Affine(), this->pivot_frame->getPose(std::move(pyrz)), *this->motion_data);
    }
}