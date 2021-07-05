#include "frankpiv/moveit_backend.hpp"
#include "frankpiv/utilities.hpp"

using namespace Eigen;
using namespace frankpiv::util;

namespace frankpiv::backend {
    MoveitBackend::MoveitBackend(const YAML::Node &config, const std::string& node_name, bool async_motion) : GeneralBackend(
            config, node_name) {
        YAML::Node moveit_config = config["moveit"];
        this->eef_step = get_config_value<float>(moveit_config, "eef_step")[0];
        this->jump_threshold = get_config_value<float>(moveit_config, "jump_threshold")[0];
        this->robot = nullptr;
    }

    void MoveitBackend::initialize() {
        this->robot = new moveit::planning_interface::MoveGroupInterface(this->getRobotName());
    }

    void MoveitBackend::finish() {
        this->robot = nullptr;
    }

    Eigen::Affine3d MoveitBackend::currentPose() {
        return to_affine(this->robot->getCurrentPose());
    }

    bool MoveitBackend::moveRobotCartesian(const Eigen::Affine3d &target_pose) {
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(to_pose_msg(target_pose));
        moveit_msgs::RobotTrajectory trajectory;
        double fraction = this->robot->computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, trajectory);
        if (fraction != 1.) {
            return false;
        }
        return this->robot->execute(trajectory) == 1;
    }
}