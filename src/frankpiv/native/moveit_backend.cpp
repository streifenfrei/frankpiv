#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "frankpiv/moveit_backend.hpp"
#include "frankpiv/utilities.hpp"
#include "frankpiv/pivot_planner.hpp"

using namespace Eigen;
using namespace frankpiv::util;

namespace frankpiv::backend {
    MoveitBackend::MoveitBackend(const YAML::Node &config, const std::string &node_name, bool async_motion) :
            GeneralBackend(config, node_name),
            max_threads(getConfigValue<unsigned int>(config["moveit"], "planner_threads")[0]),
            total_timeout(getConfigValue<double>(config["moveit"], "total_timeout")[0]),
            ik_timeout(getConfigValue<double>(config["moveit"], "ik_timeout")[0]),
            planner_max_joint_distance(getConfigValue<double>(config["moveit"], "planner_max_joint_distance")[0]),
            planner_pyrz_step_size(getConfigValue<double>(config["moveit"], "planner_pyrz_step_size")[0]) {}

    void MoveitBackend::initialize() {
        this->robot = std::make_shared<moveit::planning_interface::MoveGroupInterface>(MoveitBackend::MOVE_GROUP_NAME);
        // couldn't find a way to get the base link from moveit objects... if that's not possible TODO make it a parameter
        this->planner = std::make_shared <frankpiv::pivot_planner::PivotPlanner>(this->pivot_frame, "panda_link0", this->robot->getEndEffectorLink(), this->urdf_param, this->ik_timeout, this->max_threads);
        this->planner->max_joint_state_distance(this->planner_max_joint_distance);
        this->planner->max_pyrz_step_size(this->planner_pyrz_step_size);
    }

    void MoveitBackend::finish() {
        this->planner.reset();
        this->robot.reset();
    }

    Eigen::Affine3d MoveitBackend::currentPose() {
        return toAffine(this->robot->getCurrentPose());
    }

    bool MoveitBackend::moveRobotPYRZ(Eigen::Vector4d pyrz) {
        // prepare request
        frankpiv::pivot_planner::PivotPlanningRequest request;
        request.start_state = Map<VectorXd>(&this->robot->getCurrentJointValues()[0], this->planner->dof());
        request.goal_pyrz = std::move(pyrz);
        request.allowed_planning_time = this->total_timeout;
        // solve
        frankpiv::pivot_planner::PivotPlanningResponse response = this->planner->solve(request);
        if (response.status != frankpiv::pivot_planner::Status::Success) {
            ROS_ERROR_STREAM("Motion failed during planning (Status " << response.status << ")");
            return false;
        }
        // convert to moveit trajectory
        auto robot_model = this->robot->getRobotModel();
        robot_trajectory::RobotTrajectory trajectory(robot_model, MoveitBackend::MOVE_GROUP_NAME);
        auto joint_model_group = this->robot->getCurrentState()->getJointModelGroup(MoveitBackend::MOVE_GROUP_NAME);
        for (auto &state : response.trajectory) {
            moveit::core::RobotState state_moveit(robot_model);
            state_moveit.setJointGroupPositions(joint_model_group, state.data());
            trajectory.addSuffixWayPoint(state_moveit, 0);
        }
        // time parameterization
        trajectory_processing::IterativeParabolicTimeParameterization isp;
        //trajectory_processing::IterativeSplineParameterization isp;
        isp.computeTimeStamps(trajectory, 0.1, 0.15);
        moveit_msgs::RobotTrajectory trajectory_msg;
        trajectory.getRobotTrajectoryMsg(trajectory_msg);
        auto error_code = this->robot->execute(trajectory_msg);
        if (error_code != 1) {
            ROS_ERROR_STREAM("Motion failed during execution: code " << error_code);
            return false;
        }
        return true;
    }
}