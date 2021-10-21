#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include "frankpiv/moveit_backend.hpp"
#include "frankpiv/utilities.hpp"

using namespace Eigen;
using namespace frankpiv::util;

namespace frankpiv::backend {
    MoveitBackend::MoveitBackend(const YAML::Node &config, const std::string &node_name, bool async_motion) :
            GeneralBackend(config, node_name),
            eef_step(getConfigValue<float>(config["moveit"], "eef_step")[0]),
            jump_threshold(getConfigValue<float>(config["moveit"], "jump_threshold")[0]),
            planner_instance(std::make_shared<frankpiv::moveit_planner::PivotPlannerManager>()) {}

    void MoveitBackend::initialize() {
        this->robot = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->robot_name_);
        this->planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
        if (!this->planner_instance->initialize(this->robot->getRobotModel(), "/move_group"))
            ROS_FATAL_STREAM("Could not initialize planner instance");
        this->planner_instance->setPivotFrame(this->pivot_frame);
    }

    void MoveitBackend::finish() {
        this->planning_scene_monitor.reset();
        this->robot.reset();
    }

    Eigen::Affine3d MoveitBackend::currentPose() {
        return toAffine(this->robot->getCurrentPose());
    }

    bool MoveitBackend::moveRobotCartesian(Eigen::Affine3d target_pose) {
        const robot_state::JointModelGroup *joint_model_group = this->robot->getCurrentState()->getJointModelGroup(this->robot_name_);
        // prepare request
        planning_interface::MotionPlanRequest request;
        request.group_name = this->robot_name_;
        const robot_state::RobotState start_state = *this->robot->getCurrentState();
        moveit::core::robotStateToRobotStateMsg(start_state, request.start_state);
        robot_state::RobotState target_state(this->robot->getRobotModel());
        if (!target_state.setFromIK(joint_model_group, toPoseMsg(target_pose))) {
            ROS_ERROR_STREAM("Motion failed while calculating target joint values from inverse kinematics");
            return false;
        }
        auto target_joints = kinematic_constraints::constructGoalConstraints(target_state, joint_model_group, 0.001);
        request.goal_constraints.push_back(target_joints);
        // TODO make parameter
        request.planner_id = "RRTConnect";
        request.allowed_planning_time = 200.0;
        // solve
        planning_interface::MotionPlanResponse response;
        auto context = this->planner_instance->getPlanningContext(this->planning_scene_monitor->getPlanningScene(), request, response.error_code_);
        if (!context->solve(response)) {
            ROS_ERROR_STREAM("Motion failed during planning");
            return false;
        }
        // time parameterization
        robot_trajectory::RobotTrajectory &trajectory = *response.trajectory_;
        trajectory_processing::IterativeParabolicTimeParameterization isp;
        //trajectory_processing::IterativeSplineParameterization isp;
        isp.computeTimeStamps(trajectory, 0.1, 0.15);
        moveit_msgs::RobotTrajectory trajectory_msg;
        trajectory.getRobotTrajectoryMsg(trajectory_msg);
        // remove redundant first point ...
        if (trajectory_msg.joint_trajectory.points[0].time_from_start == trajectory_msg.joint_trajectory.points[1].time_from_start) {
            std::vector <trajectory_msgs::JointTrajectoryPoint> fixed_points;
            for (size_t i = 1; i < trajectory.getWayPointCount(); i++) {
                fixed_points.push_back(trajectory_msg.joint_trajectory.points[i]);
            }
            trajectory_msg.joint_trajectory.points = fixed_points;
        }
        auto error_code = this->robot->execute(trajectory_msg);
        if (error_code != 1) {
            ROS_ERROR_STREAM("Motion failed during execution: code " << error_code);
            return false;
        }
        return true;
    }
}