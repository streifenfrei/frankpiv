#include <pluginlib/class_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include "frankpiv/moveit_backend.hpp"
#include "frankpiv/utilities.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
using namespace Eigen;
using namespace frankpiv::util;

namespace frankpiv::backend {
    typedef boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> ClassLoaderSPtr;

    MoveitBackend::MoveitBackend(const YAML::Node &config, const std::string &node_name, bool async_motion)
            : GeneralBackend(
            config, node_name) {
        YAML::Node moveit_config = config["moveit"];
        this->eef_step = get_config_value<float>(moveit_config, "eef_step")[0];
        this->jump_threshold = get_config_value<float>(moveit_config, "jump_threshold")[0];
        this->robot = nullptr;
        this->planning_scene_monitor = nullptr;
        this->planner_instance = planning_interface::PlannerManagerPtr();
        this->loadPlanningPlugin();
    }

    void MoveitBackend::loadPlanningPlugin() {
        boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
        try {
            planner_plugin_loader.reset(
                    new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core",
                                                                                   "planning_interface::PlannerManager"));
        } catch (pluginlib::PluginlibException &ex) {
            ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
        }
        try {
            this->planner_instance.reset(planner_plugin_loader->createUnmanagedInstance("elion/ElionPlanner"));
        }
        catch (pluginlib::PluginlibException &ex) {
            ROS_ERROR_STREAM("Exception while loading planning plugin" << std::endl);
        }
    }

    void MoveitBackend::initialize() {
        this->robot = new moveit::planning_interface::MoveGroupInterface(this->getRobotName());
        this->planning_scene_monitor = new planning_scene_monitor::PlanningSceneMonitor("robot_description");
        if (!this->planner_instance->initialize(this->robot->getRobotModel(), "/move_group"))
            ROS_FATAL_STREAM("Could not initialize planner instance");
        this->moveJointUnconstrained(std::vector<double>{0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0});
    }

    void MoveitBackend::finish() {
        this->planning_scene_monitor = nullptr;
        this->robot = nullptr;
    }

    Eigen::Affine3d MoveitBackend::currentPose() {
        return to_affine(this->robot->getCurrentPose());
    }

    bool MoveitBackend::moveRobotCartesian(const Eigen::Affine3d &target_pose) {
        const robot_state::JointModelGroup* joint_model_group = this->robot->getCurrentState()->getJointModelGroup(this->robot_name);
        // prepare request
        planning_interface::MotionPlanRequest request;
        request.group_name = this->robot_name;
        const moveit::core::RobotState start_state= *this->robot->getCurrentState();
        moveit::core::robotStateToRobotStateMsg(start_state, request.start_state);
        robot_state::RobotState target_state(this->robot->getRobotModel());
        if (!target_state.setFromIK(joint_model_group, to_pose_msg(target_pose))) {
            return false;
        }
        moveit_msgs::Constraints joint_goal =
                kinematic_constraints::constructGoalConstraints(target_state, joint_model_group, 0.001);
        request.goal_constraints.clear();
        request.goal_constraints.push_back(joint_goal);
        request.planner_id = "RRTConnect";
        request.allowed_planning_time = 200.0;

        moveit_msgs::Constraints constraints;
        moveit_msgs::PositionConstraint position_constraint;
        geometry_msgs::Pose pivot_pose;
        Vector3d pivot_point = this->getReferenceFrame().translation();
        pivot_pose.position.x = pivot_point[0];
        pivot_pose.position.y = pivot_point[1];
        pivot_pose.position.z = pivot_point[2];
        position_constraint.constraint_region.primitive_poses.push_back(pivot_pose);
        constraints.position_constraints.push_back(position_constraint);
        //constraints.position_constraints.push_back(moveit_msgs::PositionConstraint());
        request.path_constraints = constraints;
        // solve
        planning_interface::MotionPlanResponse response;
        auto context = this->planner_instance->getPlanningContext(this->planning_scene_monitor->getPlanningScene(), request, response.error_code_);
        if (!context->solve(response)){
            return false;
        }
        // time parameterization
        robot_trajectory::RobotTrajectory& trajectory = *response.trajectory_;
        trajectory_processing::IterativeParabolicTimeParameterization isp;
        //trajectory_processing::IterativeSplineParameterization isp;
        isp.computeTimeStamps(trajectory, 0.1, 0.15);
        moveit_msgs::RobotTrajectory trajectory_msg;
        trajectory.getRobotTrajectoryMsg(trajectory_msg);
        // remove redundant first point ...
        std::vector<trajectory_msgs::JointTrajectoryPoint> fixed_points;
        for (int i = 1; i < trajectory.getWayPointCount(); i++) {
            fixed_points.push_back(trajectory_msg.joint_trajectory.points[i]);
        }
        trajectory_msg.joint_trajectory.points = fixed_points;
        return this->robot->execute(trajectory_msg) == 1;
    }

    bool MoveitBackend::moveJointUnconstrained(const std::vector<double> joints){
        const robot_state::JointModelGroup* joint_model_group = this->robot->getCurrentState()->getJointModelGroup(this->robot_name);
        // prepare request
        planning_interface::MotionPlanRequest request;
        request.group_name = this->robot_name;
        const moveit::core::RobotState start_state= *this->robot->getCurrentState();
        moveit::core::robotStateToRobotStateMsg(start_state, request.start_state);
        robot_state::RobotState target_state(this->robot->getRobotModel());
        target_state.setJointGroupPositions(joint_model_group, joints);
        moveit_msgs::Constraints joint_goal =
                kinematic_constraints::constructGoalConstraints(target_state, joint_model_group, 0.001);
        request.goal_constraints.clear();
        request.goal_constraints.push_back(joint_goal);
        request.planner_id = "RRTConnect";
        request.allowed_planning_time = 200.0;

        moveit_msgs::Constraints constraints;
        request.path_constraints = constraints;
        // solve
        planning_interface::MotionPlanResponse response;
        auto context = this->planner_instance->getPlanningContext(this->planning_scene_monitor->getPlanningScene(), request, response.error_code_);
        if (!context->solve(response)){
            return false;
        }
        // time parameterization
        robot_trajectory::RobotTrajectory& trajectory = *response.trajectory_;
        trajectory_processing::IterativeParabolicTimeParameterization isp;
        //trajectory_processing::IterativeSplineParameterization isp;
        isp.computeTimeStamps(trajectory, 0.1, 0.1);
        moveit_msgs::RobotTrajectory trajectory_msg;
        trajectory.getRobotTrajectoryMsg(trajectory_msg);
        // remove redundant first point ...
        std::vector<trajectory_msgs::JointTrajectoryPoint> fixed_points;
        for (int i = 1; i < trajectory.getWayPointCount(); i++) {
            fixed_points.push_back(trajectory_msg.joint_trajectory.points[i]);
        }
        trajectory_msg.joint_trajectory.points = fixed_points;
        return this->robot->execute(trajectory_msg) == 1;
    }
}