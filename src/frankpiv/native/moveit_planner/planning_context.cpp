#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "frankpiv/moveit_planner/planning_context.hpp"
#include "frankpiv/moveit_planner/validity_checker.hpp"

using namespace Eigen;

namespace frankpiv::moveit_planner {
    PivotPlanningContext::PivotPlanningContext(const std::string &name, const std::string &group, const robot_model::RobotModelConstPtr &robot_model) :
        PlanningContext(name, group),
        robot_model(robot_model),
        joint_model_group(robot_model->getJointModelGroup(group)),
        simple_setup(std::make_shared<og::SimpleSetup>(initStateSpace())) {
        this->simple_setup->setStateValidityChecker(std::make_shared<PivotStateValidityChecker>(this->simple_setup->getSpaceInformation(), this->robot_model, this->joint_model_group->getName(), planning_scene_));
    }

    std::shared_ptr<ob::RealVectorStateSpace> PivotPlanningContext::initStateSpace() const{
        unsigned int dof = this->joint_model_group->getVariableCount();
        auto state_space = std::make_shared<ob::RealVectorStateSpace>(dof);
        ob::RealVectorBounds bounds(dof);
        bounds.setLow(-2 * M_PI);
        bounds.setHigh(2 * M_PI);
        state_space->setBounds(bounds);
        return state_space;
    }

    void PivotPlanningContext::clear() {}

    bool PivotPlanningContext::solve(planning_interface::MotionPlanResponse &res) {
        this->simple_setup->setPlanner(this->selectPlanner(request_.planner_id));
        unsigned int dof = this->joint_model_group->getVariableCount();
        std::vector<double> goal_joint_positions(dof);
        for (size_t i = 0; i < dof; i++) {
            goal_joint_positions[i] = request_.goal_constraints[0].joint_constraints[i].position;
        }
        ob::ScopedState<ob::RealVectorStateSpace> start_state(this->simple_setup->getStateSpace());
        ob::ScopedState<ob::RealVectorStateSpace> goal_state(this->simple_setup->getStateSpace());
        start_state = request_.start_state.joint_state.position;
        goal_state = goal_joint_positions;
        this->simple_setup->setStartAndGoalStates(start_state, goal_state);
        bool success = this->simple_setup->solve() == ob::PlannerStatus::EXACT_SOLUTION;
        if (success) {
            this->simple_setup->simplifySolution();
            auto path = this->simple_setup->getSolutionPath();
            path.interpolate();
            res.trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(this->robot_model, this->joint_model_group->getName());
            for (std::size_t path_i = 0; path_i < path.getStateCount(); path_i++) {
                ob::RealVectorStateSpace::StateType& state_ompl = *path.getState(path_i)->as<ob::RealVectorStateSpace::StateType>();
                auto state = std::make_shared<moveit::core::RobotState>(this->robot_model);
                size_t joint_i = 0;

                for (const moveit::core::JointModel* joint_model : res.trajectory_->getGroup()->getActiveJointModels()) {
                    state->setVariablePosition(joint_model->getFirstVariableIndex(), state_ompl[joint_i++]);
                }
                res.trajectory_->addSuffixWayPoint(state, 0.0);
            }
            res.planning_time_ = this->simple_setup->getLastPlanComputationTime();
        }
        return success;
    }

    bool PivotPlanningContext::solve(planning_interface::MotionPlanDetailedResponse &res) { return false; }

    bool PivotPlanningContext::terminate() { return true; }

    void PivotPlanningContext::setPivotFrame(std::shared_ptr<frankpiv::PivotFrame> pivot_frame) {
        this->pivot_frame = pivot_frame;
        this->simple_setup->setStateValidityChecker(std::make_shared<PivotStateValidityChecker>(this->simple_setup->getSpaceInformation(), this->robot_model, this->joint_model_group->getName(), planning_scene_));
    }
    ob::PlannerPtr PivotPlanningContext::selectPlanner(const std::string& planner_id) const {
        auto space_information = this->simple_setup->getSpaceInformation();
        if (planner_id == "RRTConnect")
        {
            return std::make_shared<og::RRTConnect>(space_information);
        }
        else if (planner_id == "RRTstar")
        {
            return std::make_shared<og::RRTstar>(space_information);
        }
        else if (planner_id == "PRM")
        {
            return std::make_shared<og::PRM>(space_information);
        }
        else if (planner_id == "PRMstar")
        {
            return std::make_shared<og::PRMstar>(space_information);
        }
        else
        {
            ROS_ERROR_STREAM("Unknown planner id: " << planner_id);
            return nullptr;
        }
    }
}
