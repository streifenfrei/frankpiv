#ifndef PLANNING_CONTEXT
#define PLANNING_CONTEXT

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <ompl/base/Planner.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "frankpiv/pivot_frame.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace frankpiv::moveit_planner {
    MOVEIT_CLASS_FORWARD(PivotPlanningContext);

    class PivotPlanningContext : public planning_interface::PlanningContext {
    public:
        PivotPlanningContext(const std::string &name, const std::string &group, const moveit::core::RobotModelConstPtr &robot_model);

        ~PivotPlanningContext() override = default;

        void clear() override;

        bool solve(planning_interface::MotionPlanResponse &res) override;

        bool solve(planning_interface::MotionPlanDetailedResponse &res) override;

        bool terminate() override;

        void setPivotFrame(std::shared_ptr <frankpiv::PivotFrame> pivot_frame);

    private:
        moveit::core::RobotModelConstPtr robot_model;
        const moveit::core::JointModelGroup *joint_model_group; // why is this a raw pointer in moveit?
        std::shared_ptr <frankpiv::PivotFrame> pivot_frame;
        std::shared_ptr <og::SimpleSetup> simple_setup;

        std::shared_ptr <ob::RealVectorStateSpace> initStateSpace() const;

        ob::PlannerPtr selectPlanner(const std::string &planner_id) const;
    };
}

#endif  // PLANNING_CONTEXT