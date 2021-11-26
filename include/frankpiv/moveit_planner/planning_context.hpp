#ifndef PLANNING_CONTEXT
#define PLANNING_CONTEXT

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <ompl/base/Planner.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "frankpiv/pivot_frame.hpp"
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace frankpiv::moveit_planner {
    MOVEIT_CLASS_FORWARD(PivotPlanningContext);

    // TODO maybe decouple from moveit's planning_interface::PlanningContext
    class PivotPlanningContext : public planning_interface::PlanningContext {
    public:
        PivotPlanningContext(const std::string &name, const moveit::core::RobotModelConstPtr &robot_model, const std::string &group_name, std::shared_ptr<frankpiv::PivotFrame> pivot_frame);

        ~PivotPlanningContext() override = default;

        void clear() override;

        bool solve(planning_interface::MotionPlanResponse &res) override;

        bool solve(planning_interface::MotionPlanDetailedResponse &res) override;

        bool terminate() override;

        void preSample(double angle_resolution, double z_translation_resolution, int thread_count);

        [[nodiscard]] std::shared_ptr<frankpiv::PivotFrame> pivot_frame() const { return pivot_frame_; }
        void pivot_frame(std::shared_ptr<frankpiv::PivotFrame> pivot_frame);

    private:
        moveit::core::RobotModelConstPtr robot_model;
        const std::string &group_name;
        std::shared_ptr <frankpiv::PivotFrame> pivot_frame_;
        std::shared_ptr <og::SimpleSetup> simple_setup;
        ob::PlannerData planner_data;

        std::shared_ptr <ob::RealVectorStateSpace> initStateSpace() const;

        ob::PlannerPtr selectPlanner(const std::string &planner_id) const;
    };
}

#endif  // PLANNING_CONTEXT