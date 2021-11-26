#ifndef VALIDITY_CHECKER
#define VALIDITY_CHECKER

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/collision_detection/collision_common.h>
#include <ompl/base/StateValidityChecker.h>

#include "frankpiv/pivot_frame.hpp"

namespace ob = ompl::base;

namespace frankpiv::moveit_planner {
    class PivotStateValidityChecker : public ob::StateValidityChecker {
    public:
        PivotStateValidityChecker(const ob::SpaceInformationPtr &si, const robot_model::RobotModelConstPtr &rm,const std::string &group_name, const planning_scene::PlanningSceneConstPtr &ps, std::shared_ptr<frankpiv::PivotFrame> pivot_frame);

        bool isColliding(const moveit::core::RobotState &state) const;

        bool isPivotState(const moveit::core::RobotState &state) const;

        bool isValid(const ob::State *state) const override;

    private:
        const planning_scene::PlanningSceneConstPtr &planning_scene;
        const robot_model::RobotModelConstPtr &robot_model;
        const std::string &group_name;
        std::shared_ptr<frankpiv::PivotFrame> pivot_frame;
    };
}
#endif //VALIDITY_CHECKER
