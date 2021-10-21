#ifndef VALIDITY_CHECKER
#define VALIDITY_CHECKER

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/collision_detection/collision_common.h>
#include <ompl/base/StateValidityChecker.h>


namespace frankpiv::moveit_planner {
    namespace ob = ompl::base;

    class PivotStateValidityChecker : public ob::StateValidityChecker {
    public:
        PivotStateValidityChecker(const ob::SpaceInformationPtr &si, const robot_model::RobotModelConstPtr &rm, const std::string &group, const planning_scene::PlanningSceneConstPtr &ps);

        bool isColliding(const ob::State *state) const;

        bool isPivotState(const ob::State *state) const;

        bool isValid(const ob::State *state) const override;

    private:
        const planning_scene::PlanningSceneConstPtr &planning_scene;
        const robot_model::RobotModelConstPtr &robot_model;
        const std::string &group_name;
    };
}
#endif //VALIDITY_CHECKER
