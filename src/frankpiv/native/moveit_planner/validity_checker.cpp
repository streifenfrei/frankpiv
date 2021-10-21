#include <moveit/planning_scene/planning_scene.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "frankpiv/moveit_planner/validity_checker.hpp"


namespace frankpiv::moveit_planner {
    PivotStateValidityChecker::PivotStateValidityChecker(const ob::SpaceInformationPtr &si, const robot_model::RobotModelConstPtr &rm, const std::string &group,
                                                         const planning_scene::PlanningSceneConstPtr &ps) : ob::StateValidityChecker(si), robot_model(rm), group_name(group), planning_scene(ps) {}

    bool PivotStateValidityChecker::isColliding(const ob::State *state) const {
        double *cstate = state->as<ob::RealVectorStateSpace::StateType>()->values;
        collision_detection::CollisionRequest request;
        request.group_name = this->group_name;
        collision_detection::CollisionResult result;
        auto moveit_state = new moveit::core::RobotState(this->robot_model);
        moveit_state->setJointGroupPositions(this->group_name, cstate);
        this->planning_scene->checkCollision(request, result, *moveit_state);
        return !result.collision;
    }

    bool PivotStateValidityChecker::isPivotState(const ob::State *state) const {
        return true;
    }

    bool PivotStateValidityChecker::isValid(const ob::State *state) const {
        return isPivotState(state) && isColliding(state);
    }
}
