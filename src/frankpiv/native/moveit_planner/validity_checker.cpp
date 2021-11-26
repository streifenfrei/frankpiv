#include <moveit/planning_scene/planning_scene.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "frankpiv/moveit_planner/validity_checker.hpp"
#include "frankpiv/utilities.hpp"

using namespace frankpiv::util;

namespace frankpiv::moveit_planner {
    PivotStateValidityChecker::PivotStateValidityChecker(const ob::SpaceInformationPtr &si, const robot_model::RobotModelConstPtr &rm, const std::string &group_name, const planning_scene::PlanningSceneConstPtr &ps, std::shared_ptr <frankpiv::PivotFrame> pivot_frame) :
            ob::StateValidityChecker(si),
            robot_model(rm),
            group_name(group_name),
            planning_scene(ps),
            pivot_frame(pivot_frame) {}

    bool PivotStateValidityChecker::isColliding(const moveit::core::RobotState &state) const {
        collision_detection::CollisionRequest request;
        request.group_name = this->group_name;
        collision_detection::CollisionResult result;
        this->planning_scene->checkCollision(request, result, state);
        return !result.collision;
    }

    bool PivotStateValidityChecker::isPivotState(const moveit::core::RobotState &state) const {
        if (this->pivot_frame) {
            auto eef_pose = state.getGlobalLinkTransform("panda_link8");
            return this->pivot_frame->isValid(eef_pose);
        }
        return true;
    }

    bool PivotStateValidityChecker::isValid(const ob::State *state) const {
        auto moveit_state = toMoveitState(*state->as<ob::RealVectorStateSpace::StateType>(), this->robot_model, this->group_name);
        return this->isPivotState(moveit_state) && this->isColliding(moveit_state);
    }
}
