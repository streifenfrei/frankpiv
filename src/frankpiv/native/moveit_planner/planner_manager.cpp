#include <memory>
#include <string>
#include <vector>

#include "frankpiv/moveit_planner/planner_manager.hpp"


namespace frankpiv::moveit_planner {
    constexpr char LOGNAME[] = "pivot_planner_manager";

    PivotPlannerManager::PivotPlannerManager() : PlannerManager() {}

    bool PivotPlannerManager::initialize(const robot_model::RobotModelConstPtr &model, const std::string &ns) {
        for (const std::string &group : model->getJointModelGroupNames()) {
            planning_contexts_[group] = std::make_shared<PivotPlanningContext>("chomp_planning_context", group, model);
        }
        return true;
    }

    bool PivotPlannerManager::canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const {
        return true;
    }

    std::string PivotPlannerManager::getDescription() const {
        return "PivotOMPL";
    }

    void PivotPlannerManager::getPlanningAlgorithms(std::vector<std::string> &algs) const {
        algs.clear();
    }

    planning_interface::PlanningContextPtr PivotPlannerManager::getPlanningContext(
            const planning_scene::PlanningSceneConstPtr &planning_scene,
            const moveit_msgs::MotionPlanRequest &req, moveit_msgs::MoveItErrorCodes &error_code) const {
        if (req.group_name.empty()) {
            ROS_ERROR_NAMED(LOGNAME, "No group specified to plan for");
            error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
            return planning_interface::PlanningContextPtr();
        }

        if (!planning_scene) {
            ROS_ERROR_NAMED(LOGNAME, "No planning scene supplied as input");
            error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
            return planning_interface::PlanningContextPtr();
        }

        auto context = planning_contexts_.at(req.group_name);
        context->setPlanningScene(planning_scene);
        context->setMotionPlanRequest(req);

        return context;
    }

    void PivotPlannerManager::setPlannerConfigurations(const planning_interface::PlannerConfigurationMap &pcs) {}

    void PivotPlannerManager::setPivotFrame(std::shared_ptr<frankpiv::PivotFrame> pivot_frame) {
        for (std::pair<std::string, PivotPlanningContextPtr> context : planning_contexts_) {
            context.second->setPivotFrame(pivot_frame);
        }
    }
}

