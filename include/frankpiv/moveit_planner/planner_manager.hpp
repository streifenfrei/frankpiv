#ifndef PLANNER_MANAGER
#define PLANNER_MANAGER

#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MoveItErrorCodes.h>

#include "frankpiv/moveit_planner/planning_context.hpp"

namespace frankpiv::moveit_planner {
    MOVEIT_CLASS_FORWARD(PivotPlannerManager);

    class PivotPlannerManager : public planning_interface::PlannerManager {
    public:
        PivotPlannerManager();

        bool initialize(const robot_model::RobotModelConstPtr &model, const std::string &ns) override;

        bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const override;

        std::string getDescription() const override;

        void getPlanningAlgorithms(std::vector <std::string> &algs) const override;

        planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr &planning_scene,
                                                                  const moveit_msgs::MotionPlanRequest &req,
                                                                  moveit_msgs::MoveItErrorCodes &error_code) const override;

        void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap &pcs) override;

        void setPivotFrame(std::shared_ptr <frankpiv::PivotFrame> pivot_frame);

    private:
        std::map <std::string, PivotPlanningContextPtr> planning_contexts_;
    };
}

#endif //PLANNER_MANAGER
