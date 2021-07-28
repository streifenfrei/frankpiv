#ifndef FRANKPIV_MOVEIT_BACKEND
#define FRANKPIV_MOVEIT_BACKEND

#include "moveit/move_group_interface/move_group_interface.h"
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "frankpiv/general_backend.hpp"

namespace frankpiv::backend {
    class MoveitBackend : public GeneralBackend {
    private:
        float eef_step;
        float jump_threshold;
        moveit::planning_interface::MoveGroupInterface *robot;
        planning_scene_monitor::PlanningSceneMonitor *planning_scene_monitor;
        planning_interface::PlannerManagerPtr planner_instance;

        void loadPlanningPlugin();

    protected:
        void initialize() override;

        void finish() override;

        Eigen::Affine3d currentPose() override;

        bool moveRobotCartesian(const Eigen::Affine3d &target_pose) override;

    public:
        explicit MoveitBackend(const YAML
        ::Node &config, const std::string& node_name = "pivot_controller", bool async_motion = false);

        // for debugging
        bool moveJointUnconstrained(const std::vector<double> joints);
    };
}

#endif //FRANKPIV_MOVEIT_BACKEND