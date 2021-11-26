#ifndef FRANKPIV_MOVEIT_BACKEND
#define FRANKPIV_MOVEIT_BACKEND

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "frankpiv/general_backend.hpp"
#include "frankpiv/moveit_planner/planner_manager.hpp"

namespace frankpiv::backend {
    class MoveitBackend : public GeneralBackend {
    private:
        moveit::planning_interface::MoveGroupInterfacePtr robot;
        planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
        frankpiv::moveit_planner::PivotPlanningContextPtr planning_context;

    protected:
        // TODO make parameter
        inline static const std::string MOVE_GROUP_NAME = "panda_arm";

        void initialize() override;

        void start(Eigen::Affine3d *reference_frame) override;

        void finish() override;

        Eigen::Affine3d currentPose() override;

        bool moveRobotCartesian(Eigen::Affine3d target_pose) override;

    public:
        explicit MoveitBackend(const YAML::Node &config, const std::string &node_name = "pivot_controller", bool async_motion = false);
    };
}

#endif //FRANKPIV_MOVEIT_BACKEND