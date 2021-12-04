#ifndef FRANKPIV_MOVEIT_BACKEND
#define FRANKPIV_MOVEIT_BACKEND

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "frankpiv/general_backend.hpp"
#include "frankpiv/pivot_planner.hpp"

namespace frankpiv::backend {
    class MoveitBackend : public GeneralBackend {
    private:
        moveit::planning_interface::MoveGroupInterfacePtr robot;
        std::shared_ptr <frankpiv::pivot_planner::PivotPlanner> planner;
        double total_timeout;
        double ik_timeout;
        unsigned int max_threads;

        double planner_max_joint_distance;
        double planner_pyrz_step_size;

    protected:
        // TODO make parameter
        inline static const std::string MOVE_GROUP_NAME = "panda_arm";

        void initialize() override;

        void finish() override;

        Eigen::Affine3d currentPose() override;

        bool moveRobotPYRZ(Eigen::Vector4d pyrz) override;

    public:
        explicit MoveitBackend(const YAML::Node &config, const std::string &node_name = "pivot_controller", bool async_motion = false);
    };
}

#endif //FRANKPIV_MOVEIT_BACKEND