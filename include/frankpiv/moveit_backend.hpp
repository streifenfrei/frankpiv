#ifndef FRANKPIV_MOVEIT_BACKEND
#define FRANKPIV_MOVEIT_BACKEND

#include "moveit/move_group_interface/move_group_interface.h"

#include "frankpiv/general_backend.hpp"

namespace frankpiv::backend {
    class MoveitBackend : public GeneralBackend {
    private:
        static constexpr int SPINLOCK_WAIT = 1000;
        float eef_step;
        float jump_threshold;
        bool async_motion;
        moveit::planning_interface::MoveGroupInterface *robot;
        std::list<boost::thread::id> threads_list;
        boost::mutex threads_list_lock;
        boost::atomic<bool> terminating;

        bool movePYRZInternal(const Eigen::Vector4d &pyrz, bool degrees);

    protected:
        void initialize() override;

        void finish() override;

        Eigen::Affine3d currentPose() override;

        bool moveRobotCartesian(const Eigen::Affine3d &target_pose) override;

    public:

        int SHUTDOWN_TIMEOUT = 5000;

        explicit MoveitBackend(const YAML::Node &config, const std::string& node_name = "pivot_controller", bool async_motion = false);

        bool movePYRZ(const Eigen::Vector4d &pyrz, bool degrees = false) override;

    };
}

#endif //FRANKPIV_MOVEIT_BACKEND