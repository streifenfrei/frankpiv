#include "frankr/robot.hpp"
#include "frankr/motion.hpp"

#include "frankpiv/general_backend.hpp"

namespace frankpiv::backend {
    class FrankrBackend : public GeneralBackend {
    private:
        std::string robot_name;
        double dynamic_rel;
        Robot *robot;
        MotionData *motion_data;
    protected:
        void initialize() override;

        void finish() override;

        Eigen::Affine3d currentPose() override;

        void moveRobotCartesian(const Eigen::Affine3d &target_pose) override;

    public:
        explicit FrankrBackend(const YAML::Node &config, std::string node_name = "pivot_controller");
    };
}


