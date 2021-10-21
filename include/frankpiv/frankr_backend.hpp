#ifndef FRANKPIV_FRANKR_BACKEND
#define FRANKPIV_FRANKR_BACKEND

#include "robot.hpp"
#include "motion.hpp"

#include "frankpiv/general_backend.hpp"

namespace frankpiv::backend {
    class FrankrBackend : public GeneralBackend {
    private:
        double dynamic_rel;
        std::unique_ptr <Robot> robot;
        std::unique_ptr <MotionData> motion_data;

    protected:
        void initialize() override;

        void finish() override;

        Eigen::Affine3d currentPose() override;

        bool moveRobotCartesian(Eigen::Affine3d target_pose) override;

    public:
        explicit FrankrBackend(const YAML::Node &config, std::string node_name = "pivot_controller");

        ~FrankrBackend() override;
    };
}

#endif //FRANKR_BACKEND