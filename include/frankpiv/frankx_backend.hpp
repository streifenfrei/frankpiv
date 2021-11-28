#ifndef FRANKPIV_FRANKX_BACKEND
#define FRANKPIV_FRANKX_BACKEND

#include "frankx/frankx.hpp"

#include "frankpiv/general_backend.hpp"

namespace frankpiv::backend {
    class FrankxBackend : public GeneralBackend {
    private:
        std::string fci_ip;
        double dynamic_rel;
        std::unique_ptr <frankx::Robot> robot;
    protected:
        void initialize() override;

        void finish() override;

        Eigen::Affine3d currentPose() override;

        bool moveRobotPYRZ(Eigen::Vector4d pyrz) override;

    public:
        explicit FrankxBackend(const YAML::Node &config, std::string node_name = "pivot_controller");

        ~FrankxBackend() override;
    };
}

#endif //FRANKPIV_FRANKX_BACKEND