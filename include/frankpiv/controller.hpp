#ifndef FRANKPIV_CONTROLLER
#define FRANKPIV_CONTROLLER

#include "frankpiv/general_backend.hpp"

namespace frankpiv {
    class Controller {
    public:
        frankpiv::backend::GeneralBackend *backend;

        explicit Controller(const std::string &config_file);

        ~Controller();

        void start() const;

        void stop() const;

        void moveToPoint(const Eigen::Vector3d &point, double roll, const Eigen::Affine3d *frame = nullptr) const;

        void movePYRZ(const Eigen::Vector4d &pyrz, bool degrees = false) const;

        void movePYRZRelative(const Eigen::Vector4d &pyrz, bool degrees = false) const;
    };
}

#endif //FRANKPIV_CONTROLLER_H