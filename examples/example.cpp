#include <boost/thread.hpp>
#include <random>
#include "frankpiv/controller.hpp"

using namespace Eigen;

int main(int argc, char *argv[])
{
    assert(argc > 1);
    std::string config_file = argv[1];
    frankpiv::Controller controller(config_file);
    controller.start();
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_real_distribution<> dis(0., 1.);
    for (int i = 0; i < 10; ++i) {
        // move to random points using pitch, yaw, roll, z-translation values
        double pitch = (dis(gen)  - 0.5) * 0.7 * M_PI;
        double yaw = (dis(gen) - 0.5) * 0.7 * M_PI;
        double roll = (dis(gen) - 0.5) * 0.8 * M_PI;
        double z_translation = (dis(gen) - 0.5) * 0.05;
        controller.movePYRZ(Vector4d(pitch, yaw, roll, z_translation));
    }
    // move to some point in the pivot point frame (reference frame of the controller)
    controller.moveToPoint(Vector3d(0.15, -0.15, 0.1), M_PI);
    // move to some point in the global frame
    Affine3d reference_frame;
    reference_frame = Affine3d::Identity();
    controller.moveToPoint(Vector3d(0.2, 0.1, 0.15), M_PI, &reference_frame);
    controller.stop();
}