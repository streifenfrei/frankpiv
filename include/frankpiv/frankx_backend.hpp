#include "frankpiv/general_backend.hpp"
#include "frankx/frankx.hpp"

using namespace frankx;

namespace frankpiv::backend {
    class FrankxBackend {
    private:
        Robot robot;
        Affine clipPose(Affine& pose);

    public:
        float initial_eef_ppoint_distance;
        float tool_length;
        float max_angle;
        float roll_boundaries[2];
        float z_translation_boundaries[2];
        bool clip_to_boundaries;
        bool move_directly;

        FrankxBackend(
                float initial_eef_ppoint_distance,
                float tool_length,
                float max_angle,
                float roll_boundaries[2],
                float z_translation_boundaries[2],
                bool clip_to_boundaries,
                bool move_directly
        );

        FrankxBackend(std::string config_file);
        void start();
        void stop();
        void moveToPoint(std::array<float, 3> point, float roll, Affine frame);
        void movePYRZ(std::array<float, 4>, bool degrees);
        void movePYRZRelative(std::array<float, 4>, bool degrees);
    };
}


