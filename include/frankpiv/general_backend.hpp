#include "Eigen/Geometry"
#include "ros/ros.h"

#include "yaml-cpp/yaml.h"

using namespace Eigen;

namespace frankpiv::backend {
    class GeneralBackend {
    private:
        Vector3d static poseToPYZ(const Affine3d &pose);

        bool visualize;
        ros::Publisher *marker_publisher;

        bool clipPose(Affine3d &pose);

        void initRosNode();

        void shutdownRosNode();

        void publishMarker(const Affine3d &pose, int id = 0, int type = AXIS_MARKER);

        void deleteMarker(int id = -1);

    protected:
        Affine3d *reference_frame;
        Vector4d *current_pyrz;
        std::string node_name;
        ros::NodeHandle node_handle;
        bool ros_node_initialized;

        virtual void initialize();

        virtual void finish();

        virtual Affine3d currentPose();

        virtual void moveRobotCartesian(const Affine3d &target_pose);

    public:
        static const int AXIS_MARKER = 5;
        static const int POINT_MARKER = 2;
        double initial_eef_ppoint_distance;
        double tool_length;
        double max_angle;
        Vector2d roll_boundaries;
        Vector2d z_translation_boundaries;
        bool clip_to_boundaries;
        bool move_directly;

        explicit GeneralBackend(const YAML::Node &config, std::string node_name = "pivot_controller");

        void start();

        void stop();

        void moveToPoint(const Vector3d &point, double roll, const Affine3d *frame = nullptr);

        void movePYRZ(const Vector4d &pyrz, bool degrees = false);

        void movePYRZRelative(const Vector4d &pyrz, bool degrees = false);
    };

    class UnattainablePoseException : public std::runtime_error {
        std::string message;
    public:
        UnattainablePoseException(const std::string &message) : runtime_error(message) {
            this->message = message;
        }

        const char *what() {
            return message.c_str();
        }
    };
}
