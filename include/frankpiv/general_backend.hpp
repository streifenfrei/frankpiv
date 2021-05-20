#ifndef FRANKPIV_GENERAL_BACKEND
#define FRANKPIV_GENERAL_BACKEND

#include "Eigen/Geometry"
#include "ros/ros.h"

#include "yaml-cpp/yaml.h"

namespace frankpiv::backend {
    class GeneralBackend {
    private:
        Eigen::Vector3d static poseToPYZ(const Eigen::Affine3d &pose);

        bool visualize;
        ros::Publisher *marker_publisher;

        bool clipPose(Eigen::Affine3d &pose, double *out_angle = nullptr);

        void initRosNode();

        void shutdownRosNode();

        void publishMarker(const Eigen::Affine3d &pose, int id = 0, int type = AXIS_MARKER);

        void deleteMarker(int id = -1);

    protected:
        Eigen::Affine3d *reference_frame;
        Eigen::Vector4d *current_pyrz;
        std::string node_name;
        ros::NodeHandle *node_handle;
        bool ros_node_initialized;

        virtual void initialize() = 0;

        virtual void finish() = 0;

        virtual Eigen::Affine3d currentPose() = 0;

        virtual void moveRobotCartesian(const Eigen::Affine3d &target_pose) = 0;

    public:
        static const int AXIS_MARKER = 5;
        static const int POINT_MARKER = 2;
        double initial_eef_ppoint_distance;
        double tool_length;
        double max_angle;
        Eigen::Vector2d roll_boundaries;
        Eigen::Vector2d z_translation_boundaries;
        bool clip_to_boundaries;
        bool move_directly;

        explicit GeneralBackend(const YAML::Node &config, std::string node_name = "pivot_controller");

        void start();

        void stop();

        void moveToPoint(const Eigen::Vector3d &point, double roll, const Eigen::Affine3d *frame = nullptr);

        void movePYRZ(const Eigen::Vector4d &pyrz, bool degrees = false);

        void movePYRZRelative(const Eigen::Vector4d &pyrz, bool degrees = false);
    };

    class UnattainablePoseException : public std::runtime_error {
        std::string message;
        Eigen::Vector2d *boundaries;
        double *value;
    public:
        explicit UnattainablePoseException(const std::string &message, Eigen::Vector2d *boundaries = nullptr,
                                           double *value = nullptr) : runtime_error(message) {
            this->message = message;
            this->boundaries = boundaries;
            this->value = value;
        }

        const char *what() {
            std::stringstream what;
            what << this->message;
            if (this->boundaries) {
                what << "\nboundaries: " << this->boundaries;
            }
            if (this->value) {
                what << "\nvalue: " << this->value;
            }
            char what_char = *what.str().c_str();
            const char *what_char_pointer = &what_char;
            return what_char_pointer;
        }
    };
}

#endif //FRANKPIV_GENERAL_BACKEND