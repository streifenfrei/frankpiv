#ifndef FRANKPIV_GENERAL_BACKEND
#define FRANKPIV_GENERAL_BACKEND

#include "Eigen/Geometry"
#include "ros/ros.h"
#ifdef VISUALIZATION
#include <rviz_visual_tools/rviz_visual_tools.h>
#endif
#include "yaml-cpp/yaml.h"

namespace frankpiv::backend {
    class ROSNode {
    public:
        explicit ROSNode(const std::string &node_name) {
            int argc = 0;
            ros::init(argc, nullptr, node_name);
        }

        ~ROSNode() {
            ros::shutdown();
        }
    };

    class GeneralBackend : ROSNode{
    private:
#ifdef VISUALIZATION
        bool visualize;
        rviz_visual_tools::RvizVisualToolsPtr visual_tools;

        void reset_markers();
#endif
        Eigen::Vector3d static poseToPYZ(const Eigen::Affine3d &pose);

        bool clipPose(Eigen::Affine3d &pose, double *out_angle = nullptr);

    protected:
        Eigen::Affine3d reference_frame;
        Eigen::Vector4d current_pyrz;
        ros::NodeHandle node_handle;
        ros::AsyncSpinner spinner {1};
        std::string robot_name;

        virtual void initialize() = 0;

        virtual void finish() = 0;

        virtual Eigen::Affine3d currentPose() = 0;

        virtual void moveRobotCartesian(const Eigen::Affine3d &target_pose) = 0;

    public:
        double initial_eef_ppoint_distance;
        double tool_length;
        double max_angle;
        Eigen::Vector2d roll_boundaries;
        Eigen::Vector2d z_translation_boundaries;
        bool clip_to_boundaries;
        bool move_directly;

        explicit GeneralBackend(const YAML::Node &config, const std::string& node_name = "pivot_controller");

        virtual ~GeneralBackend();

        virtual void start();

        virtual void stop();

        virtual void moveToPoint(const Eigen::Vector3d &point, double roll, const Eigen::Affine3d *frame = nullptr);

        virtual void movePYRZ(const Eigen::Vector4d &pyrz, bool degrees = false);

        virtual void movePYRZRelative(const Eigen::Vector4d &pyrz, bool degrees = false);
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