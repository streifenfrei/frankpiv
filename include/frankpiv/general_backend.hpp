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

    class GeneralBackend : ROSNode {
    private:
        double initial_eef_ppoint_distance;
        double tool_length;
        double max_angle;
        Eigen::Vector2d roll_boundaries;
        Eigen::Vector2d z_translation_boundaries;
        bool clip_to_boundaries;
        bool move_directly;
        Eigen::Affine3d reference_frame;
        ros::NodeHandle node_handle;
        ros::AsyncSpinner spinner{1};
        std::string robot_name;
#ifdef VISUALIZATION
        bool visualize;
        rviz_visual_tools::RvizVisualToolsPtr visual_tools;

        void reset_markers();

#endif

        Eigen::Vector4d static poseToPYRZ(const Eigen::Affine3d &pose);

        bool clipPose(Eigen::Affine3d &pose, double *out_angle = nullptr);

        void fixCurrentPose();

    protected:
        [[nodiscard]] const ros::NodeHandle &getNodeHandle() const;

        [[nodiscard]] const std::string &getRobotName() const;

        virtual void initialize() = 0;

        virtual void finish() = 0;

        virtual Eigen::Affine3d currentPose() = 0;

        virtual bool moveRobotCartesian(const Eigen::Affine3d &target_pose) = 0;

    public:
        [[nodiscard]] double getInitialEefPpointDistance() const;

        [[nodiscard]] double getToolLength() const;

        [[nodiscard]] double getMaxAngle() const;

        [[nodiscard]] const Eigen::Vector2d &getRollBoundaries() const;

        [[nodiscard]] const Eigen::Vector2d &getZTranslationBoundaries() const;

        [[nodiscard]] bool isClipToBoundaries() const;

        [[nodiscard]] bool isMoveDirectly() const;

        [[nodiscard]] const Eigen::Affine3d &getReferenceFrame() const;

        [[nodiscard]] Eigen::Vector4d getCurrentPyrz();

        [[nodiscard]] double getPivotError();

#ifdef VISUALIZATION

        [[nodiscard]] bool isVisualize() const;

        void setVisualize(bool visualize_);

#endif

        void setInitialEefPpointDistance(double initialEefPpointDistance);

        void setToolLength(double toolLength);

        void setMaxAngle(double maxAngle);

        void setRollBoundaries(const Eigen::Vector2d &rollBoundaries);

        void setZTranslationBoundaries(const Eigen::Vector2d &zTranslationBoundaries);

        void setClipToBoundaries(bool clipToBoundaries);

        void setMoveDirectly(bool moveDirectly);

        void setReferenceFrame(const Eigen::Affine3d &referenceFrame);

        explicit GeneralBackend(const YAML::Node &config, const std::string &node_name = "pivot_controller");

        virtual ~GeneralBackend();

        virtual void start();

        virtual void stop();

        virtual bool moveToPoint(const Eigen::Vector3d &point, double roll, const Eigen::Affine3d *frame = nullptr);

        virtual bool movePYRZ(const Eigen::Vector4d &pyrz, bool degrees = false);

        virtual bool movePYRZRelative(const Eigen::Vector4d &pyrz, bool degrees = false);
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