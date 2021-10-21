#ifndef FRANKPIV_GENERAL_BACKEND
#define FRANKPIV_GENERAL_BACKEND

#include <Eigen/Geometry>
#include <ros/ros.h>

#ifdef VISUALIZATION

#include <rviz_visual_tools/rviz_visual_tools.h>

#endif

#include "yaml-cpp/yaml.h"
#include "frankpiv/pivot_frame.hpp"

namespace frankpiv::backend {
    class ROSNode {
    public:
        explicit ROSNode(std::string node_name) {
            int argc = 0;
            ros::init(argc, nullptr, node_name);
        }

        ~ROSNode() {
            ros::shutdown();
        }
    };

    class GeneralBackend : ROSNode {
    private:
        double initial_eef_ppoint_distance_;
        double tool_length_;
        double max_angle_;
        Eigen::Vector2d roll_boundaries_;
        Eigen::Vector2d z_translation_boundaries_;
        bool clip_to_boundaries_;
        bool move_directly_;
        ros::NodeHandle node_handle_{};
        ros::AsyncSpinner spinner_{1};

#ifdef VISUALIZATION
        bool visualize_;
        rviz_visual_tools::RvizVisualToolsPtr visual_tools;

        void resetMarkers();
#endif

        void fixCurrentPose();

    protected:
        std::string robot_name_;
        std::shared_ptr <frankpiv::PivotFrame> pivot_frame;

        double error_tolerance_;

        [[nodiscard]] const ros::NodeHandle &node_handle() const { return node_handle_; }

        virtual void initialize() = 0;

        virtual void finish() = 0;

        virtual Eigen::Affine3d currentPose() = 0;

        virtual bool moveRobotCartesian(Eigen::Affine3d target_pose) = 0;

    public:
        [[nodiscard]] double initial_eef_ppoint_distance() const { return initial_eef_ppoint_distance_; }

        void initial_eef_ppoint_distance(double initial_eef_ppoint_distance);

        [[nodiscard]] double tool_length() const { return tool_length_; }

        void tool_length(double tool_length);

        [[nodiscard]] double error_tolerance() const { return error_tolerance_; }

        void error_tolerance(double error_tolerance);

        [[nodiscard]] double max_angle() const { return max_angle_; }

        void max_angle(double max_angle);

        [[nodiscard]] Eigen::Vector2d roll_boundaries() const { return roll_boundaries_; }

        void roll_boundaries(Eigen::Vector2d roll_boundaries);

        [[nodiscard]] Eigen::Vector2d z_translation_boundaries() const { return z_translation_boundaries_; }

        void z_translation_boundaries(Eigen::Vector2d z_translation_boundaries);

        [[nodiscard]] bool clip_to_boundaries() const { return clip_to_boundaries_; }

        void clip_to_boundaries(bool clip_to_boundaries) { clip_to_boundaries_ = clip_to_boundaries; }

        [[nodiscard]] bool move_directly() const { return move_directly_; }

        void move_directly(bool move_directly) { move_directly_ = move_directly; }

        [[nodiscard]] Eigen::Affine3d reference_frame() const { return pivot_frame->reference_frame(); }

        void reference_frame(Eigen::Affine3d reference_frame);

#ifdef VISUALIZATION
        [[nodiscard]] bool visualize() const { return visualize_; };
        void visualize(bool visualize);
#endif

        [[nodiscard]] Eigen::Vector4d getCurrentPYRZ();

        explicit GeneralBackend(const YAML::Node &config, std::string node_name = "pivot_controller");

        virtual ~GeneralBackend();

        virtual void start(Eigen::Affine3d *reference_frame_ = nullptr);

        virtual void stop();

        virtual bool moveToPoint(Eigen::Vector3d point, double roll, const Eigen::Affine3d *frame = nullptr);

        virtual bool movePYRZ(Eigen::Vector4d pyrz, bool degrees = false);

        virtual bool movePYRZRelative(Eigen::Vector4d pyrz, bool degrees = false);
    };
}

#endif //FRANKPIV_GENERAL_BACKEND