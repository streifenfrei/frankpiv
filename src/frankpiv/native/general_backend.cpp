#include <unsupported/Eigen/EulerAngles>
#include <ros/ros.h>

#include "frankpiv/general_backend.hpp"
#include "frankpiv/utilities.hpp"
#include "frankpiv/exceptions.hpp"

using Euler = Eigen::EulerAngles<double, Eigen::EulerSystemXYZ>;

using namespace Eigen;
using namespace frankpiv::util;
using namespace frankpiv::exceptions;

namespace frankpiv::backend {
    // ====================================================================================================================================================
    // Maybe replace config based constructor with real one and move config initialization to wrapper class (frankpiv::Controller) or some factory function
    // ====================================================================================================================================================
    GeneralBackend::GeneralBackend(const YAML::Node &config, std::string node_name) :
            ROSNode(node_name),
            initial_eef_ppoint_distance_(getConfigValue<double>(config, "eef_ppoint_distance")[0]),
            tool_length_(getConfigValue<double>(config, "tool_length")[0]),
            error_tolerance_(getConfigValue<double>(config, "pivot_error_tolerance")[0]),
            max_angle_(rad(getConfigValue<double>(config, "max_angle")[0])),
            roll_boundaries_(Vector2d(rad(getConfigValue<double>(config, "roll_boundaries")[0]), rad(getConfigValue<double>(config, "roll_boundaries")[1]))),
            z_translation_boundaries_(Vector2d(getConfigValue<double>(config, "z_translation_boundaries")[0], getConfigValue<double>(config, "z_translation_boundaries")[1])),
            clip_to_boundaries_(getConfigValue<bool>(config, "clip_to_boundaries")[0]),
            move_directly_(getConfigValue<bool>(config, "move_directly")[0]),
            robot_name_(getConfigValue<std::string>(config, "robot_name")[0]),
            pivot_frame(std::make_shared<frankpiv::PivotFrame>(Affine3d()))
#ifdef VISUALIZATION
    ,visualize_(getConfigValue<bool>(config, "visualize")[0])
#endif
    {
        this->spinner_.start();
#ifdef VISUALIZATION
        std::string topic = getConfigValue<std::string>(config, "marker_topic")[0];
        this->visual_tools.reset(new rviz_visual_tools::RvizVisualTools("world", topic, this->node_handle_));
#endif
    }

    GeneralBackend::~GeneralBackend() {
        this->stop();
        this->spinner_.stop();
        this->node_handle_.shutdown();
    }

    void GeneralBackend::start(Affine3d *reference_frame) {
        this->initialize();
        if (reference_frame) {
            this->reference_frame(*reference_frame);
        } else {
            this->reference_frame(this->currentPose() * Translation3d(0, 0, this->initial_eef_ppoint_distance_));
        }
#ifdef VISUALIZATION
        if (this->visualize_) {
            this->resetMarkers();
        }
#endif
    }

    void GeneralBackend::stop() {
#ifdef VISUALIZATION
        if (this->visualize_) {
            this->visual_tools->deleteAllMarkers();
        }
#endif
        this->finish();
    }

    bool GeneralBackend::moveToPoint(Vector3d point, double roll, const Affine3d *frame) {
        auto pyrz = this->pivot_frame->getPYRZ(point, roll, frame);
        pyrz(3) = pyrz(3) - this->tool_length_ + this->initial_eef_ppoint_distance_;
        return this->movePYRZ(pyrz);
    }

    bool GeneralBackend::movePYRZ(Vector4d pyrz, bool degrees) {
        // prepare input
        if (degrees) {
            pyrz(0) = rad(pyrz(0));
            pyrz(1) = rad(pyrz(1));
            pyrz(2) = rad(pyrz(2));
        }
        if (!this->clip_to_boundaries_) {
            if (pyrz(2) < this->roll_boundaries_(0) || pyrz(2) > this->roll_boundaries_(1)) {
                throw UnattainablePoseException("Roll value is outside of specified boundaries", &this->roll_boundaries_, &pyrz(2));
            }
            if (pyrz(3) < this->z_translation_boundaries_(0) || pyrz(3) > this->z_translation_boundaries_(1)) {
                throw UnattainablePoseException("Z-translation value is outside of specified boundaries", &this->z_translation_boundaries_, &pyrz(3));
            }
            double angle = this->pivot_frame->getAngle(pyrz);
            if (angle < -this->max_angle_ || angle > this->max_angle_) {
                Vector2d angle_boundaries = Vector2d(-this->max_angle_, this->max_angle_);
                throw UnattainablePoseException("PY angle is outside of specified boundaries", &angle_boundaries, &angle);
            }
        }
        pyrz(2) = clip(pyrz(2), this->roll_boundaries_);
        pyrz(3) = clip(pyrz(3), this->z_translation_boundaries_) - this->initial_eef_ppoint_distance_;
        this->pivot_frame->clipAngle(pyrz, this->max_angle_);
        auto target_pose = this->pivot_frame->getPose(pyrz);
        // move the robot
#ifdef VISUALIZATION
        if (this->visualize_) {
            this->visual_tools->publishAxis(toPoseMsg(target_pose));
            this->visual_tools->publishSphere(toPoseMsg(target_pose * Translation3d(0, 0, this->tool_length_)));
            this->visual_tools->trigger();
        }
#endif
        if (!this->move_directly_) {
            auto current_pyrz = this->getCurrentPYRZ();
            double current_pitch = current_pyrz(0);
            double current_yaw = current_pyrz(1);
            double current_roll = current_pyrz(2);
            double current_z_translation = current_pyrz(3);
            double current_eef_ppoint_distance = this->initial_eef_ppoint_distance_ - current_z_translation;
            double new_eef_ppoint_distance = -pyrz(3);
            // if relative z-translation is negative do it before pitch and yaw
            auto intermediate_pyrz = new_eef_ppoint_distance > current_eef_ppoint_distance ?
                                     Vector4d(current_pitch, current_yaw, pyrz(2), pyrz(3)) :
                                     Vector4d(pyrz(0), pyrz(1), current_roll, -current_eef_ppoint_distance);
            auto intermediate_affine = this->pivot_frame->getPose(intermediate_pyrz);
            if (!this->moveRobotCartesian(std::move(intermediate_affine))) {
#ifdef VISUALIZATION
                if (this->visualize_) {
                    this->resetMarkers();
                }
#endif
                return false;
            }
        }
        bool status = this->moveRobotCartesian(std::move(target_pose));
#ifdef VISUALIZATION
        if (this->visualize_) {
            this->resetMarkers();
        }
#endif
        return status;
    }

    bool GeneralBackend::movePYRZRelative(const Vector4d pyrz, bool degrees) {
        auto absolute_pyrz = this->getCurrentPYRZ() + pyrz;
        return this->movePYRZ(std::move(absolute_pyrz), degrees);
    }

#ifdef VISUALIZATION
    void GeneralBackend::resetMarkers() {
        this->visual_tools->deleteAllMarkers();
        this->visual_tools->publishAxis(toPoseMsg(this->pivot_frame->reference_frame()));
        this->visual_tools->trigger();
    }

    void GeneralBackend::visualize(bool visualize) {
        if (this->visualize_ != visualize) {
            this->visualize_ = visualize;
            if (this->visualize_) {
                this->resetMarkers();
            } else {
                this->visual_tools->deleteAllMarkers();
                this->visual_tools->trigger();
            }
        }
    }
#endif


    void GeneralBackend::fixCurrentPose() {
        double pivot_error = this->pivot_frame->getError(this->currentPose());
        auto current_pyrz = this->getCurrentPYRZ();
        Vector4d target_pyrz{current_pyrz(0), current_pyrz(1), clip(current_pyrz(2), this->roll_boundaries_), clip(current_pyrz(3), this->z_translation_boundaries_)};
        this->pivot_frame->clipAngle(target_pyrz, this->max_angle_);
        if (current_pyrz(0) != target_pyrz(0) || current_pyrz(1) != target_pyrz(1) || current_pyrz(2) != target_pyrz(2) || current_pyrz(3) != target_pyrz(3) || pivot_error > this->error_tolerance_) {
            if (!this->clip_to_boundaries_) {
                throw UnattainablePoseException("Current pose is out of boundaries");
            }
            target_pyrz(3) -= this->initial_eef_ppoint_distance_;
            if (!this->moveRobotCartesian(this->pivot_frame->getPose(target_pyrz))) {
                throw UnattainablePoseException("Failed to fix current pose");
            }
        }
    }

    void GeneralBackend::initial_eef_ppoint_distance(double initial_eef_ppoint_distance) {
        this->initial_eef_ppoint_distance_ = initial_eef_ppoint_distance;
        this->z_translation_boundaries(this->z_translation_boundaries_);
    }

    void GeneralBackend::tool_length(double tool_length) {
        this->tool_length_ = tool_length;
        this->z_translation_boundaries(this->z_translation_boundaries_);
    }

    void GeneralBackend::error_tolerance(double error_tolerance) {
        this->error_tolerance_ = error_tolerance;
        this->fixCurrentPose();
    }

    void GeneralBackend::max_angle(double max_angle) {
        this->max_angle_ = max_angle;
        this->fixCurrentPose();
    }

    void GeneralBackend::roll_boundaries(Vector2d roll_boundaries) {
        this->roll_boundaries_ = std::move(roll_boundaries);
        this->fixCurrentPose();
    }

    void GeneralBackend::z_translation_boundaries(Vector2d z_translation_boundaries) {
        this->z_translation_boundaries_ = Vector2d(std::max(z_translation_boundaries(0), this->initial_eef_ppoint_distance_ - this->tool_length_),
                                                   std::min(z_translation_boundaries(1), this->initial_eef_ppoint_distance_));
        this->fixCurrentPose();
    }

    void GeneralBackend::reference_frame(Affine3d reference_frame) {
        this->pivot_frame->reference_frame(reference_frame);
        bool clip_to_boundaries = this->clip_to_boundaries_;
        this->clip_to_boundaries_ = true;
        this->fixCurrentPose();
        this->clip_to_boundaries_ = clip_to_boundaries;
#ifdef VISUALIZATION
        if (this->visualize_) {
            this->resetMarkers();
        }
#endif
    }

    Vector4d GeneralBackend::getCurrentPYRZ() {
        auto current_pose = this->currentPose();
        auto pyrz = this->pivot_frame->getPYRZ(current_pose, &this->error_tolerance_);
        pyrz(3) += this->initial_eef_ppoint_distance_;
        return pyrz;
    }
}