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
            z_translation_boundaries_(Vector2d(getConfigValue<double>(config, "z_translation_boundaries")[0], getConfigValue<double>(config, "z_translation_boundaries")[1])),
            clip_to_boundaries_(getConfigValue<bool>(config, "clip_to_boundaries")[0]),
            move_directly_(getConfigValue<bool>(config, "move_directly")[0]),
            pivot_frame(std::make_shared<frankpiv::PivotFrame>(Affine3d(),
                                                               rad(getConfigValue<double>(config, "max_angle")[0]),
                                                               Vector2d(rad(getConfigValue<double>(config, "roll_boundaries")[0]), rad(getConfigValue<double>(config, "roll_boundaries")[1])),
                                                               Vector2d(-this->initial_eef_ppoint_distance_ + this->z_translation_boundaries_(0), -this->initial_eef_ppoint_distance_ + this->z_translation_boundaries_(1)),
                                                               getConfigValue<double>(config, "pivot_error_tolerance")[0]))
#ifdef VISUALIZATION
    ,visualize_(getConfigValue<bool>(config, "visualize")[0])
#endif
    {
        this->spinner_.start();

#ifdef VISUALIZATION
        std::string topic = getConfigValue<std::string>(config, "marker_topic")[0];
        this->visual_tools_world.reset(new rviz_visual_tools::RvizVisualTools("world", topic, this->node_handle_));
        this->visual_tools_eef.reset(new rviz_visual_tools::RvizVisualTools("panda_link8", topic, this->node_handle_));
        this->visual_tools_eef->enableFrameLocking();
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
            this->visual_tools_world->deleteAllMarkers();
            this->visual_tools_eef->deleteAllMarkers();
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
        Vector4d unclipped_pyrz = pyrz;
        pyrz(3) -= this->initial_eef_ppoint_distance_;
        if (this->pivot_frame->clipRoll(pyrz) && !this->clip_to_boundaries_) {
            Vector2d roll_boundaries = this->pivot_frame->roll_boundaries();
            throw UnattainablePoseException("Roll value is outside of specified boundaries", &roll_boundaries, &unclipped_pyrz(2));
        }
        if (this->pivot_frame->clipZTranslation(pyrz) && !this->clip_to_boundaries_) {
            throw UnattainablePoseException("Z-translation value is outside of specified boundaries", &this->z_translation_boundaries_, &unclipped_pyrz(3));
        }
        if (this->pivot_frame->clipAngle(pyrz) && !this->clip_to_boundaries_) {
            double angle = this->pivot_frame->getAngle(pyrz);
            Vector2d angle_boundaries = Vector2d(-this->pivot_frame->max_angle(), this->pivot_frame->max_angle());
            throw UnattainablePoseException("PY angle is outside of specified boundaries", &angle_boundaries, &angle);
        }
        // move the robot
#ifdef VISUALIZATION
        if (this->visualize_) {
            auto target_pose = this->pivot_frame->getPose(pyrz);
            this->visual_tools_world->publishAxis(toPoseMsg(target_pose));
            this->visual_tools_world->publishSphere(toPoseMsg(target_pose * Translation3d(0, 0, this->tool_length_)), rviz_visual_tools::BLACK);
            this->visual_tools_world->trigger();
        }
#endif
        if (!this->move_directly_) {
            auto current_pyrz = this->getCurrentPYRZ();
            double current_eef_ppoint_distance = this->initial_eef_ppoint_distance_ - current_pyrz(3);
            double new_eef_ppoint_distance = -pyrz(3);
            // if relative z-translation is negative do it before pitch and yaw
            auto intermediate_pyrz = new_eef_ppoint_distance > current_eef_ppoint_distance ?
                                     Vector4d(current_pyrz(0), current_pyrz(1), pyrz(2), pyrz(3)) :
                                     Vector4d(pyrz(0), pyrz(1), current_pyrz(2), -current_eef_ppoint_distance);
            if (!this->moveRobotPYRZ(std::move(intermediate_pyrz))) {
#ifdef VISUALIZATION
                if (this->visualize_) {
                    this->resetMarkers();
                }
#endif
                return false;
            }
        }
        bool status = this->moveRobotPYRZ(std::move(pyrz));
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
        this->visual_tools_world->deleteAllMarkers();
        this->visual_tools_eef->deleteAllMarkers();
        this->visual_tools_world->publishAxis(toPoseMsg(this->pivot_frame->reference_frame()));
        this->visual_tools_eef->publishLine(Vector3d(), Vector3d(0, 0, this->tool_length_), rviz_visual_tools::BLACK);
        // I dont quite understand the trigger behaviour, but in this order the calls dont "cancel" each other at least
        this->visual_tools_eef->trigger();
        this->visual_tools_world->trigger();
    }

    void GeneralBackend::visualize(bool visualize) {
        if (this->visualize_ != visualize) {
            this->visualize_ = visualize;
            if (this->visualize_) {
                this->resetMarkers();
            } else {
                this->visual_tools_world->deleteAllMarkers();
                this->visual_tools_eef->deleteAllMarkers();
                this->visual_tools_world->trigger();
                this->visual_tools_eef->trigger();
            }
        }
    }
#endif

    void GeneralBackend::fixCurrentPose() {
        bool critical_pivot_error = false;
        Vector4d pyrz;
        try {
            pyrz = this->getCurrentPYRZ();
        } catch (const CriticalPivotErrorException &exception) {
            auto current_position = this->currentPose().translation();
            pyrz = this->pivot_frame->getPYRZ(current_position, 0);
            critical_pivot_error = true;
        }
        pyrz(3) -= this->initial_eef_ppoint_distance_;
        if (critical_pivot_error | this->pivot_frame->clip(pyrz)) {
            if (!this->clip_to_boundaries_) {
                throw UnattainablePoseException("Current pose is out of boundaries");
            }
            if (!this->moveRobotPYRZ(pyrz)) {
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
        this->pivot_frame->error_tolerance(error_tolerance);
        this->fixCurrentPose();
    }

    void GeneralBackend::max_angle(double max_angle) {
        this->pivot_frame->max_angle(max_angle);
        this->fixCurrentPose();
    }

    void GeneralBackend::roll_boundaries(Vector2d roll_boundaries) {
        this->pivot_frame->roll_boundaries(Vector2d(std::max(roll_boundaries(0), -M_PI), std::min(roll_boundaries(1), M_PI)));
        this->fixCurrentPose();
    }

    void GeneralBackend::z_translation_boundaries(Vector2d z_translation_boundaries) {
        this->z_translation_boundaries_ = Vector2d(std::max(z_translation_boundaries(0), this->initial_eef_ppoint_distance_ - this->tool_length_),
                                                   std::min(z_translation_boundaries(1), this->initial_eef_ppoint_distance_));
        this->pivot_frame->z_translation_boundaries(Vector2d(-this->initial_eef_ppoint_distance_ + this->z_translation_boundaries_(0),
                                                             -this->initial_eef_ppoint_distance_ + this->z_translation_boundaries_(1)));
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
        auto pyrz = this->pivot_frame->getPYRZ(current_pose);
        pyrz(3) += this->initial_eef_ppoint_distance_;
        return pyrz;
    }

}