#include "ros/ros.h"
#include "frankpiv/general_backend.hpp"
#include "frankpiv/utilities.hpp"
#include <unsupported/Eigen/EulerAngles>

using Euler = Eigen::EulerAngles<double, Eigen::EulerSystemXYZ>;

using namespace Eigen;
using namespace frankpiv::util;

namespace frankpiv::backend {
    GeneralBackend::GeneralBackend(const YAML::Node &config, const std::string &node_name) : ROSNode(node_name) {
        // configuration
        this->initial_eef_ppoint_distance = get_config_value<double>(config, "eef_ppoint_distance")[0];
        this->tool_length = get_config_value<double>(config, "tool_length")[0];
        this->max_angle = rad(get_config_value<double>(config, "max_angle")[0]);
        this->roll_boundaries = Vector2d(
                rad(get_config_value<double>(config, "roll_boundaries")[0]),
                rad(get_config_value<double>(config, "roll_boundaries")[1]));
        this->z_translation_boundaries = Vector2d(
                get_config_value<double>(config, "z_translation_boundaries")[0],
                get_config_value<double>(config, "z_translation_boundaries")[1]);
        this->clip_to_boundaries = get_config_value<bool>(config, "clip_to_boundaries")[0];
        this->move_directly = get_config_value<bool>(config, "move_directly")[0];
        // robotic stuff
        this->robot_name = get_config_value<std::string>(config, "robot_name")[0];
        this->reference_frame = Affine3d();
        this->current_pyrz = Vector4d();
        // ROS / visualization
        this->spinner.start();
        this->node_handle = ros::NodeHandle();
#ifdef VISUALIZATION
        this->visualize = get_config_value<bool>(config, "visualize")[0];
        std::string topic = get_config_value<std::string>(config, "marker_topic")[0];
        this->visual_tools.reset(new rviz_visual_tools::RvizVisualTools("world",topic, this->node_handle));
#endif
    }

    GeneralBackend::~GeneralBackend() {
        this->stop();
        this->spinner.stop();
        this->node_handle.shutdown();
    }

    void GeneralBackend::start() {
        this->initialize();
        this->reference_frame = this->currentPose() * Translation3d(0, 0, this->initial_eef_ppoint_distance);
        this->current_pyrz = Vector4d();
#ifdef VISUALIZATION
        if (this->visualize) {
            this->reset_markers();
        }
#endif
    }

    void GeneralBackend::stop() {
        this->finish();
    }

    bool GeneralBackend::clipPose(Affine3d &pose, double *out_angle) {
        Affine3d &pose_local = pose;
        Vector3d point;
        point << pose_local.translation();
        double z_translation = point[2] >= 0. ? point.norm() : -point.norm();
        Vector3d z_axis = point[2] >= 0. ? Vector3d(0, 0, 1) : Vector3d(0, 0, -1);
        double angle;
        if (point[0] == 0. && point[1] == 0. && point[2] == 0.) { // point lies in the origin -> add offset to calculate angle
            Vector3d point_offset;
            point_offset = (pose_local * Translation3d(0, 0, 1)).translation();
            angle = point_offset.dot(z_axis);
        } else {
            angle = point.dot(z_axis) / z_translation;
        }
        angle = -acos(angle);
        if (out_angle) {
            *out_angle = angle;
        }
        // clip pose if angle or z-translation are outside the specified boundaries
        if (angle < -this->max_angle || angle > this->max_angle ||
            z_translation < this->z_translation_boundaries(0) ||
            z_translation > this->z_translation_boundaries(1)) {
            // calculate clipped point by rotating along the rotation axis and translate along the z axis
            angle = clip(angle, -this->max_angle, this->max_angle);
            z_translation = clip(z_translation, this->z_translation_boundaries);
            Vector3d rotation_axis = point.cross(z_axis);
            rotation_axis /= rotation_axis.norm();
            Affine3d clipped_pose;
            clipped_pose = AngleAxisd(angle, rotation_axis) * Translation3d(0, 0, z_translation);
            pose = clipped_pose;
            return true;
        }
        return false;
    }

    Vector3d GeneralBackend::poseToPYZ(const Affine3d &pose) {
        Vector3d point = pose.translation();
        Vector3d orientation = get_rotation_euler(pose);
        double distance = point.norm();
        double z_translation = point(2) >= 0 ? distance : -distance;
        return Vector3d(orientation(0), orientation(1), z_translation);
    }

    void GeneralBackend::moveToPoint(const Vector3d &point, double roll, const Affine3d *frame) {
        if (roll < this->roll_boundaries(0) || roll > this->roll_boundaries(1)) {
            throw UnattainablePoseException("Roll value is outside of specified boundaries", &this->roll_boundaries,
                                            &roll);
        }
        roll = clip(roll, this->roll_boundaries);
        Affine3d converted_point;
        converted_point = Translation3d(point);
        if (frame) {
            // convert target point to the reference frame
            converted_point = *frame * converted_point;
            converted_point = this->reference_frame.inverse() * converted_point;
        }
        Vector3d target_point = converted_point.translation();
        Affine3d target_pose = Affine3d::Identity();
        if (target_point(0) != 0 || target_point(1) != 0 || target_point(2) != 0) {  // target point is the origin
            if (target_point(2) < 0.) {
                throw UnattainablePoseException("Point must have positive z value in the pivot point frame", nullptr,
                                                &target_point(2));
            }
            double distance = target_point.norm();
            double angle = -acos(target_point.dot(Vector3d(0, 0, 1)) / distance);
            if (angle != 0.) {
                // we rotate and translate to the target point to get the target rotation
                Vector3d rotation_axis = target_point.cross(Vector3d(0, 0, 1));
                rotation_axis /= rotation_axis.norm();
                target_pose = AngleAxis(angle, rotation_axis).toRotationMatrix();
            }
            double z_translation = distance - this->tool_length + this->initial_eef_ppoint_distance;
            target_pose = target_pose * Translation3d(0, 0, z_translation);
        }
        if (this->move_directly) {
            auto *angle = new double();
            bool clipped = this->clipPose(target_pose, angle);
            if (clipped and !this->clip_to_boundaries) {
                Vector2d angle_boundaries = Vector2d(this->max_angle, this->max_angle);
                throw UnattainablePoseException("Target point lies outside of specified boundaries", &angle_boundaries,
                                                angle);
            }

            // translate to the end effector pose and convert to global frame
            target_pose = this->reference_frame * (target_pose *
                    (Translation3d(0, 0, -this->initial_eef_ppoint_distance) *
                    Euler(0, 0, roll).toRotationMatrix()));
#ifdef VISUALIZATION
            if (this->visualize) {
                this->visual_tools->publishAxis(to_pose_msg(target_pose));
                this->visual_tools->publishSphere(to_pose_msg(target_pose * Translation3d(0, 0, this->tool_length)));
                this->visual_tools->trigger();
            }
#endif
            this->moveRobotCartesian(target_pose);
#ifdef VISUALIZATION
            if (this->visualize) {
                this->reset_markers();
            }
#endif
            Vector3d pyz = this->poseToPYZ(target_pose);
            this->current_pyrz = Vector4d(pyz(0), pyz(1), roll, pyz(2));
        } else {
            Vector3d pyz = this->poseToPYZ(target_pose);
            Vector4d target_pyrz = Vector4d(pyz(0), pyz(1), roll, pyz(2));
            this->movePYRZ(target_pyrz);
        }
    }

    void GeneralBackend::movePYRZ(const Vector4d &pyrz, bool degrees) {
        // prepare input
        double pitch = pyrz(0);
        double yaw = pyrz(1);
        double roll = pyrz(2);
        double z_translation = pyrz(3);
        if (degrees) {
            pitch = rad(pitch);
            yaw = rad(yaw);
            roll = rad(roll);
        }
        if (!this->clip_to_boundaries) {
            if (roll < this->roll_boundaries(0) || roll > this->roll_boundaries(1)) {
                throw UnattainablePoseException("Roll value is outside of specified boundaries", &this->roll_boundaries,
                                                &roll);
            }
            if (z_translation < this->z_translation_boundaries(0) ||
                z_translation > this->z_translation_boundaries(1)) {
                throw UnattainablePoseException("Z-translation value is outside of specified boundaries",
                                                &this->z_translation_boundaries, &z_translation);
            }
        }
        roll = clip(roll, this->roll_boundaries);
        z_translation = clip(z_translation, this->z_translation_boundaries);
        // calculate target pose
        double new_eef_ppoint_distance = this->initial_eef_ppoint_distance - z_translation;
        Affine3d target_affine = Euler(pitch, yaw, 0).toRotationMatrix() * Translation3d(0, 0, z_translation);
        auto *angle = new double();
        bool clipped = this->clipPose(target_affine, angle);
        if (clipped) {
            if (!this->clip_to_boundaries) {
                Vector2d angle_boundaries = Vector2d(this->max_angle, this->max_angle);
                throw UnattainablePoseException("Target point lies outside of specified boundaries", &angle_boundaries,
                                                angle);
            }
            Vector3d pyz = this->poseToPYZ(target_affine);
            pitch = pyz(0);
            yaw = pyz(1);
        }
        target_affine = this->reference_frame * (target_affine *
                                                 (Translation3d(0, 0, -this->initial_eef_ppoint_distance) *
                                                  Euler(0, 0, roll).toRotationMatrix()));
        // move the robot
#ifdef VISUALIZATION
        if (this->visualize) {
            this->visual_tools->publishAxis(to_pose_msg(target_affine));
            this->visual_tools->publishSphere(to_pose_msg(target_affine * Translation3d(0, 0, this->tool_length)));
            this->visual_tools->trigger();
        }
#endif
        if (!this->move_directly) {
            double current_pitch = this->current_pyrz(0);
            double current_yaw = this->current_pyrz(1);
            double current_roll = this->current_pyrz(2);
            double current_z_translation = this->current_pyrz(3);
            double current_eef_ppoint_distance = this->initial_eef_ppoint_distance - current_z_translation;
            // if relative z-translation is negative do it before pitch and yaw
            Affine3d intermediate_affine;
            intermediate_affine = new_eef_ppoint_distance > current_eef_ppoint_distance ?
                                  this->reference_frame * (Euler(current_pitch, current_yaw, 0).toRotationMatrix() *
                                                           (Translation3d(0, 0, -new_eef_ppoint_distance) *
                                                            Euler(0, 0, roll).toRotationMatrix())) :
                                  this->reference_frame * (Euler(pitch, yaw, 0).toRotationMatrix() *
                                                           (Translation3d(0, 0, -current_eef_ppoint_distance) *
                                                            Euler(0, 0, current_roll).toRotationMatrix()));
            this->moveRobotCartesian(intermediate_affine);
        }
        this->moveRobotCartesian(target_affine);
        this->current_pyrz = Vector4d(pitch, yaw, roll, z_translation);
#ifdef VISUALIZATION
        if (this->visualize) {
            this->reset_markers();
        }
#endif
    }

    void GeneralBackend::movePYRZRelative(const Vector4d &pyrz, bool degrees) {
        Vector4d absolute_pyrz = this->current_pyrz + pyrz;
        this->movePYRZ(absolute_pyrz, degrees);
    }

#ifdef VISUALIZATION
    void GeneralBackend::reset_markers() {
        this->visual_tools->deleteAllMarkers();
        this->visual_tools->publishAxis(to_pose_msg(this->reference_frame));
        this->visual_tools->trigger();
    }
#endif
}