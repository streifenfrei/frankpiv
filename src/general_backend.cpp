#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Header.h"
#include "visualization_msgs/Marker.h"
#include "frankpiv/general_backend.hpp"
#include "frankpiv/utilities.hpp"
#include <utility>

using namespace Eigen;
using namespace frankpiv::util;
using Euler = Eigen::EulerAngles<double, Eigen::EulerSystemXYZ>;

namespace frankpiv::backend {
    GeneralBackend::GeneralBackend(const YAML::Node &config, std::string node_name) {
        // configuration
        this->initial_eef_ppoint_distance = config["eef_ppoint_distance"].as<double>();
        this->tool_length = config["tool_length"].as<double>();
        this->max_angle = config["max_angle"].as<double>();
        this->roll_boundaries = Vector2d();
        this->roll_boundaries << config["roll_boundaries"][0].as<double>(), config["roll_boundaries"][1].as<double>();
        this->z_translation_boundaries = Vector2d();
        this->z_translation_boundaries
                << config["z_translation_boundaries"][0].as<double>(), config["z_translation_boundaries"][1].as<double>();
        this->clip_to_boundaries = config["clip_to_boundaries"].as<bool>();
        this->move_directly = config["move_directly"].as<bool>();
        // robotic stuff
        this->reference_frame = nullptr;
        this->current_pyrz = nullptr;
        // ROS / visualization
        this->visualize = config["visualize"].as<bool>();
        this->node_name = std::move(node_name);
        this->marker_publisher = nullptr;
        this->ros_node_initialized = false;
    }

    void GeneralBackend::initRosNode() {
        if (not this->ros_node_initialized) {
            int argc = 0;
            ros::init(argc, nullptr, this->node_name);
            this->ros_node_initialized = true;
        }
    }

    void GeneralBackend::shutdownRosNode() {
        if (this->ros_node_initialized) {
            ros::shutdown();
            this->ros_node_initialized = false;
        }
    }

    void GeneralBackend::start() {
        this->initialize();
        *this->reference_frame = this->currentPose() * Translation3d(0, 0, this->initial_eef_ppoint_distance);
        *this->current_pyrz << 0, 0, 0, 0;
        if (this->visualize) {
            this->initRosNode();
            if (!this->marker_publisher) {
                *this->marker_publisher = this->node_handle.advertise<visualization_msgs::Marker>(
                        "visualization_marker", 100);
            }
            this->publishMarker(*this->reference_frame, 0);
        }
    }

    void GeneralBackend::stop() {
        this->finish();
        if (this->visualize) {
            this->shutdownRosNode();
            this->deleteMarker();
            this->marker_publisher = nullptr;
        }
    }

    bool GeneralBackend::clipPose(Affine3d &pose) {
        Affine3d &pose_local = pose;
        Vector3d point;
        point << pose_local.translation();
        double z_translation = point[2] >= 0. ? point.norm() : -point.norm();
        Vector3d z_axis = point[2] >= 0. ? Vector3d(0, 0, 1) : Vector3d(0, 0, -1);
        double angle;
        if (point[0] == point[1] == point[2] == 0.) { // point lies in the origin -> add offset to calculate angle
            Vector3d point_offset;
            point_offset = (pose_local * Translation3d(0, 0, 1)).translation();
            angle = point_offset.dot(z_axis);
        } else {
            angle = point.dot(z_axis) / z_translation;
        }
        angle = -acos(angle);
        // clip pose if angle or z-translation are outside the specified boundaries
        if (angle < -this->max_angle || angle > this->max_angle ||
            z_translation < this->z_translation_boundaries(0) ||
            z_translation > this->z_translation_boundaries(1)) {
            // calculate clipped point by rotating along the rotation axis and translate along the z axis
            angle = clip(angle, -this->max_angle, this->max_angle);
            z_translation = clip(z_translation, this->z_translation_boundaries);
            Vector3d rotation_axis = point.cross(z_axis);
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
            throw UnattainablePoseException("Roll value is outside of specified boundaries");
        }
        roll = clip(roll, this->roll_boundaries);
        Affine3d converted_point;
        converted_point = Translation3d(point);
        if (frame) {
            // convert target point to the reference frame
            converted_point = *frame * converted_point;
            converted_point = this->reference_frame->inverse() * converted_point;
        }
        Vector3d target_point = converted_point.translation();
        Affine3d target_pose = Affine3d::Identity();
        if (target_point(0) == target_point(1) == target_point(2) != 0) {  // target point is the origin
            if (target_point(2) < 0.) {
                throw UnattainablePoseException("Point must have positive z value in the pivot point frame");
            }
            double distance = target_point.norm();
            double angle = -acos(target_point.dot(Vector3d(0, 0, 1)) / distance);
            if (angle != 0.) {
                // we rotate and translate to the target point to get the target rotation
                Vector3d rotation_axis = point.cross(Vector3d(0, 0, 1));
                target_pose = AngleAxis(angle, rotation_axis);
            }
            double z_translation = distance - this->tool_length + this->initial_eef_ppoint_distance;
            target_pose = target_pose * Translation3d(0, 0, z_translation);
        }
        if (this->move_directly) {
            bool clipped = this->clipPose(target_pose);
            if (clipped and !this->clip_to_boundaries) {
                throw UnattainablePoseException("Target point lies outside of specified boundaries");
            }
            // translate to the end effector pose and convert to global frame
            target_pose = *this->reference_frame *
                          (target_pose * (Translation3d(0, 0, -this->initial_eef_ppoint_distance)) *
                           Euler(0, 0, roll).toRotationMatrix());
            if (this->visualize) {
                this->publishMarker(target_pose, 1);
                this->publishMarker(target_pose * Translation3d(0, 0, this->tool_length), frankpiv::backend::GeneralBackend::POINT_MARKER, 2);
            }
            this->moveRobotCartesian(target_pose);
            if (this->visualize) {
                this->deleteMarker(1);
                this->deleteMarker(2);
            }
            Vector3d pyz = this->poseToPYZ(target_pose);
            Vector4d target_pyrz = Vector4d(pyz(0), pyz(1), roll, pyz(2));
            this->current_pyrz = &target_pyrz;
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
                throw UnattainablePoseException("Roll value is outside of specified boundaries");
            }
            if (z_translation < this->z_translation_boundaries(0) ||
                z_translation > this->z_translation_boundaries(1)) {
                throw UnattainablePoseException("Z-translation value is outside of specified boundaries");
            }
        }
        roll = clip(roll, this->roll_boundaries);
        z_translation = clip(z_translation, this->z_translation_boundaries);
        // calculate target pose
        double new_eef_ppoint_distance = this->initial_eef_ppoint_distance - z_translation;
        Affine3d target_affine = Euler(pitch, yaw, 0).toRotationMatrix() * Translation3d(0, 0, z_translation);
        bool clipped = this->clipPose(target_affine);
        if (clipped){
            if (!this->clip_to_boundaries){
                throw UnattainablePoseException("Target point lies outside of specified boundaries");
            }
            Vector3d pyz = this->poseToPYZ(target_affine);
            pitch = pyz(0);
            yaw = pyz(1);
        }
        target_affine = *this->reference_frame * (target_affine * (Translation3d(0, 0, -this->initial_eef_ppoint_distance) * Euler(0, 0, roll).toRotationMatrix()));
        // move the robot
        if (this->visualize){
            this->publishMarker(target_affine, 1);
            this->publishMarker(target_affine * Translation3d(0, 0, this->tool_length), frankpiv::backend::GeneralBackend::POINT_MARKER, 2);
        }
        if (!this->move_directly){
            double current_pitch = (*this->current_pyrz)(0);
            double current_yaw = (*this->current_pyrz)(1);
            double current_roll = (*this->current_pyrz)(2);
            double current_z_translation = (*this->current_pyrz)(3);
            double current_eef_ppoint_distance = this->initial_eef_ppoint_distance - current_z_translation;
            // if relative z-translation is negative do it before pitch and yaw
            Affine3d intermediate_affine;
            intermediate_affine = new_eef_ppoint_distance > current_eef_ppoint_distance ?
                    (*this->reference_frame) * Euler(current_pitch, current_yaw, 0).toRotationMatrix() * Translation3d(0, 0, -current_eef_ppoint_distance) * Euler(0, 0, roll).toRotationMatrix() :
                    (*this->reference_frame) * Euler(pitch, yaw, 0).toRotationMatrix() * Translation3d(0, 0, -current_eef_ppoint_distance) * Euler(0, 0, current_roll).toRotationMatrix();
            this->moveRobotCartesian(intermediate_affine);
        }
        this->moveRobotCartesian(target_affine);
        Vector4d target_pyrz = Vector4d(pitch, yaw, roll, z_translation);
        this->current_pyrz = &target_pyrz;
        if (this->visualize){
            this->deleteMarker(1);
            this->deleteMarker(2);
        }
    }

    void GeneralBackend::movePYRZRelative(const Vector4d &pyrz, bool degrees) {
        Vector4d absolute_pyrz = (*this->current_pyrz) + pyrz;
        this->movePYRZ(absolute_pyrz, degrees);
    }


    void GeneralBackend::publishMarker(const Affine3d &pose, int id, int type) {
        std_msgs::Header header;
        header.frame_id = "world";
        header.stamp = ros::Time::now();
        geometry_msgs::Point root = to_point_msg(pose);
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.id = id;
        marker.type = type;
        marker.action = 0;
        if (type == GeneralBackend::AXIS_MARKER){
            marker.scale.x = 0.01;
            geometry_msgs::Point point_x = to_point_msg(pose * Translation3d(0.05, 0, 0));
            geometry_msgs::Point point_y = to_point_msg(pose * Translation3d(0, 0.05, 0));
            geometry_msgs::Point point_z = to_point_msg(pose * Translation3d(0, 0, 0.05));
            marker.points = {root, point_x, root, point_y, root, point_z};
            marker.colors = {get_color_msg(1, 0, 0, 1), get_color_msg(1, 0, 0, 1),
                             get_color_msg(0, 1, 0, 1), get_color_msg(0, 1, 0, 1),
                             get_color_msg(0, 0, 1, 1), get_color_msg(0, 0, 1, 1)};
        } else if(type == GeneralBackend::POINT_MARKER) {
            marker.pose.position = root;
            marker.scale.x = 0.01;
            marker.scale.y = 0.01;
            marker.scale.z = 0.01;
            marker.color = get_color_msg(0, 0, 0, 1);
        } else {
            throw std::invalid_argument("No such marker type");
        }
        this->marker_publisher->publish(marker);
    }

    void GeneralBackend::deleteMarker(int id) {
        int action = id < 0 ? 2 : 3;
        visualization_msgs::Marker marker;
        marker.id = id;
        marker.action = action;
        this->marker_publisher->publish(marker);
    }
}