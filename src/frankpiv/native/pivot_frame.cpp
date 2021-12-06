#include <unsupported/Eigen/EulerAngles>

#include "frankpiv/pivot_frame.hpp"
#include "frankpiv/utilities.hpp"
#include "frankpiv/exceptions.hpp"

using namespace Eigen;
using namespace frankpiv::util;
using namespace frankpiv::exceptions;

using Euler = EulerAngles<double, EulerSystemXYZ>;

namespace frankpiv {
    PivotFrame::PivotFrame(Affine3d reference_frame, double max_angle, Vector2d roll_boundaries, Vector2d z_translation_boundaries, double error_tolerance) :
            reference_frame_(std::move(reference_frame)),
            max_angle_(max_angle),
            roll_boundaries_(std::move(roll_boundaries)),
            z_translation_boundaries_(std::move(z_translation_boundaries)),
            error_tolerance_(error_tolerance) {
    }

    Vector4d PivotFrame::getPYRZ(const Vector3d &point, double roll, const Affine3d *frame) const {
        return PivotFrame::getPYRZInternal(this->otherToLocalFrame(point, frame), roll);
    }

    Vector4d PivotFrame::getPYRZ(const Affine3d &pose, const Affine3d *frame) const {
        auto local_pose = this->otherToLocalFrame(pose, frame);
        double error = PivotFrame::getErrorInternal(local_pose);
        if (error > this->error_tolerance_)
            throw CriticalPivotErrorException(error, this->error_tolerance_);
        return PivotFrame::internalPoseToPYRZ(local_pose);
    }

    Affine3d PivotFrame::getPose(const Vector4d &pyrz, const Affine3d *frame) const {
        return PivotFrame::localToOtherFrame(PivotFrame::getPoseInternal(pyrz), frame);
    }

    double PivotFrame::getError(const Affine3d &pose, const Affine3d *frame) const {
        return PivotFrame::getErrorInternal(this->otherToLocalFrame(pose, frame));
    }

    bool PivotFrame::isValid(const Affine3d &pose, const Affine3d *frame) const {
        return PivotFrame::isValidInternal(this->otherToLocalFrame(pose, frame), this->error_tolerance_);
    }

    double PivotFrame::getAngle(const Vector4d &pyrz) {
        return PivotFrame::getAngleInternal(pyrz);
    }

    bool PivotFrame::clipAngle(Vector4d &pyrz) {
        auto point = new Vector3d();
        auto z_axis = new Vector3d();
        auto z_translation = new double();
        double angle = PivotFrame::getAngleInternal(pyrz, point, z_axis, z_translation);
        bool clipped = false;
        if (angle < -this->max_angle_ || angle > this->max_angle_) {
            // calculate clipped point by rotating along the rotation axis and translate along the z axis
            angle = frankpiv::util::clip(angle, -this->max_angle_, this->max_angle_);
            auto rotation_axis = (*point).cross(*z_axis);
            rotation_axis /= rotation_axis.norm();
            auto pose = AngleAxisd(angle, rotation_axis) * (Translation3d(0, 0, *z_translation) * Euler(0, 0, pyrz[2]).toRotationMatrix());
            pyrz = PivotFrame::internalPoseToPYRZ(pose);
            clipped = true;
        }
        delete point, z_axis, z_translation;
        return clipped;
    }

    bool PivotFrame::clipRoll(Vector4d &pyrz) {
        double roll = pyrz(2);
        pyrz(2) = frankpiv::util::clip(roll, this->roll_boundaries_);
        return roll != pyrz(2);
    }

    bool PivotFrame::clipZTranslation(Vector4d &pyrz) {
        double z_translation = pyrz(3);
        pyrz(3) = frankpiv::util::clip(z_translation, this->z_translation_boundaries_);
        return z_translation != pyrz(3);
    }

    bool PivotFrame::clip(Vector4d &pyrz) {
        return this->clipAngle(pyrz) | this->clipRoll(pyrz) | this->clipZTranslation(pyrz);
    }

    Affine3d PivotFrame::otherToLocalFrame(const Affine3d &pose, const Affine3d *frame) const {
        auto target_pose = frame ? *frame * pose : pose;
        return this->reference_frame_.inverse() * target_pose;
    }

    Vector3d PivotFrame::otherToLocalFrame(const Vector3d &point, const Affine3d *frame) const {
        Affine3d pose;
        pose = Translation3d(point);
        return this->otherToLocalFrame(pose, frame).translation();
    }

    Affine3d PivotFrame::localToOtherFrame(const Affine3d &pose, const Affine3d *frame) const {
        auto target_pose = this->reference_frame_ * pose;
        return frame ? (*frame).inverse() * target_pose : target_pose;
    }

    Vector4d PivotFrame::internalPoseToPYRZ(Affine3d &pose) {
        auto point = pose.translation();
        double z_translation = point(2) >= 0 ? point.norm() : -point.norm();
        auto in_origin = pose * Translation3d(0, 0, -z_translation);
        auto orientation = getRotationEuler(in_origin);
        return Vector4d(orientation(0), orientation(1), orientation(2), z_translation);
    }

    Vector4d PivotFrame::getPYRZInternal(const Vector3d &point, double roll) {
        Affine3d target_pose = Affine3d::Identity();
        double distance = point.norm();
        if (distance != 0) {  // target point is not in the origin
            double angle = -acos(point.dot(Vector3d(0, 0, 1)) / distance);
            if (angle != 0.) {
                // we rotate and translate to the target point to get the target rotation
                Vector3d rotation_axis = point.cross(Vector3d(0, 0, 1));
                rotation_axis /= rotation_axis.norm();
                target_pose = AngleAxis(angle, rotation_axis).toRotationMatrix();
            }
        }
        target_pose = target_pose * (Translation3d(0, 0, distance) * Euler(0, 0, roll).toRotationMatrix());
        return PivotFrame::internalPoseToPYRZ(target_pose);
    }

    Affine3d PivotFrame::getPoseInternal(const Vector4d &pyrz) {
        double pitch = pyrz(0);
        double yaw = pyrz(1);
        double roll = pyrz(2);
        double z_translation = pyrz(3);
        return Euler(pitch, yaw, 0).toRotationMatrix() * (Translation3d(0, 0, z_translation) * Euler(0, 0, roll).toRotationMatrix());
    }

    double PivotFrame::getErrorInternal(const Affine3d &pose) {
        auto point = pose.translation();
        double z_translation = point(2) >= 0 ? -point.norm() : point.norm();
        auto target_pose = pose * Translation3d(0, 0, z_translation);
        return target_pose.translation().norm();
    }

    bool PivotFrame::isValidInternal(const Affine3d &pose, double error_tolerance) {
        double error = PivotFrame::getErrorInternal(pose);
        return error < error_tolerance;
    }

    double PivotFrame::getAngleInternal(const Vector4d &pyrz, Vector3d *point, Vector3d *z_axis, double *z_translation) {
        auto pose_ = PivotFrame::getPoseInternal(pyrz);
        auto point_ = pose_.translation();
        double z_translation_ = point_[2] >= 0. ? point_.norm() : -point_.norm();
        auto z_axis_ = point_[2] >= 0. ? Vector3d(0, 0, 1) : Vector3d(0, 0, -1);
        if (z_translation_ == 0) {
            z_translation_ = 1;
            point_ = (pose_ * Translation3d(0, 0, 1)).translation();
        }
        if (point)
            *point = point_;
        if (z_axis)
            *z_axis = z_axis_;
        if (z_translation)
            *z_translation = z_translation_;
        double angle = -acos(point_.dot(z_axis_) / z_translation_);
        if (angle > M_PI / 2) {
            angle = M_PI - angle;
        } else if (angle < -M_PI / 2)
            angle = -M_PI - angle;
        return angle;
    }

}
