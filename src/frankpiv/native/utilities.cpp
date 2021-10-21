#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>

#include "frankpiv/utilities.hpp"

using Euler = Eigen::EulerAngles<double, Eigen::EulerSystemXYZ>;

using namespace Eigen;

namespace frankpiv::util {
    Vector3d getRotationEuler(const Affine3d &affine) {
        auto angles = EulerAngles<double, EulerSystemXYZ>::FromRotation<false, false, false>(affine.rotation()).angles();
        if (angles[0] > M_PI_2 || angles[0] < -M_PI_2) {
            angles << angles[0] - M_PI, M_PI - angles[1], angles[2] - M_PI;
            if (angles[0] < -M_PI) {
                angles[0] += 2 * M_PI;
            }
            if (angles[1] > M_PI) {
                angles[1] -= 2 * M_PI;
            }
            if (angles[2] > M_PI) {
                angles[2] += 2 * M_PI;
            }
        }
        return angles;
    }

    double rad(double x) {
        return x * M_PI / 180;
    }

    double deg(double x) {
        return x * 180 / M_PI;
    }

    double clip(double n, double lower, double upper) {
        return std::max(lower, std::min(n, upper));
    }

    double clip(double n, const Vector2d &boundaries) {
        return clip(n, boundaries(0), boundaries(1));
    }

    geometry_msgs::Point toPointMsg(const Affine3d &affine) {
        auto point_eigen = affine.translation();
        geometry_msgs::Point point;
        point.x = point_eigen(0);
        point.y = point_eigen(1);
        point.z = point_eigen(2);
        return point;
    }

    geometry_msgs::Quaternion toQuatMsg(const Matrix3d &rotation) {
        Quaterniond quaternion(rotation);
        geometry_msgs::Quaternion message;
        message.w = quaternion.w();
        message.x = quaternion.x();
        message.y = quaternion.y();
        message.z = quaternion.z();
        return message;
    }

    geometry_msgs::Pose toPoseMsg(const Affine3d &affine) {
        geometry_msgs::Point position = toPointMsg(affine);
        Matrix3d rotation = affine.rotation();
        geometry_msgs::Quaternion orientation = toQuatMsg(rotation);
        geometry_msgs::Pose pose;
        pose.position = position;
        pose.orientation = orientation;
        return pose;
    }

    Affine3d toAffine(const geometry_msgs::PoseStamped &msg) {
        geometry_msgs::Point position = msg.pose.position;
        geometry_msgs::Quaternion orientation = msg.pose.orientation;
        Affine3d affine = Translation3d(position.x, position.y, position.z) * Quaternion(orientation.w, orientation.x, orientation.y, orientation.z);
        return affine;
    }

    Affine3d toAffine(const std::array<double, 6> &array) {
        Affine3d affine;
        affine = Translation3d(array[0], array[1], array[2]) * Euler(array[3], array[4], array[5]).toRotationMatrix();
        return affine;
    }

    std::array<double, 6> toArray(const Eigen::Affine3d &affine) {
        Eigen::Matrix<double, 6, 1> matrix;
        matrix << affine.translation(), getRotationEuler(affine);
        return {matrix(0), matrix(1), matrix(2), matrix(3), matrix(4), matrix(5)};
    }
}