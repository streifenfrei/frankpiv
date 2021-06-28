#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>

#include "frankpiv/utilities.hpp"

using Euler = Eigen::EulerAngles<double, Eigen::EulerSystemXYZ>;

using namespace Eigen;

namespace frankpiv::util {
    Vector3d get_rotation_euler(const Affine3d &affine) {
        Vector3d angles = EulerAngles<double, EulerSystemXYZ>::FromRotation<false, false, false>(
                affine.rotation()).angles();
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

    geometry_msgs::Point to_point_msg(const Affine3d &affine) {
        Vector3d point_eigen = affine.translation();
        geometry_msgs::Point point;
        point.x = point_eigen(0);
        point.y = point_eigen(1);
        point.z = point_eigen(2);
        return point;
    }

    geometry_msgs::Quaternion to_quat_msg(const Matrix3d &rotation) {
        Quaterniond quaternion(rotation);
        geometry_msgs::Quaternion message;
        message.w = quaternion.w();
        message.x = quaternion.x();
        message.y = quaternion.y();
        message.z = quaternion.z();
        return message;
    }

    geometry_msgs::Pose to_pose_msg(const Affine3d &affine) {
        geometry_msgs::Point position = to_point_msg(affine);
        Matrix3d rotation;
        rotation = affine.rotation();
        geometry_msgs::Quaternion orientation = to_quat_msg(rotation);
        geometry_msgs::Pose pose;
        pose.position = position;
        pose.orientation = orientation;
        return pose;
    }

    Affine3d to_affine(const geometry_msgs::PoseStamped &msg) {
        geometry_msgs::Point position = msg.pose.position;
        geometry_msgs::Quaternion orientation = msg.pose.orientation;
        Affine3d affine;
        affine = Translation3d(position.x, position.y, position.z) *
                 Quaternion(orientation.w, orientation.x, orientation.y, orientation.z);
        return affine;
    }

    Affine3d to_affine(const std::array<double, 6> &array) {
        Affine3d affine;
        affine = Translation3d(array[0], array[1], array[2]) * Euler(array[3], array[4], array[5]).toRotationMatrix();
        return affine;
    }

    std_msgs::ColorRGBA get_color_msg(int r, int g, int b, int a) {
        std_msgs::ColorRGBA color;
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = a;
        return color;
    }
}