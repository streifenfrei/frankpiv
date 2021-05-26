#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>

#include "frankpiv/utilities.hpp"

namespace frankpiv::util {
    Eigen::Vector3d get_rotation_euler(const Eigen::Affine3d &affine) {
        Eigen::Vector3d angles = Eigen::EulerAngles<double, Eigen::EulerSystemXYZ>::FromRotation<false, false, false>(
                affine.rotation()).angles();
        Eigen::Vector3d angles_equal;
        angles_equal << angles[0] - M_PI, M_PI - angles[1], angles[2] - M_PI;

        if (angles_equal[0] < -M_PI) {
            angles_equal[0] += 2 * M_PI;
        }
        if (angles_equal[1] > M_PI) {
            angles_equal[1] -= 2 * M_PI;
        }
        if (angles.norm() < angles_equal.norm()) {
            return angles;
        }
        return angles_equal;
    }

    double rad(double x) {
        return x * (M_PI / 180);
    }

    double clip(double n, double lower, double upper) {
        return std::max(lower, std::min(n, upper));
    }

    double clip(double n, const Eigen::Vector2d &boundaries) {
        return clip(n, boundaries(0), boundaries(1));
    }

    geometry_msgs::Point to_point_msg(const Eigen::Affine3d &affine) {
        Eigen::Vector3d point_eigen = affine.translation();
        geometry_msgs::Point point;
        point.x = point_eigen(0);
        point.y = point_eigen(1);
        point.z = point_eigen(2);
        return point;
    }

    geometry_msgs::Quaternion to_quat_msg(const Eigen::Matrix3d &rotation) {
        Eigen::Quaterniond quaternion(rotation);
        geometry_msgs::Quaternion message;
        message.w = quaternion.w();
        message.x = quaternion.x();
        message.y = quaternion.y();
        message.z = quaternion.z();
        return message;
    }

    geometry_msgs::Pose to_pose_msg(const Eigen::Affine3d &affine) {
        geometry_msgs::Point position = to_point_msg(affine);
        Eigen::Matrix3d rotation;
        rotation = affine.rotation();
        geometry_msgs::Quaternion orientation = to_quat_msg(rotation);
        geometry_msgs::Pose pose;
        pose.position = position;
        pose.orientation = orientation;
        return pose;
    }

    Eigen::Affine3d to_affine(const geometry_msgs::PoseStamped &msg) {
        geometry_msgs::Point position = msg.pose.position;
        geometry_msgs::Quaternion orientation = msg.pose.orientation;
        Eigen::Affine3d affine;
        affine = Eigen::Translation3d(position.x, position.y, position.z) *
                 Eigen::Quaternion(orientation.w, orientation.x, orientation.y, orientation.z);
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