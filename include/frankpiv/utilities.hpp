#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>

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

    geometry_msgs::Point to_point_msg(Eigen::Affine3d affine) {
        Eigen::Vector3d point_eigen = affine.translation();
        geometry_msgs::Point point;
        point.x = point_eigen(0);
        point.y = point_eigen(1);
        point.z = point_eigen(2);
        return point;
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