#include <Eigen/Geometry>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

namespace frankpiv::util {
    Eigen::Vector3d get_rotation_euler(const Eigen::Affine3d &affine);

    double rad(double x);

    double clip(double n, double lower, double upper);

    double clip(double n, const Eigen::Vector2d &boundaries);

    geometry_msgs::Point to_point_msg(Eigen::Affine3d affine);

    std_msgs::ColorRGBA get_color_msg(int r, int g, int b, int a);
}