#ifndef FRANKPIV_UTILITIES
#define FRANKPIV_UTILITIES

#include <Eigen/Geometry>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include "yaml-cpp/yaml.h"

#include "frankpiv/exceptions.hpp"

using namespace frankpiv::exceptions;

namespace frankpiv::util {
    // geometry
    Eigen::Vector3d getRotationEuler(const Eigen::Affine3d &affine);

    double rad(double x);

    double deg(double x);

    double clip(double n, double lower, double upper);

    double clip(double n, const Eigen::Vector2d &boundaries);

    // conversion
    geometry_msgs::Point toPointMsg(const Eigen::Affine3d &affine);

    geometry_msgs::Quaternion toQuatMsg(const Eigen::Matrix3d &rotation);

    geometry_msgs::Pose toPoseMsg(const Eigen::Affine3d &affine);

    Eigen::Affine3d toAffine(const geometry_msgs::PoseStamped &msg);

    Eigen::Affine3d toAffine(const std::array<double, 6> &array);

    std::array<double, 6> toArray(const Eigen::Affine3d &affine);

    // other
    template<typename T>
    std::vector <T> getConfigValue(const YAML::Node &config, const std::string &key) {
        YAML::Node node = config[key];
        if (!node.IsDefined()) {
            throw ConfigError("Missing key: " + key);
        }
        try {
            std::vector <T> value;
            if (node.IsSequence()) {
                for (auto &&i : node) {
                    value.push_back(i.as<T>());
                }
            } else {
                value.push_back(node.as<T>());
            }
            return value;
        } catch (const YAML::BadConversion &exception) {
            throw ConfigError("Bad data type for key '" + key + "'. Expected: " + typeid(T).name());
        }
    }
}

#endif //FRANKPIV_UTILITIES