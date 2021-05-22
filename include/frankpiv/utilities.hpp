#ifndef FRANKPIV_UTILITIES
#define FRANKPIV_UTILITIES

#include <Eigen/Geometry>
#include "yaml-cpp/yaml.h"
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/thread/mutex.hpp>

namespace frankpiv {

    class ConfigError : public std::runtime_error {
        std::string message;
    public:
        explicit ConfigError(const std::string &message) : runtime_error(message) {
            this->message = message;
        }

        const char *what() {
            return this->message.c_str();
        }
    };

    namespace util {
        // geometry
        Eigen::Vector3d get_rotation_euler(const Eigen::Affine3d &affine);

        double rad(double x);

        double clip(double n, double lower, double upper);

        double clip(double n, const Eigen::Vector2d &boundaries);

        // conversion
        geometry_msgs::Point to_point_msg(const Eigen::Affine3d &affine);

        geometry_msgs::Quaternion to_quat_msg(const Eigen::Matrix3d &rotation);

        geometry_msgs::Pose to_pose_msg(const Eigen::Affine3d &affine);

        Eigen::Affine3d to_affine(const geometry_msgs::PoseStamped &msg);

        std_msgs::ColorRGBA get_color_msg(int r, int g, int b, int a);

        // other
        template<typename T>
        std::vector<T> get_config_value(const YAML::Node &config, const std::string &key) {
            YAML::Node node = config[key];
            if (!node.IsDefined()) {
                throw ConfigError("Missing key: " + key);
            }
            try {
                std::vector<T> value;
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
}

#endif //FRANKPIV_UTILITIES