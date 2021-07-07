#include <pluginlib/class_loader.h>

#include "frankpiv/moveit_backend.hpp"
#include "frankpiv/utilities.hpp"

using namespace Eigen;
using namespace frankpiv::util;

namespace frankpiv::backend {
    typedef boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> ClassLoaderSPtr;

    MoveitBackend::MoveitBackend(const YAML::Node &config, const std::string &node_name, bool async_motion)
            : GeneralBackend(
            config, node_name) {
        YAML::Node moveit_config = config["moveit"];
        this->eef_step = get_config_value<float>(moveit_config, "eef_step")[0];
        this->jump_threshold = get_config_value<float>(moveit_config, "jump_threshold")[0];
        this->robot = nullptr;
        this->planner_instance = planning_interface::PlannerManagerPtr();
        this->loadPlanningPlugin();
    }

    void MoveitBackend::loadPlanningPlugin() {
        boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
        try {
            planner_plugin_loader.reset(
                    new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core",
                                                                                   "planning_interface::PlannerManager"));
        } catch (pluginlib::PluginlibException &ex) {
            ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
        }
        try {
            this->planner_instance.reset(planner_plugin_loader->createUnmanagedInstance("elion/ElionPlanner"));
        }
        catch (pluginlib::PluginlibException &ex) {
            ROS_ERROR_STREAM("Exception while loading planning plugin" << std::endl);
        }
    }

    void MoveitBackend::initialize() {
        this->robot = new moveit::planning_interface::MoveGroupInterface(this->getRobotName());
        if (!this->planner_instance->initialize(this->robot->getRobotModel(), "/move_group"))
            ROS_FATAL_STREAM("Could not initialize planner instance");
    }

    void MoveitBackend::finish() {
        this->robot = nullptr;
    }

    Eigen::Affine3d MoveitBackend::currentPose() {
        return to_affine(this->robot->getCurrentPose());
    }

    bool MoveitBackend::moveRobotCartesian(const Eigen::Affine3d &target_pose) {
        return false;
    }
}