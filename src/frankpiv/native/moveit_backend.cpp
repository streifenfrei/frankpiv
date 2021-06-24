#include "frankpiv/moveit_backend.hpp"
#include "frankpiv/utilities.hpp"

using namespace Eigen;
using namespace frankpiv::util;

namespace frankpiv::backend {
    MoveitBackend::MoveitBackend(const YAML::Node &config, const std::string& node_name, bool async_motion) : GeneralBackend(
            config, node_name) {
        YAML::Node moveit_config = config["moveit"];
        this->eef_step = get_config_value<float>(moveit_config, "eef_step")[0];
        this->jump_threshold = get_config_value<float>(moveit_config, "jump_threshold")[0];
        this->robot = nullptr;
        this->async_motion = async_motion;
        this->threads_list = std::list<boost::thread::id>();
        this->terminating = new boost::atomic<bool>();
    }

    void MoveitBackend::initialize() {
        this->robot = new moveit::planning_interface::MoveGroupInterface(this->robot_name);
        this->terminating = false;
    }

    void MoveitBackend::finish() {
        this->terminating = true;
        boost::posix_time::ptime start = boost::posix_time::second_clock::local_time();
        while (!this->threads_list.empty()) {
            boost::posix_time::time_duration duration = boost::posix_time::second_clock::local_time() - start;
            if (duration.total_milliseconds() > this->SHUTDOWN_TIMEOUT) {
                this->robot->stop();
                break;
            }
            boost::this_thread::sleep(boost::posix_time::milliseconds(MoveitBackend::SPINLOCK_WAIT));
        }
        this->robot = nullptr;
    }

    Eigen::Affine3d MoveitBackend::currentPose() {
        return to_affine(this->robot->getCurrentPose());
    }

    void MoveitBackend::moveRobotCartesian(const Eigen::Affine3d &target_pose) {
        bool move = true;
        if (this->async_motion) {
            boost::thread::id most_recent_thread = *this->threads_list.end();
            move = boost::this_thread::get_id() == most_recent_thread;
        }
        if (move) {
            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(to_pose_msg(target_pose));
            moveit_msgs::RobotTrajectory trajectory;
            this->robot->computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, trajectory);
            if (!this->terminating) {
                this->robot->execute(trajectory);
            }
        }
    }

    void MoveitBackend::movePYRZInternal(const Eigen::Vector4d &pyrz, bool degrees) {
        this->threads_list_lock.lock();
        this->threads_list.push_back(boost::this_thread::get_id());
        this->threads_list_lock.unlock();
        this->robot->stop();
        if (!this->terminating) {
            GeneralBackend::movePYRZ(pyrz, degrees);
        }
        this->threads_list_lock.lock();
        this->threads_list.remove(boost::this_thread::get_id());
        this->threads_list_lock.unlock();
    }

    void MoveitBackend::movePYRZ(const Eigen::Vector4d &pyrz, bool degrees) {
        if (this->async_motion) {
            boost::thread(&MoveitBackend::movePYRZInternal, this, pyrz, degrees);
        } else {
            this->movePYRZInternal(pyrz, degrees);
        }
    }

}