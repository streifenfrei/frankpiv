#include <moveit/robot_state/conversions.h>

#include "frankpiv/moveit_planner/state_sampler.hpp"
#include "frankpiv/utilities.hpp"

using namespace Eigen;
using namespace frankpiv::util;

namespace frankpiv::moveit_planner {
    PivotStateSampler::PivotStateSampler(const ob::SpaceInformation *si, moveit::core::RobotModelConstPtr robot_model, const std::string &group_name, std::shared_ptr <frankpiv::PivotFrame> pivot_frame) :
            ValidStateSampler(si),
            robot_model(robot_model),
            group_name(group_name),
            pivot_frame(pivot_frame) {
    }

    PivotStateSampler::~PivotStateSampler() = default;

    bool PivotStateSampler::sample(ob::State *state) {
        // TODO account for backends pyrz constraints
        for (size_t i=0; i < this->attempts_; i++ ) {
            Vector4d pyrz{
                rng_.uniformReal(-2 * M_PI, 2 * M_PI),
                rng_.uniformReal(-2 * M_PI, 2 * M_PI),
                rng_.uniformReal(-2 * M_PI, 2 * M_PI),
                (-1, 1) };
            auto pose = this->pivot_frame->getPose(pyrz);
            robot_state::RobotState moveit_state(this->robot_model);
            // TODO use better inverse kinematics solver (one which better handles the ambiguity)
            const auto joint_model_group = this->robot_model->getJointModelGroup(this->group_name);
            if (!moveit_state.setFromIK(joint_model_group, toPoseMsg(pose))) {
                return false;
            }
            toOMPLState(state->as<ob::RealVectorStateSpace::StateType>(), moveit_state, joint_model_group);
            if (this->si_->isValid(state)) {
                return true;
            }
        }
        return false;
    }

    bool PivotStateSampler::sampleNear(ob::State *state, const ob::State *near, double distance) {
        std::cout << "hi" << std::endl;
        return false;
    }
}
